"""미션 비행: H → D(배달) → P(수거) → H, 실내 강당, 광류 위치.

동작
  1. H에서 이륙, 1.5 m 정지 (기수 = 동쪽, D 방향)
  2. H → D 직진 (0.8 m/s)
  3. D: 0.5 m까지 하강 → 5 s 정지 (택배 내려놓기 흉내) → 1.5 m 상승
  4. 오른쪽 90° 회전 → D → P (0.8 m/s)
  5. P: 0.5 m까지 하강 → 5 s 정지 (택배 집기 흉내) → 1.5 m 상승
  6. 오른쪽 135° 회전 → P → H (0.8 m/s)
  7. H 착륙

방식
  - 처음부터 끝까지 PX4 Offboard 모드로 위치 목표를 1초에 20번 보낸다 (뒤에서 계속 전송).
  - GPS가 없으므로 위도·경도가 필요한 모드(Hold, 일반 미션)는 쓰지 않는다.
    Hold 상태에서는 시동(arm)이 거부되므로, 바닥에서 Offboard로 바꾼 뒤 시동을 건다.
  - 이륙도 Offboard로 천천히 (0.3 m/s). 목표점을 조금씩 옮겨서 속도를 맞춘다.
  - H/D/P 위치는 tools/auditorium_config.py 에서 읽는다 (월드와 항상 같음).

좌표
  - PX4 로컬 좌표 NED: x = 북, y = 동, z = 아래 (원점 = 이륙 위치 H)
  - Gazebo 월드 ENU: x = 동, y = 북. 변환: 북 = 월드 y, 동 = 월드 x
  - 기수(yaw): 0° = 북, 90° = 동, 시계방향 +

사용
  1. SIM:  bash ~/parcel-drone-sim/tools/run_mydrone_flow.sh
  2. QGC:  ./QGroundControl-x86_64.AppImage  (연결 확인)
  3. CMD:  python3 scripts/mission_hdph.py
  4. 끝나면 SIM에서 pxh> shutdown, 그다음 분석
  필요: pip3 install --user --break-system-packages "mavsdk<4"
        (mavsdk 4부터는 사용법이 완전히 바뀜. 이 코드는 3.x 기준)
"""
import asyncio
import math
import sys
import time
import warnings
from pathlib import Path

warnings.filterwarnings("ignore", category=FutureWarning)   # mavsdk 이름 변경 안내 숨김 (mavsdk<4 사용)
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
import auditorium_config as C  # noqa: E402

# ── 비행 설정 (임시값. 그리퍼 설계가 나오면 수정) ─────────────────
CRUISE_ALT = 1.5        # 순항 고도 m
WORK_ALT = 0.5          # D, P에서 내려가는 고도 m (임시)
HOLD_S = 5.0            # D, P에서 멈추는 시간 s (임시)
CRUISE_SPEED = 0.8      # 수평 이동 속도 m/s (실제 제한은 run_mydrone_flow.sh의 MPC_XY_VEL_MAX)
VERT_SPEED = 0.3        # D, P에서 오르내리는 속도 m/s
TAKEOFF_SPEED = 0.3     # 이륙 상승 속도 m/s
YAW_RATE = 30.0        # 회전 속도 °/s (11/3 시험: 10 → 30)
RATE_HZ = 20            # 목표 위치를 보내는 횟수 (1초당)
ARRIVE_TOL = 0.15       # 도착 판정 거리 m (드론이 믿는 위치 기준)
CONNECT = "udpin://0.0.0.0:14540"
DT = 1.0 / RATE_HZ


def to_ned(hall_xy):
    """강당 좌표 → PX4 로컬 NED (북, 동). 원점 = H."""
    wx, wy = hall_xy[0] - C.H[0], hall_xy[1] - C.H[1]   # 월드 ENU
    return wy, wx


def heading_deg(a, b):
    """a → b 방향의 기수 (0 = 북, 시계방향 +)."""
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 360


class Mission:
    def __init__(self, drone):
        self.drone = drone
        self.sp = None          # 지금 목표 [북, 동, 아래, 기수]. stream()이 계속 보냄
        self.pos = None         # 드론이 믿는 현재 위치 (북, 동, 아래), track()이 계속 갱신
        self.t0 = time.time()

    def log(self, msg):
        print(f"[{time.time() - self.t0:6.1f} s] {msg}", flush=True)

    async def track(self):
        """드론이 믿는 위치를 계속 받아 둔다 (뒤에서 실행)."""
        async for pv in self.drone.telemetry.position_velocity_ned():
            p = pv.position
            self.pos = (p.north_m, p.east_m, p.down_m)

    async def stream(self):
        """현재 목표를 1초에 RATE_HZ번 계속 보낸다 (뒤에서 실행). 끊기면 PX4가 Offboard를 해제함."""
        while True:
            if self.sp is not None:
                await self.drone.offboard.set_position_ned(PositionNedYaw(*self.sp))
            await asyncio.sleep(DT)

    async def move_to(self, n, e, d, speed, ramp=True):
        """목표점을 (n, e, d)까지 speed m/s로 조금씩 옮기고, 도착할 때까지 기다린다.
        ramp=False: 끝점을 바로 주고 속도는 PX4가 제한 (MPC_XY_VEL_MAX, 시뮬 시간 기준). 가까워지면 PX4가 스스로 감속.
        """
        n0, e0, d0, y = self.sp
        if not ramp:
            self.sp = [n, e, d, y]
            await self.wait_arrive(n, e, d, timeout=60.0)
            return
        dist = math.dist((n0, e0, d0), (n, e, d))
        steps = max(1, int(dist / speed * RATE_HZ))
        for k in range(1, steps + 1):
            f = k / steps
            self.sp = [n0 + (n - n0) * f, e0 + (e - e0) * f, d0 + (d - d0) * f, y]
            await asyncio.sleep(DT)
        await self.wait_arrive(n, e, d)

    async def turn_to(self, yaw):
        """기수를 yaw로 돌린다 (가까운 쪽으로. 이 동선에서는 모두 오른쪽 회전)."""
        y0 = self.sp[3]
        delta = (yaw - y0 + 540) % 360 - 180
        steps = max(1, int(abs(delta) / YAW_RATE * RATE_HZ))
        for k in range(1, steps + 1):
            self.sp[3] = (y0 + delta * k / steps) % 360
            await asyncio.sleep(DT)
        await asyncio.sleep(1.0)

    async def wait_arrive(self, n, e, d, timeout=15.0):
        """드론이 믿는 위치가 목표에서 ARRIVE_TOL 안으로 들어올 때까지 기다린다."""
        t_end = time.time() + timeout
        while time.time() < t_end:
            if self.pos and math.dist(self.pos, (n, e, d)) < ARRIVE_TOL:
                return
            await asyncio.sleep(DT)
        self.log("  (도착 판정 시간 초과, 다음 단계로 진행)")


async def main():
    drone = System()
    await drone.connect(system_address=CONNECT)
    print("PX4 연결 기다리는 중...")
    async for s in drone.core.connection_state():
        if s.is_connected:
            break
    print("위치 추정(광류) 준비 기다리는 중...")
    async for h in drone.telemetry.health():
        if h.is_local_position_ok:
            break

    H, D, P = to_ned(C.H), to_ned(C.D), to_ned(C.P)
    yaw_hd, yaw_dp, yaw_ph = heading_deg(H, D), heading_deg(D, P), heading_deg(P, H)
    print(f"동선 (북, 동) m: H{H} D{D} P{P}")
    print(f"기수: H→D {yaw_hd:.0f}°, D→P {yaw_dp:.0f}°, P→H {yaw_ph:.0f}°")

    m = Mission(drone)
    tasks = [asyncio.create_task(m.track())]
    while m.pos is None:
        await asyncio.sleep(0.1)
    async for att in drone.telemetry.attitude_euler():     # heading()은 GPS 위치가 있어야 나와서 쓰지 않음
        yaw_now = att.yaw_deg % 360
        break

    # 1) 바닥에서 현재 위치를 목표로 보내기 시작 → Offboard 전환 → 시동
    m.sp = [m.pos[0], m.pos[1], m.pos[2], yaw_now]
    tasks.append(asyncio.create_task(m.stream()))
    await asyncio.sleep(1.0)
    try:
        await drone.offboard.start()
    except OffboardError as err:
        m.log(f"Offboard 전환 실패: {err._result.result}")
        return
    m.log("Offboard 전환, 시동")
    for k in range(5):                       # 시작 직후에는 거부될 수 있어 몇 번 다시 시도
        try:
            await drone.action.arm()
            break
        except ActionError as err:
            m.log(f"  시동 거부 ({err._result.result}), 2초 뒤 다시 시도 {k + 1}/5. QGC 'Not Ready' 이유 확인")
            await asyncio.sleep(2)
    else:
        m.log("시동 실패. QGC 왼쪽 위 'Not Ready'를 눌러 이유를 확인하세요.")
        return

    # 2) 이륙
    m.log(f"이륙 ({TAKEOFF_SPEED} m/s)")
    await m.move_to(H[0], H[1], -CRUISE_ALT, TAKEOFF_SPEED)
    await asyncio.sleep(2.0)

    # 3) H → D
    await m.turn_to(yaw_hd)
    m.log("H → D 이동")
    await m.move_to(D[0], D[1], -CRUISE_ALT, CRUISE_SPEED, ramp=False)
    m.log("D 도착, 하강 (택배 내려놓기)")
    await m.move_to(D[0], D[1], -WORK_ALT, VERT_SPEED)
    await asyncio.sleep(HOLD_S)
    await m.move_to(D[0], D[1], -CRUISE_ALT, VERT_SPEED)

    # 4) D → P
    m.log("오른쪽 회전 → P")
    await m.turn_to(yaw_dp)
    await m.move_to(P[0], P[1], -CRUISE_ALT, CRUISE_SPEED, ramp=False)
    m.log("P 도착, 하강 (택배 집기)")
    await m.move_to(P[0], P[1], -WORK_ALT, VERT_SPEED)
    await asyncio.sleep(HOLD_S)
    await m.move_to(P[0], P[1], -CRUISE_ALT, VERT_SPEED)

    # 5) P → H, 착륙 (Hold 모드를 거치지 않고 바로 Land)
    m.log("오른쪽 회전 → H")
    await m.turn_to(yaw_ph)
    await m.move_to(H[0], H[1], -CRUISE_ALT, CRUISE_SPEED, ramp=False)
    m.log("H 도착, 착륙")
    await asyncio.sleep(2.0)
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    m.log("착륙 완료. SIM 터미널에서 pxh> shutdown")
    for t in tasks:
        t.cancel()


if __name__ == "__main__":
    asyncio.run(main())

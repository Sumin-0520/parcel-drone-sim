"""광류 호버 분석: 드론이 "안다고 믿는 위치"(EKF 추정)와 "실제 위치"(Gazebo 정답)를 비교한다.

사용
  python3 tools/analysis/analyze_flow_log.py 결과폴더이름 [ulog 파일 경로]
  예) python3 tools/analysis/analyze_flow_log.py 261031_flow_hover_1p5m
  ulog 경로를 안 주면 PX4 SITL 로그 폴더에서 가장 최근 .ulg를 쓴다.

결과 (results/결과폴더이름/)
  1_top_view.png   위에서 본 경로: 실제(진한 선) vs 추정(점선), 호버 구간만
  2_error_time.png 시간에 따른 수평 오차와 고도
  3_sensors.png    광류 품질(0~255)과 거리계 값
  summary.txt      숫자 요약

용어
  - 실제 위치: vehicle_local_position_groundtruth (시뮬레이션만 있는 정답)
  - 추정 위치: vehicle_local_position (FC가 광류·거리계·IMU로 계산한 값, 제어에 쓰는 값)
  - 수평 오차 = |추정 - 실제| (x, y). FC는 추정 위치를 고정하려고 하므로, 이 오차가 곧 실제로 흘러간 양이다.
  - 호버 구간: 이륙 고도의 90 %에 처음 도달한 뒤 5 s부터, 착륙 하강 시작 1 s 전까지
좌표: PX4 NED (x 북, y 동, z 아래). 그림에서는 x = 동, y = 북으로 바꿔 그린다 (Gazebo 화면과 같은 방향).
"""
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyulog import ULog

REPO = Path(__file__).resolve().parents[2]   # 저장소 맨 위 (tools/analysis/ 에서 두 단계 위)
LOG_ROOT = Path.home() / "PX4-Autopilot/build/px4_sitl_default/rootfs/log"

# 차분한 색
C_TRUTH = "#3d5a80"   # 실제
C_EST = "#c08457"     # 추정
C_ERR = "#6b4e71"
C_ALT = "#5c7a5c"
C_GRID = "#d9d9d9"


def latest_ulog():
    logs = sorted(LOG_ROOT.glob("*/*.ulg"), key=lambda p: p.stat().st_mtime)
    if not logs:
        sys.exit(f"로그가 없습니다: {LOG_ROOT}")
    return logs[-1]


def topic(ulog, name):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == 0:
            return d.data
    return None


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    out = REPO / "results" / sys.argv[1]
    path = Path(sys.argv[2]) if len(sys.argv) > 2 else latest_ulog()
    out.mkdir(parents=True, exist_ok=True)

    ulog = ULog(str(path), ["vehicle_local_position", "vehicle_local_position_groundtruth",
                            "vehicle_optical_flow", "distance_sensor"])
    est = topic(ulog, "vehicle_local_position")
    tru = topic(ulog, "vehicle_local_position_groundtruth")
    if est is None or tru is None:
        sys.exit("필요한 기록(vehicle_local_position / _groundtruth)이 로그에 없습니다.")

    t0 = tru["timestamp"][0]
    tt = (tru["timestamp"] - t0) / 1e6
    te = (est["timestamp"] - t0) / 1e6

    # 추정값을 실제값 시간에 맞춤 (선형 보간)
    ex = np.interp(tt, te, est["x"]); ey = np.interp(tt, te, est["y"]); ez = np.interp(tt, te, est["z"])
    tx, ty, tz = tru["x"], tru["y"], tru["z"]

    # 이륙 시점: 실제 고도가 처음보다 0.2 m 이상 올라간 때
    lift = np.where(-(tz - np.median(tz[tt < 3])) > 0.2)[0]
    if len(lift) == 0:
        sys.exit("이 로그에는 이륙 기록이 없습니다.")
    t_lift = tt[lift[0]]

    # 기준점 = 이륙 직전 바닥에 있을 때 (로그 맨 처음은 EKF가 안정되기 전이라 쓰지 않음)
    ground = (tt > t_lift - 3.0) & (tt < t_lift - 0.5)
    if ground.sum() < 5:
        ground = tt < t_lift
    ex, ey, ez = ex - np.median(ex[ground]), ey - np.median(ey[ground]), ez - np.median(ez[ground])
    tx, ty, tz = tx - np.median(tx[ground]), ty - np.median(ty[ground]), tz - np.median(tz[ground])
    alt, alt_est = -tz, -ez                       # 바닥 기준 실제 고도, 추정 고도

    # 이륙 고도와 호버 구간 (FC가 맞추려는 값 = 추정 고도로 판단)
    params = dict(ulog.initial_parameters)
    for _, name, val in ulog.changed_parameters:
        params[name] = val
    target = float(params.get("MIS_TAKEOFF_ALT", 2.5))
    above = np.where(alt_est > 0.9 * target)[0]
    if len(above) == 0:
        sys.exit(f"추정 고도가 {target} m에 도달한 구간이 없습니다. (최고: 추정 {alt_est.max():.2f} m, 실제 {alt.max():.2f} m)")
    t_start, t_end = tt[above[0]] + 5.0, tt[above[-1]] - 1.0
    hov = (tt >= t_start) & (tt <= t_end)
    if t_end - t_start < 10:
        sys.exit(f"호버 구간이 {t_end - t_start:.0f} s로 너무 짧습니다 (15 s 이상 호버해 주세요).")

    err = np.hypot(ex - tx, ey - ty)                                  # 수평 추정 오차
    move = np.hypot(tx - tx[hov][0], ty - ty[hov][0])                 # 호버 시작점에서 실제로 움직인 거리
    dur = t_end - t_start

    # ── 그림 1: 위에서 본 경로 ──
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(ty[hov], tx[hov], color=C_TRUTH, lw=1.8, label="actual (groundtruth)")
    ax.plot(ey[hov], ex[hov], color=C_EST, lw=1.4, ls="--", label="estimated (EKF)")
    ax.plot(ty[hov][0], tx[hov][0], "o", color=C_TRUTH, ms=6, label="hover start")
    ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
    ax.set_aspect("equal", "datalim"); ax.grid(color=C_GRID); ax.legend(frameon=False)
    ax.set_title(f"Hover top view ({dur:.0f} s, {target:g} m)")
    fig.tight_layout(); fig.savefig(out / "1_top_view.png", dpi=150); plt.close(fig)

    # ── 그림 2: 시간에 따른 오차와 고도 ──
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 5.5), sharex=True)
    a1.plot(tt, err * 100, color=C_ERR, lw=1.2)
    a1.axvspan(t_start, t_end, color=C_GRID, alpha=0.5, lw=0)
    a1.set_ylabel("horizontal error [cm]"); a1.grid(color=C_GRID)
    a2.plot(tt, alt, color=C_ALT, lw=1.4, label="actual")
    a2.plot(tt, -ez, color=C_EST, lw=1.0, ls="--", label="estimated")
    a2.axvspan(t_start, t_end, color=C_GRID, alpha=0.5, lw=0)
    a2.set_ylabel("altitude [m]"); a2.set_xlabel("time [s]"); a2.grid(color=C_GRID); a2.legend(frameon=False)
    fig.suptitle("Estimation error (shaded = hover window)")
    fig.tight_layout(); fig.savefig(out / "2_error_time.png", dpi=150); plt.close(fig)

    # ── 그림 3: 센서 ──
    flow, rng = topic(ulog, "vehicle_optical_flow"), topic(ulog, "distance_sensor")
    q_mean = None
    fig, (b1, b2) = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    if flow is not None and "quality" in flow:
        tf = (flow["timestamp"] - t0) / 1e6
        b1.plot(tf, flow["quality"], color=C_TRUTH, lw=0.8)
        m = (tf >= t_start) & (tf <= t_end)
        q_mean = float(np.mean(flow["quality"][m])) if m.any() else None
    b1.set_ylabel("flow quality [0-255]"); b1.grid(color=C_GRID)
    if rng is not None:
        b2.plot((rng["timestamp"] - t0) / 1e6, rng["current_distance"], color=C_ALT, lw=1.0)
    b2.set_ylabel("rangefinder [m]"); b2.set_xlabel("time [s]"); b2.grid(color=C_GRID)
    for b in (b1, b2):
        b.axvspan(t_start, t_end, color=C_GRID, alpha=0.5, lw=0)
    fig.tight_layout(); fig.savefig(out / "3_sensors.png", dpi=150); plt.close(fig)

    # ── 요약 ──
    lines = [
        f"로그: {path}",
        f"이륙 고도 설정: {target:g} m",
        f"호버 구간: {t_start:.1f} ~ {t_end:.1f} s ({dur:.1f} s)",
        f"평균 고도: 실제 {alt[hov].mean():.2f} m, 추정 {alt_est[hov].mean():.2f} m (고도 추정 오차 {(alt_est[hov] - alt[hov]).mean() * 100:+.1f} cm)",
        "",
        "[수평 추정 오차 = 실제로 흘러간 양]",
        f"  평균 {err[hov].mean() * 100:.1f} cm, 최대 {err[hov].max() * 100:.1f} cm, 호버 끝 {err[hov][-1] * 100:.1f} cm",
        f"  호버 중 증가율 {(err[hov][-1] - err[hov][0]) * 100 / dur * 60:.1f} cm/분",
        "",
        "[호버 시작점에서 실제 이동 거리]",
        f"  최대 {move[hov].max() * 100:.1f} cm, 호버 끝 {move[hov][-1] * 100:.1f} cm",
        "",
        f"광류 품질 평균 (호버 구간): {q_mean:.0f} / 255" if q_mean is not None else "광류 품질: 기록 없음",
    ]
    (out / "summary.txt").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))
    print("\n저장:", out)


if __name__ == "__main__":
    main()

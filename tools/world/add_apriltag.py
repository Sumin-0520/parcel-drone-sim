"""택배 상자(P) 윗면에 AprilTag(tag36h11)를 붙인다. (4번: 카메라 + AprilTag 정밀 접근 준비)

사용
  python3 tools/world/add_apriltag.py              # 기본: id 0, 바깥(흰 여백 포함) 0.18 m → 검은 사각형 0.144 m
  python3 tools/world/add_apriltag.py --size 0.15  # 바깥 크기만 바꿈 (상자 윗면 0.25 x 0.20 m 안에 들어가야 함)
  python3 tools/world/add_apriltag.py remove       # 태그 떼기 (worlds/auditorium.sdf 원래대로)

하는 일
  1. models/apriltag_36h11_0/ 에 태그 판(tag.obj + tag.mtl, 그림 tag.png) 생성
     - 그림: 8 x 8 셀 태그 + 흰 여백 1셀 = 10 x 10 셀 → 검은 사각형 = 바깥 크기 x 0.8
     - 판에 그림 좌표(UV)를 직접 넣어서 위에서 봤을 때 뒤집히지 않게 함 (뒤집히면 인식 안 됨)
  2. worlds/auditorium.sdf 의 택배 상자(parcel) 윗면에 그 판을 붙임 (보이기만 함, 충돌·무게 없음)
     - 파일 경로는 이 PC의 절대 경로로 들어감 (다른 PC에서는 이 스크립트를 다시 실행)
  3. 인식 코드가 쓸 태그 크기를 tools/world/apriltag_config.txt 에 저장
"""
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
WORLD = REPO / "worlds/auditorium.sdf"
TAG_DIR = REPO / "models/apriltag_36h11_0"
CONFIG = REPO / "tools/world/apriltag_config.txt"
sys.path.insert(0, str(REPO / "tools"))
import auditorium_config as C  # noqa: E402

START, END = "<!-- apriltag start -->", "<!-- apriltag end -->"


def write_mesh(size):
    a = size / 2
    (TAG_DIR / "tag.obj").write_text(
        "mtllib tag.mtl\nusemtl tag\n"
        f"v {-a} {-a} 0\nv {a} {-a} 0\nv {a} {a} 0\nv {-a} {a} 0\n"
        "vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\n"
        "f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n")
    (TAG_DIR / "tag.mtl").write_text(
        "newmtl tag\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nNs 1\nd 1\nillum 1\nmap_Kd tag.png\n")


def strip(s):
    return re.sub(r"\s*" + re.escape(START) + r".*?" + re.escape(END), "", s, flags=re.S)


def main():
    args = sys.argv[1:]
    s = WORLD.read_text(encoding="utf-8")
    if args == ["remove"]:
        WORLD.write_text(strip(s), encoding="utf-8")
        print("태그 뗌 (worlds/auditorium.sdf)")
        return
    size = 0.18
    if "--size" in args:
        size = float(args[args.index("--size") + 1])
    bx, by, bz = C.PARCEL_SIZE
    if size > min(bx, by):
        sys.exit(f"태그 {size} m가 상자 윗면 {bx} x {by} m보다 큼")
    if not (TAG_DIR / "tag.png").exists():
        sys.exit(f"{TAG_DIR}/tag.png 가 없음 (zip을 다시 풀어 주세요)")
    write_mesh(size)

    vis = (f"\n        {START}\n"
           f'        <visual name="apriltag"><pose>0 0 {bz / 2 + 0.0008:.4f} 0 0 0</pose>'
           f"<geometry><mesh><uri>{TAG_DIR / 'tag.obj'}</uri></mesh></geometry>"
           f"<cast_shadows>false</cast_shadows></visual>\n        {END}")
    s = strip(s)
    m = re.search(r'<model name="parcel">.*?(</link>)', s, flags=re.S)
    if not m:
        sys.exit("worlds/auditorium.sdf 에서 parcel 모델을 못 찾음")
    s = s[:m.start(1)] + vis + "\n      " + s[m.start(1):]
    WORLD.write_text(s, encoding="utf-8")

    black = size * 0.8
    CONFIG.write_text(f"family tag36h11\nid 0\nouter_m {size}\nblack_m {black}\n", encoding="utf-8")
    px, py = C.P
    print(f"태그 붙임: tag36h11 id 0, 바깥 {size} m (검은 사각형 {black:.3f} m)")
    print(f"  위치: 택배 상자 윗면 = P (강당 {px}, {py}), 바닥에서 {bz:.2f} m")
    print("  시뮬을 다시 켜야 보임: bash tools/run_mydrone_flow_test.sh")


if __name__ == "__main__":
    main()

"""결과 폴더에 그 비행의 로그(flight.ulg)와 PX4 파라미터 전체(params.txt)를 붙인다. (GitHub에 올리기 전에)

사용
  python3 tools/analysis/attach_log.py results/261120_hover_2600g_cg-14_air2216
  python3 tools/analysis/attach_log.py results/폴더1 results/폴더2 ...      # 여러 개 한 번에
  python3 tools/analysis/attach_log.py results/폴더 로그경로.ulg             # 로그를 직접 지정

로그 찾는 법
  1. summary.txt 첫 줄 "로그: ..." 에 적힌 파일 (analyze_hover.py, analyze_mission_log.py 결과)
  2. 없으면 (tag_approach_test.py 결과) summary.txt를 만든 시각과 가장 가까운 로그

만드는 것
  flight.ulg        비행 로그 원본 복사 (.gitignore에 *.ulg가 있어서 올릴 때 git add -f 필요)
  params.txt        비행 당시 PX4 파라미터 전체 (이름 값)
  params_changed.txt  PX4 기본값과 다른 파라미터만 (기본값이 로그에 있을 때)
"""
import shutil
import sys
from pathlib import Path

from pyulog import ULog

LOG_ROOT = Path.home() / "PX4-Autopilot/build/px4_sitl_default/rootfs/log"


def find_log(folder):
    s = folder / "summary.txt"
    if s.exists():
        first = s.read_text(encoding="utf-8").splitlines()[0]
        if first.startswith("로그:"):
            p = Path(first.split(":", 1)[1].strip())
            if p.exists():
                return p
        t = s.stat().st_mtime
        logs = list(LOG_ROOT.glob("*/*.ulg"))
        if logs:
            return min(logs, key=lambda p: abs(p.stat().st_mtime - t))
    return None


def fmt(v):
    return f"{v:g}" if isinstance(v, float) else str(v)


def attach(folder, log=None):
    folder = Path(folder)
    if not folder.is_dir():
        print(f"[건너뜀] 폴더 없음: {folder}")
        return
    log = Path(log) if log else find_log(folder)
    if log is None or not log.exists():
        print(f"[건너뜀] {folder.name}: 로그를 못 찾음 → 경로를 직접 주세요")
        return
    shutil.copy2(log, folder / "flight.ulg")
    u = ULog(str(log), ["vehicle_status"])
    p = u.initial_parameters
    (folder / "params.txt").write_text(
        f"# {log.name} 비행 당시 PX4 파라미터 {len(p)}개\n" +
        "".join(f"{k} {fmt(p[k])}\n" for k in sorted(p)), encoding="utf-8")
    msg = f"params.txt {len(p)}개"
    try:
        d = u.get_default_parameters(0)
    except Exception:
        d = {}
    if d:
        ch = [k for k in sorted(p) if k in d and p[k] != d[k]]
        (folder / "params_changed.txt").write_text(
            "# PX4 기본값과 다른 파라미터 (이름  비행값  기본값)\n" +
            "".join(f"{k} {fmt(p[k])} {fmt(d[k])}\n" for k in ch), encoding="utf-8")
        msg += f", 기본값과 다른 것 {len(ch)}개"
    mb = (folder / "flight.ulg").stat().st_size / 1e6
    print(f"[완료] {folder.name}: {log.name} ({mb:.1f} MB), {msg}")


def main():
    args = sys.argv[1:]
    if not args:
        sys.exit(__doc__)
    if len(args) == 2 and args[1].endswith(".ulg"):
        attach(args[0], args[1])
    else:
        for a in args:
            attach(a)


if __name__ == "__main__":
    main()

# 최근 비행 기록(.ulg)에서 호버 구간의 모터 출력·토크 명령·자세를 분석하고 그래프 저장
import glob, os, sys
import numpy as np
from pyulog import ULog
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt

logs = sorted(glob.glob(os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg')), key=os.path.getmtime)
f = sys.argv[1] if len(sys.argv) > 1 else logs[-1]
tag = sys.argv[2] if len(sys.argv) > 2 else 'run'
u = ULog(f)
names = sorted({d.name for d in u.data_list})
print('기록 파일:', f)
print('관련 토픽:', [n for n in names if any(k in n for k in ('torque', 'motor', 'actuator', 'attitude'))])

def get(name):
    for d in u.data_list:
        if d.name == name: return d.data
    return None

pos = get('vehicle_local_position')
t0 = pos['timestamp'][0]
z = -pos['z']; tp = (pos['timestamp'] - t0) / 1e6
hov = tp[(z > 2.3)]                         # 2.3 m 이상 떠 있던 시간
if len(hov) < 2: sys.exit('호버 구간을 못 찾음 (2.3 m 이상 비행 기록 없음)')
a, b = hov[0] + 5, hov[-1] - 1              # 상승 직후 5초 제외
print(f'호버 구간: {a:.1f} ~ {b:.1f} s')

def win(d, key):
    t = (d['timestamp'] - t0) / 1e6; m = (t > a) & (t < b)
    return t, d[key], m

out = []
mot = get('actuator_motors')
if mot is not None:
    vals = []
    for i in range(4):
        t, v, m = win(mot, f'control[{i}]'); vals.append(v[m].mean())
    print('모터 평균 출력 (0 앞오른, 1 뒤왼, 2 앞왼, 3 뒤오른):', [round(x, 3) for x in vals])
    print(f'  앞(0,2) 평균 {np.mean([vals[0], vals[2]]):.3f} / 뒤(1,3) 평균 {np.mean([vals[1], vals[3]]):.3f}')
    out.append(('모터 출력', mot, [f'control[{i}]' for i in range(4)], ['0 앞오른', '1 뒤왼', '2 앞왼', '3 뒤오른']))
tq = get('vehicle_torque_setpoint')
if tq is not None:
    for i, ax in enumerate(['롤', '피치', '요']):
        t, v, m = win(tq, f'xyz[{i}]'); print(f'토크 명령 {ax} 평균 {v[m].mean():+.4f}')
    out.append(('토크 명령', tq, ['xyz[0]', 'xyz[1]', 'xyz[2]'], ['롤', '피치', '요']))

fig, axs = plt.subplots(len(out) + 1, 1, figsize=(9, 3 * (len(out) + 1)), sharex=True)
axs[0].plot(tp, z); axs[0].set_ylabel('altitude [m]'); axs[0].axvspan(a, b, alpha=0.15)
for axp, (title, d, keys, labels) in zip(axs[1:], out):
    t = (d['timestamp'] - t0) / 1e6
    for k, l in zip(keys, labels): axp.plot(t, d[k], label=k)
    axp.set_ylabel(title.replace('모터 출력', 'motor').replace('토크 명령', 'torque sp')); axp.legend(fontsize=7); axp.axvspan(a, b, alpha=0.15)
axs[-1].set_xlabel('time [s]')
png = os.path.expanduser(f'~/parcel-drone-sim/results/{tag}.png'); plt.tight_layout(); plt.savefig(png, dpi=120)
print('그래프 저장:', png)

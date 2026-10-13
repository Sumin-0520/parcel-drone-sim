# 설명: THR_MDL_FAC에 따른 피치 토크 명령 요약 그래프 (results/summary_thrmdl.png)
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt, os
f = [0, 0.65, 0.75, 1.0]; tq = [0.047, 0.0065, 0.0005, -0.015]
plt.figure(figsize=(6, 3.6)); plt.axhline(0, color='gray', lw=0.8)
plt.axhline(-0.067, color='#B07A5E', ls='--', lw=1, label='A: CG unknown to FC (-0.067)')
plt.plot(f, tq, 'o-', color='#3E5F8A', label='CG given to FC (B,D,E,C)')
for x, y, n in zip(f, tq, ['B', 'D', 'E', 'C']): plt.annotate(n, (x, y), textcoords='offset points', xytext=(6, 6))
plt.xlabel('THR_MDL_FAC'); plt.ylabel('pitch torque setpoint'); plt.legend(fontsize=8); plt.tight_layout()
p = os.path.expanduser('~/parcel-drone-sim/results/summary_thrmdl.png'); plt.savefig(p, dpi=150); print('저장:', p)

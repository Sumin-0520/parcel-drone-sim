import csv, sys, os, re
here = os.path.dirname(os.path.abspath(__file__))
rows = list(csv.reader(open(os.path.join(here, 'parts.csv'), encoding='utf-8')))[1:]
M = sum(float(r[1]) for r in rows)
cg = [sum(float(r[1]) * float(r[i]) for r in rows) / M for i in (2, 3, 4)]
print(f'전체 무게 {M:.0f} g')
print(f'무게중심 x {cg[0]:+.1f} mm (앞+) / y {cg[1]:+.1f} mm (왼+) / z {cg[2]:+.1f} mm (위+)')
if len(sys.argv) > 1 and sys.argv[1] == 'apply':
    f = os.path.expanduser('~/parcel-drone-sim/models/x500_mydrone_base/model.sdf')
    m = M / 1000 - 0.064; k = m / 2.0
    s = open(f).read()
    i = re.search(r"<link name=[\"']base_link[\"']>", s).end()
    a = s.index('<inertial>', i); b = s.index('</inertial>', a) + len('</inertial>')
    new = (f"<inertial><pose>{cg[0]/1000:.4f} {cg[1]/1000:.4f} {cg[2]/1000:.4f} 0 0 0</pose><mass>{m:.3f}</mass><inertia>"
           f"<ixx>{0.0216667*k:.5f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{0.0216667*k:.5f}</iyy><iyz>0</iyz><izz>{0.04*k:.5f}</izz></inertia></inertial>")
    open(f, 'w').write(s[:a] + new + s[b:])
    print('가제보 모델에 적용 완료 (몸통', round(m, 3), 'kg, 관성은 무게 비율 근사)')

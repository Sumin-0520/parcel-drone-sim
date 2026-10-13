import re, sys, os
f = os.path.expanduser('~/parcel-drone-sim/models/x500_mydrone_base/model.sdf')
cgx = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0   # 무게중심 앞(+)/뒤(-) 이동, m
m = 2.286                                                # 2.35 kg - 로터 4개(약 0.064 kg)
k = m / 2.0                                              # 관성은 무게 비율로 근사(추정)
s = open(f).read()
i = re.search(r"<link name=[\"']base_link[\"']>", s).end()
a = s.index('<inertial>', i); b = s.index('</inertial>', a) + len('</inertial>')
new = (f"<inertial><pose>{cgx} 0 0 0 0 0</pose><mass>{m}</mass><inertia>"
       f"<ixx>{0.0216667*k:.5f}</ixx><ixy>0</ixy><ixz>0</ixz>"
       f"<iyy>{0.0216667*k:.5f}</iyy><iyz>0</iyz><izz>{0.04*k:.5f}</izz></inertia></inertial>")
open(f, 'w').write(s[:a] + new + s[b:])
print('설정 완료: 질량', m, 'kg / 무게중심 앞으로', cgx * 100, 'cm')

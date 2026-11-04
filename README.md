# parcel-drone-sim — 파일 안내

자율 택배 드론 졸업설계의 가상환경(PX4 SITL + Gazebo) 저장소.
파일이 추가되거나 역할이 바뀌면 이 문서를 같이 고친다.

## 규칙
- **파일·폴더 이름은 바꾸지 않는다.** PX4가 정확한 이름으로 찾는다.
- 결과 폴더 이름: `results/날짜_실험이름/` (예: `results/261031_flow_hover_1p5m/`)
- **"생성됨" 표시 파일은 직접 고치지 않는다.** 만드는 스크립트나 설정 파일을 고친 뒤 다시 생성한다.
- 설명은 각 폴더의 `설명.md`와 이 README에 적는다.

## 폴더 구조
~~~
parcel-drone-sim/
├─ .vscode/          VS Code 설정 (터미널 자동 생성)
├─ missions/         미션 경로 (예정)
├─ scripts/          자율 임무 코드 (미션 비행)
├─ models/           Gazebo 모델 (기체, 강당 바닥)
├─ worlds/           Gazebo 월드 (강당)
├─ results/          실험 결과 (그래프, 요약)
└─ tools/            계산·생성·분석 도구
   ├─ world/         강당 월드·바닥 그림 생성
   ├─ vehicle/       기체 무게중심 계산
   └─ analysis/      비행 로그 분석
~~~

## .vscode
| 파일 | 역할 |
|---|---|
| tasks.json | 폴더를 열면 터미널 3개(SIM, QGC, CMD)를 자동으로 만든다 |

## models — 기체와 바닥
| 경로 | 역할 | 직접 수정 |
|---|---|---|
| x500_mydrone/model.sdf | **모터 4개**(회전 방향, 최대 회전수, 추력 계수) + **광류 카메라·거리계** (앞 8 cm, 하판 아래 2 cm). 위치는 `[위치]` 주석의 pose 숫자만 고침 | O |
| x500_mydrone/model.config | 모델 이름표 | X |
| x500_mydrone/thumbnails/, LICENSE | PX4에서 복사된 부속 파일 | X |
| x500_mydrone_base/model.sdf | **몸체**: 무게·무게중심·관성, 외형(기본 X500, 다리 포함), 로터 위치, 기본 센서(IMU, 기압계, 지자기, GPS). IMU는 FC 위치(하판 아랫면 위 36 mm) | 무게 부분은 `tools/vehicle/cg_calc.py apply`가 자동 수정 |
| x500_mydrone_base/model.config | 모델 이름표 | X |
| auditorium_floor/model.sdf, model.config | 강당 바닥 모델 (크기, 그림 연결) | X (생성됨) |
| auditorium_floor/materials/textures/floor_wood.jpg | 나무 마루 바닥 그림 (30 × 20 m, 코트 선 포함) | X (생성됨) |
| auditorium_floor/materials/textures/floor_mat.png | 이전 무늬 매트 그림 (지금은 안 씀) | — |

## worlds — 월드
| 파일 | 역할 | 직접 수정 |
|---|---|---|
| auditorium.sdf | 실내 강당 월드 (바닥, 벽, H/D/P 표시, 택배 상자) | X (생성됨) |
| 설명.md | 강당 크기, 동선 좌표표 | O |

## tools — 도구
### tools/ (바로 아래: 자주 쓰는 것)
| 파일 | 역할 | 실행 |
|---|---|---|
| run_mydrone_flow.sh | **우리 기체 + 광류 실행** (센서·IMU 위치, 무게중심 보정값(10/13 실험 E), 기수·나침반 설정, 수평 속도 제한, 남은 Gazebo 정리, 끝나면 Gazebo 자동 종료) | `bash tools/run_mydrone_flow.sh` |
| **auditorium_config.py** | **강당 설정값** (크기, H/D/P 위치, 택배, 판자 폭). 월드·미션·분석이 같이 읽음. 강당을 바꿀 때 여기만 고침 | — |

### tools/world/ — 강당 월드 만들기
| 파일 | 역할 | 실행 |
|---|---|---|
| make_auditorium_world.py | 설정값으로 강당 월드와 바닥 모델 생성 | `python3 tools/world/make_auditorium_world.py` |
| make_floor_texture.py | 참고 사진으로 나무 마루 그림 생성 (numpy, Pillow 필요) | `python3 tools/world/make_floor_texture.py` |
| ref/floor_ref_topview.png | 바닥 참고 사진 (학교 체육관 거리뷰 캡처, 시험용) | — |

### tools/vehicle/ — 기체 무게중심
| 파일 | 역할 | 실행 |
|---|---|---|
| **parts.csv** | **부품 무게·위치 표** (무게중심 계산 입력) | — |
| cg_calc.py | 무게중심 계산. `apply`를 붙이면 몸체 모델에 적용 | `python3 tools/vehicle/cg_calc.py apply` |

### tools/analysis/ — 비행 로그 분석
| 파일 | 역할 | 실행 |
|---|---|---|
| analyze_mission_log.py | **미션 비행 분석** (계획 경로 vs 추정 vs 실제, H·D·P 도착 오차, 기수 오차) | `python3 tools/analysis/analyze_mission_log.py 결과폴더 [로그경로]` |
| analyze_flow_log.py | 광류 호버 분석 (추정 위치 vs 실제 위치, 고도, 센서) | `python3 tools/analysis/analyze_flow_log.py 결과폴더 [로그경로]` |
| analyze_log.py | 호버 로그 분석 (모터 출력, 피치 토크) — 10/13 실험용 | `python3 tools/analysis/analyze_log.py [로그경로] [이름]` |
| plot_summary.py | THR_MDL_FAC별 피치 토크 요약 그래프 — 10/13 실험용 | `python3 tools/analysis/plot_summary.py` |

## scripts — 자율 임무
| 파일 | 역할 | 실행 |
|---|---|---|
| mission_hdph.py | **미션 비행 H → D → P → H** (Offboard, 기수 회전, D·P에서 0.5 m 하강 5초 정지). H/D/P 위치는 auditorium_config.py에서 읽음 | `python3 scripts/mission_hdph.py` (run_mydrone_flow.sh로 켠 뒤) |

## results — 실험 결과
| 폴더 | 내용 |
|---|---|
| 261013_hovering/ | 무게중심 보정 호버 실험 A~E (THR_MDL_FAC 0.75에서 피치 토크 ≈ 0) |
| 261031_mydrone_flow_hover_cg/ | 나무 마루 강당, 우리 기체 광류 호버 1.5 m. 호버 중 이동 2.6 cm, 광류 품질 218/255 |
| 261103_mission_final_5_vel08/ | **최종 미션** H→D→P→H (기수 수정 + 속도 제한 0.8 m/s). D 22 · P 19 · H 49 cm, 평균 추정 오차 26 cm, 기수 오차 2.3° 이하 |
| 설명.md | 결과 폴더 설명 |

## PX4 쪽에 연결된 것 (바로가기, 직접 수정 X)
`~/PX4-Autopilot/Tools/simulation/gz/` 안에 우리 파일을 가리키는 바로가기가 있다.
- models/x500_mydrone, models/x500_mydrone_base, models/auditorium_floor
- worlds/auditorium.sdf

## 자동 설정 (~/.bashrc)
| 줄 | 뜻 |
|---|---|
| `export PX4_PARAM_MIS_TAKEOFF_ALT=1.5` | 이륙 고도 1.5 m |
| `export PX4_PARAM_SIM_BAT_MIN_PCT=100` | 가상 배터리 무제한 (연습용. 실제 실험 때 배터리 사양으로 변경) |

## 실행
~~~bash
# CMD: 정리 (결과가 비어 있어야 함)
pkill -9 -f "gz sim"; pkill -9 -f px4; sleep 2; pgrep -a gz

# SIM: 우리 기체 + 광류 (추천, 정리까지 자동)
bash ~/parcel-drone-sim/tools/run_mydrone_flow.sh

# SIM: 다른 실행 방법
PX4_SYS_AUTOSTART=4021 PX4_GZ_WORLD=auditorium PX4_SIM_MODEL=gz_x500_mydrone ./build/px4_sitl_default/bin/px4   # 우리 기체 + 광류 (GPS 끔)
PX4_SYS_AUTOSTART=4001 PX4_GZ_WORLD=auditorium PX4_SIM_MODEL=gz_x500_mydrone ./build/px4_sitl_default/bin/px4   # 우리 기체 + GPS
PX4_GZ_WORLD=auditorium make px4_sitl gz_x500_flow                                                                 # PX4 기본 광류 기체

# QGC
./QGroundControl-x86_64.AppImage

# CMD: 미션 비행 → 끝나면 분석
python3 scripts/mission_hdph.py
python3 tools/analysis/analyze_mission_log.py 결과폴더이름 "로그경로.ulg"   # CMD: 미션 분석

# pxh>
commander takeoff      # 이륙 (1.5 m)
commander land         # 착륙 → "Landing detected" 확인
shutdown               # PX4 정상 종료 (로그 저장)
~~~

## 변경 기록
| 날짜 | 내용 |
|---|---|
| 10/13 | 저장소 생성. 우리 기체 모델(무게 2,283 g, 무게중심 x −17.7 mm), 호버 실험 A~E |
| 10/31 | 터미널 자동 생성, 강당 월드(30 × 20 m, 나무 마루), 동선 H→D 10 m→P 10 m→H, 광류 분석 도구, 우리 기체에 광류·거리계 추가 (앞 8 cm, 아래 2 cm) |
| 10/31 | IMU를 FC 위치로, 실행 스크립트 run_mydrone_flow.sh (PX4 센서 위치 설정 포함) |
| 10/31 | 실행 스크립트에 10/13 무게중심 보정값 고정 (CA_ROTOR 앞 0.192 / 뒤 −0.156, THR_MDL_FAC 0.75). 기체 번호 변경 시 초기화되던 문제 해결 |
| 10/31 | 실행 스크립트: PX4 종료 시 Gazebo 자동 종료 (메시지가 계속 찍히던 문제) |
| 10/31 | 미션 비행 스크립트 scripts/mission_hdph.py 추가 (MAVSDK 3.x, `mavsdk<4` 설치) |
| 10/31 | 미션 분석 도구 tools/analyze_mission_log.py 추가 |
| 10/31 | 실행 스크립트에 EKF2_MAG_TYPE=6 고정 (기본값 '자동'은 이륙 순간 기수가 튀어 경로가 옆으로 휘고 추락까지 발생) |
| 11/2 | 이륙 시 기수가 왼쪽으로 도는 원인 확정: 바닥·0.5 m 아래의 흐릿한 광류가 기수 추정을 튀게 함. 실행 스크립트에 SENS_FLOW_MINHGT=0.5 고정 → 이륙 후 기수 오차 11~26° → 0~6° |
| 11/2 | 나침반 설정 고정: EKF2_HEAD_NOISE=0.05, EKF2_MAG_NOISE=0.01, EKF2_MAG_DECL=-2.5 → 이륙 후 기수 오차 0.1~0.3° (시뮬레이션 전용 값, 실기체는 강당 자기장 확인 필요) |
| 11/2 | 기수 틀어짐 원인 수정 (SENS_FLOW_MINHGT 0.5, EKF2_HEAD_NOISE 0.05, MAG_NOISE 0.01, MAG_DECL −2.5), 미션 결과 기록 |
| 11/3 | 미션 회전 속도 10 → 30°/s (scripts/mission_hdph.py YAW_RATE) |
| 11/3 | 수평 속도를 PX4가 제한 (MPC_XY_VEL_MAX·MPC_XY_CRUISE 0.8, 미션 수평 이동 ramp=False) → 도착 전 감속, 고도 오차 해소. 최종 미션 5-3: D 22 · P 19 · H 49 cm |
| 11/4 | 결과 폴더 정리: 최종 미션(5-3)만 남기고 이전 미션 결과 삭제 (원본 로그는 바탕화면 드론 시뮬/logs에 보관). set_cg.py 삭제 |
| 11/4 | tools 폴더 분류: world/(월드), vehicle/(무게중심), analysis/(로그 분석). run_mydrone_flow.sh·auditorium_config.py는 tools/ 바로 아래 유지 |

# alice_lab

Isaac Lab 3.0에서 Alice M1(바퀴형 모바일 양팔 휴머노이드)을 띄워 보는 테스트 모음입니다.
1단계 목표는 **베이스를 고정한 상태에서 양팔을 움직이고, 허리 리프트가 0 자세를 유지하는지** 확인하는 것입니다.

## 구성

```
alice_lab/
├── alice_m1_cfg.py          # ArticulationCfg (USD 경로, 초기 자세, 액추에이터 그룹)
└── scripts/
    └── 01_move_arms.py      # 1단계 테스트: 양팔 사인파 + 허리·몸통 진단 출력
```

이 폴더 밖에 있는 관련 파일은 다음과 같습니다.

| 경로 | 내용 |
|---|---|
| `~/arobot_ws/alice_m1_urdf/alice_m1.urdf` | 원본 URDF. 수정하지 않습니다 |
| `~/arobot_ws/alice_m1_urdf/alice_m1_sim.urdf` | 시뮬레이션용 사본(limit·질량 보정) |
| `~/arobot_ws/alice_m1_urdf/alice_m1_sim_notes.md` | 사본의 변경 내역, 관절 방향, 고정 프레임 변환값 |
| `~/arobot_ws/alice_m1_usd/alice_m1_sim/alice_m1_sim.usda` | `alice_m1_sim.urdf`를 변환한 USD. cfg가 이 파일을 읽습니다 |

## 필요 환경

- `~/vla_ws/IsaacLab`: Isaac Lab `release/3.0.0` 체크아웃. uv 환경과 `isaacsim` extra가 설치되어 있어야 합니다.
- 확인한 장비: NVIDIA GB10(aarch64), Ubuntu 24.04. 이 장비에서는 `LD_PRELOAD`(아래 참고)가 필요합니다.
- 기본 지면 USD는 실행할 때 Isaac Lab Nucleus에서 받아옵니다. 오프라인이면 `--local_ground`를 쓰세요.

## 실행

```bash
cd ~/vla_ws/IsaacLab
export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1
uv run --extra isaacsim isaaclab -p ~/arobot_ws/alice_lab/scripts/01_move_arms.py --viz kit
```

- 창을 닫으면 끝납니다(또는 `Ctrl+C`).
- 로그를 남기려면 명령 끝에 `2>&1 | tee /tmp/alice.log`를 붙이세요.
- 동작은 다음과 같습니다.
  - 양팔 `sh_p`는 0 ~ −1.0 rad, `el_p`는 0 ~ −1.2 rad 사이를 4초 주기 사인파로 왕복합니다.
  - 나머지 관절은 모두 목표 0입니다.

### 옵션

| 옵션 | 기본값 | 설명 |
|---|---|---|
| `--period` | 4.0 | 팔 왕복 주기 [s] |
| `--arm_scale` | 1.0 | 팔 진폭 배율. 0이면 팔을 멈추고 허리만 관찰 |
| `--print_interval` | 5.0 | 몸통 기울기와 팔 각도 출력 간격 [s] |
| `--diag_interval` | 1.0 | 허리 진단 블록 출력 간격 [s] |
| `--local_ground` | 꺼짐 | Nucleus 지면 대신 로컬 충돌 박스(윗면 z=0) 사용 |
| `--waist_vel_limit` | 없음 | (실험) 허리 관절 속도 제한 [rad/s] |
| `--waist_armature` | 없음 | (실험) 허리 관절 armature |
| `--solver_pos_iters`, `--solver_vel_iters` | 없음 | (실험) PhysX articulation solver 반복 횟수 |
| `--device`, `--viz` 등 | | Isaac Lab 공통 launcher 인자 |

## 출력 읽는 법

시작할 때 1회 출력되는 것은 다음과 같습니다.
- 관절 순서와 body 이름 목록
- 허리·팔 관절의 stiffness, damping, effort limit, 위치 limit. 모두 시뮬레이터에서 읽어 온 값입니다.
- 사인파를 보내는 관절의 인덱스

`[DIAG t=…]` 블록은 1초마다 출력됩니다.

| 열 | 의미 |
|---|---|
| `q` | 현재 각도 [rad] |
| `q_target` | 사용자가 쓴 목표 |
| `q_sim` | solver에 넘어간 목표 |
| `qd` | 각속도 [rad/s] |
| `effort_est` | Isaac Lab이 PD 식으로 추정한 토크. solver가 실제로 낸 토크는 아닙니다 |
| `g(q)` | 중력 보상 토크 |
| `limit` | 위치 limit 초과 여부 |

블록 끝에는 목표가 0이 아닌 관절 이름 목록이 붙습니다.

**정상 기준**
- `torso_link z`가 1.500 m 근처에 머뭅니다.
- 허리 관절이 ±0.01 rad 이내이고, `OUT OF LIMIT`이 없습니다.
- `torso tilt`가 1° 미만입니다.
- 팔 실제 각도가 목표를 따라갑니다. PD와 중력 때문에 목표보다 약간 덜 갑니다(예: 목표 −0.50에서 −0.41).

## 모델·설정 메모

**베이스와 시작 위치**
- 베이스는 world에 고정됩니다. USD의 `root_joint`와 cfg의 `fix_root_link=True`가 함께 작동합니다.
- 시작 높이는 z = 0.246 m입니다. `base_footprint`가 바퀴 접지면보다 0.246 m 위에 있기 때문입니다.

**self collision**
- 꺼져 있습니다.
- `articulation_props`는 `{"/.*": [...]}` 형태로 넘겨야 합니다. ArticulationRootAPI가 하위 프림인 `base_footprint`에 있어서, 리스트로만 넘기면 적용되지 않습니다.

**mimic 관절 12개**
- 변환된 USD에는 `NewtonMimicAPI`로 들어 있고, PhysX가 이를 articulation mimic 구속으로 만듭니다.
- 허리 follower(`lower_torso_mimic`, `upper_torso_mimic`)는 **drive 없이(stiffness 0) mimic 구속만으로** 따라갑니다.
  - follower에도 강한 drive를 걸면 팔을 멈춘 상태에서도 허리 리프트가 계속 흔들렸습니다(2026-10-04 실험).
- 손가락 distal 10개는 아직 손가락 그룹 drive(stiffness 2)에 들어 있습니다. 손가락을 움직일 때는 허리처럼 분리해야 합니다.

**허리 구조**
- 허리 pitch 2개(`waist_lower_pitch`, `waist_upper_pitch`)는 상체를 숙이는 관절이 아닙니다.
- 상체를 수직으로 유지한 채 높이와 앞뒤 위치를 바꾸는 평행링크 리프트입니다.

**추정값**
- 모든 gain, armature, effort, 팔꿈치 굽힘 방향(굽힘 = −)은 하드웨어로 검증하지 않은 추정값입니다.

**쿼터니언 순서**
- Isaac Lab 3.0은 **(x, y, z, w)**를 씁니다. `alice_m1_sim_notes.md`의 변환값은 (w, x, y, z) 순서이니 순서를 바꿔서 쓰세요.

**병합으로 사라진 프레임**
- 손바닥 `*_base_link`는 `*_arm_link_7`에 병합되었습니다. 손바닥을 엔드이펙터로 쓰려면 notes의 변환값(z −0.077, 회전)을 오프셋으로 붙여야 합니다.

## USD 다시 만들기

`alice_m1_sim.urdf`를 고친 뒤에는 USD를 다시 변환해야 합니다. 기존 출력의 `config.yaml` 기준 옵션은 아래와 같습니다.

```bash
cd ~/vla_ws/IsaacLab
export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1
uv run --extra isaacsim isaaclab -p scripts/tools/convert_urdf.py \
    ~/arobot_ws/alice_m1_urdf/alice_m1_sim.urdf ~/arobot_ws/alice_m1_usd \
    --merge_joints --fix_base --joint_damping 10
```

- 변환할 때 넣는 drive gain은 cfg의 액추에이터 설정이 덮어씁니다.
- 출력 경로가 바뀌면 `alice_m1_cfg.py`의 `ALICE_M1_USD_PATH`도 같이 고치세요.

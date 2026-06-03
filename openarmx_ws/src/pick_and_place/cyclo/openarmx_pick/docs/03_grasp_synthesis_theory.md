# Top-down Grasp 합성 이론

> 분석일: 2026-06-03 · 패키지: openarmx_pick

본 문서는 `openarmx_pick` 패키지의 Stage B 노드(`grasp_pose_node`)가 박스 윗면 포인트 클라우드로부터 6-DoF(six degrees of freedom, 6 자유도) top-down grasp pose를 **해석적/기하학적(analytic/geometric)** 으로 합성하는 수학과 그 이론적 정당성을 다룬다. 모든 수식·주장은 실제 코드(`openarmx_pick/grasp_pose_node.py`)를 인용해 근거를 제시하며, 코드로 직접 검증하지 못한 부분은 명시적으로 '추정'으로 표기한다.

---

## 분석 범위

- **포함**: top-down grasp의 기하학적 정의(grasp point, approach direction, yaw), box-top XY 평면에서의 PCA(Principal Component Analysis, 주성분 분석)를 통한 yaw 결정, `_grasp_rotation`의 Gram-Schmidt 직교정규화와 desired/tool frame 구성, `R_base_tool = B @ Tᵀ` 의 유도, `_quat_from_matrix`의 회전행렬→quaternion 변환(trace 기반 4-case), 그리고 "테이블 위 박스는 top plane + footprint 로 완전 구속되므로 learned grasp network 가 불필요하다"는 논거의 이론적 설명. 관련 배경 이론(PCA, analytic vs. learned grasp synthesis)도 간략히 정리한다.
- **제외**: 비전 파이프라인 내부(YOLO-World, `box_plane` RANSAC inlier 추출), TF(transform) 룩업·좌표변환의 ROS2 메커니즘, QP(Quadratic Program, 이차계획법)+CBF(Control Barrier Function, 제어 장벽 함수) MoveL 솔버 내부, MoveL 디바운스(debounce)·QoS(Quality of Service)·노드 수명주기 등 실행 인프라. 이들은 본 문서와 범위가 겹치지 않도록 다루지 않는다.
- **검증 방법**: 본 문서의 수식은 `grasp_pose_node.py`의 코드를 그대로 재현한 numpy 스크립트로 수치 검증했다(아래 [부록 A](#부록-a-수치-검증-결과) 참조). 검증 입력은 패키지에 포함된 합성 테스트(`scripts/verify_grasp.py:24-26`)의 박스(긴 변 +Y 0.16 m, 짧은 변 +X 0.08 m)이다.

---

## 1. 문제 정의: top-down grasp 합성

입력은 박스 윗면(box top)에 속하는 inlier 포인트 클라우드이며, 카메라 광학 프레임(`camera_color_optical_frame`)에서 들어와 robot base frame(`openarmx_body_link0`)으로 변환된 뒤 `Nx3` 행렬 `pts`로 표현된다(`grasp_pose_node.py:177-178`). 출력은 base frame에서 표현된 grasp pose, 즉 위치 3-벡터와 방향 quaternion이다.

top-down grasp는 다음 세 요소로 완전히 정의된다(노드 docstring `grasp_pose_node.py:8-12`):

1. **grasp point** = box-top centroid(중심점), 단 `grasp_depth` 만큼 윗면 아래로 하강.
2. **approach direction**(접근 방향) = 수직 하강(straight down), 즉 base frame의 `-z`.
3. **yaw**(요, 수직축 회전) = box top의 principal axis(주축)를 PCA로 구해, 그리퍼 opening이 박스의 짧은 변(short dimension)을 가로지르도록(straddle) 정렬.

이 세 요소가 SE(3) pose(위치 3 + 방향 3 = 6 DoF)를 모두 결정한다. 위치는 centroid가, approach가 회전의 2 DoF(어느 방향을 내려다보는가)를, yaw가 나머지 1 DoF(접근축 둘레의 회전)를 고정한다.

---

## 2. Grasp point: centroid 하강

centroid는 base frame 포인트들의 산술 평균이다(`grasp_pose_node.py:180`):

```python
centroid = pts.mean(axis=0)
```

수식으로:

$$ \mathbf{c} = \frac{1}{N}\sum_{i=1}^{N}\mathbf{p}_i,\qquad \mathbf{p}_i\in\mathbb{R}^3 $$

실제 grasp point와 pre-grasp(예비 접근) point는 centroid의 z만 각각 하강·상승시켜 만든다(`grasp_pose_node.py:193-194`):

```python
grasp_xyz = centroid.copy(); grasp_xyz[2] -= self.grasp_depth   # 윗면 아래로
pre_xyz   = centroid.copy(); pre_xyz[2]   += self.pregrasp_h    # 윗면 위로
```

- `grasp_depth` 기본값 0.005 m(`grasp_pose_node.py:110`): 그리퍼 손가락이 윗면 모서리를 살짝 물도록 표면 아래로 5 mm 내려 잡는다. 박스 윗면 법선이 base `+z`라는 가정 하에서, z만 내리면 그대로 표면 안쪽이 된다.
- `pregrasp_height` 기본값 0.10 m(`grasp_pose_node.py:109`): centroid 위 10 cm 지점을 hover(공중 정지) 목표로 삼는다. Stage B는 이 pre-grasp만 MoveL로 송신하고, 실제 하강/파지/들어올리기 FSM(Finite State Machine, 유한 상태 기계)은 후속 단계로 명시되어 있다(`grasp_pose_node.py:16-17`, README `Not yet done` 2번).

**이론적 근거.** centroid를 grasp point로 삼는 것은, 평행 그리퍼(parallel-jaw gripper)로 평면 직사각형 물체를 잡을 때 **force closure(힘 폐쇄)** 를 만족하는 안정 파지가 윗면의 기하 중심 부근에서 형성된다는 고전적 사실에 근거한다(추정 — 코드에 force-closure 계산은 없고, centroid 채택이 기하적 휴리스틱임). 중심에서 잡으면 두 손가락 접촉선이 물체의 무게중심(균질 박스라면 footprint 중심의 연직 위)과 정렬되어 파지 토크가 최소화된다.

---

## 3. Approach direction: 수직 하강

approach는 상수로 고정된다(`grasp_pose_node.py:189`):

```python
approach = np.array([0.0, 0.0, -1.0])   # straight down
```

근거는 docstring에 명시되어 있다(`grasp_pose_node.py:9-10`): 테이블 위에 놓인 박스의 윗면 법선은 base frame의 `+z`이므로, 그리퍼 도구축(tool axis)을 `-z_base`에 정렬해 윗면 법선과 정반대로 마주 보며 내려가야 한다. 이는 top-down grasp의 정의 그 자체이며, 카메라가 박스를 비스듬히 보더라도 approach는 박스 윗면 자세와 무관하게 항상 연직 하강으로 고정된다(추정 — 윗면 법선 추정 대신 base `-z`를 상수로 쓰므로, 윗면이 수평이라는 가정에 의존).

> 윗면 법선을 클라우드에서 추정(예: 평면 normal)하지 않고 `-z`를 하드코딩한 것은, 박스가 수평 테이블 위에 정립(upright)해 있다는 강한 사전 가정을 단순화로 채택했음을 뜻한다. 기울어진 박스에는 부정확해진다(추정).

---

## 4. Yaw: box-top XY 평면에서의 PCA

### 4.1 공분산 행렬과 eigendecomposition

approach가 연직으로 고정되면 남는 자유도는 그 축 둘레의 yaw 1개뿐이다. 이를 박스 윗면의 평면 형상에서 결정하기 위해 XY 평면으로 투영한 2D 점들의 공분산 구조를 분석한다(`grasp_pose_node.py:181-184`):

```python
xy = pts[:, :2] - centroid[:2]
cov = xy.T @ xy / max(len(xy) - 1, 1)
evals, evecs = np.linalg.eigh(cov)
long_axis = evecs[:, int(np.argmax(evals))]   # box long direction (XY)
```

여기서:

- `xy`는 centroid를 뺀 **mean-centered**(평균 제거) 2D 좌표. PCA는 평균이 제거된 데이터의 공분산을 다루므로 이 정규화가 필수다.
- `cov = Xᵀ X / (N-1)` 는 표본 공분산 행렬(sample covariance matrix), 2×2 대칭 양의 준정부호(symmetric positive semi-definite) 행렬.

  $$ \Sigma = \frac{1}{N-1}\sum_{i=1}^{N}(\mathbf{q}_i-\bar{\mathbf{q}})(\mathbf{q}_i-\bar{\mathbf{q}})^\top,\quad \mathbf{q}_i\in\mathbb{R}^2 $$

- `np.linalg.eigh`는 **대칭(Hermitian) 행렬 전용** eigensolver로, eigenvalue를 **오름차순(ascending)** 으로, 그에 대응하는 정규직교 eigenvector를 열로 반환한다. 따라서 가장 큰 eigenvalue에 대응하는 eigenvector를 고르려면 `argmax(evals)`로 인덱싱해야 하며, 코드가 정확히 그렇게 한다.

**PCA 이론.** 공분산 행렬의 최대 eigenvalue 방향은 데이터 분산(흩어짐)이 가장 큰 방향, 즉 직사각형 윗면의 **긴 변(long axis)** 방향이다. 두 번째(작은) eigenvalue 방향은 짧은 변이다. 직사각형 균일 분포 점들에서는 분산이 변 길이의 제곱에 비례하므로(긴 변일수록 분산이 큼), `argmax` eigenvector가 곧 긴 변 방향이 된다. 부록 A의 수치 검증에서 긴 변(+Y, 0.16 m)의 eigenvalue가 0.00213, 짧은 변(+X, 0.08 m)의 eigenvalue가 0.00053으로, 변 길이의 제곱비(0.16²:0.08² = 4:1)에 근사함을 확인했다.

이어서 long axis를 z=0으로 강제해 3D 수평 벡터로 만든다(`grasp_pose_node.py:185`):

```python
long_axis = np.array([long_axis[0], long_axis[1], 0.0])
```

### 4.2 Opening 방향: cross product 로 short axis straddle

평행 그리퍼의 opening(손가락 벌어짐 방향)은 박스의 **짧은 변**을 가로질러야 한다. 두 손가락이 짧은 변 양쪽에서 다가와 닫히면 더 짧은 폭을 물게 되어 그리퍼 stroke(개폐 폭) 내에 들어오고 파지가 안정적이다. opening은 cross product로 구한다(`grasp_pose_node.py:186-187`):

```python
opening = np.cross(np.array([0.0, 0.0, 1.0]), long_axis)   # span the short axis
opening /= np.linalg.norm(opening)
```

$$ \mathbf{o} = \frac{\hat{\mathbf{z}}\times \mathbf{\ell}}{\lVert\hat{\mathbf{z}}\times \mathbf{\ell}\rVert} $$

여기서 `ẑ = (0,0,1)`, `ℓ = long_axis`. 수평면(XY) 안에 있는 long axis와 수직 `ẑ`의 외적은, 두 벡터에 모두 수직이므로 다시 XY 평면 안에 있으면서 long axis에 직교한다 — 즉 정확히 short axis 방향이다. 부록 A에서 long_axis = (0,1,0)에 대해 opening = (−1,0,0)으로, +X(짧은 변)에 정렬됨을 확인했다(부호는 cross product의 방향성으로 −X가 되며, 평행 그리퍼는 양방향 대칭이라 ±부호는 무의미하다).

> **요약**: long axis = 분산 최대 = 긴 변. opening = ẑ × long = 짧은 변 가로지름. 따라서 그리퍼는 긴 변과 평행하게 손목을 정렬하고, 짧은 변을 양쪽에서 집는다 — 직사각형 박스 top-down 파지의 정석 yaw.

---

## 5. `_grasp_rotation`: desired frame 과 tool frame 의 정합

`_grasp_rotation`(`grasp_pose_node.py:83-97`)은 위에서 얻은 base-frame의 desired approach/opening 방향에, 그리퍼 도구 좌표계의 approach/opening 축을 맞추는 회전행렬 `R_base_tool`을 구성한다.

### 5.1 Gram-Schmidt 직교정규화

먼저 desired frame을 구성한다(`grasp_pose_node.py:87-92`):

```python
a = approach / np.linalg.norm(approach)
o = opening - np.dot(opening, a) * a       # Gram-Schmidt: approach 성분 제거
if np.linalg.norm(o) < 1e-6:               # opening ∥ approach 인 퇴화 처리
    o = np.array([1.0, 0.0, 0.0]) - np.dot([1.0, 0.0, 0.0], a) * a
o /= np.linalg.norm(o)
B = np.column_stack((o, np.cross(a, o), a))   # [opening, a×o, approach]
```

여기서 핵심은 **Gram-Schmidt 직교화**:

$$ \mathbf{o}_\perp = \mathbf{o} - (\mathbf{o}\cdot\mathbf{a})\,\mathbf{a} $$

이 한 줄은 opening 벡터에서 approach 방향 성분을 빼서 두 축을 정확히 직교시킨다. PCA로 얻은 opening은 수평면 안에 있고 approach는 연직이라 원래 거의 직교하지만, 수치 오차나 입력 비직교성을 흡수하기 위한 안전한 정규화다. `if np.linalg.norm(o) < 1e-6` 분기는 opening이 approach와 평행해 직교 성분이 사라지는 퇴화(degenerate) 경우에 대비해 임의 보조축 `(1,0,0)`으로 대체한다.

세 번째 축은 외적 `a×o`로 만들어 **right-handed orthonormal basis(우수계 정규직교 기저)** 를 보장한다. 따라서:

$$ B = \begin{bmatrix} \mathbf{o} & \mathbf{a}\times\mathbf{o} & \mathbf{a} \end{bmatrix} $$

열 순서는 `[opening, binormal, approach]`로, 코드 주석(`grasp_pose_node.py:92`)이 명시한다. `B`는 desired frame의 기저벡터를 base frame 좌표로 적은 행렬, 즉 **base ← desired** 회전이다.

tool frame도 동일하게 구성한다(`grasp_pose_node.py:93-96`):

```python
ta = tool_approach / np.linalg.norm(tool_approach)
to = tool_opening - np.dot(tool_opening, ta) * ta
to /= np.linalg.norm(to)
T = np.column_stack((to, np.cross(ta, to), ta))
```

기본 도구축은 `tool_approach_axis = [0,0,1]`, `tool_opening_axis = [1,0,0]`(`grasp_pose_node.py:120-121`), 즉 TCP(Tool Center Point, 도구 중심점) 로컬 좌표에서 **+z로 접근하고 +x로 손가락을 벌린다**는 컨벤션(`grasp_pose_node.py:19-21`). `T`는 tool 기저를 tool 자신의 좌표로 적은 행렬이다.

### 5.2 `R_base_tool = B @ Tᵀ` 의 유도

반환식은(`grasp_pose_node.py:97`):

```python
return B @ T.T
```

이 식의 의미는 좌표계 정합으로 유도된다. `B`와 `T`는 각각 desired·tool의 동일한 의미축(opening, binormal, approach)을 정렬한 정규직교 기저다. tool의 i번째 의미축이 desired의 i번째 의미축에 일치해야 하므로, tool 좌표의 한 점을 base 좌표로 보내는 회전 `R`은 다음을 만족해야 한다:

$$ R\,T = B \;\Longrightarrow\; R = B\,T^{-1} = B\,T^{\top} $$

마지막 등식은 `T`가 정규직교(orthonormal)라 `T⁻¹ = Tᵀ`이기에 성립한다. 결과 `R_base_tool`은 tool frame 벡터를 base frame으로 회전시키며, 특히:

- tool +z(approach 축) → desired approach `(0,0,-1)` (연직 하강)
- tool +x(opening 축) → desired opening (짧은 변)

부록 A에서 기본 도구축과 본 예제 박스에 대해 `R_base_tool = diag(−1, 1, −1)`이 나오고, `det(R)=+1`(우수계 보존), `‖RRᵀ−I‖=0`(정규직교)임을 확인했으며, `R @ (0,0,1)ᵀ = (0,0,−1)` (tool +z → approach), `R @ (1,0,0)ᵀ = (−1,0,0)` (tool +x → opening)으로 의도대로 매핑됨을 검증했다.

---

## 6. `_quat_from_matrix`: 회전행렬 → quaternion (trace 기반 4-case)

ROS2 메시지(`geometry_msgs/PoseStamped.orientation`)는 회전을 quaternion `[x,y,z,w]`로 요구하므로, `R_base_tool`을 quaternion으로 변환한다(`grasp_pose_node.py:65-80`). 표준 Shepperd 방식의 수치 안정 변환으로, 회전행렬 trace와 대각 성분 크기에 따라 4개 case로 분기한다.

```python
tr = m[0,0] + m[1,1] + m[2,2]
if tr > 0:                                   # case 1
    s = sqrt(tr + 1.0) * 2                    # s = 4w
    w = 0.25*s; x = (m[2,1]-m[1,2])/s; ...
elif m[0,0] > m[1,1] and m[0,0] > m[2,2]:    # case 2: x 최대
    s = sqrt(1 + m[0,0] - m[1,1] - m[2,2]) * 2  # s = 4x
    ...
elif m[1,1] > m[2,2]:                         # case 3: y 최대
    s = sqrt(1 + m[1,1] - m[0,0] - m[2,2]) * 2  # s = 4y
    ...
else:                                         # case 4: z 최대
    s = sqrt(1 + m[2,2] - m[0,0] - m[1,1]) * 2  # s = 4z
    ...
```

**왜 4-case 인가.** quaternion 성분은 회전행렬과
$$ w=\tfrac{1}{2}\sqrt{1+\mathrm{tr}(R)},\quad x=\tfrac{1}{2}\sqrt{1+R_{00}-R_{11}-R_{22}},\ \dots $$
로 연결되지만, 단일 공식만 쓰면 제곱근 안 인자가 0에 가까워질 때 0으로 나누거나 정밀도가 붕괴한다. 따라서 **분모로 쓰는 성분이 가장 큰 case** 를 골라야 수치적으로 안정하다:

- **case 1** (`tr > 0`): `w`가 충분히 크므로 `s = 4w`를 분모로. 나머지 `x,y,z`는 반대칭 성분 `(m[2,1]−m[1,2])` 등으로 복원.
- **case 2~4**: trace가 음수일 때(180°에 가까운 회전 등)는 `w`가 작아지므로, 대각 성분 `m[0,0], m[1,1], m[2,2]` 중 **가장 큰 것**을 골라 그에 대응하는 quaternion 성분(`x`, `y`, 또는 `z`)을 분모로 삼는다. `elif` 사다리가 정확히 이 "가장 큰 대각 성분" 선택을 구현한다.

각 case에서 비대각(off-diagonal) 합·차 항(`(m[0,1]+m[1,0])`, `(m[2,1]−m[1,2])` 등)으로 나머지 성분을 채우며, 부호 규약은 표준 Hamilton quaternion과 일치한다.

**검증.** 부록 A에서 `R_base_tool = diag(−1,1,−1)`은 trace = −1 < 0 이므로 case 1(`tr > 0`)이 아니다. 이어서 `m[0,0]=−1`은 `m[1,1]=1`보다 작으므로 case 2(`x` 최대)도 아니다. 그 다음 `m[1,1]=1 > m[2,2]=−1`이 참이므로 **case 3: y 최대(`elif m[1,1] > m[2,2]`) 분기**로 진입한다(마지막 else/z 최대가 아님). 결과 quaternion = `[0,1,0,0]` (norm 1.0), 이를 다시 회전행렬로 복원하면 원본과 오차 0으로 일치했다(round-trip 정확). 이는 180° yaw 회전(x축이 뒤집힌 채 z 하강)을 정확히 표현한 결과다. `scripts/verify_grasp.py:72-73`도 동일한 quaternion→축 복원으로 approach·opening을 역산해 PASS를 확인하는 독립 검증을 둔다.

---

## 7. "learned grasp network 불필요" 논거의 이론적 설명

README(`README.md:7-10`)와 노드 docstring(`grasp_pose_node.py:14-15`)은 다음을 주장한다:

> "No learned grasp network is needed — a box resting on a table is fully constrained by its top plane + footprint."

이 주장의 이론적 핵심은 **자유도 계산(degrees-of-freedom counting)** 이다. SE(3) grasp pose는 6 DoF를 가진다(위치 3 + 방향 3). 테이블 위 정립 박스에서는 이 6 DoF가 두 개의 기하 관측만으로 모두 결정된다:

| Grasp DoF | 결정 요소 | 근거 |
|---|---|---|
| 위치 x, y (2) | top plane footprint 의 **centroid** | `grasp_pose_node.py:180,193` |
| 위치 z (1) | top plane 의 높이 − `grasp_depth` | `grasp_pose_node.py:193` |
| 방향: approach 2 DoF | top plane 법선 = base `+z` → approach = `−z` (연직) | `grasp_pose_node.py:189` |
| 방향: yaw 1 DoF | footprint 의 **PCA principal axis** (긴 변) | `grasp_pose_node.py:183-187` |

즉 **top plane**(법선 + 높이)이 approach 2 DoF와 위치 z를, **footprint**(centroid + principal axis)가 위치 x,y와 yaw를 닫는다. 6개 자유도가 빠짐없이 채워지므로, 가능한 grasp 후보 집합의 모호성(ambiguity)이 남지 않는다.

**learned grasp network 가 필요한 경우와의 대비.** GraspNet/Dex-Net류 학습 기반 합성은 임의 형상, 가림(occlusion), 비정형 표면, 다중 접촉점 후보 등 **해석적으로 닫히지 않는 모호한 grasp 분포**를 데이터로 학습해 점수화한다. 그러나 본 시나리오는:

1. 물체 클래스가 박스(직육면체)로 고정 — 형상 prior가 강함.
2. 윗면이 평면이고 수평 — 법선·approach가 상수.
3. 평행 그리퍼 + top-down — 접촉 모드가 단일.

이 세 가정 하에서 grasp 분포는 사실상 **유일한 해**(또는 ±yaw 대칭만 갖는 해)로 붕괴한다. 따라서 학습 모델의 일반화 능력이 불필요하고, **closed-form geometric synthesis**(centroid + PCA + 고정 approach)가 동등한 정확도를 더 적은 계산·데이터·불확실성으로 달성한다. 이것이 "learned network 불필요"의 본질이다. (이는 가정이 깨질 때 — 기울어진 박스, 비박스 물체, 가려진 윗면 — 본 방법이 부정확해질 수 있음을 동시에 함의한다. 추정.)

---

## 8. 관련 이론 정리

### 8.1 PCA (Principal Component Analysis, 주성분 분석)

PCA는 mean-centered 데이터의 공분산 행렬 Σ를 eigendecomposition하여, 분산이 큰 순서로 정규직교 주축을 찾는 기법이다. 본 노드는 2D(XY 평면)에 적용해 직사각형 윗면의 긴 변/짧은 변을 추출한다. 핵심 성질:

- Σ가 대칭 양의 준정부호이므로 eigenvalue는 실수·비음수, eigenvector는 정규직교(따라서 `eigh` 사용이 정당).
- 최대 eigenvalue 방향 = 최대 분산 방향 = 긴 변. 이 성질이 yaw 결정의 전부다.
- 정사각형(긴 변 = 짧은 변)에서는 두 eigenvalue가 같아져 주축이 비결정(degenerate)이 된다 — yaw가 불안정해질 수 있다(추정).

### 8.2 Analytic / Geometric grasp synthesis

해석적 grasp 합성은 물체의 기하(평면, 모서리, 대칭)와 그리퍼 운동학으로부터 force closure / form closure를 만족하는 grasp를 **수식으로 직접 계산**한다. 학습 기반과 달리 훈련 데이터·추론 모델이 없고, 가정이 성립하는 영역에서 결정론적·재현 가능·해석 가능하다. 본 노드는 그 전형적 사례로, "centroid 위치 + PCA yaw + 고정 top-down approach"라는 세 휴리스틱으로 6-DoF grasp를 닫는다.

### 8.3 좌표 정합과 Gram-Schmidt / quaternion

두 정규직교 frame(`B`, `T`)을 정렬하는 회전은 `R = B Tᵀ`로 닫힌 형태로 얻어지며, Gram-Schmidt가 입력 축의 비직교성을 흡수해 `R`의 직교성·우수성(det=+1)을 보장한다. ROS2 메시지 직렬화를 위해 회전행렬을 quaternion으로 옮길 때는 trace 기반 4-case Shepperd 변환으로 수치 안정성을 확보한다 — 본 노드의 `_grasp_rotation`과 `_quat_from_matrix`가 이 표준 파이프라인을 그대로 구현한다.

---

## 부록 A. 수치 검증 결과

`grasp_pose_node.py`의 `_grasp_rotation`, `_quat_from_matrix`, PCA 블록(`:181-191`)을 그대로 재현한 numpy 스크립트를, 패키지 합성 테스트의 박스(긴 변 +Y 0.16 m, 짧은 변 +X 0.08 m, centroid (0.50, 0.10, 0.20); `scripts/verify_grasp.py:24-32`) 입력으로 실행한 결과:

| 항목 | 결과 | 기대/의미 |
|---|---|---|
| `eigh` eigenvalue (오름차순) | `[0.000533, 0.002135]` | 큰 값이 긴 변(+Y); 비 ≈ 4:1 = (0.16/0.08)² |
| `long_axis` | `(0, 1, 0)` | 긴 변 = +Y, 정확 |
| `opening = ẑ × long` | `(−1, 0, 0)` | 짧은 변 +X 가로지름(±부호는 그리퍼 대칭) |
| `R_base_tool` | `diag(−1, 1, −1)` | det = +1, ‖RRᵀ−I‖ = 0 (정규직교·우수계) |
| `R @ (0,0,1)` (tool +z) | `(0, 0, −1)` | approach = 연직 하강, 정확 |
| `R @ (1,0,0)` (tool +x) | `(−1, 0, 0)` | opening = 짧은 변, 정확 |
| `_quat_from_matrix` | `[x,y,z,w] = [0, 1, 0, 0]` | norm = 1.0 |
| quaternion → R 복원 오차 | `0` | round-trip 정확 |
| trace(R) | `−1.0` | `tr>0` 분기 아님; `m[1,1]=1 > m[2,2]=−1`이므로 **case 3: y 최대(`elif m[1,1] > m[2,2]`)** 진입 |

이 결과는 본문의 모든 수식적 주장(PCA argmax 선택, opening cross product, `B @ Tᵀ`의 우수 정규직교성, trace 4-case 분기)을 직접 뒷받침한다.

---

## 부록 B. 참조 코드 위치 (패키지 루트 기준)

| 주제 | 위치 |
|---|---|
| top-down grasp 3요소 정의 | `openarmx_pick/grasp_pose_node.py:8-12` |
| centroid 및 grasp/pre-grasp z 하강·상승 | `openarmx_pick/grasp_pose_node.py:180,193-194` |
| approach = `(0,0,-1)` 상수 | `openarmx_pick/grasp_pose_node.py:189` |
| PCA: 공분산·`eigh`·argmax long axis | `openarmx_pick/grasp_pose_node.py:181-185` |
| opening = `ẑ × long_axis` | `openarmx_pick/grasp_pose_node.py:186-187` |
| `_grasp_rotation` (Gram-Schmidt, `B @ Tᵀ`) | `openarmx_pick/grasp_pose_node.py:83-97` |
| tool 축 컨벤션 파라미터 | `openarmx_pick/grasp_pose_node.py:19-21,120-121` |
| `_quat_from_matrix` (trace 4-case) | `openarmx_pick/grasp_pose_node.py:65-80` |
| "learned network 불필요" 논거 | `openarmx_pick/grasp_pose_node.py:14-15`, `README.md:7-10` |
| 합성 검증 박스·기대값 | `scripts/verify_grasp.py:8-13,24-32,72-90` |

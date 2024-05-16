# Drone Tracking Robot

## Preview


https://github.com/spring98/drone-tracking-robot/assets/92755385/92e1d344-143c-4592-9c2e-b84396f86e11

<img src="https://github.com/spring98/drone-tracking-robot/assets/92755385/ec3595ac-1477-459f-91f5-ccdbcf1c3945" align="center" width="100%">

![드론 트레킹 실험](https://github.com/spring98/drone-tracking-robot/assets/92755385/8df1b261-4534-4210-ad07-ccf7a1d65935)


## Research Topic
컴퓨터 비전 AI 모델을 활용한 드론 추적 로봇 개발 및 PID(Proportional Integral Derivative), SMC(Sliding Mode Control), MPC(Model Predictive Control) 전략 성능 비교

## Scheme

## Specification
<p align="center">
  <img src="https://github.com/spring98/drone-tracking-robot/assets/92755385/a069562a-7fc3-40f0-b46e-a51008345307" align="center" width="50%">  
</p>

 <table>
    <caption>Link Length</caption>
    <tr>
      <th></th>
      <th>$$d_1$$</th>
      <th>$$a_2$$</th>
      <th>$$a_3$$</th>
      <th>$$a_3'$$</th>
    </tr>
    <tr>
      <td>$$Length (m)$$</td>
      <td>$$0.07$$</td>
      <td>$$0.03$$</td>
      <td>$$0.055$$</td>
      <td>$$0.085$$</td>
    </tr>
  </table>

<table>
    <caption>Link Mass</caption>
    <tr>
      <th></th>
      <th>$$m_1$$</th>
      <th>$$m_2$$</th>
    </tr>
    <tr>
      <td>$$Mass (kg)$$</td>
      <td>$$0.075$$</td>
      <td>$$0.316$$</td>
    </tr>
  </table>

## Kinematics
<p align="center">  
  <img src="https://github.com/spring98/drone-tracking-robot/assets/92755385/b4e4af97-9759-496a-94bf-f8377e7e92ec" align="center" width="30%">
</p>

아래부터 순서대로 { ${base}$ }, { ${1}$ }, { ${2}$ }, { ${tool}$ } 의 local coordinate 이며 빨간축이 $x$축, 초록축이 $y$축, 파란축이 $z$축 입니다.
<br/><br/>

### DH Parameter
| $$i$$  | $$\alpha_{i-1}$$ | $$a_{i-1}$$ | $$d_i$$ | $$\theta_i$$ |
|---|---|---|---|---|
|$$1$$|$$0$$|$$0$$|$$d_1 + a_2$$|$$\theta_1$$|
|$$2$$|$$\pi/2$$|$$0$$|$$0$$|$$\theta_2 + \pi/2$$|
|$$Tool (Laser)$$|$$\pi/2$$|$$a_3$$|$$0$$|$$\pi/2$$|

DH Parameter 는 위의 표와 같습니다.
<br/><br/>

### Forward Kinematics
{ ${base}$ } 계에서 { ${tool}$ } 계를 표현하고자 할 때 

DH Parameter 표를 이용해서 $i = 1$ 부터 $tool$ 까지 곱으로 표현할 수 있으며 아래 식과 같습니다.

<br/>

$${}_{base}^{tool} T = \begin{bmatrix}
\sin \theta_1 & \sin \theta_2 \cos \theta_1 & \cos \theta_1 \cos \theta_2 & -a_3 \sin \theta_2 \cos \theta_1 \\
-\cos \theta_1 & \sin \theta_1 \sin \theta_2 & \sin \theta_1 \cos \theta_2 & -a_3 \sin \theta_1 \sin \theta_2 \\
0 & -\cos \theta_2 & \sin \theta_2 & a_2 + a_3 \cos \theta_2 + d_1 \\
0 & 0 & 0 & 1
\end{bmatrix}$$


<br/><br/>

### Inverse Kinematics
<p align="center">  
  <img src="https://github.com/spring98/drone-tracking-robot/assets/92755385/4d263b5d-cde5-444e-a2ae-311e66163468" align="center" width="50%">
</p>

<br/>

드론의 위치를 $x, y, z$ 라고 할 때, $\theta_1$ 과 $\theta_2$ 는 아래와 같이 기하학적인 방식으로 유도할 수 있습니다.

<br/>

$$\theta_1 = \arctan\left(\frac{y}{x}\right)$$

$$\theta_2 = \arccos\left(L_2 A + \sqrt{(L_2 A)^2 + (A^2 + B)(B - L_2^2)}\right)$$

$$where, \quad A = z - L_1, \quad B = x^2 + y^2, \quad L_1 = d_1 + a_2, \quad L_2 = a_3$$


<br/><br/>

## Trajectory

## Dynamics
<p align="center">
  <img src="https://github.com/spring98/drone-tracking-robot/assets/92755385/59052504-5f18-490c-b873-6064c58fadaf" align="center" width="35%">  
</p>

빨간점을 각각의 질점으로 가정하여 위치에너지, 운동에너지를 구할 수 있습니다.

<br/>

$$ E_p = m_1 g l_1 + m_2 g ( l_1 + l_2 \cos \theta_2 + l_3 \sin \theta_2 ) $$

$$ E_k = \frac{1}{2} m_2 \dot{\theta}_1^2 \left( l_3^2 \cos^2 \theta_2 + l_2^2 \sin^2 \theta_2 - 2 l_2 l_3 \cos \theta_2 \sin \theta_2 \right) + \frac{1}{2} m_2 \dot{\theta}_2^2 \left( l_2^2 + l_3^2 \right) $$

<br/>

 운동에너지와 위치에너지의 차로 라그랑지안을 정의하고 라그랑주 방정식으로 운동방정식을 유도할 수 있습니다.

<br/>

$$ \frac{d}{dt} \frac{\partial L}{\partial \dot{q}_i} - \frac{\partial L}{\partial q_i} = \tau \quad (i = 1, 2)$$

$$ M(q) \ddot{q} + V(q, \dot{q}) + G(q) = \tau $$

<br/>

$$ \therefore
\begin{bmatrix}
m_2 A^2 & 0 \\
0 & m_2 (l_2^2 + l_3^2)
\end{bmatrix}
\begin{bmatrix}
\ddot{\theta}_1 \\
\ddot{\theta}_2
\end{bmatrix}
+
\begin{bmatrix}
2m_2 AB \dot{\theta}_1 \dot{\theta}_2 \\
-m_2 AB \dot{\theta}_1^2
\end{bmatrix}
+
\begin{bmatrix}
0 \\
-m_2 A g
\end{bmatrix}
= \begin{bmatrix}
\tau_1 \\
\tau_2
\end{bmatrix} $$

$$ where, \quad A = l_2 \sin \theta_2 - l_3 \cos \theta_2 \quad \text{and} \quad B = l_2 \cos \theta_2 + l_3 \sin \theta_2 $$


## Control
### PID Design
![PID 블록선도 drawio](https://github.com/spring98/drone-tracking-robot/assets/92755385/3de4c052-973d-4ff3-891e-c789fb44c17b)

로봇의 PID제 어 시스템은 입력된 현재 각도와 각속도에 대한 정 보를 피드백 받고, 목표하는 각도와 각속도의 값을 비교하여 오차 값을 계산합니다. 

오차 값은 적절한 이득 값들과 곱해져서 τ'(토크 수정 값)을 생성합니다. 이후, 현재 각도 정보는 질량 매트릭스를 통해 처 리되고, 이 결과는 τ' 와 곱해져 최종토크 τ 를 형성합니다. 

차이를 에러 각(e)으로 두고, 폐루프 시스템의 특성을 이용하면 오차방정식을 구할 수 있습니다.


$$ MV(t) = K_p e(t) + K_i \int_{0}^{t} e(t) \, dt + K_d \frac{de}{dt} $$


### SMC Design
![SMC블록선도 drawio](https://github.com/spring98/drone-tracking-robot/assets/92755385/bcb763d6-7b74-499a-868d-ad8812b96a05)

상태공간좌표계에 상태값들이 0 으로 수렴하도록 설계된 슬라이딩표면을 따라가도록 불연속적인 제어입력을 가합니다.

사용된 상태 공간좌표계는 모터의 참조값, 현재값의 차이 각도오차, 각속도 오차를 사용하고 있으며 제어기의 게인을 통해서 오차수렴속도, 채터링, 입력크기 등을 조절하는데 사용할 수 있습니다. 

동역학 모델링으로부터 상태공간 방정식을 도출한 식은 아래와 같습니다.

$$ x_1 = \theta_1, \quad x_2 = \dot{\theta}_1, \quad x_3 = \theta_2, \quad x_4 = \dot{\theta}_2 $$ 

$$ \dot{x}_2 = -\frac{2B}{A} x_2 x_4 + \frac{1}{m_2 A^2} \tau_1 $$

$$ \dot{x}_4 = \frac{AB}{C} x_2^2 + \frac{Ag}{C} + \frac{1}{m_2 C} \tau_2 $$ 

$$ where, \quad A = l_2 \sin \theta_2 - l_3 \cos \theta_2 \quad B = l_2 \cos \theta_2 + l_3 \sin \theta_2 \quad C = l_2^2 + l_3^2 $$

슬라이딩표면 설계는 아래와 같습니다.

$$ s_1 = c_1 e_1 + e_2, \quad s_2 = c_2 e_3 + e_4  $$ 

$$ where, \quad c_1 > 0, \quad c_2 > 0 $$ 

<br/>

$$ e_1 = \theta_{d1} - \theta_1 $$

$$ e_2 = \dot{\theta}_{d1} - \dot{\theta}_1$$ 

$$ e_3 = \theta_{d2} - \theta_2$$ 

$$ e_4 = \dot{\theta}_{d2} - \dot{\theta}_2 $$ 

## Result
<p align="center">
  <img width="40%" src="https://github.com/spring98/drone-tracking-robot/assets/92755385/fff4aea4-27ee-4e74-8d52-da3b52b72f5c">
  <img width="38%" src="https://github.com/spring98/drone-tracking-robot/assets/92755385/1b4b0041-9a5a-4779-aecc-c262a4d0bdac">
</p>

<p align="center">
  <img width="40%" src="https://github.com/spring98/drone-tracking-robot/assets/92755385/51ae24e8-69ea-4814-a3f5-b4595b866dfc">
  <img width="38%" src="https://github.com/spring98/drone-tracking-robot/assets/92755385/9bb199f0-c4a2-44c8-9f38-fb36acd07d11">
</p>

<p align="center">
  <img width="40%" src="https://github.com/spring98/drone-tracking-robot/assets/92755385/1547c6de-a242-4ba3-ba0c-1fa9d1fc932c">
  <img width="38%" src="https://github.com/spring98/drone-tracking-robot/assets/92755385/ea3814b6-3ed4-4597-a27d-2becaff3a7c9">
</p>

RMSE로 계산하였을 때 SMC가 XY Plane 0.809, YZ Plane 0.493으로 가장 적은 RMSE를 가져 다른 제어기보다더 좋은 추적성능을 보여주는 것을 확인할 수 있었습니다. 그러나 전력소모량이 가장 높은 것을 알 수 있습니다. 

구간별 추적 성능으로 보았을 SMC는 경로에서 벗어나지 않으며 정확하게 추적하는 성능을 보이고 PID와 MPC 제어기에서는 경로에서 벗어나는 것을 보입니다. 

성능적인 측면에서 SMC가 가장 우수하지만, 경제적인 측면을 고려하였을 때 전력소모량이 적고 비교적 추적오차가 적은 PID가 우수하다고 볼 수 있습니다.



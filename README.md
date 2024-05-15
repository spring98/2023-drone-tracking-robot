# Drone Tracking Robot

## Preview


https://github.com/spring98/drone-tracking-robot/assets/92755385/92e1d344-143c-4592-9c2e-b84396f86e11


![드론 트레킹 실험](https://github.com/spring98/drone-tracking-robot/assets/92755385/8df1b261-4534-4210-ad07-ccf7a1d65935)


## Research Topic
컴퓨터 비전 AI 모델을 활용한 드론 추적 로봇 개발 및 PID(Proportional Integral Derivative), SMC(Sliding Mode Control), MPC(Model Predictive Control) 전략 성능 비교

## Scheme

## Specification
<img width="720" alt="스크린샷 2024-05-15 11 29 32" src="https://github.com/spring98/drone-tracking-robot/assets/92755385/a069562a-7fc3-40f0-b46e-a51008345307">

링크 길이
|   | $$d_1$$ | $$a_2$$ | $$a_3$$ | $$a_3'$$|
|---|---|---|---|---|
|$$Length (m)$$|$$0.07$$|$$0.03$$|$$0.055$$|$$0.085$$|

링크 질량
|   | $$m_1$$ | $$m_2$$ |
|---|---|---|
|$$Mass (kg)$$|$$3$$|$$3$$|

## Kinematics
<img width="481" alt="스크린샷 2024-05-15 11 29 07" src="https://github.com/spring98/drone-tracking-robot/assets/92755385/89d0baaa-1565-4af4-9b76-d77b52ffc5ca">

<img width="705" alt="스크린샷 2024-05-15 11 23 34" src="https://github.com/spring98/drone-tracking-robot/assets/92755385/ec3595ac-1477-459f-91f5-ccdbcf1c3945">

### DH Parameter
| $$i$$  | $$\alpha_{i-1}$$ | $$a_{i-1}$$ | $$d_i$$ | $$\theta_i$$ |
|---|---|---|---|---|
|$$1$$|$$0$$|$$0$$|$$d_1 + a_2$$|$$\theta_1$$|
|$$2$$|$$\pi/2$$|$$0$$|$$0$$|$$\theta_2 + \pi/2$$|
|$$Tool (Laser)$$|$$\pi/2$$|$$a_3$$|$$0$$|$$\pi/2$$|

### Forward Kinematics
$${}_{O}^{Laser} T = \begin{bmatrix}
\sin \theta_1 & \sin \theta_2 \cos \theta_1 & \cos \theta_1 \cos \theta_2 & -a_3 \sin \theta_2 \cos \theta_1 \\
-\cos \theta_1 & \sin \theta_1 \sin \theta_2 & \sin \theta_1 \cos \theta_2 & -a_3 \sin \theta_1 \sin \theta_2 \\
0 & -\cos \theta_2 & \sin \theta_2 & a_2 + a_3 \cos \theta_2 + d_1 \\
0 & 0 & 0 & 1
\end{bmatrix}$$

### Inverse Kinematics
![드론 추적 기하학](https://github.com/spring98/drone-tracking-robot/assets/92755385/4d263b5d-cde5-444e-a2ae-311e66163468)


$$\theta_1 = \arctan\left(\frac{y}{x}\right)$$

$$\theta_2 = \arccos\left(L_2 A + \sqrt{(L_2 A)^2 + (A^2 + B)(B - L_2^2)}\right)$$

$$where, \quad A = z - L_1, \quad B = x^2 + y^2, \quad L_1 = d_1 + a_2, \quad L_2 = a_3$$


## Dynamics

$$ E_p = m_1 g l_1 + m_2 g ( l_1 + l_2 \cos \theta_2 + l_3 \sin \theta_2 ) $$

$$ E_k = \frac{1}{2} m_2 \dot{\theta}_1^2 \left( l_3^2 \cos^2 \theta_2 + l_2^2 \sin^2 \theta_2 - 2 l_2 l_3 \cos \theta_2 \sin \theta_2 \right) + \frac{1}{2} m_2 \dot{\theta}_2^2 \left( l_2^2 + l_3^2 \right) $$

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

## Result





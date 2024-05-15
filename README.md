# Drone Tracking Robot

## Preview


https://github.com/spring98/drone-tracking-robot/assets/92755385/92e1d344-143c-4592-9c2e-b84396f86e11


![드론 트레킹 실험](https://github.com/spring98/drone-tracking-robot/assets/92755385/8df1b261-4534-4210-ad07-ccf7a1d65935)


## Research Topic
컴퓨터 비전 AI 모델을 활용한 드론 추적 로봇 개발 및 PID(Proportional Integral Derivative), SMC(Sliding Mode Control), MPC(Model Predictive Control) 전략 성능 비교

## Scheme

## Specification
<img width="720" alt="스크린샷 2024-05-15 11 29 32" src="https://github.com/spring98/drone-tracking-robot/assets/92755385/a069562a-7fc3-40f0-b46e-a51008345307">

## Kinematics
<img width="481" alt="스크린샷 2024-05-15 11 29 07" src="https://github.com/spring98/drone-tracking-robot/assets/92755385/89d0baaa-1565-4af4-9b76-d77b52ffc5ca">

<img width="705" alt="스크린샷 2024-05-15 11 23 34" src="https://github.com/spring98/drone-tracking-robot/assets/92755385/ec3595ac-1477-459f-91f5-ccdbcf1c3945">


## Dynamics

## Control

## Result



$$\begin{bmatrix}
\sin \theta_1 & \sin \theta_2 \cos \theta_1 & \cos \theta_1 \cos \theta_2 & -a_3 \sin \theta_2 \cos \theta_1 \\
-\cos \theta_1 & \sin \theta_1 \sin \theta_2 & \sin \theta_1 \cos \theta_2 & -a_3 \sin \theta_1 \sin \theta_2 \\
0 & -\cos \theta_2 & \sin \theta_2 & a_2 + a_3 \cos \theta_2 + d_1 \\
0 & 0 & 0 & 1
\end{bmatrix}$$


$$\sum_{i=1}^{10} t_i$$

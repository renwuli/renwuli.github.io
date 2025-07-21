---
layout: post
title: VINS-Wheel
date: 2022-10-10 12:00:00
description: 
tags: IMU, Wheel, Preintegration, VINS
categories: SLAM
toc: true
---

Paper: Visual-Inertial Odometry Tightly Coupled with Wheel Encoder Adopting Robust Initialization and Online Extrinsic Calibration

PDF: <a href="https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8967607">https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8967607</a>

## Main Contributions:
- Fusion of IMU and wheel encoder at the preintegration stage, achieving more accurate scale through 4-DoF nonlinear optimization
- Joint initialization method for IMU-Camera-Wheel encoder
- Online calibration of IMU-Wheel encoder extrinsic parameters

## Hardware Setup:
Rear-wheel-drive four-wheeled vehicle (rear wheels maintain fixed orientation when front wheels rotate), wheel encoder installed on rear-left wheel. Velocity direction of left rear wheel always aligns with the y-axis.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/1.jpg" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 1
</div>

## Workflow
Initialization → Align sensor coordinate systems with gravity direction, create initial map → IMU and wheel encoder preintegration, feature extraction and tracking → Sliding window nonlinear optimization → Compute current frame's pvq

After initialization completes, extrinsic parameters are fixed and no longer updated.

### A. Preintegration
Extension of VINS-MONO preintegration formulas (incorporating wheel encoder)

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/2.jpg" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 2
</div>

At initial stage, $$ \hat{\alpha}_i^i, \hat{\beta}^i_i, \hat{\eta}^i_i $$ are all zero, while $$ \hat{\gamma}^i_i $$ is the identity quaternion.

> Joan Sola. Quaternion kinematics for the error-state kalman filter. arXiv preprint arXiv:1711.02508, 2017.

Using perturbation methods to derive kinematic equations and compute covariance matrix:

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/3.jpg" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 3
</div>

$$
\delta z^i_{l+1} = B_{i,l}n＋A_{i,l}\delta z_{l}^i
$$

$$
\Sigma_{i, l+1} = B_{i,l}QB_{i,l}^T+A_{i,l}\Sigma_{i,l}A_{i,l}^T
$$

Comparison with VINS-MONO:

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/4.jpg" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 4
</div>

Discusses VINS-MONO initialization limitations:
> The initialization procedure of VINS is well-designed, but prone to error for a car with a monocular camera facing forward moving at approximately constant velocity.

This indicates that when a vehicle moves forward at near-constant velocity, VINS-MONO often fails.

### B. Initialization

#### Gyroscope Bias
First perform SFM like VINS-MONO to obtain up-to-scale visual structure, then calibrate with IMU using rotation constraints via least squares to compute gyroscope bias:

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/5.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 5
</div>

After obtaining new gyroscope bias $$ \mathrm{b}_w $$, recompute preintegration to avoid accumulated errors from inaccurate bias.

#### Refine Gravity Direction and Initialize Velocity

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/6.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 6
</div>

Equations similar to VINS-MONO:

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/7.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 7
</div>

Since wheel encoder's XY plane is defined level with car body, its Z-axis approximately aligns with gravity direction. Transform to body-IMU frame for initial gravity:

$$
g_0^{b0} = R_o^b[0 ~ 0 ~ g]^T
$$

Refine gravity direction (known magnitude) similar to VINS-MONO by over-parameterizing in tangent plane with two new tangent vectors:

$$
g^{b0} = g_0^{b0}+B\triangle g
$$

where $B$ is the basis formed by these tangent vectors.

### C. Nonlinear Optimization
Cost function comprises three terms: marginalization term, reprojection error term, and IMU-wheel encoder term.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/8.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 8
</div>

The term $$ e_s^k $$ (IMU-wheel encoder residual) is our focus:

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/9.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 9
</div>

Differentiate w.r.t $$ b_{a_k}, b_{w_k} $$ and $$ R_o^b $$, then solve for optimal solution.

### D. Online Extrinsic Calibration
Two extrinsics: camera-IMU and IMU-odometry. Authors note that dynamic extrinsic adjustment may fail under poor constraints (e.g., lack of rotation, insufficient IMU excitation).

> Consider that one cannot distinguish the direction of local gravity from that of the accelerometer bias when there is no rotational motion, which is pointed out in [6]. That is to say, the lack of constraints will result in the unstable estimation of accelerometer bias. Conversely, the convergence of accelerometer bias indicates that the system has become well-constrained. 

**Convergence of accelerometer bias indicates sufficient system constraints.**

## References
[1] Tong Qin, Peiliang Li, and Shaojie Shen. Vins-mono: A robust and versatile monocular visual-inertial state estimator. IEEE Transactions on Robotics, 34(4):1004–1020, 2018.

[2] Meixiang Quan, Songhao Piao, Minglang Tan, and Shi-Sheng Huang. Tightly-coupled monocular visual-odometric slam using wheels and a mems gyroscope. arXiv preprint arXiv:1804.04854, 2018.

[3] Shaojie Shen, Nathan Michael, and Vijay Kumar. Tightly-coupled monocular visual-inertial fusion for autonomous flight of rotorcraft mavs. In Robotics and Automation (ICRA), 2015 IEEE International Conference on, pages 5303–5310. IEEE, 2015.

[4] 轮式编码器与VIO的融合（一）. https://zhuanlan.zhihu.com/p/149484507

[5] 胡占义-中国科学院大学-UCAS. https://people.ucas.ac.cn/~huzhanyi
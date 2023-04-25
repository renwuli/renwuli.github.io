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

## 主要贡献：
- 在预积分阶段融合IMU和轮速计，4-DoF非线性优化得到更加准确的尺度
- IMU-Camera-轮速计的联合初始化方法
- 在线标定IMU-轮速计外参

## 硬件设置：
后驱四轮车（当前面两个轮子旋转时，后面两个轮子的朝向不改变），轮速计安装在后左车轮。左后轮的速度方向始终朝向y轴。

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/1.jpg" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 1
</div>


## 流程
初始化 → 传感器坐标系与重力方向对齐，创建初始地图 → IMU、轮速计预积分，特征点提取与跟踪 → 滑动窗口非线性优化 → 当前帧的pvq计算得到

当初始化结束之后，外参便写死不再改变



### A. 预积分
其实是VINS-MONO预积分公式的扩展（加入轮速计）

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/2.jpg" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 2
</div>

在初始阶段，$$ \hat{\alpha}_i^i, \hat{\beta}^i_i, \hat{\eta}^i_i $$ 均为0，而 $$ \hat{\gamma}^i_i $$ 为单位四元数。

> Joan Sola. Quaternion kinematics for the error-state kalman filter. arXiv preprint arXiv:1711.02508, 2017.

参考上文，使用扰动方式计算出运动学公式，推导出协方差矩阵：

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

和VINS-MONO的对比下来看一下：

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/4.jpg" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 4
</div>

这里面也讨论了一下VINS-MONO初始化的问题：
> The initailization procedure of VINS is well-designed, but prone to error for a car with a monocular camera facing forward moving at approximately constant velocity.

也就是说，如果车子以近似恒速的状态向前运动，VINS-MONO往往会出错。

### B. 初始化

#### 陀螺仪bias

先像VINS-MONO一样做SFM，得到up-to-scale的视觉structure，然后与IMU进行手眼标定，通过旋转约束最小二乘计算得到陀螺仪的bias：

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/5.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 5
</div>

得到新的陀螺仪bias $$ \mathrm{b}_w $$ 之后，重新预积分，以避免使用不精确的陀螺仪bias引入的累计误差。


#### 修正重力方向和初始化速度

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/6.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 6
</div>

依然是和VINS-MONO类似的方程：

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/7.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 7
</div>

因为轮速计的$XY$平面在定义时是和car的机壳水平的，所以可以近似地认为轮速计的 $$ Z $$ 方向是和重力方向一致的，将其转到body-imu坐标系下得到重力的初始值：

$$
g_0^{b0} = R_o^b[0 ~ 0 ~ g]^T
$$

之后进行重力方向的修正（重力的大小已知），和VINS-MONO类似，将重力方向在切平面处过参数化，引入两个新的切向量，进行优化：

$$
g^{b0} = g_0^{b0}+B\triangle g
$$

其中，$B$即为这两个切向量的基。

### C. 非线性优化
cost function由三个部分组成：边缘化的term，重投影误差的term和IMU-轮速计的term。

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/8.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 8
</div>

其中 $$ e_s^k $$ 是IMU-轮速计的residual，是我们重点关注的：

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/vins-wheel/9.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 9
</div>

上式对 $$ b_{a_k}, b_{w_k} $$ 和 $$ R_o^b $$ 求导，再求解到最优解。

### D. 在线外参标定
分为两个外参：camera-imu外参和imu-odemetry外参。但是作者也说了，当整个系统没有良好的约束时（比如缺乏旋转、IMU没有良好的激励），动态地调整外参可能会导致系统运行失败。

> Consider that one cannot distinguish the direction of local gravity from that of the accelerometer bias when there is no rotational motion, which is pointed out in [6]. That is to say, the lack of constraints will result in the unstable estimation of accelerometer bias. Conversely, the convergence of accelerometer bias indicates that the system has become well-constrained. 

**当加速度计的bias很好地收敛，那就意味着整个系统有良好的约束。**


## 参考文献
[1] Tong Qin, Peiliang Li, and Shaojie Shen. Vins-mono: A robust and versatile monocular visual-inertial state estimator. IEEE Transactions on Robotics, 34(4):1004–1020, 2018.

[2] Meixiang Quan, Songhao Piao, Minglang Tan, and Shi-Sheng Huang. Tightly-coupled monocular visual-odometric slam using wheels and a mems gyroscope. arXiv preprint arXiv:1804.04854, 2018.

[3] Shaojie Shen, Nathan Michael, and Vijay Kumar. Tightly-coupled monocular visual-inertial fusion for autonomous flight of rotorcraft mavs. In Robotics and Automation (ICRA), 2015 IEEE International Conference on, pages 5303–5310. IEEE, 2015.

[4] 轮式编码器与VIO的融合（一）. https://zhuanlan.zhihu.com/p/149484507

[5] 胡占义-中国科学院大学-UCAS. https://people.ucas.ac.cn/~huzhanyi
---
layout: post
title: IMU Preintegration
date: 2022-10-01 10:00:00
description: Dive into IMU preintegration
tags: IMU, Preintegration, ORBSLAM3
categories: SLAM
toc: true
---

本文基于Foster2016年发表在 *IEEE Transactions on Robotics* 上的一篇文章：*On-Manifold Preintegration for Real-Time Visual-Inertial Odometry*。

## IMU噪声模型
6自由度的IMU有三轴加速度和三轴角速度。

$$
_{B}\widetilde{\omega}_{WB}(t) = _{B}\omega_{WB}(t) + \mathrm{b}^{g}(t) + \mathrm{\eta}^{g}(t)
$$

$$
_{B}\widetilde{a}(t) = \mathrm{R}_{WB}^{T}(t)(_{W}\mathrm{a}(t) - _{W}\mathrm{g}) + \mathrm{b}^{a}(t) + \mathrm{\eta}^{a}(t)
$$

以上分别为IMU坐标系下的角速度和加速度，IMU系的加速度为世界坐标系下的加速度去除重力加速的影响再转到IMU坐标系，上面的表达都忽略了地球的自转。加速度和角速度都各自受其偏置和随机游走噪声影响，这里认为偏置和随机游走噪声都是关于时间的函数，也就是说偏置和噪声不是一成不变的。

## IMU运动模型

$$
\dot{\mathrm{R}}_{WB} = \mathrm{R}_{WB} ~ _{B} \omega_{WB}^{\wedge}
$$

$$
_{W} \dot{\mathrm{v}} = _{W}\mathrm{a}
$$

$$
_{W} \dot{\mathrm{p}} = _{W}\mathrm{v}
$$


## 相邻时刻的RVP递推
IMU自身坐标系到世界坐标系的旋转：

$$
\mathrm{R}_{WB}(t + \triangle t) = \mathrm{R}_{WB} \mathrm{Exp}\left( \int_{t}^{t + \triangle t} { _{B}\omega_{WB}(\tau) d\tau} \right)
$$

世界坐标系下的速度：

$$
_{W} \mathrm{v}(t + \triangle t) = _{W} \mathrm{v}(t) + \int_{t}^{t + \triangle t} {_{W} \mathrm{a} (\tau) d\tau}
$$

世界坐标系下的位移：

$$
_{W} \mathrm{p}(t + \triangle t) = _{W} \mathrm{p}(t) + \int_{t}^{t + \triangle t} {_{W} \mathrm{v} (\tau) d\tau} + \iint_{t}^{t + \triangle t} {_{W} \mathrm{a}(\tau) d\tau^{2}}
$$

以上是相邻时刻IMU的三个状态的递推，如果我们假设在$$ [t, t + \triangle t] $$内，世界坐标系下的加速度 $$ _{W}\mathrm{a} $$ 和IMU坐标系下的角速度 $$ _{B}\omega _{WB} $$ 保持恒定，那么可以将上述连续形式下的递推改写为如下离散形式：

$$
\mathrm{R}_{WB} (t + \triangle t) = \mathrm{R}_{WB} (t) \mathrm{Exp} \left( _{B}\omega _{WB}(t) \triangle t \right)
$$

$$
_{W}\mathrm{v}(t + \triangle t) = _{W}\mathrm{v}(t) + _{W}\mathrm{a}(t) \triangle t
$$

$$
_{W}\mathrm{p}(t + \triangle t) = _{W}\mathrm{p}(t) + _{W}\mathrm{v}(t) \triangle t + \frac{1}{2} _{W}\mathrm{a}(t) \triangle t^{2}
$$

将$$ \mathrm{R}_{WB} $$ 记作$$ \mathrm{R} $$，默认速度 $$\mathrm{v} $$、位移 $$\mathrm{p}$$、重力加速度 $$\mathrm{g}$$ 和加速度 $$\mathrm{a}$$ 为世界坐标系下的量，而角速度 $$\omega$$ 为IMU坐标系下的量，则：

$$
\mathrm{R} (t + \triangle t) = \mathrm{R} (t) \mathrm{Exp} \left( \omega (t) \triangle t \right)
$$

$$
\mathrm{v}(t + \triangle t) = \mathrm{v}(t) + \mathrm{a}(t) \triangle t
$$

$$
\mathrm{p}(t + \triangle t) = \mathrm{p}(t) + \mathrm{v}(t) \triangle t + \frac{1}{2} \mathrm{a}(t) \triangle t^{2}
$$

考虑噪声模型，有：
$$
\mathrm{R} (t + \triangle t) = \mathrm{R} (t) \mathrm{Exp} \left( \left( \tilde{\omega} - \mathrm{b}^{g}(t) - \mathrm{\eta}^{gd}(t) \right)  (t) \triangle t \right)
$$

$$
\mathrm{v}(t + \triangle t) = \mathrm{v}(t) + \mathrm{g}\triangle t + \left( \tilde{\mathrm{a}}(t) - \mathrm{b}^{a}(t) - \mathrm{\eta}^{ad}(t) \right)  \triangle t
$$

$$
\mathrm{p}(t + \triangle t) = \mathrm{p}(t) + \mathrm{v}(t) \triangle t + \frac{1}{2}  \mathrm{g}\triangle t^{2} + \frac{1}{2} \mathrm{R}(t) \left( \tilde{\mathrm{a}}(t) - \mathrm{b}^{a}(t) - \mathrm{\eta}^{ad}(t) \right) \triangle t^{2}
$$


## 关键帧之间的RVP递推

$$
\mathrm{R}_{j} = \mathrm{R}_{i} \prod_{k=i}^{j-1} \mathrm{Exp} \left( \left( \tilde{\omega}_{k} - \mathrm{b}_{k}^{g} - \mathrm{\eta}_{k}^{gd} \right) \triangle t \right)
$$

$$
\mathrm{v}_{j} = \mathrm{v}_{i} + \mathrm{g} \triangle t_{ij} + \sum_{k=i}^{j-1} \mathrm{R}_{k} \left( \tilde{\mathrm{a}}_k - \mathrm{b}_k^{a} - \mathrm{\eta}_k^{ad} \right) \triangle t 
$$

$$
\mathrm{p}_{j} = \mathrm{p}_{i} + \sum_{k=i}^{j-1} \left[ \mathrm{v}_{k} \triangle t + \frac{1}{2} \mathrm{g} \triangle t^{2} + \frac{1}{2} \mathrm{R}_{k} \left( \tilde{\mathrm{a}}_{k} - \mathrm{b}_{k}^{a} - \mathrm{\eta}_{k}^{ad} \right) \triangle t^{2}  \right]
$$

借助上面三个公式，假如已知 $$i$$ 时刻的RVP状态，就可以推得 $$j$$ 时刻的RVP状态，但是在优化过程中，一旦 $$\mathrm{R}_{i}$$ 发生变化，要求得 $$j$$ 时刻的RVP状态，需要重新进行高计算量的积分，因此引入预积分，只需要将 $$\triangle  \mathrm{R}_{ij}$$ 这个中间增量求出来即可。

## 预积分

将第 $$i$$ 时刻的状态和重力加速度分离出来

$$
\triangle  \mathrm{R}_{ij} = \mathrm{R}_{i}^{T} \mathrm{R}_{j} = \prod_{k=i}^{j-1} \mathrm{Exp} \left( \left( \tilde{\omega}_{k} - \mathrm{b}_{k}^{g} - \mathrm{\eta}_{k}^{gd} \right) \triangle t \right)
$$

$$
\mathrm{v}_{j} - \mathrm{v}_{i} - \mathrm{g} \triangle t_{ij} = \sum_{k=i}^{j-1} \mathrm{R}_{k} \left( \tilde{\mathrm{a}}_k - \mathrm{b}_k^{a} - \mathrm{\eta}_k^{ad} \right) \triangle t 
$$

转换到第 $$i$$ 时刻的IMU坐标系，得：

$$
\begin{align*}
\triangle \mathrm{v}_{ij} &= \mathrm{R}_{i}^{T} \left( \mathrm{v}_{j} - \mathrm{v}_{i} - \mathrm{g} \triangle t_{ij} \right) \\
&= \sum_{k=i}^{j-1} \mathrm{R}_{ik} \left( \tilde{\mathrm{a}}_k - \mathrm{b}_k^{a} - \mathrm{\eta}_k^{ad} \right) \triangle t 
\end{align*}
$$

同理：

$$
\begin{align*}
\triangle \mathrm{p}_{ij} &= \mathrm{R}_{i}^{T} \left( \mathrm{p}_{j} - \mathrm{p}_{i} - \mathrm{v}_{i} \triangle t_{ij} - \frac{1}{2} \mathrm{g} \triangle t_{ij}^{2}  \right) \\
&= \sum_{k=i}^{j-1} \left[ \mathrm{v}_{ik} \triangle t + \frac{1}{2} \triangle \mathrm{R}_{ik} \left( \tilde{\mathrm{a}}_{k} - \mathrm{b}_{k}^{a} - \mathrm{\eta}_{k}^{ad} \right) \triangle t^{2} \right]
\end{align*}
$$

由此，上面三个式子的 rhs 便与第 $$i$$ 时刻和重力加速度无关，并且可以直接用第 $$i$$ 关键帧和第 $$j$$ 关键帧之间的IMU数据积分而来。

### 分离噪声

将上述预积分公式中的噪声分离出来，我们得到一个 **与噪声无关的项 + 噪声**的形式：

$$
\begin{align*}
  \triangle \mathrm{R}_{ij} &= \prod_{k=i}^{j-1} \left[ 
  \mathrm{Exp} \left( 
    \left( 
      \tilde{\omega}_{k} - \mathrm{b}_{i}^{g} \triangle t
     \right)
   \right)
  \mathrm{Exp} \left( 
    -\mathrm{J}_{r}^{k} \mathrm{\eta}_{k}^{gd} \triangle t
   \right)
 \right] \\
 &= \triangle \tilde{\mathrm{R}}_{ij} \prod_{k=i}^{j-1} \mathrm{Exp} \left( 
  - \triangle \tilde{\mathrm{R}}_{k+1 j}^{T} \mathrm{J}_{r}^{k}\mathrm{\eta}_{k}^{gd} \triangle t
  \right) \\
 &= \triangle \tilde{\mathrm{R}}_{ij} \mathrm{Exp} \left( -\delta \phi_{ij} \right)
\end{align*}
$$

其中，$$ \triangle \tilde{\mathrm{R}}_{ij} = \prod_{k=i}^{j-1} \mathrm{Exp} \left(\left(\tilde{\omega}_{k} - \mathrm{b}_{i}^{g}\right) \triangle t \right) $$。

将分离噪声之后的 $$ \triangle \tilde{\mathrm{R}}_{ij} $$ 带入到本节开头的预积分公式中去，可以得到速度和加速度预积分的第二个形式：

$$
\triangle \mathrm{v}_{ij} = \triangle \tilde{\mathrm{v}}_{ij} - \delta \mathrm{v}_{ij}
$$

其中：

$$
\triangle \tilde{\mathrm{v}_{ij}} = \sum_{k=i}^{j-1} \left[ 
  \triangle \tilde{\mathrm{R}}_{ij} \left( 
    \tilde{\mathrm{a}}_{k} - \mathrm{b}_{i}^{a}
   \right) \triangle t
 \right]
$$

对于位移，同样有：

$$
\triangle \mathrm{p}_{ij} = \triangle \tilde{\mathrm{p}}_{ij} - \delta \mathrm{p}_{ij}
$$

将上述三式带入到本节开头的预积分公式中，也就是：

$$
\triangle \mathrm{R}_{ij} = \mathrm{R}_{i}^{T} \mathrm{R}_{j}
$$

$$
\triangle \mathrm{v}_{ij} = \mathrm{R}_{i}^{T} \left( \mathrm{v}_{j} - \mathrm{v}_{i} - \mathrm{g} \triangle t_{ij} \right)
$$

$$
\triangle \mathrm{p}_{ij} = \mathrm{R}_{i}^{T} \left( \mathrm{p}_{j} - \mathrm{p}_{i} - \mathrm{v}_{i} \triangle t_{ij} - \frac{1}{2} \mathrm{g} \triangle t_{ij}^{2}  \right)
$$
得：
$$
\triangle \tilde{\mathrm{R}}_{ij} = \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathrm{Exp} \left( \delta \phi_{ij} \right)
$$

$$
\triangle \tilde{\mathrm{v}}_{ij} = \mathrm{R}_{i}^{T} \left( \mathrm{v}_{j} - \mathrm{v}_{i} - \mathrm{g} \triangle t_{ij} \right) + \delta \mathrm{v}_{ij}
$$

$$
\triangle \tilde{\mathrm{p}}_{ij} = \mathrm{R}_{i}^{T} \left( \mathrm{p}_{j} - \mathrm{p}_{i} - \mathrm{v}_{i} \triangle t_{ij} - \frac{1}{2} \mathrm{g} \triangle t_{ij}^{2}  \right) + \delta \mathrm{p}_{ij}
$$

那么预积分噪声向量便可定义为：

$$
\mathrm{\eta}_{ij}^{\triangle} =
\begin{bmatrix}
  \delta \phi_{ij}, \delta \mathrm{v}_{ij}, \delta \mathrm{p}_{ij}
\end{bmatrix}^{T}
$$

### 噪声传播
对预积分噪声 $$ \mathrm{\eta}_{ij}^{\triangle} $$ 做一阶近似，得:

$$
\delta \phi_{ij} \simeq \sum_{k=i}^{j-1} \triangle \tilde{\mathrm{R}}_{k+1 j}^{T} \mathrm{J}_{r}^{k} \eta_{k}^{gd} \triangle t
$$

$$
\delta \mathrm{v}_{ij} \simeq \sum_{k=i}^{j-1} \left[ 
  - \triangle \tilde{\mathrm{R}}_{ik} \left( 
    \tilde{\mathrm{a}}_{k} - \mathrm{b}_{i}^{a}
   \right)^{\wedge} \delta \phi_{ik} \triangle t + 
   \triangle \tilde{\mathrm{R}}_{ik} \eta_{k}^{ad} \triangle t
 \right]
$$

$$
\delta \mathrm{p}_{ij} \simeq \sum_{k=i}^{j-1} \left[ 
  \delta \mathrm{v}_{ik} \triangle t - \frac{1}{2} \triangle \tilde{\mathrm{R}}_{ik} \left( 
    \tilde{\mathrm{a}}_{k} - \mathrm{b}_{i}^{a}
   \right)^{\wedge}  \delta \phi_{ik} \triangle t^{2} + \frac{1}{2} \triangle \tilde{\mathrm{R}}_{ik} \eta_{k}^{ad} \triangle t^{2}
 \right]
$$

上式说明预积分噪声 $$ \mathrm{\eta}_{ij}^{\triangle} $$ 是关于IMU测量模型 $$ \mathrm{\eta}_{k}^{d} = \left[ \mathrm{\eta}_{k}^{gd}, \mathrm{\eta}_{k}^{ad} \right] $$ 的线性函数，因此，若已知IMU噪声 $$ \mathrm{\eta}_{k}^{d} $$ 的协方差矩阵，那么预积分噪声的协方差矩阵可以自然而然推出来。

### 偏置更新

在之前的内容中都假设偏置在关键帧 $$ i $$ 和关键帧  $$ j $$ 内是一成不变的，但实际上这是不可能滴。如果在偏置每次改变的时候都重新计算一下IMU观测和预积分，那是大大得耗费资源，是不可取滴。所以这里也把预积分项改写为关于偏置的一阶线性近似函数：

$$
\begin{align*}
\triangle \tilde{\mathrm{R}}_{ij}(\mathrm{b}_{i}^{g}) \simeq \triangle \tilde{\mathrm{R}}_{ij} (\bar{\mathrm{b}}_{i}^{g}) \mathrm{Exp} \left( 
  \frac{\partial{\triangle \tilde{\mathrm{R}}_{ij}}}{\partial{\mathrm{b}^{g}}} \delta \mathrm{b}^{g}
 \right) \\
\triangle \tilde{\mathrm{v}}_{ij}(\mathrm{b}_{i}^{g}, \mathrm{b}_{i}^{a}) \simeq \triangle \tilde{\mathrm{v}}_{ij} (\bar{\mathrm{b}}_{i}^{g}, \bar{\mathrm{b}}_{i}^{a}) + \frac{\partial{\tilde{\mathrm{v}}_{ij}}}{\partial{\mathrm{b}^{g}}} \delta \mathrm{b}^{g} + \frac{\partial{\tilde{\mathrm{v}}_{ij}}}{\partial{\mathrm{b}^{a}} } \delta \mathrm{b}^{a} \\
\triangle \tilde{\mathrm{p}}_{ij}(\mathrm{b}_{i}^{g}, \bar{\mathrm{b}}_{i}^{a}) \simeq \triangle \tilde{\mathrm{p}}_{ij} (\bar{\mathrm{b}}_{i}^{g}, \bar{\mathrm{b}}_{i}^{a}) + \frac{\partial{\tilde{\mathrm{p}}_{ij}}}{\partial{\mathrm{b}^{g}}} \delta \mathrm{b}^{g} + \frac{\partial{\tilde{\mathrm{p}}_{ij}}}{\partial{\mathrm{b}^{a}} } \delta \mathrm{b}^{a}
\end{align*}
$$

### 预积分因子

$$
\mathrm{r}_{\triangle \mathrm{R}_{ij}} = \mathrm{Log} \left( 
  \left( 
    \triangle \tilde{\mathrm{R}}_{ij} (\bar{\mathrm{b}}_{i}^{g}) \mathrm{Exp} \left( 
  \frac{\partial{\triangle \tilde{\mathrm{R}}_{ij}}}{\partial{\mathrm{b}^{g}}} \delta \mathrm{b}^{g}
 \right)
   \right)^{T}
  \mathrm{R}_{i}^{T} \mathrm{R}_{j}
 \right)
$$

$$
\mathrm{r}_{\triangle \mathrm{v}_{ij}} = \mathrm{R}_{i}^{T} \left( \mathrm{v}_{j} - \mathrm{v}_{i} - \mathrm{g} \triangle t_{ij} \right) - \left[ 
  \triangle \tilde{\mathrm{v}}_{ij} (\bar{\mathrm{b}}_{i}^{g}, \bar{\mathrm{b}}_{i}^{a}) + \frac{\partial{\tilde{\mathrm{v}}_{ij}}}{\partial{\mathrm{b}^{g}}} \delta \mathrm{b}^{g} + \frac{\partial{\tilde{\mathrm{v}}_{ij}}}{\partial{\mathrm{b}^{a}} } \delta \mathrm{b}^{a}
 \right]
$$

$$
\mathrm{r}_{\triangle \mathrm{p}_{ij}} = \mathrm{R}_{i}^{T} \left( \mathrm{p}_{j} - \mathrm{p}_{i} - \mathrm{v}_{i} \triangle t_{ij} - \frac{1}{2} \mathrm{g} \triangle t_{ij}^{2}  \right) - \left[ 
  \triangle \tilde{\mathrm{p}}_{ij} (\bar{\mathrm{b}}_{i}^{g}, \bar{\mathrm{b}}_{i}^{a}) + \frac{\partial{\tilde{\mathrm{p}}_{ij}}}{\partial{\mathrm{b}^{g}}} \delta \mathrm{b}^{g} + \frac{\partial{\tilde{\mathrm{p}}_{ij}}}{\partial{\mathrm{b}^{a}} } \delta \mathrm{b}^{a}
 \right]
$$

## 坐标系
<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/imu-preintegrated/frames.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Coordinate systems
</div>


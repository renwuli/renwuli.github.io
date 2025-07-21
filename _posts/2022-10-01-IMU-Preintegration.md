---
layout: post
title: IMU Preintegration
date: 2022-10-01 10:00:00
description: Dive into IMU preintegration
tags: IMU, Preintegration, ORBSLAM3
categories: SLAM
toc: true
---

This article is based on Foster's 2016 paper *On-Manifold Preintegration for Real-Time Visual-Inertial Odometry* published in *IEEE Transactions on Robotics*.

## IMU Noise Model
A 6-DOF IMU measures three-axis acceleration and three-axis angular velocity.

$$
_{B}\widetilde{\omega}_{WB}(t) = _{B}\omega_{WB}(t) + \mathrm{b}^{g}(t) + \mathrm{\eta}^{g}(t)
$$

$$
_{B}\widetilde{a}(t) = \mathrm{R}_{WB}^{T}(t)(_{W}\mathrm{a}(t) - _{W}\mathrm{g}) + \mathrm{b}^{a}(t) + \mathrm{\eta}^{a}(t)
$$

The equations above represent angular velocity and acceleration in the IMU frame. The IMU-frame acceleration is obtained by removing the gravitational acceleration from the world-frame acceleration and then transforming it to the IMU frame. Earth's rotation is neglected. Both acceleration and angular velocity are affected by their respective biases and random walk noise, modeled as time-varying quantities.

## IMU Motion Model

$$
\dot{\mathrm{R}}_{WB} = \mathrm{R}_{WB} ~ _{B} \omega_{WB}^{\wedge}
$$

$$
_{W} \dot{\mathrm{v}} = _{W}\mathrm{a}
$$

$$
_{W} \dot{\mathrm{p}} = _{W}\mathrm{v}
$$

## Recursive Propagation of R, V, P Between Consecutive Timesteps
Rotation from IMU frame to world frame:

$$
\mathrm{R}_{WB}(t + \triangle t) = \mathrm{R}_{WB} \mathrm{Exp}\left( \int_{t}^{t + \triangle t} { _{B}\omega_{WB}(\tau) d\tau} \right)
$$

Velocity in world frame:

$$
_{W} \mathrm{v}(t + \triangle t) = _{W} \mathrm{v}(t) + \int_{t}^{t + \triangle t} {_{W} \mathrm{a} (\tau) d\tau}
$$

Position in world frame:

$$
_{W} \mathrm{p}(t + \triangle t) = _{W} \mathrm{p}(t) + \int_{t}^{t + \triangle t} {_{W} \mathrm{v} (\tau) d\tau} + \iint_{t}^{t + \triangle t} {_{W} \mathrm{a}(\tau) d\tau^{2}}
$$

The above equations propagate the IMU state (Rotation, Velocity, Position) between consecutive timesteps. Assuming constant world-frame acceleration $$ _{W}\mathrm{a} $$ and constant IMU-frame angular velocity $$ _{B}\omega _{WB} $$ over $$ [t, t + \triangle t] $$, we discretize:

$$
\mathrm{R}_{WB} (t + \triangle t) = \mathrm{R}_{WB} (t) \mathrm{Exp} \left( _{B}\omega _{WB}(t) \triangle t \right)
$$

$$
_{W}\mathrm{v}(t + \triangle t) = _{W}\mathrm{v}(t) + _{W}\mathrm{a}(t) \triangle t
$$

$$
_{W}\mathrm{p}(t + \triangle t) = _{W}\mathrm{p}(t) + _{W}\mathrm{v}(t) \triangle t + \frac{1}{2} _{W}\mathrm{a}(t) \triangle t^{2}
$$

Adopting simplified notation: $$\mathrm{R}_{WB} \rightarrow \mathrm{R}$$, with $$\mathrm{v}$$ (velocity), $$\mathrm{p}$$ (position), $$\mathrm{g}$$ (gravity), and $$\mathrm{a}$$ (acceleration) in world frame, and $$\omega$$ (angular velocity) in IMU frame:

$$
\mathrm{R} (t + \triangle t) = \mathrm{R} (t) \mathrm{Exp} \left( \omega (t) \triangle t \right)
$$

$$
\mathrm{v}(t + \triangle t) = \mathrm{v}(t) + \mathrm{a}(t) \triangle t
$$

$$
\mathrm{p}(t + \triangle t) = \mathrm{p}(t) + \mathrm{v}(t) \triangle t + \frac{1}{2} \mathrm{a}(t) \triangle t^{2}
$$

Incorporating the noise model:
$$
\mathrm{R} (t + \triangle t) = \mathrm{R} (t) \mathrm{Exp} \left( \left( \tilde{\omega} - \mathrm{b}^{g}(t) - \mathrm{\eta}^{gd}(t) \right)  (t) \triangle t \right)
$$

$$
\mathrm{v}(t + \triangle t) = \mathrm{v}(t) + \mathrm{g}\triangle t + \mathrm{R}(t)\left( \tilde{\mathrm{a}}(t) - \mathrm{b}^{a}(t) - \mathrm{\eta}^{ad}(t) \right)  \triangle t
$$

$$
\mathrm{p}(t + \triangle t) = \mathrm{p}(t) + \mathrm{v}(t) \triangle t + \frac{1}{2}  \mathrm{g}\triangle t^{2} + \frac{1}{2} \mathrm{R}(t) \left( \tilde{\mathrm{a}}(t) - \mathrm{b}^{a}(t) - \mathrm{\eta}^{ad}(t) \right) \triangle t^{2}
$$

## R, V, P Propagation Between Keyframes

$$
\mathrm{R}_{j} = \mathrm{R}_{i} \prod_{k=i}^{j-1} \mathrm{Exp} \left( \left( \tilde{\omega}_{k} - \mathrm{b}_{k}^{g} - \mathrm{\eta}_{k}^{gd} \right) \triangle t \right)
$$

$$
\mathrm{v}_{j} = \mathrm{v}_{i} + \mathrm{g} \triangle t_{ij} + \sum_{k=i}^{j-1} \mathrm{R}_{k} \left( \tilde{\mathrm{a}}_k - \mathrm{b}_k^{a} - \mathrm{\eta}_k^{ad} \right) \triangle t 
$$

$$
\mathrm{p}_{j} = \mathrm{p}_{i} + \sum_{k=i}^{j-1} \left[ \mathrm{v}_{k} \triangle t + \frac{1}{2} \mathrm{g} \triangle t^{2} + \frac{1}{2} \mathrm{R}_{k} \left( \tilde{\mathrm{a}}_{k} - \mathrm{b}_{k}^{a} - \mathrm{\eta}_{k}^{ad} \right) \triangle t^{2}  \right]
$$

These equations propagate the state from keyframe $$i$$ to keyframe $$j$$. However, during optimization, if $$\mathrm{R}_{i}$$ changes, recomputing the integral for $$j$$ is computationally expensive. Preintegration solves this by computing intermediate increments ($$\triangle  \mathrm{R}_{ij}$$, $$\triangle \mathrm{v}_{ij}$$, $$\triangle \mathrm{p}_{ij}$$) only once.

## Preintegration

Factor out the state at time $$i$$ and gravity:

$$
\triangle  \mathrm{R}_{ij} = \mathrm{R}_{i}^{T} \mathrm{R}_{j} = \prod_{k=i}^{j-1} \mathrm{Exp} \left( \left( \tilde{\omega}_{k} - \mathrm{b}_{k}^{g} - \mathrm{\eta}_{k}^{gd} \right) \triangle t \right)
$$

$$
\mathrm{v}_{j} - \mathrm{v}_{i} - \mathrm{g} \triangle t_{ij} = \sum_{k=i}^{j-1} \mathrm{R}_{k} \left( \tilde{\mathrm{a}}_k - \mathrm{b}_k^{a} - \mathrm{\eta}_k^{ad} \right) \triangle t 
$$

Transform to the IMU frame at time $$i$$:

$$
\begin{align*}
\triangle \mathrm{v}_{ij} &= \mathrm{R}_{i}^{T} \left( \mathrm{v}_{j} - \mathrm{v}_{i} - \mathrm{g} \triangle t_{ij} \right) \\
&= \sum_{k=i}^{j-1} \mathrm{R}_{i}^{T}\mathrm{R}_{k} \left( \tilde{\mathrm{a}}_k - \mathrm{b}_k^{a} - \mathrm{\eta}_k^{ad} \right) \triangle t  \\
&= \sum_{k=i}^{j-1} \triangle \mathrm{R}_{ik} \left( \tilde{\mathrm{a}}_k - \mathrm{b}_k^{a} - \mathrm{\eta}_k^{ad} \right) \triangle t
\end{align*}
$$

Similarly for position:

$$
\begin{align*}
\triangle \mathrm{p}_{ij} &= \mathrm{R}_{i}^{T} \left( \mathrm{p}_{j} - \mathrm{p}_{i} - \mathrm{v}_{i} \triangle t_{ij} - \frac{1}{2} \mathrm{g} \triangle t_{ij}^{2}  \right) \\
&= \sum_{k=i}^{j-1} \left[ \mathrm{R}_{i}^{T}\mathrm{v}_{k} \triangle t + \frac{1}{2} \mathrm{R}_{i}^{T}\mathrm{R}_{k} \left( \tilde{\mathrm{a}}_{k} - \mathrm{b}_{k}^{a} - \mathrm{\eta}_{k}^{ad} \right) \triangle t^{2} \right] \\
&= \sum_{k=i}^{j-1} \left[ \triangle \mathrm{v}_{ik} \triangle t + \frac{1}{2} \triangle \mathrm{R}_{ik} \left( \tilde{\mathrm{a}}_{k} - \mathrm{b}_{k}^{a} - \mathrm{\eta}_{k}^{ad} \right) \triangle t^{2} \right]
\end{align*}
$$

The right-hand sides (rhs) of these equations are independent of the state at $$i$$ and gravity. They can be preintegrated directly using IMU measurements between keyframes $$i$$ and $$j$$.

### Noise Separation

Separate noise from the preintegration terms to obtain a form: **noise-independent term + noise**:

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

Where $$ \triangle \tilde{\mathrm{R}}_{ij} = \prod_{k=i}^{j-1} \mathrm{Exp} \left(\left(\tilde{\omega}_{k} - \mathrm{b}_{i}^{g}\right) \triangle t \right) $$ is the noise-free relative rotation increment.

Substituting the noise-separated $$ \triangle \tilde{\mathrm{R}}_{ij} $$ into the original preintegration formulas yields similar forms for velocity and position:

$$
\triangle \mathrm{v}_{ij} = \triangle \tilde{\mathrm{v}}_{ij} - \delta \mathrm{v}_{ij}
$$
where
$$
\triangle \tilde{\mathrm{v}}_{ij} = \sum_{k=i}^{j-1} \left[ 
  \triangle \tilde{\mathrm{R}}_{ik} \left( 
    \tilde{\mathrm{a}}_{k} - \mathrm{b}_{i}^{a}
   \right) \triangle t
 \right]
$$

$$
\triangle \mathrm{p}_{ij} = \triangle \tilde{\mathrm{p}}_{ij} - \delta \mathrm{p}_{ij}
$$

Substituting these three expressions back into the original preintegration definitions:

$$
\triangle \mathrm{R}_{ij} = \mathrm{R}_{i}^{T} \mathrm{R}_{j}
$$

$$
\triangle \mathrm{v}_{ij} = \mathrm{R}_{i}^{T} \left( \mathrm{v}_{j} - \mathrm{v}_{i} - \mathrm{g} \triangle t_{ij} \right)
$$

$$
\triangle \mathrm{p}_{ij} = \mathrm{R}_{i}^{T} \left( \mathrm{p}_{j} - \mathrm{p}_{i} - \mathrm{v}_{i} \triangle t_{ij} - \frac{1}{2} \mathrm{g} \triangle t_{ij}^{2}  \right)
$$

Yields the relationships:

$$
\triangle \tilde{\mathrm{R}}_{ij} = \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathrm{Exp} \left( \delta \phi_{ij} \right)
$$

$$
\triangle \tilde{\mathrm{v}}_{ij} = \mathrm{R}_{i}^{T} \left( \mathrm{v}_{j} - \mathrm{v}_{i} - \mathrm{g} \triangle t_{ij} \right) + \delta \mathrm{v}_{ij}
$$

$$
\triangle \tilde{\mathrm{p}}_{ij} = \mathrm{R}_{i}^{T} \left( \mathrm{p}_{j} - \mathrm{p}_{i} - \mathrm{v}_{i} \triangle t_{ij} - \frac{1}{2} \mathrm{g} \triangle t_{ij}^{2}  \right) + \delta \mathrm{p}_{ij}
$$

The preintegration noise vector is thus defined as:

$$
\mathrm{\eta}_{ij}^{\triangle} =
\begin{bmatrix}
  \delta \phi_{ij}, \delta \mathrm{v}_{ij}, \delta \mathrm{p}_{ij}
\end{bmatrix}^{T}
$$

### Noise Propagation
Applying first-order approximation to the preintegration noise $$ \mathrm{\eta}_{ij}^{\triangle} $$:

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

These equations show that the preintegration noise $$ \mathrm{\eta}_{ij}^{\triangle} $$ is a linear function of the IMU measurement noise $$ \mathrm{\eta}_{k}^{d} = \left[ \mathrm{\eta}_{k}^{gd}, \mathrm{\eta}_{k}^{ad} \right] $$. Therefore, if the covariance matrix of the IMU noise $$ \mathrm{\eta}_{k}^{d} $$ is known, the covariance matrix of the preintegration noise can be derived.

### Bias Update

Previous derivations assumed constant biases between keyframes $$i$$ and $$j$$. In reality, biases drift. Recomputing preintegrals upon every bias change is computationally expensive. Instead, express the preintegrated terms as first-order approximations with respect to bias:

$$
\begin{align*}
\triangle \tilde{\mathrm{R}}_{ij}(\mathrm{b}_{i}^{g}) \simeq \triangle \tilde{\mathrm{R}}_{ij} (\bar{\mathrm{b}}_{i}^{g}) \mathrm{Exp} \left( 
  \frac{\partial{\triangle \tilde{\mathrm{R}}_{ij}}}{\partial{\mathrm{b}^{g}}} \delta \mathrm{b}^{g}
 \right) \\
\triangle \tilde{\mathrm{v}}_{ij}(\mathrm{b}_{i}^{g}, \mathrm{b}_{i}^{a}) \simeq \triangle \tilde{\mathrm{v}}_{ij} (\bar{\mathrm{b}}_{i}^{g}, \bar{\mathrm{b}}_{i}^{a}) + \frac{\partial{\tilde{\mathrm{v}}_{ij}}}{\partial{\mathrm{b}^{g}}} \delta \mathrm{b}^{g} + \frac{\partial{\tilde{\mathrm{v}}_{ij}}}{\partial{\mathrm{b}^{a}} } \delta \mathrm{b}^{a} \\
\triangle \tilde{\mathrm{p}}_{ij}(\mathrm{b}_{i}^{g}, \bar{\mathrm{b}}_{i}^{a}) \simeq \triangle \tilde{\mathrm{p}}_{ij} (\bar{\mathrm{b}}_{i}^{g}, \bar{\mathrm{b}}_{i}^{a}) + \frac{\partial{\tilde{\mathrm{p}}_{ij}}}{\partial{\mathrm{b}^{g}}} \delta \mathrm{b}^{g} + \frac{\partial{\tilde{\mathrm{p}}_{ij}}}{\partial{\mathrm{b}^{a}} } \delta \mathrm{b}^{a}
\end{align*}
$$

Here, $$\bar{\mathrm{b}}_{i}^{g}, \bar{\mathrm{b}}_{i}^{a}$$ are the biases at the linearization point, and $$\delta \mathrm{b}^{g}, \delta \mathrm{b}^{a}$$ are small updates.

### Preintegration Factors

The residual errors (factors) used in optimization (e.g., Bundle Adjustment) are:

**Rotation Factor:**
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

**Velocity Factor:**
$$
\mathrm{r}_{\triangle \mathrm{v}_{ij}} = \mathrm{R}_{i}^{T} \left( \mathrm{v}_{j} - \mathrm{v}_{i} - \mathrm{g} \triangle t_{ij} \right) - \left[ 
  \triangle \tilde{\mathrm{v}}_{ij} (\bar{\mathrm{b}}_{i}^{g}, \bar{\mathrm{b}}_{i}^{a}) + \frac{\partial{\tilde{\mathrm{v}}_{ij}}}{\partial{\mathrm{b}^{g}}} \delta \mathrm{b}^{g} + \frac{\partial{\tilde{\mathrm{v}}_{ij}}}{\partial{\mathrm{b}^{a}} } \delta \mathrm{b}^{a}
 \right]
$$

**Position Factor:**
$$
\mathrm{r}_{\triangle \mathrm{p}_{ij}} = \mathrm{R}_{i}^{T} \left( \mathrm{p}_{j} - \mathrm{p}_{i} - \mathrm{v}_{i} \triangle t_{ij} - \frac{1}{2} \mathrm{g} \triangle t_{ij}^{2}  \right) - \left[ 
  \triangle \tilde{\mathrm{p}}_{ij} (\bar{\mathrm{b}}_{i}^{g}, \bar{\mathrm{b}}_{i}^{a}) + \frac{\partial{\tilde{\mathrm{p}}_{ij}}}{\partial{\mathrm{b}^{g}}} \delta \mathrm{b}^{g} + \frac{\partial{\tilde{\mathrm{p}}_{ij}}}{\partial{\mathrm{b}^{a}} } \delta \mathrm{b}^{a}
 \right]
$$

## Coordinate Systems
<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/imu-preintegrated/frames.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Coordinate systems involved in visual-inertial odometry.
</div>
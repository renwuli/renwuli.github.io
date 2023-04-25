---
layout: post
title: Wheel Preintegration
date: 2022-11-11 20:00:00
description: 
tags: IMU, Wheel, Preintegration, VINS
categories: SLAM
toc: true
---

## 轮速计噪声模型

$$
_{O}\widetilde{\mathbf{u}}(t) = _{O}\mathbf{u}(t) + \mathbf{\eta}^{u}(t)
$$

## 轮速计运动模型

$$
_{W} \dot{\mathbf{o}} = _{W}\mathbf{u}
$$

$$
_{W} \dot{\mathbf{o}} = \mathrm{R}_{WB} \cdot \mathrm{R}^{B}_{O} \cdot _{O} \mathbf{u}
$$

## 相邻时刻的轮速测量模型

$$
_{W}\mathbf{o}(t + \triangle t) = _{W}\mathbf{o}(t) + _{W}\mathbf{u} (t) \triangle t
$$

将 $$ \mathrm{R}_{WB} $$ 记作 $$ \mathrm{R} $$，默认位移 $$ \mathbf{o} $$ 和速度 $$ \mathbf{v} $$ 为世界坐标系下的量，则：

$$
\mathbf{o}(t + \triangle t) = \mathbf{o}(t) + \mathrm{R}_{k} \cdot \mathrm{R}^{B}_{O} \cdot \mathbf{u} \triangle t
$$

考虑噪声模型，有：

$$
\mathbf{o}(t + \triangle t) = \mathbf{o}(t) + \mathrm{R}_{k} \cdot \mathrm{R}^{B}_{O} \cdot \left( 
  \tilde{\mathbf{u}}(t) - \mathbf{\eta}^{ud}(t)
 \right) \triangle t
$$

## 关键帧之间的轮速测量模型

$$
\mathbf{o}_{j} = \mathbf{o}_{i} + \sum_{k=i}^{j-1} \mathrm{R}_{k} \cdot \mathrm{R}^{B}_{O} \left( 
  \tilde{\mathbf{u}}_{k} - \mathbf{\eta}^{ud}_{k}
 \right) \triangle t
$$

## 预积分

将第 $$ i $$ 时刻的状态分离出来：

$$
\mathbf{o}_{j} - \mathbf{o}_{i} = \sum_{k=i}^{j-1} \mathrm{R}_{k} \cdot \mathrm{R}^{B}_{O} \left( 
  \tilde{\mathbf{u}}_{k} - \mathbf{\eta}^{ud}_{k}
 \right) \triangle t
$$

并转换到第 $$ i $$ 时刻的IMU坐标系，得：

$$
\begin{align*}
\triangle \mathbf{o}_{ij} &= \mathrm{R}_{i}^{T} \left( \mathbf{o}_{j} - \mathbf{o}_{i} \right) \\
&= \sum_{k=i}^{j-1} \triangle  \mathrm{R}_{ik} \cdot \mathrm{R}^{B}_{O} \left( 
  \tilde{\mathbf{u}}_{k} - \mathbf{\eta}^{ud}_{k}
 \right) \triangle t
\end{align*}
$$

$$
{\color{red}
\triangle \mathbf{o}_{ij} = \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B}
}
$$

### 分离噪声

将上述预积分公式中得噪声分离出来：

$$
\begin{align*}
  \triangle \mathbf{o}_{ij} &= \sum_{k=i}^{j-1} \triangle \mathrm{R}_{ik} \cdot \mathrm{R}^{B}_{O} \left( 
  \tilde{\mathbf{u}}_{k} - \mathbf{\eta}^{ud}_{k}
 \right) \triangle t \\
&= \sum_{k=i}^{j-1} \triangle \tilde{\mathrm{R}}_{ik} \left( 
  \mathbf{I} - \delta\phi_{ik}^{\wedge} \right) \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \triangle t
   - \triangle  \tilde{\mathrm{R}}_{ik} \mathrm{R}^{B}_{O} \mathbf{\eta}_{k}^{ud} \triangle t \\
&= \sum_{k=i}^{j-1} \triangle \tilde{\mathrm{R}}_{ik} \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \triangle t + \sum_{k=i}^{j-1} \left[ 
  \triangle \tilde{\mathrm{R}}_{ik} \left( 
     \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k}
   \right)^{\wedge} \delta \phi_{ik} \triangle t - \triangle \tilde{\mathrm{R}}_{ik} \mathrm{R}^{B}_{O} \mathbf{\eta}_{k}^{ud} \triangle t
 \right] \\ 
&= \triangle \tilde{\mathbf{o}}_{ij} - \delta \mathbf{o}_{ij}
\end{align*}
$$

其中：

$$
\begin{align*}
\triangle \tilde{\mathbf{o}}_{ij} &= \sum_{k=i}^{j-1} \triangle \tilde{\mathrm{R}}_{ik} \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \triangle t \\
&= \triangle \mathbf{o}_{ij} + \delta \mathbf{o}_{ij}
\end{align*}
$$


### 噪声传播

$$
\begin{align*}
 \delta \mathbf{o}_{ij} &= \sum_{k=i}^{j-1} \left[ 
  -\triangle \tilde{\mathrm{R}}_{ik} \left( 
     \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k}
   \right)^{\wedge} \delta \phi_{ik} \triangle t + \triangle \tilde{\mathrm{R}}_{ik} \mathrm{R}^{B}_{O} \mathbf{\eta}_{k}^{ud} \triangle t
 \right] \\
&= \sum_{k=i}^{j-2} \left[
  -\triangle \tilde{\mathrm{R}}_{ik} \left( 
     \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k}
   \right)^{\wedge} \delta \phi_{ik} \triangle t + \triangle \tilde{\mathrm{R}}_{ik} \mathbf{\eta}_{k}^{ud} \mathrm{R}^{B}_{O} \triangle t
 \right] \\
&- \triangle \tilde{\mathrm{R}}_{ij-1} \left( 
     \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{j-1}
   \right)^{\wedge} \delta \phi_{ij-1} \triangle t + \triangle \tilde{\mathrm{R}}_{ij-1} \mathrm{R}^{B}_{O} \mathbf{\eta}_{j-1}^{ud} \triangle t \\
&= \delta \mathbf{o}_{ij-1} - \triangle \tilde{\mathrm{R}}_{ij-1} \left( 
     \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{j-1}
   \right)^{\wedge} \delta \phi_{ij-1} \triangle t + \triangle \tilde{\mathrm{R}}_{ij-1} \mathrm{R}^{B}_{O} \mathbf{\eta}_{j-1}^{ud} \triangle t
\end{align*}
$$

令预积分的噪声向量为:

$$
\delta \mathbf{\eta}_{ik}^{\triangle} = \left[ \delta \mathbf{\phi}_{ik}, \delta \mathbf{v}_{ik}, \delta \mathbf{p}_{ik}, \delta \mathbf{o}_{ik} \right] \in \mathbb{R}^{12}
$$

传感器噪声为：

$$
\mathbf{\eta}_{k}^{d} = \left[ \mathbf{\eta}_{k}^{gd}, \mathbf{\eta}_{k}^{ad}, \mathbf{\eta}_{k}^{ud} \right] \in \mathbb{R}^{3 \times 3} =  \in \mathbb{R}^{9}
$$

则预积分噪声的递推公式为：

$$
\delta \mathbf{\eta}_{ij}^{\triangle} = \mathbf{A}_{j-1} \delta \mathbf{\eta}_{ij-1}^{\triangle} + \mathbf{B}_{j-1} \mathbf{\eta}_{j-1}^{d}
$$ 

则：

$$
\mathbf{A}_{j-1} = \begin{bmatrix}
  \triangle \tilde{\mathrm{R}}_{j-1 j}^{T} & 0 & 0 & 0 \\
  -\triangle \tilde{\mathrm{R}}_{i j-1} \left( \tilde{\mathbf{a}}_{j-1} - \mathbf{b}_{i}^{a} \right)^{\wedge} \triangle t & \mathbf{I} & 0 & 0 \\
  -\frac{1}{2} \triangle \tilde{\mathrm{R}}_{i j-1} \left( \tilde{\mathbf{a}}_{j-1} - \mathbf{b}_{i}^{a} \right)^{\wedge} \triangle t^{2} & \triangle t \mathbf{I} & 0 & 0 \\
  -\triangle \tilde{\mathrm{R}}_{ij-1} \left( 
     \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{j-1}
   \right)^{\wedge} \triangle t & 0 & 0 & 0
\end{bmatrix}  \in \mathbb{R}^{12 \times 12}
$$

$$
\mathbf{B}_{j-1} = \begin{bmatrix}
  \mathbf{J}_{r}^{j-1} \triangle t & 0 & 0 \\
  0 & \triangle \tilde{\mathrm{R}}_{i j-1} \triangle t & 0 \\
  0 & \frac{1}{2}  \triangle \tilde{\mathrm{R}}_{i j-1} \triangle t^{2} & 0 \\
  0 & 0 & \triangle \tilde{\mathrm{R}}_{i j-1} \mathrm{R}^{B}_{O} \triangle t
\end{bmatrix}  \in \mathbb{R}^{12 \times 9}
$$

协方差矩阵递推公式为：

$$
\mathbf{\Sigma}_{ij} = \mathbf{A}_{j-1} \mathbf{\Sigma}_{ij-1} \mathbf{A}_{j-1}^{T} + \mathbf{B}_{j-1} \mathbf{\Sigma}_{\mathbf{\eta}}\mathbf{B}_{j-1}^{T}  \in \mathbb{R}^{12 \times 12}
$$

$$
\mathbf{\Sigma}_{\mathbf{\eta}} \in \mathbb{R}^{9 \times 9}
$$

### 偏置更新

$$
\triangle \tilde{\mathbf{o}}_{ij} \left( \mathbf{b}_{i}^{g} \right) \simeq \triangle \tilde{\mathbf{o}}_{ij} \left( \bar{\mathbf{b}}_{i}^{g} \right) + \frac{\partial{\bar{\mathbf{o}}_{ij}}}{\partial{\mathbf{b}_{i}^{g}}} \delta \mathbf{b}_{i}^{g}
$$

其中：

$$
\bar{\mathbf{o}}_{ij} = \tilde{\mathbf{o}}_{ij} \left( \bar{\mathbf{b}}_{i} \right)
$$

当偏置进行更新：

$$
\hat{\mathbf{b}}_{i} \leftarrow \bar{\mathbf{b}}_{i} + \delta \mathbf{b}_{i}
$$

相应的预积分进行更新：

$$
\begin{align*}
\tilde{\mathbf{o}}_{ij} (\hat{\mathbf{b}}_{i}) &= \sum_{k=i}^{j-1} \triangle \tilde{\mathrm{R}}_{ik} (\hat{\mathbf{b}}_{i}) \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \triangle t \\
&= \sum_{k=i}^{j-1} \triangle \bar{\mathrm{R}}_{ik} \mathbf{Exp} \left( \frac{\partial{\triangle \bar{\mathrm{R}}_{ik}}}{\partial{\mathbf{b}^{g}}} \delta \mathbf{b}_{i}^{g} \right) \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \triangle t \\
&= \sum_{k=i}^{j-1} \triangle \bar{\mathrm{R}}_{ik} \left( 
  \mathbf{I} + \left( 
    \frac{\partial{\triangle \bar{\mathrm{R}}_{ik}}}{\partial{\mathbf{b}^{g}}} \delta \mathbf{b}_{i}^{g}
   \right)^{\wedge}
 \right) \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \triangle t \\
&= \sum_{k=i}^{j-1} \triangle \bar{\mathrm{R}}_{ik} \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \triangle t + \sum_{k=i}^{j-1} \triangle  \bar{\mathrm{R}}_{ik} \left( 
    \frac{\partial{\triangle \bar{\mathrm{R}}_{ik}}}{\partial{\mathbf{b}^{g}}} \delta \mathbf{b}_{i}^{g}
   \right)^{\wedge} \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \triangle t \\
&= \triangle \bar{\mathbf{o}}_{ij} + \sum_{k=i}^{j-1} \triangle \bar{\mathrm{R}}_{ik} \left( 
    \frac{\partial{\triangle \bar{\mathrm{R}}_{ik}}}{\partial{\mathbf{b}^{g}}} \delta \mathbf{b}_{i}^{g}
   \right)^{\wedge} \left( \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \right) \triangle t \\
&= \triangle \bar{\mathbf{o}}_{ij} - \sum_{k=i}^{j-1} \triangle \bar{\mathrm{R}}_{ik} \left( \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \right)^{\wedge} \left( 
    \frac{\partial{\triangle \bar{\mathrm{R}}_{ik}}}{\partial{\mathbf{b}^{g}}} \delta \mathbf{b}_{i}^{g}
   \right) \triangle t \\
&= \triangle \bar{\mathbf{o}}_{ij} - \sum_{k=i}^{j-1} \triangle \bar{\mathrm{R}}_{ik} \left( \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \right)^{\wedge} 
    \frac{\partial{\triangle \bar{\mathrm{R}}_{ik}}}{\partial{\mathbf{b}^{g}}} \triangle t  \cdot \delta \mathbf{b}_{i}^{g} \\
&= {\color{red} \triangle \bar{\mathbf{o}}_{ij} + \frac{\partial{\triangle \bar{\mathbf{o}}_{ij}}}{\partial{\mathbf{b}^{g}}} \delta \mathbf{b}_{i}^{g}}
\end{align*}
$$

所以：

$$
\frac{\partial{\triangle \bar{\mathbf{o}}_{ij}}}{\partial{\mathbf{b}^{g}}} = - \sum_{k=i}^{j-1} \triangle \bar{\mathrm{R}}_{ik} \left( \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \right)^{\wedge} 
    \frac{\partial{\triangle \bar{\mathrm{R}}_{ik}}}{\partial{\mathbf{b}^{g}}} \triangle t
$$

### 预积分残差项

$$
\begin{align*}
\mathbf{r}_{\triangle \mathbf{o}_{ij}} &= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} \\
&- \left( \triangle \tilde{\mathbf{o}}_{ij} \left( \bar{\mathbf{b}}_{i}^{g} \right) + \frac{\partial{\bar{\mathbf{o}}_{ij}}}{\partial{\mathbf{b}_{i}^{g}}} \delta \mathbf{b}_{i}^{g} \right)  \\
&= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} \\
&- \left( \triangle \bar{\mathbf{o}}_{ij} + \frac{\partial{\bar{\mathbf{o}}_{ij}}}{\partial{\mathbf{b}_{i}^{g}}} \delta \mathbf{b}_{i}^{g} \right) 
\end{align*}
$$

#### 雅各比求解

$$
\begin{array}{l}
\mathrm{R}_{i} \leftarrow \mathrm{R}_{i} \operatorname{Exp}\left(\delta \boldsymbol{\phi}_{i}\right), \quad \mathrm{R}_{j} \leftarrow \mathrm{R}_{j} \operatorname{Exp}\left(\delta \boldsymbol{\phi}_{j}\right) \\[2mm]
\mathbf{p}_{i} \leftarrow \mathbf{p}_{i}+\mathrm{R}_{i} \delta \mathbf{p}_{i}, \quad \mathbf{p}_{j} \leftarrow \mathbf{p}_{j}+\mathrm{R}_{j} \delta \mathbf{p}_{j} \\[2mm]
\mathbf{v}_{i} \leftarrow \mathbf{v}_{i}+\delta \mathbf{v}_{i}, \quad \mathbf{v}_{j} \leftarrow \mathbf{v}_{j}+\delta \mathbf{v}_{i} \\[2mm]
\delta \mathbf{b}_{i}^{g} \leftarrow \delta \mathbf{b}_{i}^{g}+\tilde{\delta} \mathbf{b}_{i}^{g}, \quad \delta \mathbf{b}_{i}^{a} \leftarrow \delta \mathbf{b}_{i}^{a}+\tilde{\delta} \mathbf{b}_{i}^{a} \\
\end{array}
$$

- 对 $$ \delta \mathbf{p}_{i} $$

$$
\begin{align*}
\mathbf{r}_{\triangle \mathbf{o}_{ij}} (\mathbf{p}_{i} + \mathrm{R}_{i} \delta \mathbf{p}_{i}) &= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} - \mathrm{R}_{i} \delta \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C \\
&= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C - \delta \mathbf{p}_{i} \\
&= \mathbf{r}_{\triangle \mathbf{o}_{ij}} (\mathbf{p}_{i}) + (-\mathbf{I}_{3 \times 1}) \delta \mathbf{p}_{i}
\end{align*}
$$

所以：

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\delta \mathbf{p}_{i}}} = -\mathbf{I}_{3 \times 1}
$$

- 对 $$ \delta \mathbf{p}_{j} $$

$$
\begin{align*}
\mathbf{r}_{\triangle \mathbf{o}_{ij}} (\mathbf{p}_{j} + \mathrm{R}_{j} \delta \mathbf{p}_{j}) &= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} + \mathrm{R}_{j} \delta \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C \\
&= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C + (\mathrm{R}_{i}^{T} \mathrm{R}_{j}) \delta \mathbf{p}_{j} \\
&= \mathbf{r}_{\triangle \mathbf{o}_{ij}} (\mathbf{p}_{i}) + (\mathrm{R}_{i}^{T} \mathrm{R}_{j}) \delta \mathbf{p}_{i}
\end{align*}
$$

所以：

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\delta \mathbf{p}_{i}}} = \mathrm{R}_{i}^{T} \mathrm{R}_{j}
$$

- 对 $$ \delta \mathbf{v}_{i} $$

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\delta \mathbf{v}_{i}}} = 0 
$$

- 对 $$ \delta \mathbf{v}_{j} $$

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\delta \mathbf{v}_{j}}} = 0 
$$

- 对 $$ \delta \mathbf{\phi}_{i} $$

$$
\begin{align*}
\mathbf{r}_{\triangle \mathbf{o}_{ij}} \left( \mathrm{R}_{i} \mathbf{Exp} (\delta \phi_{i}) \right) &= \left[ \mathrm{R}_{i} \mathbf{Exp} (\delta \phi_{i}) \right]^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \left[ \mathrm{R}_{i} \mathbf{Exp} (\delta \phi_{i}) \right]^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C \\[2mm]
&= \left( \mathbf{I} - \delta \phi_{i}^{\wedge} \right) \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \left( \mathbf{I} - \delta \phi_{i}^{\wedge} \right) \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C \\[2mm]
&= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C \\
&+ \left( -\delta \phi_{i}^{\wedge} \right) \mathrm{R}_{i}^{T} \left( 
   \mathbf{p}_{j} - \mathbf{p}_{i} + \mathrm{R}_{j} \mathbf{t}_{O}^{B}
 \right) \\[2mm]
&= \mathbf{r}_{\triangle \mathbf{o}_{ij}} + \left[ \mathrm{R}_{i}^{T} \left( 
   \mathbf{p}_{j} - \mathbf{p}_{i} + \mathrm{R}_{j} \mathbf{t}_{O}^{B}
 \right) \right]^{\wedge} \delta \phi_{i}
\end{align*}
$$

注：

$$
\mathbf{Exp} (\phi)^{T} \simeq (\mathbf{I} + \phi^{\wedge})^{T} = \mathbf{I} + (\phi ^{\wedge})^{T} = \mathbf{I} - \phi^{\wedge}
$$

- 对 $$ \delta \mathbf{\phi}_{j} $$

$$
\begin{align*}
\mathbf{r}_{\triangle \mathbf{o}_{ij}} \left( \mathrm{R}_{j} \mathbf{Exp} (\delta \phi_{j}) \right)  &= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{Exp} (\delta \phi_{j}) \mathbf{t}_{O}^{B} + C \\[2mm]
&= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} (\mathbf{I} + \delta \phi_{j}^{\wedge}) \mathbf{t}_{O}^{B} + C \\[2mm]
&=  \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C + \mathrm{R}_{i}^{T} \mathrm{R}_{j}\delta \phi_{j}^{\wedge} \mathbf{t}_{O}^{B} \\[2mm]
&= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C - \mathrm{R}_{i}^{T} \mathrm{R}_{j} (\mathbf{t}_{O}^{B})^{\wedge} \delta \phi_{j}
\end{align*}

$$

所以：

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\delta \phi_{j}}} = - \mathrm{R}_{i}^{T} \mathrm{R}_{j} (\mathbf{t}_{O}^{B})^{\wedge}
$$

- 对 $$ \tilde{\delta} \mathbf{b}_{i}^{g} $$

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\tilde{\delta} \mathbf{b}_{i}^{g}}} = -\frac{\partial{\bar{\mathbf{o}}_{ij}}}{\partial{\mathbf{b}_{i}^{g}}}
$$

- 对 $$ \tilde{\delta} \mathbf{b}_{i}^{a} $$

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\tilde{\delta} \mathbf{b}_{i}^{a}}} = 0
$$
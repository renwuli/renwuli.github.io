---
layout: post
title: Wheel Preintegration
date: 2022-11-11 20:00:00
description:
tags: IMU, Wheel, Preintegration, VINS
categories: SLAM
toc: true
---

## Wheel Encoder Noise Model

$$
_{O}\widetilde{\mathbf{u}}(t) = _{O}\mathbf{u}(t) + \mathbf{\eta}^{u}(t)
$$

## Wheel Encoder Motion Model

$$
_{W} \dot{\mathbf{o}} = _{W}\mathbf{u}
$$

$$
_{W} \dot{\mathbf{o}} = \mathrm{R}_{WB} \cdot \mathrm{R}^{B}_{O} \cdot _{O} \mathbf{u}
$$

## Wheel Measurement Model Between Consecutive Timesteps

$$
_{W}\mathbf{o}(t + \triangle t) = _{W}\mathbf{o}(t) + _{W}\mathbf{u} (t) \triangle t
$$

Let $$ \mathrm{R}\_{WB} $$ be denoted as $$ \mathrm{R} $$, with default position $$ \mathbf{o} $$ and velocity $$ \mathbf{v} $$ in world frame:

$$
\mathbf{o}(t + \triangle t) = \mathbf{o}(t) + \mathrm{R}_{k} \cdot \mathrm{R}^{B}_{O} \cdot \mathbf{u} \triangle t
$$

Considering noise model:

$$
\mathbf{o}(t + \triangle t) = \mathbf{o}(t) + \mathrm{R}_{k} \cdot \mathrm{R}^{B}_{O} \cdot \left(
  \tilde{\mathbf{u}}(t) - \mathbf{\eta}^{ud}(t)
 \right) \triangle t
$$

## Wheel Measurement Model Between Keyframes

$$
\mathbf{o}_{j} = \mathbf{o}_{i} + \sum_{k=i}^{j-1} \mathrm{R}_{k} \cdot \mathrm{R}^{B}_{O} \left(
  \tilde{\mathbf{u}}_{k} - \mathbf{\eta}^{ud}_{k}
 \right) \triangle t
$$

## Preintegration

Factor out state at time $$ i $$:

$$
\mathbf{o}_{j} - \mathbf{o}_{i} = \sum_{k=i}^{j-1} \mathrm{R}_{k} \cdot \mathrm{R}^{B}_{O} \left(
  \tilde{\mathbf{u}}_{k} - \mathbf{\eta}^{ud}_{k}
 \right) \triangle t
$$

Transform to IMU frame at time $$ i $$:

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

### Noise Separation

Separate noise from preintegration:

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

Where:

$$
\begin{align*}
\triangle \tilde{\mathbf{o}}_{ij} &= \sum_{k=i}^{j-1} \triangle \tilde{\mathrm{R}}_{ik} \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \triangle t \\
&= \triangle \mathbf{o}_{ij} + \delta \mathbf{o}_{ij}
\end{align*}
$$

### Noise Propagation

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

Define preintegration noise vector:

$$
\delta \mathbf{\eta}_{ik}^{\triangle} = \left[ \delta \mathbf{\phi}_{ik}, \delta \mathbf{v}_{ik}, \delta \mathbf{p}_{ik}, \delta \mathbf{o}_{ik} \right] \in \mathbb{R}^{12}
$$

Sensor noise:

$$
\mathbf{\eta}_{k}^{d} = \left[ \mathbf{\eta}_{k}^{gd}, \mathbf{\eta}_{k}^{ad}, \mathbf{\eta}_{k}^{ud} \right] \in \mathbb{R}^{3 \times 3} =  \in \mathbb{R}^{9}
$$

Preintegration noise recursion:

$$
\delta \mathbf{\eta}_{ij}^{\triangle} = \mathbf{A}_{j-1} \delta \mathbf{\eta}_{ij-1}^{\triangle} + \mathbf{B}_{j-1} \mathbf{\eta}_{j-1}^{d}
$$

Where:

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

Covariance matrix recursion:

$$
\mathbf{\Sigma}_{ij} = \mathbf{A}_{j-1} \mathbf{\Sigma}_{ij-1} \mathbf{A}_{j-1}^{T} + \mathbf{B}_{j-1} \mathbf{\Sigma}_{\mathbf{\eta}}\mathbf{B}_{j-1}^{T}  \in \mathbb{R}^{12 \times 12}
$$

$$
\mathbf{\Sigma}_{\mathbf{\eta}} \in \mathbb{R}^{9 \times 9}
$$

### Bias Update

$$
\triangle \tilde{\mathbf{o}}_{ij} \left( \mathbf{b}_{i}^{g} \right) \simeq \triangle \tilde{\mathbf{o}}_{ij} \left( \bar{\mathbf{b}}_{i}^{g} \right) + \frac{\partial{\bar{\mathbf{o}}_{ij}}}{\partial{\mathbf{b}_{i}^{g}}} \delta \mathbf{b}_{i}^{g}
$$

Where:

$$
\bar{\mathbf{o}}_{ij} = \tilde{\mathbf{o}}_{ij} \left( \bar{\mathbf{b}}_{i} \right)
$$

When bias updates:

$$
\hat{\mathbf{b}}_{i} \leftarrow \bar{\mathbf{b}}_{i} + \delta \mathbf{b}_{i}
$$

Corresponding preintegration update:

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

Thus:

$$
\frac{\partial{\triangle \bar{\mathbf{o}}_{ij}}}{\partial{\mathbf{b}^{g}}} = - \sum_{k=i}^{j-1} \triangle \bar{\mathrm{R}}_{ik} \left( \mathrm{R}^{B}_{O} \tilde{\mathbf{u}}_{k} \right)^{\wedge}
    \frac{\partial{\triangle \bar{\mathrm{R}}_{ik}}}{\partial{\mathbf{b}^{g}}} \triangle t
$$

### Preintegration Residual

$$
\begin{align*}
\mathbf{r}_{\triangle \mathbf{o}_{ij}} &= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} \\
&- \left( \triangle \tilde{\mathbf{o}}_{ij} \left( \bar{\mathbf{b}}_{i}^{g} \right) + \frac{\partial{\bar{\mathbf{o}}_{ij}}}{\partial{\mathbf{b}_{i}^{g}}} \delta \mathbf{b}_{i}^{g} \right)  \\
&= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} \\
&- \left( \triangle \bar{\mathbf{o}}_{ij} + \frac{\partial{\bar{\mathbf{o}}_{ij}}}{\partial{\mathbf{b}_{i}^{g}}} \delta \mathbf{b}_{i}^{g} \right)
\end{align*}
$$

#### Jacobian Derivation

$$
\begin{array}{l}
\mathrm{R}_{i} \leftarrow \mathrm{R}_{i} \operatorname{Exp}\left(\delta \boldsymbol{\phi}_{i}\right), \quad \mathrm{R}_{j} \leftarrow \mathrm{R}_{j} \operatorname{Exp}\left(\delta \boldsymbol{\phi}_{j}\right) \\[2mm]
\mathbf{p}_{i} \leftarrow \mathbf{p}_{i}+\mathrm{R}_{i} \delta \mathbf{p}_{i}, \quad \mathbf{p}_{j} \leftarrow \mathbf{p}_{j}+\mathrm{R}_{j} \delta \mathbf{p}_{j} \\[2mm]
\mathbf{v}_{i} \leftarrow \mathbf{v}_{i}+\delta \mathbf{v}_{i}, \quad \mathbf{v}_{j} \leftarrow \mathbf{v}_{j}+\delta \mathbf{v}_{i} \\[2mm]
\delta \mathbf{b}_{i}^{g} \leftarrow \delta \mathbf{b}_{i}^{g}+\tilde{\delta} \mathbf{b}_{i}^{g}, \quad \delta \mathbf{b}_{i}^{a} \leftarrow \delta \mathbf{b}_{i}^{a}+\tilde{\delta} \mathbf{b}_{i}^{a} \\
\end{array}
$$

- W.r.t $$ \delta \mathbf{p}\_{i} $$

$$
\begin{align*}
\mathbf{r}_{\triangle \mathbf{o}_{ij}} (\mathbf{p}_{i} + \mathrm{R}_{i} \delta \mathbf{p}_{i}) &= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} - \mathrm{R}_{i} \delta \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C \\
&= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C - \delta \mathbf{p}_{i} \\
&= \mathbf{r}_{\triangle \mathbf{o}_{ij}} (\mathbf{p}_{i}) + (-\mathbf{I}_{3 \times 1}) \delta \mathbf{p}_{i}
\end{align*}
$$

Thus:

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\delta \mathbf{p}_{i}}} = -\mathbf{I}_{3 \times 1}
$$

- W.r.t $$ \delta \mathbf{p}\_{j} $$

$$
\begin{align*}
\mathbf{r}_{\triangle \mathbf{o}_{ij}} (\mathbf{p}_{j} + \mathrm{R}_{j} \delta \mathbf{p}_{j}) &= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} + \mathrm{R}_{j} \delta \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C \\
&= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C + (\mathrm{R}_{i}^{T} \mathrm{R}_{j}) \delta \mathbf{p}_{j} \\
&= \mathbf{r}_{\triangle \mathbf{o}_{ij}} (\mathbf{p}_{i}) + (\mathrm{R}_{i}^{T} \mathrm{R}_{j}) \delta \mathbf{p}_{i}
\end{align*}
$$

Thus:

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\delta \mathbf{p}_{i}}} = \mathrm{R}_{i}^{T} \mathrm{R}_{j}
$$

- W.r.t $$ \delta \mathbf{v}\_{i} $$

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\delta \mathbf{v}_{i}}} = 0
$$

- W.r.t $$ \delta \mathbf{v}\_{j} $$

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\delta \mathbf{v}_{j}}} = 0
$$

- W.r.t $$ \delta \mathbf{\phi}\_{i} $$

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

Note:

$$
\mathbf{Exp} (\phi)^{T} \simeq (\mathbf{I} + \phi^{\wedge})^{T} = \mathbf{I} + (\phi ^{\wedge})^{T} = \mathbf{I} - \phi^{\wedge}
$$

- W.r.t $$ \delta \mathbf{\phi}\_{j} $$

$$
\begin{align*}
\mathbf{r}_{\triangle \mathbf{o}_{ij}} \left( \mathrm{R}_{j} \mathbf{Exp} (\delta \phi_{j}) \right)  &= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{Exp} (\delta \phi_{j}) \mathbf{t}_{O}^{B} + C \\[2mm]
&= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} (\mathbf{I} + \delta \phi_{j}^{\wedge}) \mathbf{t}_{O}^{B} + C \\[2mm]
&=  \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C + \mathrm{R}_{i}^{T} \mathrm{R}_{j}\delta \phi_{j}^{\wedge} \mathbf{t}_{O}^{B} \\[2mm]
&= \mathrm{R}_{i}^{T} \left( \mathbf{p}_{j} - \mathbf{p}_{i} \right) - \mathbf{t}_{O}^{B} + \mathrm{R}_{i}^{T} \mathrm{R}_{j} \mathbf{t}_{O}^{B} + C - \mathrm{R}_{i}^{T} \mathrm{R}_{j} (\mathbf{t}_{O}^{B})^{\wedge} \delta \phi_{j}
\end{align*}


$$

Thus:

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\delta \phi_{j}}} = - \mathrm{R}_{i}^{T} \mathrm{R}_{j} (\mathbf{t}_{O}^{B})^{\wedge}
$$

- W.r.t $$ \tilde{\delta} \mathbf{b}\_{i}^{g} $$

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\tilde{\delta} \mathbf{b}_{i}^{g}}} = -\frac{\partial{\bar{\mathbf{o}}_{ij}}}{\partial{\mathbf{b}_{i}^{g}}}
$$

- W.r.t $$ \tilde{\delta} \mathbf{b}\_{i}^{a} $$

$$
\frac{\partial{\mathbf{r}_{\triangle \mathbf{o}_{ij}}}}{\partial{\tilde{\delta} \mathbf{b}_{i}^{a}}} = 0
$$

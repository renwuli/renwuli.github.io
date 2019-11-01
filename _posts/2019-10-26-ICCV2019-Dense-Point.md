---
layout: post
title: "ICCV2019 Dense Point"
subtitle: "Point Cloud"
date: 2019-10-26 00:00:00 UTC
background: '/img/posts/DensePoint_motivation.png'
---

## From

- Title: 《DensePoint: Learning Densely Contextual Representation for Efficient Point Cloud Processing》
- Source: ICCV2019 Oral
- Subject: Point Cloud Convolution
- Link: https://arxiv.org/pdf/1909.03669.pdf

## 简介

丰富的上下文语义信息对点云处理是至关重要的，但是之前的工作大多对此重视不足。本文提出了`DensePoint`，一种能够充分挖掘点云语义信息的方法。从技术上来看，是通过定义点云卷积来获取点云的语义信息，从网络结构上来看，是充分借鉴了二维图像领域里的`DenseNet`，通过稠密连接来获取多层级和多尺度的语义信息。

![DensePoint](https://renwuli.github.io/img/posts/DensePoint.png)

## How it works

卷积算子：和大部分的点云卷积算子类似，对点云中特定的点$x$进行卷积，也就是对该点的邻域点云$\mathcal{N}(x)$做特征提取$\phi$和特征聚集$\rho$。像$PointNet$使用了`Shared MLP`作为特征提取的手段，而本文为了提升效率采用了`single-layer perceptron(SLP)`，特征聚集一般是一个对称函数，常用的比如有`sum`和`max`。

$$\mathbf{f}_{\mathcal{N}(x)} = \rho({\phi(\mathbf{f}_{x_n})}, \forall x_{n} \in \mathcal{N}(x))$$

从网络结构上来看，极大借鉴了`DenseNet`，赋予网络强大的表达能力。

## Summary:

优势：网络模型小、速度快，更好地利用contexual信息，对稀疏点云更加鲁棒

做法：定义了一个点云卷积算子，借鉴DenseNet，采用稠密连接的方式加深网络深度，获取到更多层次的语义信息，但是同时带来了复杂度的提升，经引入分组卷积和用单层感知机替换常见的多层感知机来提取每个点的特征降低了复杂度
---
layout: post
title: SymmetryNet
date: 2022-04-01 15:00:00
description: 
tags: Symmetry
categories: Geometry, NN
toc: true
---

Paper: SymmetryNet: Learning to Predict Reflectional and Rotational
Symmetries of 3D Shapes from Single-View RGB-D Images

Author: YIFEI SHI, JUNWEN HUANG, HONGJIA ZHANG, XIN XU, SZYMON RUSINKIEWICZ, KAI XU

PDF: <a href="https://arxiv.org/pdf/2008.00485.pdf">https://arxiv.org/pdf/2008.00485.pdf</a>

Code: <a href="https://github.com/GodZarathustra/SymmetryNet">https://github.com/GodZarathustra/SymmetryNet</a>

## Overview

- 输入：RGB-D
- 输出：$$M^{\text{ref}}$$ 个镜面对称和 $$M^{\text{rot}}$$ 个旋转对称

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/symmetrynet/1.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 1
</div>

总体方法：
1. 特征提取：RGB图像输入CNN网络中提取逐像素特征，Depth深度图转成点云输入PointNet提取逐点特征，之后将图像特征和点云特征进行Fusion。
2. 以每个点的特征作为输入，逐点进行对称性预测。
3. 最后通过aggregation和可视性验证对逐点对称性进行过滤和集成，得到最终的对称性预测结果。

## Introduction

对称性检测可以通过纯几何信息进行求解，比如先建立点到点之间的对应关系，得到大量的点对之间的对称性变换，然后通过霍夫投票得到全局的对称性 $$ \rightarrow $$ 但是这种方法在single-view（几何信息不足，partial observation and object occlusion）的情况下面临挑战，有可能找不到局部的对称点对来支撑全局的对称性 $$ \rightarrow $$ 所以当前的对称性检测不仅仅需要依赖几何信息，也需要来自于统计分析（也就是从大量数据中去学习patten）。

## Methods

该文claim一旦3D模型的几何已知，那么得到它的对称性是分分钟的事情（真的这么简单吗？我看未必）。传统的对称性检测方法通常先建立点或者组件之间的correspondence，然后再aggragate得到对称性。但是single-view的表征通常是不完整和视角受限的。在不完整几何上做对称性检测是病态的。通常我们去识别一个不完整物体对称性的时候，会去靠先验来判别这个对称性是不是歧义的。但是对于一个我们不认识的物体或者说辨别不出来是什么种类的物体，我们没有先验，那么只能通过建立对称的correspondence来推测出对称性。这篇文章主要聚焦的问题就是：**对于认识或者不认识的物体，通过将对称性预测和对称映射耦合起来，构造一个统一的single-view的对称性检测方案**。

### 逐点的对称性预测

因为对称性是non-local的，所以需要共同使用全局特征和局部特征来进行逐点对称性的预测。因为avg-pooling对于对称性预测是冗余的，max-pooling可能会丢失太多信息，该文采用weighted pooling，也就是对每个点的特征赋予一个权重，然后进行加权求和得到全局特征，该权重也是通过一个小网络学出来的，叫做spatially weighted pooling layer。

为了提高精度和泛化性，采用multi-task策略进行训练，也就是：
1. 分类，判别对称性种类：no symmetry / reflective symmetry / rotational symmetry
2. 回归对称性的参数
3. 回归该点的对称点位置
4. 分类，判别该点是不是某个点的对称点

为了让网络更加好训，所有预测的3D坐标均是相对于当前点的局部坐标，对于以上4部分，分别设计loss如下：
1. cross-entropy
2. 当前点 $$ P_i $$ 投影到到预测的对称平面（对称轴）的点与真实的投影点之间的2范数
3. 2范数
4. cross-entropy

对于旋转对称来说，我们去做上面 3 的 loss 不太容易，因为一个点的旋转对称点有可能有多个（旋转对称的阶数有穷）或无穷多个（连续的旋转对称），所以该文选择不直接对 3 做 L2 的 loss，而是去预测某点在不在该点对应的旋转对称轨道上的概率。并且通过将预测旋转对称的阶数转换成一个分类的问题，也就是 0~R 类，0 代表连续旋转对称，R 代表所能预测的最大旋转对称阶数，该文将 R 取作 10，也就是说该文能预测的最大的旋转对称的阶数为 10。

### 处理任意数目的对称性
如果要处理任意数目的对称性预测，要不设计一个循环神经网络（显然是不现实的，因为我要知道到底循环多少次？），要么是引出 M 个分支来预测 M 个对称性（其中 M 是设定的最大对称性的数目）。但是采用后者的策略需要将 M 个分支各自区分开来（也就是定义顺序），该文采用基于 optimal assignment 的方法来训练网络。也就是将M个输出与GT对应起来。

对于那些已经被判断对称性种类的分类器验证（输出不为0）的对称性预测值，找到它所对应的GT对称性，然后求loss。

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/symmetrynet/2.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 2
</div>

虽然这里是说处理任意数目的对称性，但是显然，依然受上一节中 R=10 的限制。

### 对称性推理

在推理阶段，首先编码得到 RGB-D 的特征，然后对每个点进行对称性预测，然后使用聚类的方式得到最终的全局对称性预测结果。因为每个点预测对称性的准确性有差异，这里通过对 symmetry type classifier 的最后一层接一个概率层，得到每个点预测结果的权重，之后将这个概率作为 DBSCAN 中的密度权重，通过聚类得到最终的预测结果。

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/symmetrynet/3.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 3
</div>

得到预测的对称性结果之后，还需要做的是对称性的检验，通过检验的对称性留下，没有通过检验的去除。将深度图转换为提速表示，将空间划分为三部分：可视部分，空气部分和未知部分，然后得到可视部分的对称部分，将对称部分与空气部分求交，便得到了 mismatch 的部分（很容易理解，如果对称部分是合理的，那么我摄像头应该本来就能看到，按理来说应该是可视部分，但是现在却为空气部分，说明不合理）。如果 mismatch 的部分太大，就说明该对称性预测的不对。这个策略作者也在训练阶段作为一项额外的约束进行了尝试，但是发现收敛太慢，所以就只在推理完之后作为一个验证手段。


## 评价标准

该文使用 precision-recall 指标来评价对称性预测的好与坏，其中 precision 代表我预测的对称性中有多大比例是正确的，而 recall 代表 GT 中的对称性我有多大的比例正确预测出来了。

关于如何衡量预测的对称性是正确还是错误，作者给出了一个简单的评估指标：将模型按照预测的对称性对称过去，然后求原模型和对称模型的距离，该距离与 GT 对称性求得的距离做损失来打分。

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/symmetrynet/4.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 4
</div>

## Implementation

读完 paper 有一个问题，如果每个模型都预测 $$M$$ 个对称性，但是每个模型的实际对称性个数可不是都一样的，有的模型 1 个对称，有的模型 2 个对称，有的模型可能更多，那么怎么做批处理，计算 loss 反传呢？读作者的代码，从 https://github.com/GodZarathustra/SymmetryNet/blob/HEAD/lib/loss.py#L10-L11 可以看到作者在每次计算 loss 的时候，实际上 batch size 为 1，在 https://github.com/GodZarathustra/SymmetryNet/blob/HEAD/tools/train_shapenet.py#L81-L82 也可以看到 dataloader 的 batch size 也被设为 1 了，那么事情就变得明了，作者在实际实现的时候，是一个模型一个模型训练的，没有采用批处理，这也算是无奈之举。

## 总结

Pros.
1. it handles RGB-D inputs, and can deal with incomplete and partial observation
2. end-to-end deep learning method

Cons:
1. strong supervision
2. limited with pre-defined maximum of number of symmetries per object
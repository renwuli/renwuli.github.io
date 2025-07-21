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

- Input: RGB-D
- Output: $$M^{\text{ref}}$$ mirror symmetries and $$M^{\text{rot}}$$ rotational symmetries

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/symmetrynet/1.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 1
</div>

General approach:

1. Feature extraction: RGB image is input to a CNN to extract per-pixel features, depth map is converted to point cloud and input to PointNet to extract per-point features, then image and point cloud features are fused.
2. Each point's feature is used to predict symmetry at the point level.
3. Finally, aggregation and visibility validation are used to filter and integrate pointwise symmetry predictions to obtain the final symmetry prediction results.

## Introduction

Symmetry detection can be solved using pure geometric information, e.g., first establishing point-to-point correspondences to obtain many symmetric transformations between point pairs, then using Hough voting to obtain global symmetries. $$\rightarrow$$ However, this method faces challenges in single-view (insufficient geometric information, partial observation and object occlusion) scenarios, where it may be impossible to find enough local symmetric point pairs to support global symmetry. $$\rightarrow$$ Therefore, current symmetry detection needs to rely not only on geometric information but also on statistical analysis (i.e., learning patterns from large amounts of data).

## Methods

The paper claims that once the 3D geometry of a model is known, obtaining its symmetries is trivial (is it really that simple? I doubt it). Traditional symmetry detection methods usually first establish correspondences between points or components, then aggregate to obtain symmetries. However, single-view representations are usually incomplete and viewpoint-limited. Detecting symmetries on incomplete geometry is ill-posed. When recognizing the symmetry of an incomplete object, we often rely on priors to judge whether the symmetry is ambiguous. But for an unknown object or one whose category cannot be determined, we have no prior, so we can only infer symmetry by establishing symmetric correspondences. This paper focuses on the problem: **For known or unknown objects, by coupling symmetry prediction and symmetry mapping, construct a unified single-view symmetry detection scheme.**

### Pointwise Symmetry Prediction

Since symmetry is non-local, both global and local features are needed for pointwise symmetry prediction. Since avg-pooling is redundant for symmetry prediction and max-pooling may lose too much information, the paper uses weighted pooling, i.e., assigning a weight to each point's feature and then performing weighted sum to obtain the global feature. The weights are also learned by a small network, called the spatially weighted pooling layer.

To improve accuracy and generalization, a multi-task strategy is used for training:

1. Classification: determine symmetry type (no symmetry / reflective symmetry / rotational symmetry)
2. Regression: symmetry parameters
3. Regression: symmetric point position for each point
4. Classification: determine whether a point is the symmetric point of another point

To make the network easier to train, all predicted 3D coordinates are relative to the current point's local coordinates. The losses for the above four parts are:

1. cross-entropy
2. L2 norm between the projection of point $$ P_i $$ onto the predicted symmetry plane (or axis) and the ground truth projection
3. L2 norm
4. cross-entropy

For rotational symmetry, it's not easy to directly apply the L2 loss in part 3, because a point's rotationally symmetric counterpart may have multiple (finite-order) or infinitely many (continuous) possibilities. Therefore, the paper chooses not to directly apply L2 loss for 3, but instead predicts the probability that a point lies on the rotational symmetry orbit of another point. The predicted order of rotational symmetry is converted into a classification problem, i.e., 0~R classes, where 0 means continuous rotational symmetry, and R is the maximum order the network can predict (set to 10 in the paper).

### Handling Arbitrary Numbers of Symmetries

To handle arbitrary numbers of symmetry predictions, one could design a recurrent neural network (not practical, since you don't know how many times to loop), or introduce M branches to predict M symmetries (where M is the maximum number of symmetries). However, the latter requires distinguishing the M branches (i.e., defining an order). The paper uses an optimal assignment-based method for training, i.e., matching the M outputs to the ground truth symmetries.

For those symmetry predictions validated by the symmetry type classifier (output not 0), find the corresponding ground truth symmetry and compute the loss.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/symmetrynet/2.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 2
</div>

Although this claims to handle arbitrary numbers of symmetries, it is still limited by the R=10 constraint above.

### Symmetry Inference

During inference, first encode the RGB-D features, then predict symmetry for each point, then use clustering to obtain the final global symmetry predictions. Since the accuracy of each point's symmetry prediction varies, the last layer of the symmetry type classifier is followed by a probability layer, giving each point's prediction a weight. This probability is used as the density weight in DBSCAN clustering to obtain the final predictions.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/symmetrynet/3.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 3
</div>

After obtaining the predicted symmetries, a symmetry validation step is performed: only symmetries that pass the validation are kept. The depth map is converted to a voxel representation, and the space is divided into three parts: visible, air, and unknown. The symmetric part of the visible region is obtained, and its intersection with the air region gives the mismatch region (intuitively, if the symmetry is correct, the symmetric part should be visible, but if it falls in the air region, it's likely incorrect). If the mismatch region is too large, the symmetry prediction is considered incorrect. The authors also tried using this as an extra constraint during training, but found convergence too slow, so it is only used as a post-inference validation.

## Evaluation Metrics

The paper uses precision-recall to evaluate symmetry prediction. Precision is the proportion of predicted symmetries that are correct, and recall is the proportion of ground truth symmetries that are correctly predicted.

To determine whether a predicted symmetry is correct, the authors propose a simple metric: reflect the model according to the predicted symmetry, then compute the distance between the original and reflected models, and compare this to the distance using the ground truth symmetry.

<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/symmetrynet/4.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 4
</div>

## Implementation

After reading the paper, one question arises: if each model predicts $$M$$ symmetries, but the actual number of symmetries per model varies (some have 1, some 2, some more), how is batch processing and loss backpropagation handled? Looking at the code (https://github.com/GodZarathustra/SymmetryNet/blob/HEAD/lib/loss.py#L10-L11), the batch size is actually 1 during loss computation. In https://github.com/GodZarathustra/SymmetryNet/blob/HEAD/tools/train_shapenet.py#L81-L82, the dataloader batch size is also set to 1. So, in practice, the author trains one model at a time, not using batch processing, which is a necessary compromise.

## Summary

Pros:

1. Handles RGB-D inputs and can deal with incomplete and partial observations
2. End-to-end deep learning method

Cons:

1. Strong supervision required
2. Limited by the pre-defined maximum number of symmetries per object (R=10)

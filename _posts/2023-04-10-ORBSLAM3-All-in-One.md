---
layout: post
title: ORBSLAM3 All in One
date: 2023-04-10 14:00:00
description: Dive into ORBSLAM3
tags: ORBSLAM3
categories: SLAM
toc: true
---


写这篇文章的初衷有两个，一是目前的简中互联网实在是没有对ORBSLAM3庖丁解牛解析很全面的博客，尽管有很多从理论和代码层面都分析的不错的文章和视频，还有开源的带注释的源码，但都不够系统，且有些资源是收费的；二是想要消化一下对ORBSLAM3，加深对此算法的理解，做到知其然并知其所以然。我希望能够通过我这篇长文，能够让希望了解ORBSLAM3算法的朋友不用再费心费力去找别的资料，一文就能搞懂理论和代码，所以我给这篇文章起了个题目：*ORBSLAM3 All in One*。

不同于其他文章可能会上来就讲前端，诸如ORB特征点是什么啊，为什么ORB特征具有旋转不变性啊，怎么用八叉树均匀地将特征点铺满整张图上啊，我觉得这些问题都非常细，一上来就看这些容易让人陷入局部极小值跳不出来，无法从宏观的角度去俯瞰整个世界，导致对ORBSLAM3失去兴趣。我将会以 *算法总览* -- *算法详解* -- *问题* 的思路，去介绍ORBSLAM3。


## 算法总览

看到这篇文章的朋友应该都已经将ORBSLAM3的文章读过了，如果还没有读过的话，最好先赶紧去读一遍再说：<a href="https://arxiv.org/pdf/2007.11898.pdf">https://arxiv.org/pdf/2007.11898.pdf</a>

### 传感器支持

| 相机模型 | 相机组合 | 运行模式 |
| :-------: | :-------: | :--------: |
| 针孔相机<br>鱼眼相机 | 单目<br>双目<br>RGB-D | 视觉（Visual SLAM）<br>视觉-惯性（Visual-Inertial SLAM） |

从上表中可以看到 ORBSLAM3支持的传感器和运行模式那是相当丰富。当然，如果本文将以上的运行模式全都介绍一遍那是相当的冗余和啰嗦，所以本文以 *单目-针孔相机-视觉惯性* 模式为例展开介绍。其他的传感器组合和运行模式读者可以参考理解，触类旁通。

### 系统架构

下图是算法整体的一个pipeline，可以看到一共分为了四个部分：

- Tracking: 也就是跟踪，其输入是 **图像帧** 和 **IMU帧**，目的是得到当前帧相对于上一（关键）帧的相对位姿，如果我们知道了第一个（关键）帧的位姿，那么从前往后我们便可以一帧一帧串起来，得到当前帧在世界坐标系中的绝对位姿。
- Local Mapping: 得到当前帧相对于上一帧（参考帧）的相对位姿之后我们便可以通过三角化计算出当前帧视觉特征点的位置，如果当前帧检测出来了1000个特征点，那么局部地（从当前帧往前追溯一直到没有断掉为止），我们便得到了一张具有1000个特征点的地图。
- Loop & Map Merging：即回环检测和地图合并。Local Mapping 毕竟是一个局部地图，如果我们在 Tracking 的时候突然丢失了怎么办？ORBSLAM3的做法是再建立一个局部的地图。而这些局部地图的合集，我们称作 Atlas，翻译为中文就是“地图册”。当运行一段时间之后，如果我们发现又回到了之前到过的地方（回环检测），便可以将之前的局部地图和目前的局部地图进行合并，得到一张更大、特征点更多的地图。
- Full BA：最后便是将位姿和特征点坐标进行联合优化，期待能够得到更加准确的位姿估计和更加准确的特征点地图。


<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/orbslam3-all-in-one/fig1.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 1
</div>

## 算法详解

### 名词概念

- 帧 (Frame)

帧 (Frame) 可以简单地理解为采集到视频流的每一帧图像。

- 关键帧 (Key Farme)

关键帧 (Key Frame) 是帧 (Frame) 的一个子集，一般情况下是真子集。使用基于关键帧的视觉 SLAM 方法会仅使用一小部分图像帧来估计地图，丢弃掉冗余的信息，降低系统的复杂度。具体来说，当系统检测到场景中出现了显著的运动时，就会将当前图像帧作为一个新的关键帧，并用它来更新地图和计算相机的位姿。

关键帧的作用主要有以下几个方面：

1. 建图：通过对不同关键帧的匹配，可以在三维空间中重建场景，从而得到一个稠密的点云地图。

2. 定位：当相机在新的位置进行拍摄时，系统可以通过匹配当前帧与已有的关键帧，来估计相机的位姿和当前位置。

3. 优化：关键帧可以用于优化相机位姿和地图，从而提高系统的精度和稳定性。

总之，关键帧是视觉SLAM系统中的重要组成部分，它能够为后续的建图和定位提供重要的信息，从而实现更加准确和可靠的定位和建图。

- 地图点 (Map Point) 与 地图 (Map)

地图点 (Map Point) 是指在三维空间中被重建出来的具有代表性的特征点，并且通过特征匹配算法来识别和跟踪这些特征点，它们对应于场景中的真实物体或者场景结构的点。而地图 (Map) 则是由这些地图点构成的三维点云地图，它记录了场景中物体的位置和姿态信息。地图点和地图之间的关系是非常密切的，地图点是构成地图的基本元素，地图则是地图点的集合。在建立地图的过程中，ORB-SLAM3算法通过将每个图像帧与前面的关键帧匹配，然后通过三角化算法将匹配到的ORB特征点转化为地图点，从而逐渐构建出三维地图。地图点的数量和分布对地图的质量和精度有着重要的影响。

- 地图册 (Atlas)

地图册 (Atlas) 内包含了多个地图，系统内会始终存在一个活跃的地图和一些不活跃的地图，地图之间是不连通的，而每个地图描述了一个联通的局部区域，并包含了该区域的关键帧、地图点以及它们之间的约束信息。跟踪线程不断估计到来的关键帧位姿，并将关键帧插入到当前活跃的地图，并持续优化关键帧的位姿，局部建图线程会持续扩充当前活跃地图。系统还构建了一个唯一的 DBoW2 关键帧数据库，用于重定位、回环检测和地图合并。

Atlas的主要作用是管理和优化大规模地图，因为ORB-SLAM3可以将地图分解为多个局部地图，并在这些局部地图之间建立约束关系，从而实现了地图的高效管理和优化。在定位时，ORB-SLAM3可以利用Atlas中的约束信息来提高定位的精度和鲁棒性。

- 共视图 (Covisibility Graph)

共视图 (Covisibility Graph) 表达了关键帧之间的关系。共视图中的节点为关键帧，如果两个节点（关键帧）之间有足够的共视关系，即他们能够同时观测到一些相同的地图点，则这两个节点之间存在一条边，边的权重为两个关键帧中共同观测到的地图点数量。在局部地图优化中，ORB-SLAM3 选择与当前关键帧具有很强共视关系的关键帧进行优化；在回环检测中，也会利用与当前关键帧具有很强共视关系的关键帧来确定是否存在回环。

- 生成树 (Spanning Tree)

在图论中，一个连通图的生成树是该图的一个极小连通子图，它包含图中所有的 n 个节点，但只构成一棵树的 n-1 条边。ORB-SLAM3 中的生成树 (Spanning Tree) 便是共视图的一个生成树。生成树的目的是将共视图中所有的关键帧连接成一颗树，以减少地图优化和回环检测的计算复杂度。由于共视图中的所有关键帧之间都有一定程度的视觉重叠，因此直接使用共视图进行地图优化和回环检测会导致大量冗余计算。而生成树则可以使计算集中在树形结构中，避免冗余计算。

- 本质图 (Essential Graph)

本质图 (Essential Graph) 的概念在 ORB-SLAM3 中被一笔带过，我们可以通过最初版本的 ORB-SLAM 中找到相关的介绍：

> In order to correct a loop we perform a pose graph optimization that distributes the loop closing error along the graph. In order not to include all the edges provided by the covisibility graph, which can be very dense, we propose to build an Essential Graph that retains all the nodes (keyframes), but less edges, still preserving a strong network that yields accurate results. The system builds incrementally a spanning tree from the initial keyframe, which provides a connected subgraph of the covisibility graph with minimal number of edges. When a new keyframe is inserted, it is included in the tree linked to the keyframe which shares most point observations, and when a keyframe is erased by the culling policy, the system updates the links affected by that keyframe. The Essential Graph contains the spanning tree, the subset of edges from the covisibility graph with high covisibility (min = 100), and the loop closure edges, resulting in a strong network of cameras.

也就是说，由于共视图的边太过稠密，给优化带来了不小的麻烦，而本质图的节点和共视图保持一致，但是拥有更少的边，并且在本质图上进行优化也能够提供足够精确的结果。新的关键帧会在生成树中被链接到与它拥有最多共视关系的关键帧上，而当该关键帧被删除时，也会在生成树中删掉与该关键帧的链接关系。本质图中的边保留了共视图中权重（共视关系）大于一定阈值的边，并在此基础上加入了回环边。

下图给出了关键帧、共视图、生成树和本质图之间的关系。可以看出，共视图最稠密，本质图次之，生成树最稀疏。


<div class="row mt-3">
    <div class="col-sm mt-3 mt-md-0">
        {% include figure.html path="assets/img/posts/orbslam3-all-in-one/fig2.png" class="img-fluid rounded z-depth-1" zoomable=true %}
    </div>
</div>
<div class="caption">
    Figure 2
</div>


### 跟踪

<center>
{% mermaid %}
---
title: 跟踪线程
---
flowchart TB
    初始化 --> 跟踪 --> 记录位姿信息
    subgraph 跟踪
        direction TB
        恒速模型跟踪 --> 局部地图跟踪
        参考关键帧跟踪 --> 局部地图跟踪
        局部地图跟踪 --> 判断当前帧是否为关键帧 -- 是 --> 插入关键帧
    end
{% endmermaid %}
</center>


如上图所示跟踪线程大体上包括三个主要部分：
1. 初始化
2. 跟踪：跟踪又主要包括从上一（关键）帧到当前帧的位姿跟踪，和当前帧与局部地图之间的跟踪
3. 记录位姿信息：跟踪并优化得到当前帧的位姿之后，记录下来

上面流程图的重要模块的主要内容和流程分别如下所示：

- 初始化

<center>
{% mermaid %}
---
title: 初始化
---
flowchart TB
    last_frame[[第一帧]]
    frame[[第二帧]]

    frame --> IMU序列
    last_frame --> IMU序列
    IMU序列 --> 预积分

    last_frame -- ORB特征提取 --> last_orb[第一帧ORB特征]
    frame -- ORB特征提取 --> cur_orb[第二帧ORB特征]

    last_orb --> match[特征匹配]
    cur_orb --> match[特征匹配]
    match --> corr[初始对应关系]

    corr --> 单应矩阵 --> 模型选择
    corr --> 基础矩阵 --> 模型选择

    模型选择 --> 计算位姿 --> 特征点三角化 --> BA优化
{% endmermaid %}
</center>

- 恒速模型跟踪

<center>
{% mermaid %}
---
title: 恒速模型跟踪
---
flowchart TB
    更新上一帧位姿 --> 上一帧位姿 --> 相对位姿
    
    subgraph 当前帧初始位姿估计
        direction TB
        IMU是否完成初始化 -- 是 --> IMU运动模型 --> 当前帧初始位姿
        IMU是否完成初始化 -- 否 --> 恒速模型 --> 当前帧初始位姿
    end

    subgraph 特征匹配
        direction TB
        上一帧特征点 --> 上一帧特征点在当前帧的匹配
        当前帧初始位姿 --> 相对位姿 --> 上一帧特征点在当前帧的匹配
    end
    
    上一帧特征点在当前帧的匹配 -- 2D-3D --> BA优化
    BA优化 --> 剔除当前帧观测地图点中外点 --> 合法地图点个数大于10

    合法地图点个数大于10 --> 是 --> 跟踪成功
    合法地图点个数大于10 --> 否 --> 跟踪失败
{% endmermaid %}
</center>

- 参考关键帧跟踪

<center>
{% mermaid %}
---
title: 参考关键帧跟踪
---
flowchart TB
    subgraph BoW描述子提取
        direction TB
        参考关键帧 --> 参考关键帧BoW
        当前帧 --> 当前帧BoW

        参考关键帧BoW --> 特征点匹配
        当前帧BoW --> 特征点匹配
    end


    subgraph 初始位姿设置
        direction TB
        上一帧位姿 --> 当前帧位姿
    end

    参考关键帧位姿 --> 相对位姿
    当前帧位姿 --> 相对位姿

    特征点匹配 --> BA优化
    相对位姿 --> BA优化

    BA优化 --> 剔除当前帧观测地图点中外点 --> 合法地图点个数大于10

    合法地图点个数大于10 --> 是 --> 跟踪成功
    合法地图点个数大于10 --> 否 --> 跟踪失败
{% endmermaid %}
</center>

- 局部地图跟踪

<center>
{% mermaid %}
---
title: 局部地图跟踪
---
flowchart TB
    更新局部地图 --> 更新局部关键帧和局部地图点 --> 局部地图点投影到当前帧 --> IMU是否已经初始化
    IMU是否已经初始化 -- 否 --> 仅优化位姿 --> 更新当前帧地图点被观测关系
    IMU是否已经初始化 -- 是 --> 视觉和惯性联合优化 --> 更新当前帧地图点被观测关系

    更新当前帧地图点被观测关系 --> 剔除外点 --> 统计跟踪匹配数目 --> 判断是否跟踪成功
{% endmermaid %}
</center>

- 判断当前帧是否为关键帧


<center>
{% mermaid %}
---
title: 判断当前帧是否为关键帧
---
flowchart TB
    A{使用IMU\n并且当前地图的\nIMU还未初始化} 
    A -- 是 --> B{当前帧距离\n上一帧\n超过0.25秒} -- 是 --> C((关键帧))
    B -- 否 --> D((非关键帧))
    A -- 否 --> E{跟踪模式} -- 是 --> D((非关键帧))
    E -- 否 --> F{局部建图是否\n被回环检测停止} -- 是 --> D((非关键帧))
    F -- 否 --> G{距上次重定位\n很近并且当前地图\n的关键帧足够多} -- 是 --> D((非关键帧))
    G -- 否 --> H[计算条件]

    H --> Ha[A. \n距离上一关键帧\n已经过去了\n MaxFrames 帧] --> C((关键帧))
    H --> Hb[B. \n距离上一关键帧\n已经过去了\n MinFrames 帧\n并且\n局部建图空闲] --> C((关键帧))
    H --> Hc[C. \n跟踪比较弱] --> C((关键帧))
    H --> Hd[D. \n同参考关键帧\n相比跟踪到\n很少地图点] --> C((关键帧))
{% endmermaid %}
</center>

### 局部建图

- 插入新关键帧

- 地图点更新

- 局部 BA

- IMU 初始化

- 局部关键帧剔除

- IMU 尺度优化

### 回环检测与地图合并

- 回环检测
    - 数据库查询
    - 计算相对位姿
    - 本质图优化
    - 回环

- 地图合并
    - 本质图优化
    - BA
    - 合并地图

### 全局优化

**Deep Learning for 3D Point Clouds A Survey-3D Object Detection**

------

# 数据集

## 室内

- [11]A.  Dai,  A.  X.  Chang,  M.  Savva,  M.  Halber,  T.  Funkhouser,  andM.  Nießner,  “ScanNet:  Richly-annotated  3D  reconstructions  ofindoor scenes,” inCVPR, 2017
- [25]S.  Song,  S.  P.  Lichtenberg,  and  J.  Xiao,  “Sun  RGB-D:  A  RGB-Dscene understanding benchmark suite,” inCVPR, 2015

## 室外

- [14]A.  Geiger,  P.  Lenz,  and  R.  Urtasun,  “Are  we  ready  for  au-tonomous driving,” inCVPR, 2012
- [28]R.  Kesten,  M.  Usman,  J.  Houston,  T.  Pandya,  K.  Nadhamuni,A.  Ferreira,  M.  Yuan,  B.  Low,  A.  Jain,  P.  Ondruskaetal.,  “Lyftlevel 5 av dataset 2019,” 2019.
- [30]P.  Sun,  H.  Kretzschmar,  X.  Dotiwalla,  A.  Chouard,  V.  Patnaik,P. Tsui, J. Guo, Y. Zhou, Y. Chai, B. Caine, V. Vasudevan, W. Han,J. Ngiam, H. Zhao, A. Timofeev, S. Ettinger, M. Krivokon, A. Gao,A. Joshi, Y. Zhang, J. Shlens, Z. Chen, and D. Anguelov, “Scalabil-ity in perception for autonomous driving: Waymo open dataset,”inCVPR, 2020
- [31]H.  Caesar,  V.  Bankiti,  A.  H.  Lang,  S.  Vora,  V.  E.  Liong,  Q.  Xu,A.  Krishnan,  Y.  Pan,  G.  Baldan,  and  O.  Beijbom,  “nuscenes:  Amultimodal dataset for autonomous driving,” inCVPR, 2020

------

| Name and Refence | Year | #Scenes | Classes | Annotated Frames | 3D Boxes |  Secne Type   |  Sensors   |
| :--------------: | ---- | :-----: | :-----: | :--------------: | :------: | :-----------: | :--------: |
|    KITTI[14]     | 2012 |   22    |    8    |       15K        |   200k   | Urban (Drivin | RGB & LiDA |
|  SUN RGB-D[25]   | 2015 |   47    |   37    |        5K        |   65k    |    Indoor     |   RGB-D    |
|  ScanNetV2[11]   | 2018 |  1.5K   |   18    |        -         |    -     |    Indoor     | RGB & Mesh |
|     H3D[26]      | 2019 |   160   |    8    |       27K        |   1.1m   | Urban (Drivin | RGB & LiDA |
|  Argoverse[27]   | 2019 |   113   |   15    |       44k        |   993k   | Urban (Drivin | RGB & LiDA |
|   Lyft L5[28]    | 2019 |   366   |    9    |       46k        |   1.3m   | Urban (Drivin | RGB & LiDA |
|     A*3D[29]     | 2019 |    -    |    7    |       39k        |   230m   | Urban (Drivin | RGB & LiDA |
|  Waymo Open[30]  | 2020 |   1K    |    4    |       200k       |   12m    | Urban (Drivin | RGB & LiDA |
|   nuScenes[31]   | 2020 |   1K    |   23    |       40k        |   1.4m   | Urban (Drivin | RGB & LiDA |



# 评价指标

- Average Precision AP 平均精度：

It is calculated as thearea under the precision-recall curve.

它被计算为精确调用曲线下的面积。

- Precision and Success 精准度和成功率

used to evaluate the overall performance of a 3D single object tracker 单对象跟踪器的整体性能

- Average Multi-Object Tracking Accuracy (AMOTA) and Average Multi-Object Tracking Precision(AMOTP) 

used criteria for the evaluation of 3D multi-object tracking

多对象跟踪器的整体性能





# 现存方法

## 基于区域提议的方法 region proposal based

### 基于多视图 multi-view based

- 对小目标具有高召回率：

[126]  J.  Ku,  M.  Mozifian,  J.  Lee,  A.  Harakeh,  and  S.  L.  Waslander,“Joint  3D  proposal  generation  and  object  detection  fro

- 连续卷积实现图像与3D LiDAR特征图在不同分辨率的融合

[127]  M. Liang, B. Yang, S. Wang, and R. Urtasun, “Deep continuousfusion for multi-sensor 3D object detection,” inECCV, 2018

- 多任务多传感器3D目标检测网络

[128]  M. Liang, B. Yang, Y. Chen, R. Hu, and R. Urtasun, “Multi-taskmulti-sensor fusion for 3D object detection,” inCVPR, 2019

在2D,3D和BEV检测任务上优于

[129]  B.  Yang,  W.  Luo,  and  R.  Urtasun,  “PIXOR:  Real-time  3D  objectdetection from point clouds,” inCVPR, 2018.

[130]  W.  Luo,  B.  Yang,  and  R.  Urtasun,  “Fast  and  furious:  Real  timeend-to-end 3D detection, tracking and motion forecasting with asingle convolutional net,” inCVPR, 2018

- 引入空间通道信息 SCA 探索多尺度上下文信息，还提出了扩展空间非采样（ESU）模块，以通过组合多尺度的低级特征获得具有丰富空间信息的高级特征

[39]H. Lu, X. Chen, G. Zhang, Q. Zhou, Y. Ma, and Y. Zhao, “SCANet:Spatial-channel  attention  network  for  3D  object  detection,”  inICASSP, 2019

- [131]预RoI池卷积来提高[4]的效率，将大多数卷积运算移至RoIpooling模块之前

[131]  Y.  Zeng,  Y.  Hu,  S.  Liu,  J.  Ye,  Y.  Han,  X.  Li,  and  N.  Sun,“RT3D:  Real-time  3D  vehicle  detection  in  lidar  point  cloud  forautonomous driving,”IEEERAL, 2018

[4]X. Chen, H. Ma, J. Wan, B. Li, and T. Xia, “Multi-view 3D objectdetection network for autonomous driving,” inCVPR, 2017

### 基于分段 segmentation based

- 2D分割网络来预测前景像素，并将其投影到点云中以去除大多数背景点，在预测的前景点上生成提案，并设计了一个名为PointsIoU的新准则，以减少提案的冗余性和歧义性

[132]  Z.  Yang,  Y.  Sun,  S.  Liu,  X.  Shen,  and  J.  Jia,  “IPOD:  Intensivepoint-based  object   detector  for   point  cloud,”arXivpreprintarXiv:1812.05276, 201

- 直接分割三维点云以获得前景点，然后融合语义特征和局部空间特征产生高质量3d框

[133]  S.  Shi,  X.  Wang,  and  H.  Li,  “PointRCNN:  3D  object  proposalgeneration and detection from point cloud,” inCVPR, 2019

- 利用GraphConvolution网络（GCN）进行3D对象检测

[134]  Z.  Jesus,  G.  Silvio,  and  G.  Bernard,  “PointRGCN:  Graph  con-volution  networks  for  3D  vehicles  detection  refinement,”arXivpreprintarXiv:1911.12236, 201

- 将点云投影到基于图像的分段网络的输出中，并将语义预测分数附加到这些点上

[135]  V. Sourabh, L. Alex H., H. Bassam, and B. Oscar, “PointPainting:Sequential fusion for 3D object detection,” inCVPR, 2020.

- 将每个点与球形锚点关联。 然后使用每个点的语义评分来删除多余的锚点

[138]  Z. Yang, Y. Sun, S. Liu, X. Shen, and J. Jia, “STD: Sparse-to-dense3D object detector for point cloud,” inICCV, 2019

### 基于视锥 frustum based

- 学习每个3D视锥的点云特征，以进行amodal三维盒体估计

[139]  C.  R.  Qi,  W.  Liu,  C.  Wu,  H.  Su,  and  L.  J.  Guibas,  “FrustumPointNets for 3D object detection from RGB-D data,” inCVPR,2018.

- 提出一个点集模型来预测一组比例因子，并进一步用于自适应地突出有用的特征和抑制信息较少的特征。

[140]  X. Zhao, Z. Liu, R. Hu, and K. Huang, “3D object detection usingscale  invariant  and  feature  reweighting  networks,”  inAAAI,2019.

- 利用2D图像区域及其对应的平截头体点来精确地回归3Dbox

[142] D.  Xu,  D.  Anguelov,  and  A.  Jain,  “PointFusion:  Deep  sensorfusion for 3D bounding box estimation,” inCVPR, 2018

- 首先从2D图像中估计对象的2D边界框和3D姿态，然后提取多个几何上可行的候选对象

[143] K.  Shin,  Y.  P.  Kwon,  and  M.  Tomizuka,  “RoarNet:  A  robust  3Dobject detection based on region approximation refinement,” inIEEEIV, 2019.

- 沿着视锥轴为每个2D区域生成了一系列视锥，并应用PointNet [5]为每个视锥提取特征

[144]  Z. Wang and K. Jia, “Frustum convNet: Sliding frustums to ag-gregate local point-wise features for amodal 3D object detection,”inIROS, 2019

- 在BEV图上获得了初步的检测结果，然后根据BEV预测提取了小点子集，应用局部优化网络来学习补丁的局部特征

[145]  L. Johannes, M. Andreas, A. Thomas, H. Markus, N. Bernhard,and H. Sepp, “Patch refinement - localized 3D object detection,”arXivpreprintarXiv:1910.04093, 20

### 其他方法

- [146]将两个3D旋转边界框的IoU集成到几个最新的检测器中[133]，[137]，[158]，以实现一致的性能改进。  

 D. Zhou, J. Fang, X. Song, C. Guan, J. Yin, Y. Dai, and R. Yang,“Iou loss for 2D/3D object detection,” in3DV, 2019

- [147]提出了一种两阶段网络架构，以同时使用点云和体素表示

 Y. Chen, S. Liu, X. Shen, and J. Jia, “Fast point r-cnn,” inICCV,2019.

- [148]提出了PointVoxel-RCNN（PV-RCNN）来利用3D卷积网络和基于PointNet的集合抽象来学习点云特征

 S.  Shi,  C.  Guo,  L.  Jiang,  Z.  Wang,  J.  Shi,  X.  Wang,  and  H.  Li,“PV-RCNN:  Point-voxel  feature  set  abstraction  for  3D  objectdetection,” inCVPR, 2020.

- [124]提出了VoteNet直接投票支持点云中对象的虚拟中心点，并通过汇总投票特征来生成一组高质量的3D对象建议

  C. R. Qi, O. Litany, K. He, and L. J. Guibas, “Deep hough votingfor 3D object detection in point clouds,”ICCV, 2019.

- [149]增加了方向矢量的辅助分支，以提高虚拟中心点和3D候选框的预测精度

 M. Feng, S. Z. Gilani, Y. Wang, L. Zhang, and A. Mian, “Relationgraph  network  for  3D  object  detection  in  point  clouds,”arXivpreprintarXiv:1912.00202, 2019.

- [150]通过将2D对象检测线索（例如，几何和语义/纹理线索）融合到3D投票管道中，提出了一种ImVoteNet检测器

 C.  R.  Qi,  X.  Chen,  O.  Litany,  and  L.  J.  Guibas,  “ImVoteNet:Boosting 3D object detection in point clouds with image votes,”inCVPR, 2020

- [151]提出了Part-A2Net，它由一个部分感知阶段和一个部分聚集阶段组成

 S.  Shi,  Z.  Wang,  X.  Wang,  and  H.  Li,  “From  points  to  parts:3D object detection from point cloud with part-aware and part-aggregation network,”TPAMI, 2020

## 单次检测方法 single shot

### 基于BEV BEV-based

- 离散化具有相等间隔的单元格的场景的点云，并以类似的方式对反射率进行编码，从而产生规律的表示。 然后应用全卷积网络（FCN）来估计物体的位置和航向角

[129] B.  Yang,  W.  Luo,  and  R.  Urtasun,  “PIXOR:  Real-time  3D  objectdetection from point clouds,” inCVPR, 2018

- 利用高清（HD）映射提供的几何和语义先验信息来提高[129]的鲁棒性和检测性能

[152]  B. Yang, M. Liang, and R. Urtasun, “HDNET: Exploiting hd mapsfor 3D object detection,” inCoRL, 2018.

- 提出了归一化图来考虑不同LiDAR传感器之间的差异

[153] J. Beltr ́an, C. Guindel, F. M. Moreno, D. Cruzado, F. Garc ́ıa, andA.  De  La  Escalera,  “BirdNet:  a  3D  object  detection  frameworkfrom lidar information,” inITSC, 2018.

### 基于离散化 discretization-based

- [154]提出了使用FCN进行3D对象检测

 B. Li, T. Zhang, and T. Xia, “Vehicle detection from 3D lidar usingfully  convolutional  network,”arXivpreprintarXiv:1608.07916,2016.

- [155]将点云离散为具有长度，宽度，高度和通道尺寸的4D张量，并将基于2 FCN的检测技术扩展到3D域以进行3D对象检测

  B.  Li,  “3D  fully  convolutional  network  for  vehicle  detection  inpoint cloud,” inIROS, 2017

- [156]利用一个以特征为中心的投票方案为每个非空体生成一组投票，并通过累积投票获得卷积结果。

 M.  Engelcke,  D.  Rao,  D.  Z.  Wang,  C.  H.  Tong,  and  I.  Posner,“Vote3Deep: Fast object detection in 3D point clouds using effi-cient convolutional neural networks,” inICRA, 2017

- [157]通过堆叠多个稀疏3DCNN构造了3D骨干网络,此方法旨在通过充分利用体素的稀疏性来节省内存并加速计算

 X. Li, J. E. Guivant, N. Kwok, and Y. Xu, “3D backbone networkfor 3D object detection,” inCoRR, 2019.

- [136]提出了一个基于体素的端到端可训练框架VoxelNet。 他们将点云划分为等距的体素，并将每个体素中的特征编码为4D张量

  Y. Zhou and O. Tuzel, “VoxelNet: End-to-end learning for pointcloud based 3D object detection,” inCVPR, 2018

- [158]使用稀疏卷积网络[166]来提高[136]的推理效率。

  Y. Yan, Y. Mao, and B. Li, “SECOND: Sparsely embedded convo-lutional detection,”Sensors, 2018

- [159]通过在早期融合图像和点云功能扩展了VoxelNet

 V.  A.  Sindagi,  Y.  Zhou,  and  O.  Tuzel,  “MVX-Net:  Multimodalvoxelnet for 3D object detection,” inICRA, 2019

- [137]提出了一种名为PointPillars的3D对象检测器

 A.  H.  Lang,  S.  Vora,  H.  Caesar,  L.  Zhou,  J.  Yang,  and  O.  Bei-jbom, “PointPillars: Fast encoders for object detection from pointclouds,” inCVPR, 2019.

- [160]提出了一种SA-SSD检测器，以利用细粒度的结构信息来提高定位精度

 C.  He,  H.  Zeng,  J.  Huang,  X.-S.  Hua,  and  L.  Zhang,  “Structureaware  single-stage  3D  object  detection  from  point  cloud,”  inCVPR, 2020.

### 基于点 point-based

- 3DSSD [161]是朝着这个方向的开拓性工作。 它介绍了一种用于距离FPS（D-FPS）和Feature-FPS（F-FPS）的融合采样策略，以删除耗时的Feature Propagation（FP）层和[133]中的细化模块

 Z. Yang, Y. Sun, S. Liu, and J. Jia, “3DSSD: Point-based 3D singlestage object detector,” inCVPR, 2020

### 其他方法

- [162]提出了一种称为LaserNet的高效3D对象检测器

  G. P. Meyer, A. Laddha, E. Kee, C. Vallespi-Gonzalez, and C. K.Wellington, “LaserNet: An efficient probabilistic 3D object detec-tor for autonomous driving,”CVPR, 2019.

- [163]然后扩展LaserNet [162]以利用RGB图像（例如50至70米）提供的密集纹理

 G. P. Meyer, J. Charland, D. Hegde, A. Laddha, and C. Vallespi-Gonzalez, “Sensor fusion for joint 3D object detection and seman-tic segmentation,”CVPRW, 2019

- [164]提出了anovelHotspot表示和第一个基于热点的无锚检测器

 Q.  Chen,  L.  Sun,  Z.  Wang,  K.  Jia,  and  A.  Yuille,  “Object  ashotspots: An anchor-free 3D object detection approach via firingof hotspots,”arXivpreprintarXiv:1912.12791, 20

- [125]提出了一种图神经网络Point-GNN来检测来自激光点云的3D对象

  W. Shi and R. Rajkumar, “Point-GNN: Graph neural network for3D object detection in a point cloud,” inCVPR, 2020

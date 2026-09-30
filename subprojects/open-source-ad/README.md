# 子项目：研究开源自动驾驶算法

这个子项目**和主工程分开**，目的是搞懂业界（尤其是国内团队）开源的
端到端自动驾驶和世界模型是怎么做的，看看有没有能反哺主工程的思路。

主工程是"手写每个模块、把原理搞懂"；这里是"读别人的代码、学设计"。

## 为什么单独开一块

主工程（CARLA + ROS 2）和这些开源项目的**数据形态根本不一样**：

| | 我们主工程 | UniAD / VAD / SparseDrive |
|---|---|---|
| 传感器 | 1 路前视 720p + 32/64 线 LiDAR | **6 路环视相机** + LiDAR + 5 路雷达 |
| 数据格式 | 直接吃 CARLA | nuScenes 数据集格式 |
| 坐标系 | ROS 右手系 | nuScenes 自有一套标定 |
| 输出 | 我们自己定义的路径 | 规划轨迹（也要转成 nuScenes 格式评估） |

所以**不能直接把模型搬进主工程**，但可以：
1. 用真数据把 nuScenes 格式吃透，理解 6 路环视是怎么标定和融合的；
2. 读懂 UniAD 的"规划导向"、VAD 的向量化表示、SparseDrive 的稀疏计算；
3. 把学到的设计用在主工程的 M8 ST-Graph 上。

## 数据集：nuScenes

**好消息：`v1.0-mini` 不需要注册就能下**（实测 HTTP 200，4.17 GB，
支持断点续传）。完整版 trainval 才需要注册（约 300 GB）。

| 版本 | 大小 | 场景数 | 说明 |
|---|---|---|---|
| **v1.0-mini** | 4.17 GB | 10 | 够把格式和 6 路环视吃透 |
| v1.0-trainval | ~300 GB | 850 | 训练用，需注册 |

下载（复用主工程的多连接下载脚本）：

```bash
cd /home/tom/Desktop/ROS2
bash setup/par_download.sh \
  https://www.nuscenes.org/data/v1.0-mini.tgz \
  subprojects/open-source-ad/data/v1.0-mini.tgz \
  10
```

解压后会得到 `v1.0-mini/`，里面有 `samples/`（6 路相机 jpg）、
`sweeps/`（LiDAR/雷达点云）、`v1.0-mini/`（标注 json）。

## 6 路环视相机是怎么装的

nuScenes 的 6 个相机（前、前左、前右、后、后左、后右），
水平 FOV 都是 70°，**相邻相机只重叠很小一点**，所以必须做 BEV 融合。
这跟"一路前视打天下"是两种完全不同的技术路线——
这也是为什么 UniAD 这类模型我们接不进来。

## 目录

```
open-source-ad/
├── README.md              本文件
├── data/                  数据集（不入库，见 .gitignore）
├── notes/                 读代码/读论文的笔记
└── scripts/
    ├── fetch_data.sh      下载 nuScenes mini
    └── inspect_nuscenes.py 用 devkit 加载，打印 6 路相机标定和样本
```


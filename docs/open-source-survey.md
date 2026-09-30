# 开源调研：世界模型 / 端到端自动驾驶

调研时间：2026-09-30。目的是看**有没有能直接接进我们这个 CARLA + ROS 2 工程的
开源东西**，尤其是国内公司的。

## 一、端到端自动驾驶（学术/大厂）

| 项目 | 星数 | 来源 | 是什么 |
|---|---|---|---|
| [OpenDriveLab/UniAD](https://github.com/OpenDriveLab/UniAD) | 4.8k | 上海AI Lab | CVPR'23 最佳论文，**规划导向**的端到端：感知-预测-规划一条链 |
| [OpenDriveLab/End-to-end-Autonomous-Driving](https://github.com/OpenDriveLab/End-to-end-Autonomous-Driving) | 3.7k | 上海AI Lab | T-PAMI 综述，想入门先看这个 |
| [OpenDriveLab/Vista](https://github.com/OpenDriveLab/Vista) | 901 | 上海AI Lab | NeurIPS'24，**驾驶世界模型**，可泛化 |
| [hustvl/VAD](https://github.com/hustvl/VAD) | 1.4k | 华中科大 | 向量化场景表示，比栅格快很多 |
| [hustvl/DiffusionDrive](https://github.com/hustvl/DiffusionDrive) | 1.5k | 华中科大 | CVPR'25 Highlight，扩散模型做实时端到端 |
| [swc-17/SparseDrive](https://github.com/swc-17/SparseDrive) | 1.0k | 地平线系 | 稀疏表示端到端，含 TensorRT 部署 |
| [autonomousvision/transfuser](https://github.com/autonomousvision/transfuser) | 1.6k | 图宾根 | Transformer 多传感器融合 |

## 二、世界模型（生成/仿真）

| 项目 | 星数 | 来源 | 是什么 |
|---|---|---|---|
| [nvidia-cosmos/cosmos-predict2.5](https://github.com/nvidia-cosmos/cosmos-predict2.5) | 1.4k | NVIDIA | 最新的**世界基础模型**，物理感知 |
| [nv-tlabs/Cosmos-Drive-Dreams](https://github.com/nv-tlabs/Cosmos-Drive-Dreams) | 544 | NVIDIA | 用 Cosmos 批量生成驾驶数据 |
| [Tencent-Hunyuan/HunyuanWorld-1.0](https://github.com/Tencent-Hunyuan/HunyuanWorld-1.0) | 2.9k | 腾讯混元 | 文字生成可探索的 3D 世界 |
| [Tencent-Hunyuan/HunyuanWorld-Voyager](https://github.com/Tencent-Hunyuan/HunyuanWorld-Voyager) | 1.6k | 腾讯混元 | 交互式 RGBD 视频生成 |
| [Tencent-Hunyuan/HunyuanWorld-Mirror](https://github.com/Tencent-Hunyuan/HunyuanWorld-Mirror) | 1.2k | 腾讯混元 | 快速 3D 重建 |

## 三、清单类（找论文用）

- [LMD0311/Awesome-World-Model](https://github.com/LMD0311/Awesome-World-Model) 2.3k
- [Thinklab-SJTU/Awesome-LLM4AD](https://github.com/Thinklab-SJTU/Awesome-LLM4AD) 1.9k
- [leofan90/Awesome-World-Models](https://github.com/leofan90/Awesome-World-Models) 2.0k

## 四、我的判断：什么能接、什么不能

### 能直接接的（已经接了）

**CARLA 官方 `PythonAPI/carla/agents`** —— 这个才是性价比之王，1995 行，
`GlobalRoutePlanner` / `LocalPlanner` / `BehaviorAgent` / `VehiclePIDController`
全都有，自带避障和红绿灯。我们已经在用（`behavior_driver.py`）。

### 看着很香但接不进来的

**UniAD / VAD / SparseDrive 这类端到端模型**，原因有三：
1. 它们吃的是 **nuScenes 数据格式**：6 路环视相机（固定内外参）+ 激光雷达，
   我们只有一个前视 720p；
2. 都是**训练好的权重绑定特定数据集**，换到 CARLA 上要么重新训，
   要么输出完全不可信；
3. 算力上，UniAD 推理要几十毫秒起，而且依赖一堆自定义 CUDA 算子，
   在当前这套 conda 环境里编译风险很高。

结论：**它们的思路可以学，模型搬不过来**。真正值得学的是：
- UniAD 的"规划导向"——感知模块的梯度由规划损失反传，而不是各自为战；
- VAD 的向量化场景表示——不用栅格图，直接操作矢量的车道线/障碍物；
- SparseDrive 的稀疏表示——只算有用的地方。

这三条思路**不依赖它们的权重**，可以直接影响我们的 M8 ST-Graph 设计。

### 世界模型：值得单独做一条支线

Cosmos / HunyuanWorld 这类，价值不在"开车"，而在**造数据**：
- 我们现在要雨天/白天/黄昏，是改 CARLA 天气参数（已实现）；
- 但想造"夜雨 + 逆光 + 行人突然横穿"这种**长尾场景**，
  世界模型能按提示词生成，这是 CARLA 参数调不出来的。

对当前阶段来说这是**支线**，等主线（M8）做完可以试：
用 Cosmos-Drive-Dreams 生成一批极端天气图，用来测我们的检测在域外数据上的表现。

## 五、结论

**这个项目最该"复用"的从来不是某个大模型，而是 CARLA 官方 agents。**
端到端大模型是"另一条技术路线"，和"手写每个模块、把原理搞懂"的定位冲突，
硬接进来只会变成一个看不懂的黑盒。

所以路线不变：**继续手写 M8 ST-Graph**，但把 UniAD/VAD/SparseDrive 的
设计思想吸收进来（规划导向、向量化表示、稀疏计算）。


# 论文笔记：端到端自动驾驶，哪些思路能用在我们 M8 上

论文在 `papers/`（6 篇，PDF + 首页 txt），都是 arXiv 公开版。

## 清单

| 论文 | arXiv | 一句话 |
|---|---|---|
| UniAD | 2212.10156 | CVPR'23 最佳论文，**规划导向** |
| VAD | 2303.12077 | 向量化场景表示，比栅格快且更安全 |
| SparseDrive | 2405.19620 | 稀疏表示 + **多模态规划 + 碰撞重打分** |
| DiffusionDrive | 2411.15139 | 截断扩散 + 多模态锚点 |
| Vista | 2405.17398 | 驾驶世界模型 |
| Survey (T-PAMI) | 2306.16927 | 270+ 篇综述，列了五大挑战 |

---

## 一、能直接改我们 M8 的三条

### 1. 规划是多模态的，不是一条轨迹（SparseDrive）

> "models planning as a **multi-modal** problem… a hierarchical planning
> selection strategy, which incorporates a **collision-aware rescore module**,
> to select a rational and safe trajectory"

**我们现在的做法**：ST-Graph DP 只输出**一条**最优速度曲线。

**问题**：DP 的代价函数是手调的权重，换个场景就不一定对。而且"绕/让/超"
本来就是三种合理选择，硬压成一条曲线会把多模态信息丢光。

**改法**：一次生成 **K 条候选**（比如"保持速度通过 / 减速让行 / 加速抢过 /
靠边停车"），每条算完再统一打分：

```
总代价 = 跟踪期望速度的代价 + 加速度代价 + 碰撞风险代价(重加权)
```

关键是**碰撞风险要单独重打分**（这就是 collision-aware rescore）——
不能混在优化目标里被速度项稀释掉。选总代价最小且无碰撞的那条。

### 2. 障碍物要用"向量"，不是点（VAD）

> "exploits the vectorized agent motion and map elements as explicit
> **instance-level planning constraints**"
> "vectorized map provides road structure (traffic flow, drivable boundary,
> lane direction), and helps the autonomous vehicle narrow down the…"

**我们现在的做法**：障碍物在 ST 图里被当成一个**点**，加固定的安全距离。

**问题**：障碍物有长度，而且在动。一个 4.5 m 长的车以 8 m/s 前进，
在 ST 图上留下的是**一条斜的带子**，不是点。当成点会误判。

**改法**：每个障碍物在 ST 平面上投影成矩形：
```
s 方向占据 [s0 - L/2, s0 + L/2] + 安全余量
t 方向从 0 延伸到时域末端，且中心线斜率 = 障碍物速度
```
即 `s_center(t) = s0 + v_obs·t`，带宽 `L_obs + L_ego + margin`。
这样"前面有辆慢车"和"前面有堵墙"在图上就是两种不同的形状。

### 3. 地图用来缩小搜索空间（VAD）

> "vectorized map … helps the autonomous vehicle **narrow down the** [search space]"

**我们的对应**：路线文件里的车道中心线 + 车道宽度。
横向选择不需要在整条马路上搜，只在"当前车道 ± 一条相邻车道"里选。
这能把候选轨迹的数量压下来。

---

## 二、暂时用不上的（但要知道）

### UniAD 的"规划导向"

所有感知模块的梯度都由规划的损失反传——听起来很美，但**前提是整条链可微**。
我们的感知是 LiDAR 聚类 + YOLO，都不是可微模块，硬套不了。

**能借的**：思想上——**评估感知质量的标准应该是"规划用起来好不好"**，
而不是 mAP。所以我们的 `obstacle_detector` 的调参目标应该从
"检出率高"改成"规划不因为漏检而撞"。

### DiffusionDrive 的截断扩散

用扩散模型生成多模态轨迹，靠"预置锚点 + 截断去噪"做到实时。
多模态这一点和 SparseDrive 一致，但它需要 GPU 上跑神经网络，
和我们的"手写、可解释"路线冲突。**用它的多模态思想就够了**，
不需要真的上扩散模型。

### Vista / Cosmos 这类世界模型

价值在**造数据**（生成 CARLA 参数调不出来的长尾场景），
不在开车。属于支线，主线做完再说。

### Survey 里点名的五大挑战

多模态 / 可解释性 / **因果混淆(causal confusion)** / 鲁棒性 / 世界模型。

其中**因果混淆**值得警惕：模型可能"因为打了转向灯所以左转"而不是
"因为路口所以要左转"——学到的是相关不是因果。我们的手写栈天然规避了
这个问题（每一步的物理含义都是明确的），这是手写路线的一个隐性好​​处。

---

## 三、落地计划

按优先级：

1. ✅ **障碍物从"点"改成"斜带"**（VAD）——直接改 `st_planner_node._blocked()`
2. ✅ **多模态候选 + 碰撞感知重打分**（SparseDrive）——`plan()` 输出 K 条再选
3. ⬜ 用车道宽度限制横向候选（VAD）
4. ⬜ 把"检出率"指标换成"规划安全性"指标（UniAD 的启发）


# TurtleBot3 Nav2 Demo

## 0 功能总览
- 基于 TurtleBot3 + Nav2 的完整导航示例包  
- 一键启动 Gazebo 仿真、导航栈、预配置 RViz  
- 已内置“地图漂移”减缓参数与 `/map` 显示修复方案  

---

## 1 安装与编译
```bash
# 1. 进入工作空间
cd ~/ros2_ws

# 2. 编译本包（第一次或修改后）
colcon build --packages-select turtlebot3_nav2_demo

# 3. 加载环境（**每个新终端都要执行**）
source install/setup.bash
```

---

## 2 一键启动（3 个终端）
| 终端 | 命令 | 作用 |
| ---- | ---- | ---- |
| T1 | `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` | 启动 Gazebo 仿真世界 |
| T2 | `ros2 launch turtlebot3_nav2_demo nav.launch.py` | 启动 Nav2（map_server、amcl、controller、planner …） |
| T3 | `ros2 launch turtlebot3_nav2_demo rviz.launch.py` | 打开已配置好的 RViz（含地图、代价层、工具栏） |

> 启动顺序：**Gazebo → Nav2 → RViz**  
> 全部 `source install/setup.bash` 后再执行！

---

## 3 常见问题速查

### 3.1 RViz 里 “No map received” 或 Map 插件全黑
**原因**：map_server 使用 *Transient Local* QoS，只发一次；RViz 默认 *Volatile* 收不到。  
**解决**：
1. 在 RViz 左侧 **Map** 插件 → 底部 **QoS Settings**  
   - History = Keep Last **1**  
   - Reliability = **Reliable**  
   - Durability = **Transient Local**  
2. **删除旧 Map → 重新 Add → 选 Topic `/map`**，地图瞬间变绿。  
3. 立即 **File → Save Config** 覆盖本包 `rviz/nav2_demo.rviz`，下次启动自动生效。

### 3.2 建图时出现漂移、重影、错位
- **线速度 ≤ 0.2 m/s，角速度 ≤ 0.5 rad/s**（已写在参数里）  
- **缓慢、多次覆盖同一路径**，帮助 SLAM 回环闭合  
- **急转弯/急停**会直接拉偏地图，务必平稳推杆。

---

## 4 如何保存自己的 RViz 布局
1. 按上面 QoS 改好后，随意添加/删除插件、调视角。  
2. **File → Save Config As →** 选择  
   `<本包路径>/rviz/nav2_demo.rviz` 覆盖保存。  
3. 重新编译（资源文件不需要 `colcon build`，但为了安装）：
   ```bash
   colcon build --packages-select turtlebot3_nav2_demo
   source install/setup.bash
   ```
4. 以后只跑 `ros2 launch turtlebot3_nav2_demo rviz.launch.py` 即可 **100% 还原**当前布局。

---

## 5 初始位姿微调（可选）
若启动后机器人定位偏差大，在 RViz 用 **2D Pose Estimate** 点一次即可；  
或修改 `config/nav2_params.yaml` 里 amcl 的 `initial_pose.*` 数值后重新 launch。

添加初始位姿（initial pose）的意义一句话就能概括：

> **让 AMCL 在“睁眼”第一秒就知道机器人在地图的哪里、朝哪个方向，从而立刻发布正确的 `map→odom` 变换，避免后续定位漂移、导航失败或地图显示异常。**

------------------------------------------------
展开来说，它解决三大痛点：

1. **TF 链条不断**  
   Nav2 的 global_costmap、map_server 都依赖 `map→odom→base_link` 链条。  
   没有初始位姿 → AMCL 不发布 `map→odom` → 链条断裂 →  
   - map_server 的 `/map` 发不出来（你会看到 “No map received”）  
   - 全局规划器报错 “can’t transform”  
   - RViz 里全局代价地图全黑。

2. **防止粒子云“炸开”**  
   AMCL 默认把粒子均匀撒满整张图。  
   如果开局就告诉你 “我在原点，朝向 X 轴”，粒子只在 ( -2, -0.5, 0 rad ) 附近出生，  
   激光一扫描就能迅速收敛，**不会整段定位飘走**。

3. **节省人工操作**  
   不写初始位姿，每次 launch 后必须 **手动 2D Pose Estimate** 点一下；  
   写进 yaml → **全自动**，CI/远程部署也能一次成功。

------------------------------------------------
一句话记忆  
“初始位姿” = 给 AMCL 一张 **出生证明**，告诉它 “你生在这，别乱猜”。

---

## 6 更多帮助
| 链接 | 内容 |
| ---- | ---- |
| [Nav2 官方文档](https://navigation.ros.org/) | 参数详解、行为树定制 |
| [TurtleBot3 中文网](https://turtlebot3.robotis.com/) | 固件、模型、仿真世界下载 |
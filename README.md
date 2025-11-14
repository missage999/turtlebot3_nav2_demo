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

## 2 启动（5 个终端）
| 终端 | 命令 | 作用 |
| ---- | ---- | ---- |
| T1 | `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` | 启动 Gazebo 仿真世界 |
| T2 | `ros2 launch turtlebot3_nav2_demo spawn_obstacle.launch.py` | 可以跟随仿真世界启动，先确认障碍物是否运行正确，再启动导航栈和其他内容 |
| T3 | `ros2 launch turtlebot3_nav2_demo nav.launch.py` | 启动 Nav2（map_server、amcl、controller、planner …） |
| T4 | `ros2 launch turtlebot3_nav2_demo rviz.launch.py` | 打开已配置好的 RViz（含地图、代价层、工具栏） |
| T5 | `ros2 run turtlebot3_nav2_demo multi_goal_patrol.py` | 运行多目标巡逻脚本 |

> 启动顺序：**Gazebo → 障碍物 → Nav2 → RViz → 巡逻脚本**  
> 全部 `source ~/ros2_ws/install/setup.bash` 后再执行！

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

> **让 AMCL 在“睁眼”第一秒就知道机器人在地图的哪里、朝哪个方向，从而立刻发布正确的 `map→odom` 变换，避免后续定位漂移、导航失败或地图显示异常。**

### 5.1 解决三大痛点

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

> **“初始位姿” = 给 AMCL 一张出生证明，告诉它 “你生在这，别乱猜”。**

---

## 6 更多帮助
| 链接 | 内容 |
| ---- | ---- |
| [Nav2 官方文档](https://navigation.ros.org/) | 参数详解、行为树定制 |
| [TurtleBot3 中文网](https://turtlebot3.robotis.com/) | 固件、模型、仿真世界下载 |

---

## 7 超时重试功能

### 7.1 功能概述
多目标巡逻脚本 (`multi_goal_patrol.py`) 实现了智能的超时重试机制，能够自动处理导航过程中的各种异常情况，包括：
- 目标被拒绝
- 导航超时
- 导航失败
- 通信中断

### 7.2 工作原理
脚本通过以下机制实现超时重试功能：

1. **超时计时器**：为每个导航目标设置超时计时器，默认30秒
2. **重试机制**：导航失败后自动重试，默认最多2次。无论失败和超时都会走超时通道，避免竞争。
3. **重试间隔**：每次重试间隔5秒，避免连续失败
4. **智能处理**：根据失败类型采取不同的处理策略

### 7.3 可配置参数
所有超时重试参数都可以通过ROS参数动态配置：

| 参数名 | 默认值 | 说明 |
| ---- | ---- | ---- |
| `timeout_duration` | 30.0 秒 | 导航超时时间 |
| `max_retry_count` | 2 | 最大重试次数 |
| `retry_interval` | 5.0 秒 | 重试间隔时间 |

### 7.4 使用示例
运行时可以通过命令行参数修改默认配置：
```bash
# 修改超时时间为60秒，最大重试次数为3次，重试间隔为10秒
ros2 run turtlebot3_nav2_demo multi_goal_patrol.py --ros-args -p timeout_duration:=60.0 -p max_retry_count:=3 -p retry_interval:=10.0
```

### 7.5 功能特点
- **自动重试**：无需人工干预，自动处理临时性故障
- **状态监控**：实时监控导航状态，及时发现异常
- **资源管理**：正确管理计时器资源，避免内存泄漏
- **日志记录**：详细记录重试过程，便于问题排查
- **优雅降级**：超过重试次数后自动跳过目标，保证巡逻任务继续进行

---

## 8 移动障碍物系统

移动障碍物系统由四个核心文件组成，协同实现周期性运动的障碍物，用于导航避障测试：

### 8.1 spawn_obstacle.launch.py
**功能**：负责在Gazebo仿真环境中生成移动障碍物模型，并启动运动控制器。
- 设置障碍物初始位置（-2, 0.5, 0）
- 加载障碍物模型文件
- 延迟10秒启动控制器，确保模型完全加载
- 协调模型生成与控制器启动时序

### 8.2 model.sdf
**功能**：定义移动障碍物的物理模型和运动特性。
- 构建红色立方体底盘（0.2×0.2×0.2m）
- 配置左右驱动轮和差速驱动系统
- 设置低摩擦系数（0.001）确保平滑运动
- 定义关节动力学参数和速度控制接口

### 8.3 moving_obstacle_controller.py
**功能**：实现障碍物的周期性往复运动控制逻辑，并且等到模型订阅速度后再启动，保证运动正确。
- 40秒周期（20秒前进+20秒后撤）的循环运动
- 0.2m/s的恒定线速度控制
- 实时发布速度指令到cmd_vel话题
- 异常时自动停止，确保安全

### 8.4 model.config
**功能**：提供模型的元数据描述和版本信息。
- 定义模型名称和版本号
- 指定SDF文件关联
- 包含作者信息和功能描述
- 作为Gazebo模型库的入口文件

### 8.5 演示视频
**移动障碍物避障测试视频**：

[动态避障视频演示](videos/移动障碍物和超时重试的测试.mp4)

该视频展示了：
- 移动障碍物在场景中的周期性往复运动
- TurtleBot3机器人对移动障碍物的检测和避障行为
- 导航系统的实时响应和路径重新规划
- 超时重试机制的实际工作效果

---

## 9 常见问题解决方案

### 9.1 无法找到 moving_obstacle_controller.py文件
**解决**：
请确保 `moving_obstacle_controller.py` 文件位于 `scripts/` 目录下，并且在 `CMakeLists.txt` 中正确安装。

1. 检查文件路径：确认 `moving_obstacle_controller.py` 是否位于 `turtlebot3_nav2_demo/scripts/` 目录下。
2. 检查 CMakeLists.txt：确认 `CMakeLists.txt` 中是否包含以下安装指令：
   ```cmake
   install(PROGRAMS
     scripts/moving_obstacle_controller.py
     DESTINATION lib/${PROJECT_NAME}
   )
   ```
> **注意**：不能安装在相同的scripts文件夹：`lib/${PROJECT_NAME}/scripts/`，而是安装在：`lib/${PROJECT_NAME}/`
> 
> 原因：因为 ros2 run 不会递归搜索子目录，它只看 lib/<pkg>/ 下的直接文件

### 9.2 模型坐标系问题
**model.sdf世界坐标系** (Gazebo全局)
```
└── 模型坐标系 (<model><pose>)
    ├── 底盘坐标系 (<link name="chassis"><pose>)
    ├── 左轮坐标系 (<link name="left_wheel"><pose>)
    ├── 右轮坐标系 (<link name="right_wheel"><pose>)
    └── 万向轮坐标系 (<link name="caster_wheel"><pose>)
```

**关键点**：
- 所有`<link>`的`<pose>`都相对于模型坐标系
- 即使有`<parent>chassis</parent>`，left_wheel的pose仍然相对于模型坐标系，不是相对于chassis
- chassis和left_wheel都有各自独立的坐标系，都相对于模型坐标系定义
- 在启动文件中，已经设置了坐标，如果在model.sdf中再次设立了相同的坐标，就会有2倍关系

**检查model.sdf语法的命令**：
```bash
gz sdf -p model.sdf
```

### 9.3 关节定义错误
**问题**：`[gzserver-1] [ERROR] [1762516486.401337091] [moving_obstacle.diff_drive]: Joint [left_wheel_joint] not found, plugin will not work.`

**原因**：Gazebo的SDF解析器对`<dynamics>`和`<axis>`标签要求必须使用完整嵌套格式。

**错误写法**：
```xml
<dynamics damping="0.1" friction="0.1"/>
```

**正确写法**：
```xml
<dynamics><damping>0.1</damping><friction>0.1</friction></dynamics>
```

### 9.4 时间同步问题
**问题**：速度控制的启动和障碍物运行的初始时间不一致，导致第一个周期运动出现问题。

**原因**：控制器节点在Gazebo模型完全就绪前就已启动计时器，导致时间基准错乱。

**解决**：在Launch文件中增加启动延迟

### 9.5 XML解析错误
**问题**：`Spawn status: Failed to parse XML string`

**原因**：从聊天界面复制粘贴时，中文字符被自动转成了HTML实体！

**解决**：转换成utf-8编码或重新编写。

### 9.6 障碍物运动偏移
**问题**：障碍物运动应该沿着平行x轴的方向周期运动，但是初始位姿和运行路线都有些偏移，偏向东北方向。打开gazebo的view中的contacts查看力线，非常不稳定，快速闪烁。

当前问题：障碍物运动时间有时候会导致运动出错乃至偏移，因此可以设置障碍物在-2,0.5和2,0.5之间运动，也可以延长等待时间。
**原因分析**：
1. 可以转动的万向轮导致运动偏移，改成圆柱支撑
2. 轮子的旋转方向并不精确，导致细微的偏移
3. 轮子和障碍物有接触，导致内部碰撞
4. 障碍物主体离地，以及前后轮的物体摆放和摩擦
5. 轮子底部的接触不稳定

**解决方案**：
- 注意：轮子的旋转方向会影响运动方向，不要一正一负，而且旋转轴为0 0 -1才是朝前运动，右手法则判断。
- 把障碍物和后轮平齐地面
- 把轮子底部做成略微凹进地面，避免接触不良
- **最终方案**：障碍物设成光滑，不用支撑
  - 注意：设置低摩擦系数，和删除摩擦完全不一样，后者会导致卡顿

### 9.7 运动时间控制
**问题**：障碍物运动时间有时候会导致运动出错乃至偏移

**解决**：可以设置障碍物在-2,0.5和2,0.5之间运动，也可以延长等待时间。也可以添加状态确认，等待障碍物模型开始订阅速度后再启动控制器，也是当前的做法。

### 9.8 避障响应慢
**问题**：机器人碰到障碍物时，避障不够迅速

**解决**：可以调整控制器和局部代价地图更新频率、障碍物敏感度、响应速度优化、预测能力增强、速度空间搜索优化、安全距离膨胀。
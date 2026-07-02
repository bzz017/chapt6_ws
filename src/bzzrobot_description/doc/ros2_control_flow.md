
# ROS2 Control 两轮差速控制器运作流程图

## 一、当前方式（Gazebo 原生插件）

```
┌──────────────────────┐     ┌─────────────────────────────────┐
│   teleop_twist_      │     │     libgazebo_ros_diff_drive    │
│   keyboard           │────▶│          .so 插件                │
│   (发布/cmd_vel)     │     │  在 Gazebo 进程内部运行          │
└──────────────────────┘     └─────────────────────────────────┘
                                        │
                                        ▼
┌──────────────────────┐     ┌─────────────────────────────────┐
│    /odom             │◀────│         Gazebo 物理引擎         │
│    (里程计话题)       │     │  - 更新轮子位置/速度            │
│    /tf (odom→base)   │     │  - 计算机器人运动               │
└──────────────────────┘     │  - 检测碰撞                     │
                             └─────────────────────────────────┘
                                        │
                                        ▼
                             ┌─────────────────────────────────┐
                             │     /joint_states (轮子状态)     │
                             └─────────────────────────────────┘
```

**流程说明：**
1. 键盘控制节点发布 `/cmd_vel` 话题
2. Gazebo 插件直接订阅 `/cmd_vel`，计算左右轮速度
3. 插件直接控制 Gazebo 中的关节
4. 插件发布里程计和关节状态

---

## 二、ros2_control 方式（标准化架构）

### 整体架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              ROS2 Control 架构                              │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌──────────────────┐                                                      │
│   │ 1. 命令输入层     │                                                      │
│   │ teleop_twist_    │──────▶ /cmd_vel (geometry_msgs/Twist)                │
│   │ keyboard         │                                                      │
│   └──────────────────┘                                                      │
│                                    │                                        │
│                                    ▼                                        │
│   ┌──────────────────────────────────────────────────────────────────┐      │
│   │ 2. 控制器层 (Controller Layer)                                   │      │
│   │                                                                  │      │
│   │   ┌─────────────────────────────────────────────────────────┐    │      │
│   │   │  diff_drive_controller/DiffDriveController              │    │      │
│   │   │                                                         │    │      │
│   │   │  输入: /cmd_vel (linear.x, angular.z)                   │    │      │
│   │   │                                                         │    │      │
│   │   │  算法:                                                  │    │      │
│   │   │    v_left  = v_linear - (ω * wheel_sep) / 2            │    │      │
│   │   │    v_right = v_linear + (ω * wheel_sep) / 2            │    │      │
│   │   │                                                         │    │      │
│   │   │  输出: joint velocity commands                          │    │      │
│   │   └─────────────────────────────────────────────────────────┘    │      │
│   │                                                                  │      │
│   │   ┌─────────────────────────────────────────────────────────┐    │      │
│   │   │  joint_state_broadcaster                                │    │      │
│   │   │                                                         │    │      │
│   │   │  输入: joint states from hardware                       │    │      │
│   │   │                                                         │    │      │
│   │   │  输出: /joint_states (供 robot_state_publisher 使用)    │    │      │
│   │   └─────────────────────────────────────────────────────────┘    │      │
│   └──────────────────────────────────────────────────────────────────┘      │
│                                    │                                        │
│                                    ▼                                        │
│   ┌──────────────────────────────────────────────────────────────────┐      │
│   │ 3. 控制器管理器 (Controller Manager)                             │      │
│   │                                                                  │      │
│   │   职责:                                                          │      │
│   │   - 加载/启动/停止/卸载控制器                                      │      │
│   │   - 管理控制器生命周期                                             │      │
│   │   - 协调硬件接口读写                                              │      │
│   │                                                                  │      │
│   │   命令行接口:                                                     │      │
│   │   - ros2 control list_controllers                                │      │
│   │   - ros2 control load_controller diff_drive_controller           │      │
│   │   - ros2 control set_controller_state diff_drive_controller active│      │
│   └──────────────────────────────────────────────────────────────────┘      │
│                                    │                                        │
│                                    ▼                                        │
│   ┌──────────────────────────────────────────────────────────────────┐      │
│   │ 4. 硬件接口层 (Hardware Interface)                               │      │
│   │                                                                  │      │
│   │   ┌─────────────────────────────────────────────────────────┐    │      │
│   │   │  gazebo_ros2_control/GazeboSystem                       │    │      │
│   │   │                                                         │    │      │
│   │   │  命令接口 (Command Interface):                           │    │      │
│   │   │    - velocity (left_wheel_joint)                        │    │      │
│   │   │    - velocity (right_wheel_joint)                       │    │      │
│   │   │                                                         │    │      │
│   │   │  状态接口 (State Interface):                             │    │      │
│   │   │    - position (left_wheel_joint)                        │    │      │
│   │   │    - velocity (left_wheel_joint)                        │    │      │
│   │   │    - position (right_wheel_joint)                       │    │      │
│   │   │    - velocity (right_wheel_joint)                       │    │      │
│   │   └─────────────────────────────────────────────────────────┘    │      │
│   └──────────────────────────────────────────────────────────────────┘      │
│                                    │                                        │
│                                    ▼                                        │
│   ┌──────────────────────────────────────────────────────────────────┐      │
│   │ 5. 硬件层 (Hardware Layer)                                      │      │
│   │                                                                  │      │
│   │   ┌─────────────────────────────────────────────────────────┐    │      │
│   │   │              Gazebo 物理引擎                            │    │      │
│   │   │                                                         │    │      │
│   │   │  执行关节命令:                                           │    │      │
│   │   │    - 应用速度控制到轮子关节                               │    │      │
│   │   │    - 计算物理模拟                                         │    │      │
│   │   │    - 返回关节状态 (位置、速度)                             │    │      │
│   │   └─────────────────────────────────────────────────────────┘    │      │
│   └──────────────────────────────────────────────────────────────────┘      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 三、详细数据流（时序图）

```
时间轴 ─────────────────────────────────────────────────────────────────────────▶

teleop_twist      diff_drive      controller      gazebo_ros2      Gazebo
   │                  │              │                │             │
   │──/cmd_vel───────▶│              │                │             │
   │                  │              │                │             │
   │                  │──commands───▶│                │             │
   │                  │              │                │             │
   │                  │              │──write()───────▶│             │
   │                  │              │                │             │
   │                  │              │                │──setJoint──▶│
   │                  │              │                │             │
   │                  │              │                │◀─getJoint───│
   │                  │              │                │             │
   │                  │              │◀─read()────────│             │
   │                  │              │                │             │
   │                  │◀─states──────│                │             │
   │                  │              │                │             │
   │◀──/joint_states──│              │                │             │
   │                  │              │                │             │
   │                  │              │                │             │
```

---

## 四、配置文件结构

```
bzzrobot_description/
├── urdf/
│   ├── bzz2robot/
│   │   ├── bzz002.urdf.xacro
│   │   ├── bzzbot.ros2_control.xacro    # 硬件接口声明
│   │   └── plugins/
│   │       └── gazebo_ros2_control.xacro # Gazebo插件配置
│   └── ...
├── config/
│   └── controller_config.yaml           # 控制器参数
└── launch/
    └── gazebo_sim.launch.py             # 启动文件
```

### 4.1 硬件接口声明 (bzzbot.ros2_control.xacro)

```xml
<ros2_control name="bzz002_hardware" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  
  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### 4.2 控制器配置 (controller_config.yaml)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.3
    wheel_radius: 0.1
    linear.x.max_velocity: 1.0
    angular.z.max_velocity: 1.57
```

---

## 五、两种方式对比

| 特性 | Gazebo原生插件 | ros2_control |
|------|---------------|--------------|
| **架构** | 单层直接控制 | 四层标准化架构 |
| **硬件抽象** | 仅Gazebo | 可切换真实硬件 |
| **控制器扩展** | 固定差速驱动 | 支持多种控制器 |
| **生命周期管理** | 无 | controller_manager |
| **配置复杂度** | 低 | 高 |
| **灵活性** | 低 | 高 |
| **代码复用** | 仿真专用 | 仿真/真实通用 |

---

## 六、关键概念

### 6.1 命令接口 (Command Interface)
- 控制器→硬件：发送控制指令
- 类型：velocity（速度）、position（位置）、effort（力/力矩）

### 6.2 状态接口 (State Interface)
- 硬件→控制器：反馈当前状态
- 类型：position（位置）、velocity（速度）、effort（力/力矩）

### 6.3 控制器管理器 (Controller Manager)
- 统一管理所有控制器
- 处理控制器间的资源竞争
- 提供命令行和服务接口

### 6.4 硬件抽象层 (HAL)
- 屏蔽底层硬件差异
- 仿真和真实硬件使用相同接口
- 支持热插拔

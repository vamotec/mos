# MOS 云网关代理 (mos-cloud-agent)

## 1. 组件定位 (Component Positioning)

本组件是 `mos` 系统中的一个关键后台服务（Agent），作为守护进程在机器人实机上运行。

它的核心职责是充当一个**数据中继**，将机器人本地的 ROS2 可视化数据，安全、高效地传输到 `MOS Cloud`，以支持云端的可视化服务。

## 2. 核心职责 (Core Responsibilities)

- **连接本地数据源:** 连接到本地运行的 `rosbridge_server` 的 WebSocket 服务。
- **订阅核心话题:** 通过 `rosbridge` 订阅对可视化至关重要的核心 ROS2 话题。
- **建立云端隧道:** 主动与 `MOS Cloud` 的可视化接入点建立一个加密的、持久的 WebSocket (WSS) 连接。
- **双向数据中继:**
    - 将从 `rosbridge` 订阅到的 ROS2 话题数据，通过云端隧道发送到 `MOS Cloud`。
    - （未来）将从 `MOS Cloud` 下发的控制指令（如动态调整订阅话题等），转发给本地的 `rosbridge`。
- **状态监控与重连:** 监控与本地 `rosbridge` 和云端的连接状态，在断开时自动进行重连。

## 3. 技术规格 (Technical Specifications)

### 3.1. 本地连接 (Local Connection)

- **目标服务:** `rosbridge_server`
- **协议:** WebSocket (ws)
- **地址:** `ws://127.0.0.1:9090` (默认，应可配置)

### 3.2. 订阅的ROS2话题 (Subscribed ROS2 Topics)

为了实现基础的机器人姿态可视化，本代理默认需要订阅以下话题：

- `/tf`: 机器人的坐标系变换树，用于确定各部件的相对位置。
- `/tf_static`: 静态坐标变换。
- `/robot_description`: URDF 机器人模型描述，用于在前端渲染机器人外观。
- `/joint_states`: 机器人关节状态，用于驱动模型关节运动。

*注：未来应支持通过云端指令动态增减订阅的话题。*

### 3.3. 云端连接 (Cloud Connection)

- **目标服务:** `MOS Cloud` 可视化接入点
- **协议:** Secure WebSocket (wss)
- **地址:** `wss://viz.mos-cloud.com/ingress` (示例地址，应可配置)
- **认证方式:** 连接时需在请求头中携带从 `Cloud Agent` 获取的、由云端颁发的有效JWT（JSON Web Token）。

### 3.4. 数据格式 (Data Format)

- **上行 (机器人 -> 云端):**
    - 代理将从 `rosbridge` 收到的原始 JSON 消息体，封装在一个新的 JSON 结构中，并增加元数据。
    - 示例:
      ```json
      {
        "type": "ros_message",
        "timestamp_ms": 1668888888,
        "payload": {
          // rosbridge 原始消息体
          "op": "publish",
          "topic": "/tf",
          "msg": { ... }
        }
      }
      ```

- **下行 (云端 -> 机器人):**
    - 遵循 `rosbridge` 的标准协议格式，代理直接将云端下发的消息体转发给本地 `rosbridge`。
    - 示例:
      ```json
      {
        "op": "subscribe",
        "topic": "/scan",
        "type": "sensor_msgs/LaserScan"
      }
      ```

# MOS 云核心代理 (mos-cloud-agent)

## 1. 组件定位 (Component Positioning)

本组件是 `mos` 系统中最核心的后台服务之一，负责机器人与 `MOS Cloud` 之间**高可靠、低带宽**的通信。

它是机器人作为“物联网智能终端”的**数字身份和管理控制生命线**。它的稳定运行是保证设备可被远程管理、监控和运维的基石。

## 2. 核心职责 (Core Responsibilities)

- **建立云端连接:** 主动与 `MOS Cloud` 的 MQTT 接入点（Broker）建立一个加密的、持久的 MQTTS 连接。
- **设备认证:** 在连接时，使用设备唯一的客户端证书和私钥进行身份认证。
- **发布心跳:** 定期向云端发布心跳消息，以表明设备在线状态。
- **发布遥测数据:** 定期或在状态变化时，向云端发布设备的关键遥测（Telemetry）数据，如：
    - 电池电量
    - CPU/内存使用率
    - 当前执行的任务状态
    - 机器人位置
- **订阅控制指令:** 订阅云端下发的控制指令话题，并根据指令执行相应操作，如：
    - 触发 OTA (Over-the-Air) 软件更新。
    - 执行紧急停止指令。
    - 接收新的任务或调度。
- **状态监控与重连:** 监控与云端的连接状态，在断开时自动进行指数退避重连。

## 3. 技术规格 (Technical Specifications)

### 3.1. 云端连接 (Cloud Connection)

- **目标服务:** `MOS Cloud` MQTT Broker
- **协议:** MQTTS (MQTT over TLS)
- **地址:** `mqtts://mqtt.mos-cloud.com:8883` (示例地址，应可配置)
- **QoS 等级:**
    - 对于关键指令，应使用 QoS 1 (至少一次)。
    - 对于遥测数据，可使用 QoS 0 (至多一次) 或 QoS 1。

### 3.2. 话题（Topic）结构设计 (示例)

采用层级结构，清晰地划分功能。

- **设备上报 (Publish):**
    - 心跳: `mos/devices/{deviceID}/events/heartbeat`
    - 状态: `mos/devices/{deviceID}/events/status`
    - 日志: `mos/devices/{deviceID}/events/log`

- **设备接收 (Subscribe):**
    - 指令: `mos/devices/{deviceID}/commands/json` (接收所有JSON格式的指令)
    - OTA: `mos/devices/{deviceID}/commands/ota`

*`{deviceID}` 将在设备认证后由云端确认，或使用客户端ID。*

### 3.3. 数据格式 (Data Format)

- **协议:** 所有消息体（Payload）均使用 **JSON** 格式。

- **上报状态示例 (Payload for `.../events/status`):**
  ```json
  {
    "timestamp_ms": 1668889999,
    "battery_percent": 85.5,
    "is_charging": false,
    "current_task_id": "task-123",
    "cpu_load_percent": 25.0
  }
  ```

- **下发指令示例 (Payload for `.../commands/json`):**
  ```json
  {
    "command_id": "cmd-abc-789",
    "action": "E_STOP", // or "REBOOT", "PAUSE_TASK"
    "params": {}
  }
  ```

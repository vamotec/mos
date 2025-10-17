# Skill 是什么？

首先，用户开发的 Skill 不是直接编译进 mos-core
的原生代码。它被编译成一个独立的、可移植的、沙箱化的二进制文件：一个 .wasm 文件。

这么做的好处是：
- 安全： WASM 运行在一个严格的沙箱环境中。一个有 Bug 的 Skill 无法访问它不该访问的内存，也无法搞垮整个 mos-core
  主程序。这对于硬件部署至关重要。
- 便携： 开发者可以用 Rust、C++、Go 等多种语言编写 Skill，只要能编译成 WASM 即可。
- 热更新： 我们可以随时替换或添加新的 .wasm 文件（通过 OTA），而无需重启整个机器人。

  ---

## 1. Skill 存放在哪里？

当一个 Skill 通过 OTA 下载到硬件上时，它就是一个 .wasm 文件。

mos-core 会将这个文件保存在机器人操作系统的文件系统中一个特定的目录下。例如，可能会是：
- `/var/lib/mos/skills/`
- `/opt/mos/skills/`

mos-core 启动时会扫描这个目录，加载所有可用的 Skill 的元信息（名称、版本等）。

  ---

## 2. Skill 如何被执行？

这正是 wasm_runtime.rs 和 ffi.rs 发挥作用的地方。

执行流程如下：

- 1. 请求执行： 外部（比如通过 gRPC 或 Web 请求）向 mos-core 发出一个指令：“执行名为 ‘pick_and_place’ 的 Skill”。

- 2. 加载WASM： mos-core 中的 WasmRuntime (wasm_runtime.rs) 负责从文件系统中读取 pick_and_place.wasm
   文件的二进制内容。

- 3. 创建沙箱实例： WasmRuntime 会创建一个独立的、与主程序隔离的沙箱实例来运行这个 Skill。

- 4. 注入API (FFI)： 这是最关键的一步。Skill 需要控制机器人，但它在沙箱里，无法直接调用 mos-core
   的函数。因此，mos-core 会通过 FFI (Foreign Function Interface)，将一组预先定义好的函数（例如 host_move_to,
   host_get_joint_state）“注入”到沙箱中。这些函数的定义就在 ffi.rs 中。

- 5. 开始执行： mos-core 调用 WASM 模块中一个约定的入口函数，比如 run()。

- 6. Skill 内部逻辑： pick_and_place.wasm 的代码开始运行。当它需要移动机器人时，它会调用它所知道的 host_move_to()
   函数。

- 7. 返回主机： 这个调用会穿过 FFI 边界，回到 mos-core 的 WasmRuntime 中。mos-core 接收到这个调用后，就知道“哦，Skill
   想要移动机器人”，然后它会去调用真正的 RobotController (也就是我们的 gRPC 客户端)，将命令发送给 mos-ros2。

下面是这个流程的示意图：

    1 +-------------------------------- mos-core (主机 Host) --------------------------------+
    2 |                                                                                     |
    3 |  +-----------------+      +---------------------+      +--------------------------+ |
    4 |  | Skill Scheduler |----->|   WasmRuntime       |----->| RobotController (gRPC)   | |
    5 |  +-----------------+      | (from wasm_runtime.rs)|      +--------------------------+ |
    6 |                           +---------------------+                 |                |
    7 |                                     ^                           (to mos-ros2)      |
    8 |                                     | FFI Call (e.g., host_move_to)                |
    9 |                           +---------------------+                                  |
    10 |                          |  Skill.wasm (访客)  |                                  |
    11 |                          |   (in Sandbox)      |                                  |
    12 |                          +---------------------+                                  |
    13 |                                                                                     |
    14 +-------------------------------------------------------------------------------------+

# 总结：

mos-core 像一个“操作系统”，而 Skill 是运行于其上的、受严格管制的“App”。通过 WASM 和
FFI，你的设计实现了一个既安全又灵活的 Skill 执行架构，非常适合硬件部署的场景。
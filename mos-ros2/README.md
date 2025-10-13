# 一、docker ros2 环境准备
每天开始工作前，在根目录mos下运行一次
```bash
./scripts/run_ros2_env
```
前提是docker镜像已经准备好，如果没有，就先准备镜像
```bash
docker build -t mos-ros2-env mos-ros2
```
# 二、docker下编译c++项目
进入docker命令行终端后，先编译项目
```bash
colcon build --packages-select mos_ros2
```
每次编译后，都需要 source 一下新生成的 setup.bash 文件，这样ROS2系统才能找到我们刚刚创建的可执行文件。
```bash
source install/setup.bash
```
然后 使用 ros2 run 命令来启动您的节点
```bash
ros2 run mos_ros2 ros2_grpc_server
```
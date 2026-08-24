# kuavo_back

Kuavo ROS 控制、MPC/WBC、仿真、硬件驱动和开发工具的主工程。

## 从这里开始

- [完整构建、仿真、实机与接口文档](readme.md)
- [Topic 与节点接口](docs/readme_topics.md)
- [安全仿真示例](examples/README.md)
- [变更记录](CHANGELOG.md)

仓库同时包含 MuJoCo/Gazebo/Isaac Sim 与真实机器人入口。首次使用应先在 Docker 仿真中
确认 `ROBOT_VERSION`、总质量、末端执行器配置和控制接口；实机校准、上电、站立、VR
或键盘控制必须由现场操作员按照完整文档执行，并保持急停与扶持条件。

不要把 `~/.config/lejuconfig`、零点、设备凭据、ROS 日志、bag、构建目录或机器专属路径
提交到仓库。仿真可运行不代表模型精度或真机安全已经验证。

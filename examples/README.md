# Kuavo 安全仿真示例

## 1. 只读预检

```bash
test -n "${ROBOT_VERSION:-}"
test -f "${HOME}/.config/lejuconfig/TotalMassV${ROBOT_VERSION}"
git submodule status --recursive
```

确认 `src/kuavo_assets/config/kuavo_v$ROBOT_VERSION/kuavo.json` 的机器人版本、总质量和
`EndEffectorType` 与目标一致。配置不明确时停止，不猜测默认值。

## 2. Docker 编译

```bash
./docker/run.sh
source installed/setup.zsh
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
catkin build humanoid_controllers
```

## 3. MuJoCo 仿真

```bash
source devel/setup.zsh
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch joystick_type:=sim
```

仿真中先验证 reset、关节限位、控制频率、停止键和异常退出。不要在相同终端把仿真 launch
替换为 `load_kuavo_real.launch`。实机需要独立的零点/圈数、急停、扶持、上电和现场口令
检查，详见 [完整文档](../readme.md)。

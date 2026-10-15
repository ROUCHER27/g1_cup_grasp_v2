# G1 Cup Grasp V2

V2 将现有 RGB-D、YOLO 和手眼矩阵接到一个新的最小抓取流程：

```text
YOLO 分割
→ 同一局部像素与深度取点
→ 杯子基座坐标
→ 5 个侧抓角度
→ 每个路径点做 URDF IK
→ 选择关节运动较小的可行候选
→ 保存预览与 JSON 报告
→ 可选分阶段实机执行
```

## 已生成的主要文件

- `config/grasp_v2.yaml`：V2 感知、机器人、IK、抓取和执行配置。
- `models/robot/G1_120s.urdf`：智元官方 G1 模型。
- `src/g1_cup_grasp/kinematics.py`：URDF 正运动学和右臂 IK。
- `src/g1_cup_grasp/grasp_planner.py`：多角度候选、全路径 IK 和评分。
- `src/g1_cup_grasp/io/robot.py`：GDK 状态读取和分阶段执行；不会自动设置 SERVO。
- `tools/grasp_v2.py`：统一规划和执行入口。

原相机、YOLO、深度取点和坐标变换模块继续复用。旧项目
`g1_cup_grasp_gdk` 没有被修改。

## 环境

V2 暂时复用已经可以连接 GDK 和相机的旧环境，不需要重新安装：

```bash
source /home/mskjuzi/a2d_sdk/env.sh
source /home/mskjuzi/projects/g1_cup_grasp_gdk/.venv/bin/activate
cd /home/mskjuzi/projects/g1_cup_grasp_v2
export PYTHONPATH="$PWD/src"
```

V2 使用的 NumPy、SciPy、OpenCV、PyYAML、Ultralytics 和 GDK 已在该环境中。

## 第一步：只查看相机与取点

```bash
python tools/live_yolo_preview.py --device cpu
```

这个命令只使用相机，不创建机器人控制器，也不发送运动命令。

## 第二步：只规划

```bash
python tools/grasp_v2.py --device cpu
```

默认行为：

- 读取实时 RGB-D 和 YOLO 结果。
- 使用局部真实像素及其深度计算目标点。
- 比较 URDF 正运动学与 GDK `arm_right_link7` 反馈。
- 模型一致时生成 5 个候选，并检查全部路径点的 IK。
- 保存 `outputs/plans/*_preview.jpg` 和 `outputs/plans/*.json`。
- 不发送机械臂或夹爪命令。

返回状态：

- `PLAN_READY`：找到完整可行路线。
- `MODEL_MISMATCH`：URDF 与当前机器人反馈不一致，需要先修正模型映射。
- `NO_FEASIBLE_GRASP`：候选均未通过 IK 或工作区检查。
- `NAVIGATION_REQUIRED`：杯子超出当前验证工作区。

## 第三步：模型和抓取偏移校准

`config/grasp_v2.yaml` 初始保持：

```yaml
runtime:
  enable_real_motion: false
calibration:
  enabled: false
grasp:
  calibrated: false
```

先完成两项验证：

1. 在几个安全姿态比较 URDF FK 与 GDK `arm_right_link7`，位置误差应小于
   10 mm，姿态误差应小于 2°。
2. 重新示教当前取点方法对应的 `anchor_to_link7_offset_m`，并用另一个杯子
   位置验证。

完成后才能将上面三个配置开关设为 `true`。

## 第四步：分阶段实机执行

先只到预抓取点：

```bash
python tools/grasp_v2.py \
  --device cpu \
  --execute \
  --enable-real \
  --stop-after pregrasp
```

确认位置后，可依次验证 `grasp`、`lift`、`return` 和 `retreat`。全部验证后：

```bash
python tools/grasp_v2.py --device cpu --execute --enable-real
```

执行路径为：张开夹爪 → clearance → pregrasp → grasp → 闭合 → 抬升
20 mm → 放回 → 张开 → retreat。程序只检查 `SERVO` 状态，不会自动切换
机器人模式。

## URDF 来源

`models/robot/G1_120s.urdf` 来自智元官方仓库
`AgibotTech/genie_sim`，固定到提交
`6ca11c7593ecf6b7dae58c28fd19f4a789a0dd46`。当前文件 SHA-256：
`865dc8a81a9d93e60f5163a941ca62d67cad61679e9c1cf35db0e9c1bcf5e11b`。

## 当前边界

- 仅处理竖直圆柱杯和右臂侧抓。
- IK 检查关节限制和末端误差，没有完整三维碰撞模型。
- 候选范围只覆盖配置中的工作区；区域外返回 `NAVIGATION_REQUIRED`。
- 当前手眼矩阵和抓取偏移仍是待验证候选，不能直接开启真实运动。

## 侧抓示教采集

确认修正后的只规划结果正确后，将机械臂切换到官方示教方式，手动把右夹爪
摆到杯子中部的理想侧抓姿态。保持杯子、头部和腰部不动，然后运行：

```bash
python tools/teach_side_grasp.py --device cpu
```

它只读取相机和机器人反馈，不发送运动或夹爪命令。结果保存在
`outputs/side_teach/`，其中的 `base_rpy_rad` 和
`anchor_to_link7_offset_m` 是当前取点方式对应的候选值。移动杯子后再次只规划，
确认独立位置也正确，才能把 `grasp.calibrated` 设为 `true`。

## 导航步骤 1：停靠点与只读检查

`config/transport.yaml` 保存地图 6（CUP）的底盘停靠目标：
A 为 `(1.542, 1.344) m, 0 rad`，B 为 `(0.646, 1.098) m, pi rad`。
这是地图中的底盘位置，杯子放置点仍需在 B 桌单独示教。

在原 GDK 环境下运行，只读取地图列表、底盘状态和 HMI 配置：

```bash
cd /home/mskjuzi/projects/g1_cup_grasp_v2
python tools/check_navigation.py --station B
```

机器人实际在 A 点时，使用 `--station A`。工具不会切换模式、发送导航任务、
控制机械臂或夹爪；报告保存在 `outputs/navigation_check/`。
报告包括五次更新反馈、目标位置/朝向误差、定位变化、速度以及机器人配置的地图 ID。
`INSPECTION_RECORDED` 仅表示数据已记录、目标地图存在且机器人配置关联该地图，
不表示已验证导航、抓取可达性或放置安全性。普通 GDK 状态消息不提供运行时地图 ID；使用 `--native-feedback` 从机器人原生
`/hal/position` 获取实际 `map_id`，报告要求连续更新的反馈。充电状态和导航模式可在报告的 HMI 状态中查看。

地图 6 的导航入口为 `tools/navigate_station.py`。默认只读取反馈；`--preview`
只在离线计算请求参数，不初始化 SDK、不访问机器人：

```bash
python tools/navigate_station.py --station B --preview
python tools/navigate_station.py --station B
```

2026-10-13 携杯导航发生碰撞，未到达 B。已核对实际机器人上的 HAL 服务实现：
此版本 `navigate_to_pose(x, y, theta)` 请求采用 **毫米／度**，
`get_chassis_position()` 的地图位置反馈采用 **米／弧度**，两者单位不同。
配置仍保存米／弧度，发送前只转换一次：A=`(1542,1344,0)`，
B=`(646,1098,180)`。原 B 请求 `(0.646,1.098,pi)` 经服务端整数截断后
实际被编码为 `(0,1,3)`，造成目标解释错误。接口使用当前地图的绝对坐标；
`odom_x/y/angle` 属于里程计坐标系，不能用作 A/B 目标。

证据见 `outputs/navigation_contract/verification.md` 与该目录的服务器二进制、
反汇编记录。这一契约只针对已核对的 HAL 版本，升级后必须重新验证。

`enable_real_navigation: false` 保持真实派发禁用。即使开启该配置，执行检查仍要求
连续更新的原生 `map_id` 与地图 6 一致；HMI 配置或控制器选择的地图不能替代这项证据。
当前安装的 GDK 没有透传该字段；完整携杯入口通过原生反馈核对地图。工具不会自动切换模式、
重定位或恢复行走。

真实导航派发前，`tools/navigate_station.py` 会先调用 GDK 的
`set_agv_col_level(agv_collision_level)` 设置底盘避障等级，再调用
`set_agv_col_ctrl(True)` 开启底盘避障；只有两项响应都成功才会发送导航目标。
`config/transport.yaml` 当前设为 3（指南说明等级越高越灵敏）。这只接入底盘避障，
不代表机械臂轨迹已有碰撞规划；机械臂规划边界见下文。

未来执行分支要求：底盘 Idle、定位置信度至少 50、A 点位置/朝向符合容差、
静止反馈稳定、充电状态已知且未充电、自动模式与无急停/触变异常均确认。
它监测定位置信度低于 50（不再比较相对起点的下降幅度）、反馈中断、接近目标后又远离、持续无进展和实际报告速度。
按用户要求，代码不再因线速度或角速度超过试验上限而暂停，
也不再要求控制器的自动线速度配置不超过 100 mm/s。实际速度由底盘控制器决定。
速度数据仍记录并检查有效性；到达与停车确认仍要求线速度、角速度近零。
到达需要连续至少三次新反馈、持续至少一秒满足 Idle、位置误差 50 mm、
朝向误差 5°和速度近零。异常暂停响应与实际零速度确认分别记录，不能把响应成功当作停车。

原始故障轨迹保存在 `outputs/navigation_check/20261013_092408_948878_B_navigation.json`。
修复经过离线请求预览与故障轨迹回放测试；用户已完成空载 A→B 验证；携杯运输尚未进行实机验收。


## 携杯 A→B：到达后按示教姿态放杯

`tools/transport_cup.py` 每次重新识别并规划抓取，抬杯后移到运输姿态，
然后向地图 6 派发 B=`(646,1098,180)` 毫米／度目标。
已配置 B 桌 `place` 示教姿态时，到位停车后移动至该姿态并松开右夹爪；未配置时保持夹持。
机械臂控制状态必须正在更新；程序不会启动控制器或自动切换 SERVO。

默认只读取状态和规划，不执行动作：

```bash
cd /home/mskjuzi/projects/g1_cup_grasp_v2
export LOCATOR_IP=10.42.0.10
export AORTA_DISCOVERY_URI=http://10.42.0.101:2379
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
export DYLOG_DEFAULT_LEVEL=FATAL
/home/mskjuzi/projects/g1_cup_grasp_gdk/.venv/bin/python tools/transport_cup.py --device cpu
```

用户现场执行完整流程（会抓取、移动机械臂并导航）：

```bash
/home/mskjuzi/projects/g1_cup_grasp_gdk/.venv/bin/python tools/transport_cup.py --device cpu --execute --enable-real
```

终端会询问机器人 SSH/sudo 密码，不保存密码。完整入口要求同时提供两个执行参数；
`enable_real_navigation: false` 只控制旧的独立导航入口，不替代完整入口的显式执行参数。
可使用 `--stop-after carry` 抬杯并保持运输姿态，随后十分钟内用
`--resume-navigation --execute --enable-real` 继续；恢复导航不会再次抓取或打开夹爪。
报告保存于 `outputs/transport/`。离线运动学通过只证明计算可达，
尚不证明真实持杯稳定性、机械臂跟踪或环境碰撞净距。B 桌放杯示教已录入，尚未实机验收。


### 运动状态误报恢复（2026-10-13）

如果机械臂已恢复 SERVO，携杯入口仍报 `motion-control status is not publishing/updating`，
旧版 `FreshTransportArm` 的重复订阅检查可能误报：当前 GDK 的同一 Node 不允许
重复订阅相同话题，第二个订阅返回 `None`，检查的接收时间始终为零。
已修改 `src/g1_cup_grasp/io/transport_arm.py`，复用原 SDK 订阅，
先调用原状态缓存回调，再记录本机单调时钟接收时间。无需重启机器人或更改 SDK 安装。

在上述 GDK 环境和 v2 项目目录中，可只读验证状态恢复：

```bash
/home/mskjuzi/projects/g1_cup_grasp_gdk/.venv/bin/python outputs/navigation_contract/verify_arm_recovery.py
```

报告 `motion_status_available: true`、五次更新的 `SERVO`、`execution_ready: true`
及 `model.passed: true` 表示状态接收和模型核对通过；此工具不发送运动命令。
结果保存在 `outputs/navigation_contract/arm_recovery_verified.json`。


### 使用示教的持杯运输姿态

2026-10-13 已录入右臂运输示教姿态：`position_m=[0.631880,-0.226864,0.683523]`，
`rpy_rad=[-1.557688,0.005777,-0.067131]`，坐标系为 GDK `base_link`。
记录位于 `outputs/carry_teach/carry_teach_20261013_210132_609892.json`。
`config/transport.yaml` 的运输目标已更新为这一姿态；地图 6、A/B 底盘目标保持原值。
规划先从试抬杯姿态垂直抬升，再过渡到示教姿态，逐点验证 IK、关节余量和杯子倾斜。
不会再使用旧的固定侧移点；示教时夹爪反馈只作记录，不作为运输时的夹爪命令。
配置 B 桌放杯姿态后，到 B 并停车再放杯。

以后重新示教时，将右臂通过官方示教方式摆到运输姿态，保持标定头部和腰部位置，然后运行：

```bash
/home/mskjuzi/projects/g1_cup_grasp_gdk/.venv/bin/python tools/teach_carry_pose.py
```

该工具只保存姿态候选，不自动覆盖当前运输配置，也不控制夹爪或机械臂；
夹爪可以不夹杯。录入新的目标后仍须重新识别杯子并规划抓取到运输目标的路径。
原配置备份为 `outputs/carry_teach/transport_before_teach_20261013_210132.yaml`。


### B 桌示教放杯（2026-10-13）

已录入 `outputs/place_teach/place_teach_20261013_221756_885856.json`，配置位于
`config/transport.yaml` 的 `place`。示教末端位置为 `[0.586697,-0.194993,0.530769] m`，
朝向为 `[-1.684836,0.071606,-0.311424] rad`（GDK `base_link`）。同时记录地图中的
底盘位置和朝向，以便到 B 后按实际底盘位姿换算同一个放杯落点。

完整执行命令仍为 `tools/transport_cup.py --device cpu --execute --enable-real`，
现在包含：重新识别抓杯 → 示教运输姿态 → A 到 B → 到位停车 → 示教放杯位置 → 松开右夹爪。
放杯后不增加收臂动作。成功发送放杯动作记录为 `ARRIVED_B_CUP_RELEASED`，
不表示已通过视觉确认杯子稳定落桌。抓取规划只保留到 `lift`，
不再因源桌原地放回后的 `retreat` 不可达而拒绝整个运输任务。

最低定位置信度已调整为 50，50 可通过，低于 50 停止；相对下降幅度不限制。
只读录入新的放杯候选可运行 `tools/teach_place_pose.py`，不会打开夹爪、移动机械臂
或自动覆盖配置。示教时应保持杯子落点和底盘位置不动。

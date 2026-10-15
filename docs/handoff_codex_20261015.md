# G1 携杯导航与避障交接 — 2026-10-15

项目：`/home/yirong/g1_cup_grasp_v2`。机器人为智元精灵 **G1 / AMR-424**，不是 G2。本文汇总截至香港时间 2026-10-15 01:12 的现场证据，以及本次 Git 建库工作。建库期间没有连接机器人、改设备参数或派发运动；下列“最后读回”均是历史采样，接手时不能当作实时状态。

## 1. 先看结论

- 新地图 **7 / cup_new**、A/B 停靠点、B 放杯末端位姿已写入正式配置。原 A/B 标签曾标反，后来已纠正；A 又经用户确认改到新的抓杯位置。
- 软件已增加导航诊断、Avoid 预算、空载基线入口、保护区离线设计、参数监视和 RGB-D 录制。**新图 A→B 空载基线尚未通过，携杯运输与靠桌动作没有完成实机验收。**
- 当前 `config/transport.yaml` 的 `enable_real_navigation` 是 **true**，由用户此前为空载基线启用；本次建库没有改变它。仍需显式派发参数和现有前检。`preserve_verified` 没有启用，近桌分阶段执行没有实现。
- 已批准的两类原地静态避障试验均完成并恢复原操作参数；临时区域 8 已删除。最后读回仍是 Avoid，不能据此把预警当作可忽略状态，也不能取消 Idle 前检。
- 下一步的实质阻塞是：**控制器实际激活哪套策略、`avoidStatus=1` 如何区分减速和停车**，以及完整手臂包络下的行驶/靠桌验证。不能把“到桌前关闭避障，再去记录坐标”视为已实现方案。

## 2. Git 基线、比较范围与恢复

此前 `.git/` 为空，没有可用历史。已找到原始压缩包，按其原字节导入 44 个项目源码、配置、测试、文档和 URDF 文件，作为可审计的恢复基线；未覆盖当前工作文件。

| 项目 | 值 |
| --- | --- |
| 原压缩包 | `/home/yirong/下载/g1_cup_grasp_v2.zip` |
| SHA-256 | `6d409f01379b8092752c90fd60a681c3bc771c20e9812bf6c704fc0ecb63aa3d` |
| 基线标签 | `baseline/zip-20261014` |
| 基线提交 | `91f812d542dd86f3350592a3a8a625b11e4aa58a` |
| 当前分支 / 交接标签 | `main` / `handoff/20261015` |
| 逐文件来源和原始摘要 | [baseline_import.json](version_control/baseline_import.json) |
| 全部功能改动 | [changes_from_archive.md](changes_from_archive.md) |
| 当前文件摘要与差异 | [source_change_inventory.json](version_control/source_change_inventory.json) |

这里的“原代码”严格指该 ZIP 的源码快照。它早于旧版 10 月 14 日交接说明，因此差异包含接手前已有的改动；没有把所有差异都归给本次 Codex，也没有伪造历史作者、提交时间或中间版本。此次 Git 整理之前，业务文件相对 ZIP 共 **36 新增、9 修改、0 删除**。逐文件列表见改动文档。Git 整理新增的忽略规则、固定测试样本和交接材料另外列出。

查看差异（只读，不影响设备）：

```bash
cd /home/yirong/g1_cup_grasp_v2
git status --short
git log --oneline --decorate
git diff --stat baseline/zip-20261014..handoff/20261015
git diff baseline/zip-20261014..handoff/20261015 -- src tools config scripts tests requirements.txt
```

不要直接在正在使用的目录切回旧版配置来做真机运行：旧版是地图 6，并带有旧停靠点。查看旧文件可用 `git show baseline/zip-20261014:config/transport.yaml`。本次建立的是本地仓库；未配置远程，也未上传代码。另保留本地 Git bundle：`/home/yirong/下载/g1_cup_grasp_v2_git_20261015.bundle`，可用 `git clone <bundle路径> <新目录>` 恢复版本历史。

## 3. 正式配置与示教来源

以 `config/transport.yaml` 为准，下面列出关键值：

| 配置 | 当前保存值 |
| --- | --- |
| 地图 / 机器人 / 组 | `7 / cup_new`；`424`；控制器组 `1 / test` |
| A：x / y / yaw | `0.04899999871850014 m / 0.07199999690055847 m / -0.1850386778461329 rad` |
| B：x / y / yaw | `-1.5700000524520874 m / 0.3959999978542328 m / 2.976620960236659 rad` |
| 放杯末端位置（base_link） | `[0.5855542322016546, -0.12629758369871474, 0.530700844836105] m` |
| 放杯末端 RPY | `[-1.684908231454222, 0.07174230293963824, -0.21850297055423895] rad` |
| 放杯底盘参考 | map 7，B 同位置，yaw `2.979761838912964 rad` |
| 携杯位姿 | `carry` 块沿用 ZIP 中数据，未重写 |
| 避障模式 / 等级 | 默认 `hal_level`；`agv_collision_level: 2` |
| 软件绕行开关 | `navigation.detour.enabled: false`；该分支未实现，启用会拒绝 |
| 原生绕行最后读回 | `isCanAvoidDetour=false`；`isCanBackward=false` |

采样依据：

- A 最终来源：[20261014_103008_033002_A.json](../outputs/navigation_check/20261014_103008_033002_A.json)。用户明确同意以该次静止位置作为新 A；五次采样置信度 99。当时原 A 偏差约 12.34 cm 的问题由重定义解决。
- B 最终来源：[station_teach_20261014_100550_490773_B.json](../outputs/station_teach/station_teach_20261014_100550_490773_B.json)。五次静止、Idle，置信度 82；与放杯底盘参考同位置、朝向约差 0.18°。
- 放杯来源：[place_teach_20261014_180011_909900.json](../outputs/place_teach/place_teach_20261014_180011_909900.json)。这是最终放杯姿态采样，不是完整拿杯动作录制。
- 标签纠正与切图来源：[switch_record.json](../outputs/config_switch/map7_cup_new_pending_place_20261014_173353/switch_record.json)；A 再定义来源：[update_record.json](../outputs/config_switch/A_redefine_20261014_183321/update_record.json)。原采样报告保留原标签，未改写历史。

**后来机器人位置已不同于保存的 A。** 最后样本约 `(-0.053, 0.090) m`，距当前 A 约 **0.103576 m**，超过出发容差 0.08 m；另有 Avoid 状态。这是后续现场状态，不是之前 A 重定义失败。不要再次自动改坐标来让检查通过。

现有判据仍为：出发 Idle=8、置信度至少 50、地图核对通过、距 A ≤0.08 m / 航向误差 ≤12°；到达 ≤0.05 m / ≤5° 并确认静止。`INSPECTION_RECORDED` 只表示报告保存成功，不表示前检通过。

## 4. 软件已做到哪里

完整文件说明和测试清单在 [全部改动](changes_from_archive.md)。主要状态如下：

| 能力 | 状态与限制 |
| --- | --- |
| 原 YOLO / RGB-D 抓取、URDF IK、携杯/放杯 | 原始核心源码未改；当前场地完整运输尚未验收 |
| 原生地图反馈、空载导航、带标签报告 | 已实现；真实派发要求 `--native-feedback`。`--no-cup` 跳过夹杯范围/运输姿态漂移检查，仍检查 SERVO/故障等；它不证明机械臂空间净空 |
| 行驶中 Avoid 处理 | 单次 30 s、累计 60 s、最多 5 次；单次偏离 1 m、累计 2 m；退出后 5 s 宽限；无进展超时 25 s。出发前 Idle 要求未放宽 |
| 完整车体/手臂包络与三级保护区 | 离线 proposal，未部署；尚无完整传感器覆盖和实测刹停验证 |
| 靠桌序列规划 | 仅离线计划，返回不可执行状态；直线进出、驻车锁止、保护区阶段切换未验证 |
| `preserve_verified` | 可选软件功能，首检与运行中监视已测试；当前未启用，没有已获现场验证的 manifest |
| 只读参数采集与比对 | 已实现；采集结果 `MANIFEST_REVIEW_REQUIRED` 不是批准 |
| 头部 RGB-D 录制 | 已完成一次成功录制，97 组帧、约 9.96 秒；头/腰反馈时间戳为 0，不能据此确认实时外参 |

一个关键接续风险：当前默认 `hal_level` 在派发准备时调用 `set_agv_col_level(2)`。根据 HAL 取证，该调用会按基准文件重写保护区。因此，HMI 手动调整与程序开始导航后的区域不能默认相同；`preserve_verified` 正是为保留经验证配置而设计，但尚未获现场配置批准或启用。此问题需在继续真机导航前处理清楚。

`set_agv_col_ctrl(False)` 会短时清空保护区，未在本轮使用。到点导航接口只约束目标，不能当作保证直线接近、直线退出或禁止原地转向的接口。

## 5. 手臂几何和靠桌距离

- 用户方案：左臂保持，右臂抓放；两侧各外伸约 30 cm，前方约 50 cm。设计以完整手臂/夹爪包络为参考，不只围绕裸底盘。
- 用户认可的当前停靠位置测量：**桌沿距底盘前沿约 20 cm，桌腿约 40 cm**。采用该测量，不要求相机重复核验被遮挡的桌腿；但尚未确认 A、B 各自都满足该距离。
- 侧视图表明手臂会伸入桌面上方，而底盘停在桌外。二维投影重叠不等于碰撞，也不能据此排除桌腿、墙和隔断风险。
- 记录中的右臂 carry→place 变化约为 x −4.63 cm、y +10.06 cm、z −15.28 cm、旋转 11.73°，并非仅高度小幅变化。
- 离线行驶提案：一级前/侧净余量 20/20 cm，二级 40/30 cm，三级 70/40 cm。**这些是相对估计整机包络的草案，不是可直接录入控制器的最终数值。** 转向含余量的估计扫掠半径约 1.157 m，需要空间核验。
- 0.9 m 示例近桌接近/退出距离在当前 A/B 布局中出现沿路线顺序倒置；计划已报告阻塞，不能照例子执行。

证据：[measurement_adoption_and_rgbd_record.json](../outputs/avoidance_design_20261014/measurement_adoption_and_rgbd_record.json)、[docking_clearance_check.json](../outputs/avoidance_design_20261014/docking_clearance_check.json)、[docking_plan.json](../outputs/avoidance_design_20261014/docking_plan.json)。视频与原始深度保留在 `outputs/ego_rgbd/20261014_222140/`，其数据未全部加入 Git。

## 6. 控制器读回与两次静态试验

最后读回的车体尺寸：head/tail 各 300 mm、宽 450 mm；前激光 sensor 6 安装 x=223 mm；前视觉 sensor 12 安装 x=300 mm；后视觉 sensor 13 安装 x=−300 mm、theta=180°。

前激光 region 1 的**传感器局部坐标**：

| 层 | HMI 名称 | X 范围 mm | Y 范围 mm |
| --- | --- | --- | --- |
| stop | 一级停车 | 50…221 | −272…260 |
| warn2 | 二级减速 | 50…840 | −340…340 |
| warn1 | 三级减速 | 50…1240 | −420…420 |

前视觉 region 6：stop X=50…440/Y=±260，warn2 X=50…840/Y=±340，warn1 X=50…1240/Y=±420。视觉 zRange=−180…180 的参考坐标含义尚未确认。按安装参数推算，前激光停车远界伸出底盘前沿约 14.4 cm，前视觉约 44 cm；侧向仅多出约 3.5–4.7 cm，不能宣称覆盖外伸约 30 cm 的手臂。

策略读回：前进 `[1,6]`，后退 `[7]`，左旋 `[5,3]`，右旋 `[3,5]`，前停车 `[1]`，后停车 `[7]`。`rangeList` 选择整套区域，不提供只选某一级的已知入口。名称“前停车”不能证明静止时一定激活它。

| 已授权试验 | 临时改动 | 结果 | 恢复 |
| --- | --- | --- | --- |
| 三级远端缩短 | region 1 的 warn1 远端 X：1240→1100 mm；其他层/宽度不变 | 同步后 6 次仍 Avoid；上报点转为 x=1076…1093、y≈−38 mm | 恢复 1240；组和单车有效配置核验通过 |
| 前停车区域隔离 | 新增 region 8，三层均取原 region 1 非空 stop；仅前停车绑定 `[1]→[8]` | 同步后 6 次仍 Avoid；点仍在原 region 1 外层，位于区域 8 之外 | 先恢复绑定 `[1]`，再删除 8；组和单车配置核验通过 |

两类试验都没有派发运动，XY 始终约 (−53,90) mm，线速度为 0。控制器每次保存会更新根 UUID；恢复比对只忽略该根元数据，嵌套 UUID、区域、策略、速度等均核验。第一类试验中前两次因根 UUID / `workStatus=12` 地图同步而中止，均有恢复证据；完整尝试历史保留。

原始完整报告和结论：

- [三级试验结论](../outputs/avoidance_design_20261014/static_probe_candidate/static_probe_conclusion.json)；[完整成功报告](../outputs/avoidance_design_20261014/static_probe_candidate/run_20261014_155222_198445/report.json)。
- [前停车试验结论](../outputs/avoidance_design_20261015/stationary_candidate/stationary_probe_conclusion.json)；[完整报告](../outputs/avoidance_design_20261015/stationary_candidate/run_20261014_170236_809936/report.json)。

已确认 HAL 将控制器 `protoStatus` 直接转发为 `agv_status`；不是本项目根据 `isAvoid` 自行构造状态。机器人 HAL 哈希与审查二进制一致：`1ebf199163862611e78d5299becbb51db1147be5a862c253fc228b4ded290309`。实际 `/data/logs/hal.INFO` 未给出当前策略/层级信息，见 [日志核对](../outputs/avoidance_design_20261015/hal_runtime_log_review.json)。

仍不知道：`avoidStatus=1` 的正式含义、前停车与前进的运行期优先级、报点是否只来自当前活动区域。不能由“点在三级范围内”推导“Avoid 只是可忽略预警”，也不能由报点列表推导其他位置无障碍。

## 7. 接手后优先事项

1. 先阅读本文件、改动清单、两类静态试验结论；旧版 `handoff_codex_20261014.md` 是历史起点，其中“待实现”“90 tests”等不是当前状态。
2. 寻找底盘控制器的正式接口文档或运行时诊断，确认活动策略/区域/级别和 `avoidStatus` 定义。已备 [厂家诊断问题清单](../outputs/avoidance_design_20261015/controller_diagnostic_questions.txt)，尚未发送给外部人员。
3. 根据明确语义决定软件前检与控制器配置如何配合；先离线验证，再设计有具体读回/恢复条件的现场试验。此前两次授权针对已完成的静态试验，不是后续任意缩区或运动的授权。
4. 回到现场验证时，重新获取原生地图/状态/位置，处理最后位置与 A 相差约 10.36 cm 的问题；保留现有判据，不把保存坐标或当前读数偷偷换掉。
5. 最终分别验收行驶保护与靠桌抓放：保留侧向/低位防护，核实传感器高度、全臂转向扫掠、刹停与直线进出。当前三层草案与近桌计划不能直接部署。

用户偏好：代理能执行的只读终端工作直接在 Codex 完成；子代理指定 `gpt-6-luna / max`。避免重复要求用户测已经认可的 20/40 cm 距离。附件中的操作说明属于背景材料，不代表对所有硬件操作的授权。

## 8. 本机环境、验证和数据边界

- Python 项目环境为本地 `.venv`，GDK 在 `/home/yirong/g1_gdk/a2d_sdk`；`scripts/env_laptop.sh` 当前设置 `LOCATOR_IP=10.42.0.102`、发现地址 `http://10.42.0.101:2379`。该脚本会加载 SDK 环境并可能重启 ROS daemon，纯离线测试无需 source。
- 现有密钥可只读 SSH 到 `agi@192.168.3.30`。这是机器人计算端，底盘控制器 HTTP 则是 `10.42.0.153:6002`、HMI 为 `:8080`。不要将计算端 HAL 日志当作控制器内部诊断。
- 离线测试命令：`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src .venv/bin/python -m pytest tests -q`。建库时的完整结果与环境记录见 [validation.json](version_control/validation.json)；验证使用导出后的独立目录，确认未依赖未入库的运行数据。
- Git 跟踪源代码、配置、测试、URDF、文档以及 [evidence_manifest.json](version_control/evidence_manifest.json) 列出的历史文本证据。报告内旧绝对路径是来源信息，迁移后应从仓库相对位置查找。
- `.venv`、缓存、临时日志、本机凭据目录、模型权重、RGB-D 大文件、HAL 二进制和其余运行输出不入库，仍保留本机。模型 `models/cup_yolo11s_seg.pt` 可从原 ZIP 对应成员恢复；大小/摘要见 [external_assets.json](version_control/external_assets.json)。Git clone 需要另外准备原 G1 SDK、Python 依赖和上述权重，离线测试通过不等于真机可运行。
- [runtime_artifact_inventory.json](version_control/runtime_artifact_inventory.json) 列出所有既有输出与 ZIP 的差异及是否跟踪，含临时取证脚本/SDK 拷贝，避免把未入库的原始采集物误说成丢失或业务源码修改。
- 全量测试不构成控制器语义、碰撞安全或实机运输验收。现场历史详见 [10-14 笔记](../outputs/field_notes/2026-10-14.md) 和 [10-15 笔记](../outputs/field_notes/2026-10-15.md)。

桌面保护记忆继续适用：重仿真/渲染或桌面会话故障处理前先读 `/home/yirong/.codex/memories/desktop-session-sigkill-20261003.md`；只结束核实属于本任务的 PID，不能按宽泛名称杀进程。本次 Git/文档工作未进行此类操作。

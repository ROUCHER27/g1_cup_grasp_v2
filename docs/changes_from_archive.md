# 相对原始 ZIP 的全部代码改动

基线、恢复和当前真机状态见 [2026-10-15 handoff](handoff_codex_20261015.md)。本表由 ZIP 原始文件与 Git 建库前的工作目录逐文件摘要比对生成：**9 个修改、36 个新增、0 个删除**。范围包含源码、入口、配置、脚本、测试、项目文档和 URDF；运行输出另见第 5 节。所有新增/修改逐项列出，没有用“等”省略文件。

## 1. 已有文件的修改（9）

| 文件 | 相对 ZIP 的变化 |
| --- | --- |
| [README.md](../README.md) | 补充 5090 环境、地图 7/cup_new、重新示教来源、独立空载导航、避障取证与离线设计/参数监视用法；说明已完成范围与未验证项。 |
| [config/transport.yaml](../config/transport.yaml) | 地图 6/CUP→7/cup_new；A/B 与 place 整块换为新采样；known_hosts 改本机路径；独立导航 false→true；等级 3→2；新增 Avoid/超时预算和禁用的 detour 配置。carry 块保持原值。 |
| [requirements.txt](../requirements.txt) | ruckig==0.14.0 改为 >=0.14.0，并记录 0.19.4 wheel 的环境验证；增加 CPU torch 安装说明。此文件仍非完全锁定的依赖清单。 |
| [tests/test_navigation_check.py](../tests/test_navigation_check.py) | 将测试配置改用固定 transport fixture，不再要求生产地图/坐标保持旧值。 |
| [tests/test_navigation_execution.py](../tests/test_navigation_execution.py) | 扩充配置严格校验、原生反馈/派发门禁、空载、报告标签、Avoid 预算/恢复、超时、速度参数读回/补偿和相关失败路径测试；测试配置固定化。 |
| [tests/test_transport.py](../tests/test_transport.py) | 将真实运输配置依赖改为 fixture，补充配置/地图不一致时拒绝执行的断言；保留既有抓放流程测试。 |
| [tools/check_navigation.py](../tools/check_navigation.py) | HMI 检查报告新增 isAvoid、avoidStatus、avoidSensorList；原生反馈能力在 ZIP 版本已经存在，不应归为该文件本次新增。 |
| [tools/navigate_station.py](../tools/navigate_station.py) | 新增严格配置验证、原生反馈入口、--no-cup、--label、可配置 Avoid 看门狗/动态超时、可选 chassis_info 读回与失败补偿、可选 preserve_verified 监视及原生暂停确认。详细行为见下节。 |
| [tools/transport_cup.py](../tools/transport_cup.py) | 入口增加运输配置校验与 manifest 预检；地图错误消息改为实际配置值；经共享 execute_navigation 接入新监视功能。抓取/放置主流程未重写。 |

## 2. 新增文件（36）

| 文件 | 作用 / 部署状态 |
| --- | --- |
| [config/avoidance_control.example.yaml](../config/avoidance_control.example.yaml) | preserve_verified 示例，只描述选择方式与 manifest 引用；未合入当前正式配置。 |
| [config/avoidance_profile.example.yaml](../config/avoidance_profile.example.yaml) | 整机/手臂包络、三级净距与传感器安装信息的离线示例。 |
| [config/docking_clearance.example.yaml](../config/docking_clearance.example.yaml) | 桌沿/桌腿间距及分层靠桌计算输入；使用用户认可的测量并保留未核验条件。 |
| [config/docking_plan.example.yaml](../config/docking_plan.example.yaml) | 近桌阶段序列示例；不是可执行运动计划。 |
| [docs/field_test_runbook.md](../docs/field_test_runbook.md) | 现场顺序、只读检查、示教/切图/空载基线与报告判读；后续增加当前状态和离线设计限制。 |
| [docs/handoff_codex_20261014.md](../docs/handoff_codex_20261014.md) | 保存接手时的历史任务与进度；不是当前状态的唯一依据。 |
| [scripts/env_laptop.sh](../scripts/env_laptop.sh) | 5090 本机 venv/GDK 环境加载；LOCATOR_IP 最终修正为 10.42.0.102；发现服务、protobuf 与 pytest 插件设置。 |
| [src/g1_cup_grasp/avoidance_comparison.py](../src/g1_cup_grasp/avoidance_comparison.py) | 只读历史快照与整机提案比较，保留真实边界、传感器变换、覆盖差异和不可应用的阻塞项。 |
| [src/g1_cup_grasp/avoidance_guard.py](../src/g1_cup_grasp/avoidance_guard.py) | 严格 manifest 身份/字段/摘要/引用验证、后台配置监视、过期/漂移/异常终止路径；只证明配置匹配，不证明空间安全。 |
| [src/g1_cup_grasp/avoidance_profile.py](../src/g1_cup_grasp/avoidance_profile.py) | 整机二维包络、嵌套停车/减速区、传感器局部坐标变换及离线提案。 |
| [src/g1_cup_grasp/docking_clearance.py](../src/g1_cup_grasp/docking_clearance.py) | 依据底盘/传感器安装位置、桌沿与桌腿距离做分层投影和净空计算，显式区分几何估计与传感器/动作验证。 |
| [src/g1_cup_grasp/docking_plan.py](../src/g1_cup_grasp/docking_plan.py) | 抓放前后接近/退出/转向候选顺序及阻塞项；接口语义未验证时返回不可执行计划。 |
| [src/g1_cup_grasp/io/controller_avoidance.py](../src/g1_cup_grasp/io/controller_avoidance.py) | 有界 HTTP 只读参数采集，检查响应/数值结构并构造待审查快照。 |
| [tests/fixtures/navigation_avoid_long_trace.json](../tests/fixtures/navigation_avoid_long_trace.json) | 超长 Avoid 轨迹回归样本。 |
| [tests/fixtures/navigation_avoid_near_goal_trace.json](../tests/fixtures/navigation_avoid_near_goal_trace.json) | 接近终点后出现 Avoid 的轨迹回归样本。 |
| [tests/fixtures/navigation_avoid_trace.json](../tests/fixtures/navigation_avoid_trace.json) | 普通 Avoid→恢复轨迹回归样本。 |
| [tests/fixtures/transport_config_test.yaml](../tests/fixtures/transport_config_test.yaml) | 固定旧地图/示教数据供离线测试，解除与当前现场配置的耦合。 |
| [tests/test_avoidance_comparison.py](../tests/test_avoidance_comparison.py) | 真实固定快照下的传感器变换、边界保留、后向/旋转不适用、候选阻塞及 CLI 比较测试。 |
| [tests/test_avoidance_control_integration.py](../tests/test_avoidance_control_integration.py) | 验证两种避障模式在导航/运输入口的首检、禁止覆盖、异常暂停路径。 |
| [tests/test_avoidance_guard.py](../tests/test_avoidance_guard.py) | manifest 无效值/身份/引用/摘要与监视线程超时、漂移、旧快照、启动关闭竞态等回归。 |
| [tests/test_avoidance_profile.py](../tests/test_avoidance_profile.py) | 几何包络、嵌套关系、坐标变换、输入校验与提案生成。 |
| [tests/test_avoidance_profile_cli.py](../tests/test_avoidance_profile_cli.py) | 只读采集 CLI 的候选状态与错误报告。 |
| [tests/test_chassis_pause.py](../tests/test_chassis_pause.py) | 只读/释放模式区分、现场确认、状态/地图/原生反馈、请求失败与读回等路径。 |
| [tests/test_controller_avoidance_snapshot.py](../tests/test_controller_avoidance_snapshot.py) | 控制器只读采集的响应处理、边界与数据有效性。 |
| [tests/test_docking_clearance.py](../tests/test_docking_clearance.py) | 分层靠桌计算、输入验证、投影局限、离线 CLI。 |
| [tests/test_docking_plan.py](../tests/test_docking_plan.py) | 阶段顺序、航向/几何条件、阻塞项和不可执行状态。 |
| [tests/test_station_tools.py](../tests/test_station_tools.py) | 地图清单与只读示教的连续采样、地图一致性、错误报告和不改正式配置。 |
| [tools/chassis_pause.py](../tools/chassis_pause.py) | 暂停状态检查；只有 --release 与 --confirm-path-clear 组合才进入释放请求，另受前置状态检查约束。属于可能写设备的工具。 |
| [tools/check_avoidance_profile.py](../tools/check_avoidance_profile.py) | 生成控制器参数快照和待审查 manifest；--live 为只读采集，不批准也不部署。 |
| [tools/check_docking_clearance.py](../tools/check_docking_clearance.py) | 分层靠桌净空计算 CLI，仅离线报告。 |
| [tools/compare_avoidance.py](../tools/compare_avoidance.py) | 当前快照与候选包络比较 CLI，不修改控制器。 |
| [tools/map_manager.py](../tools/map_manager.py) | 只读地图列表和 HMI/控制器/原生地图信息核对，不建图、不切图。 |
| [tools/plan_avoidance.py](../tools/plan_avoidance.py) | 离线生成三级几何 proposal，不应用。 |
| [tools/plan_docking.py](../tools/plan_docking.py) | 离线生成近桌序列与阻塞信息，不发运动。 |
| [tools/record_ego_rgbd.py](../tools/record_ego_rgbd.py) | 只读订阅与保存头部彩色/无损深度、同步时间和状态；NaN 元数据转换 null 并记录字段，逐帧留档。 |
| [tools/teach_station.py](../tools/teach_station.py) | 在现场停靠点只读多次采样，核验地图/静止/置信度并输出 A/B 候选，不自动覆盖正式配置。 |

## 3. 导航入口的重要行为变化

- `navigation_settings`、`validate_transport_config` 拒绝未知配置、非有限数、非法类型、地图/放杯参考不一致和启用未实现的 detour；配置错误在构造硬件执行对象前检查。
- 独立导航新增 `--native-feedback`；真实 `--dispatch` 强制要求它，并在采样、导航反馈、暂停确认中使用原生源。GDK Position 缺少地图字段，不能伪造为已验证。
- `--no-cup` 仅跳过夹杯反馈范围和运输姿态漂移监视，保留机械臂准备状态检查；`--carry-clearance-verified` 仍是现场确认而非自动碰撞检测。
- `--label` 限制为安全文件名字符；保存报告时添加后缀。
- 行驶中新增 Avoid=5 预算、episode 报告、退出宽限期和进展基线重置；默认不把静止 Walking=7 当 Avoid。出发 Idle=8 与置信度≥50、地图、位置判据保持。
- 无进展超时从 ZIP 的 10 s 改为默认 25 s；总导航时限按距离、基础时间和 Avoid 总预算计算并封顶。到达仍要求 Idle、距离/航向达标及连续静止反馈。
- 可选 chassis_info 写入前读取旧值，写后等待反馈匹配，未匹配时补发旧值并报错；当前正式配置没有启用该写入块。HAL 的 min_collision_distance 未接入保护区多边形，不能靠该参数调真实范围。
- 默认 hal_level 保留 set_agv_col_level 和 True 控制调用；新增 preserve_verified 模式时跳过等级/覆盖写入，先核验 manifest，运行中监视漂移/超时，失败走现有暂停确认。当前未启用。
- `execute_navigation` 增加 leg、可选请求/容差和结构化 Avoid 结果，最终释放监视线程；发生已尝试派发后的异常，使用原生反馈确认暂停，无法确认则报告 STOP_UNCONFIRMED。

## 4. 哪些原代码未改、改动何时已有

ZIP 中的核心抓取和运动学实现均保持字节一致：`src/g1_cup_grasp/carry.py`、`config.py`、`domain.py`、`grasp_planner.py`、`kinematics.py`、`geometry/*`、原相机/机械臂/原生底盘适配器、`perception/*`；`tools/grasp_v2.py`、`check_rgbd.py`、`live_yolo_preview.py`、`native_chassis_feedback.py`、原有 `teach_carry_pose.py` / `teach_place_pose.py` / `teach_side_grasp.py`；`config/grasp_v2.yaml`、`config/perception.yaml`、URDF 和 YOLO 权重也未变。完整逐文件摘要在 [source_change_inventory.json](version_control/source_change_inventory.json)，权重在 [external_assets.json](version_control/external_assets.json)。

版本归属按证据分层，不能把 ZIP 后所有改动都称为本次新写：

1. 10-14 旧交接文档已记载 Avoid 看门狗、地图/站点/暂停工具与本机环境工作；原生反馈独立入口、空载、标签、计时器与测试配置解耦在接手时仍列为待办，当前代码已完成。依赖迁移也早于 20:38 的离线设计实现。
2. 当天示教/切图、A/B 标签纠正、LOCATOR_IP 修复、A 再定义与启用空载基线均有 `outputs/config_switch/` 备份和更新记录。
3. 20:38 的 [implementation_record.json](../outputs/avoidance_design_20261014/implementation_record.json) 记录整机保护区、docking 提案、preserve_verified 及入口整合，附文件哈希和 200 测试结果。它的“没有修改控制器”仅描述该次离线实现。
4. 22:31 的 [measurement_adoption_and_rgbd_record.json](../outputs/avoidance_design_20261014/measurement_adoption_and_rgbd_record.json) 记录后续参数比较、分层净空、RGB-D 录制及 224 测试结果。
5. 随后的两类静态真机试验确实临时写过控制器，并已恢复；这是实验配置变化，不是持续部署的软件改动。

## 5. 运行输出中的脚本与证据

Git 另保存两支会临时写参数的静态实验脚本，及精确候选、完整执行/恢复报告：

- `outputs/avoidance_design_20261014/static_probe_candidate/run_confirmed_probe.py`：组成员/自动模式/静止/任务前检，提交结果未知时补偿，组和单车有效参数多次读回，根 UUID 元数据处理，有限地图同步观察与恢复异常记录。
- `outputs/avoidance_design_20261015/stationary_candidate/run_stationary_probe.py`：复用前者，临时区域唯一身份、前停车绑定切换、先恢复引用后删除自有区域、并发引用保护。两者不是通用生产部署入口，历史批准不授权重跑。

`implementation_backup/` 的三份 tools 源码是改前副本，不是额外的新业务实现；一并跟踪用于复查。`outputs/navigation_contract/robot_probe/` 下的 SDK/protobuf 源码是取证拷贝，`build_report.py` / `hal_static/analyze_elf.py` 是临时取证辅助，未接入运行入口；它们保留本机并列入输出清单。

完整输出清单 [runtime_artifact_inventory.json](version_control/runtime_artifact_inventory.json) 比较了 ZIP 中的 280 个历史输出和当前采集物：原 280 个摘要均未变，后续增加 912 项（含媒体/协议拷贝/报告）。它标明每个文件是否作为 Git 证据跟踪，不能将未跟踪采集物当成遗漏的生产源码。

## 6. 本次 Git 整理新增变化

- 原 ZIP 导入独立基线提交；当前源码、测试、文档和选定历史文本证据提交在 main，并打交接标签。未重写业务配置、未触碰设备。
- `.gitignore` 补充环境/缓存/运行输出/日志/凭据目录/模型权重规则。已有历史证据按清单显式跟踪，新采集输出默认忽略。
- 新增 `tests/fixtures/controller_avoidance_snapshot_20261014.json`，与原只读快照字节一致；`tests/test_avoidance_comparison.py` 改读固定 fixture，避免新采集覆盖测试输入或 Git 恢复后缺文件。来源哈希见 [fixture_provenance.json](version_control/fixture_provenance.json)。
- README 与现场手册增加当前 handoff 入口；新增本文件、`docs/handoff_codex_20261015.md` 和 `docs/version_control/` 的来源/文件/证据/外部资产/验证清单。
- 使用 Git 导出的独立目录运行全套离线测试，验证所跟踪文件足以重现测试；结果见 [validation.json](version_control/validation.json)。

这里不提供虚构的逐次历史提交。要查看任何行级差异，以 `git diff baseline/zip-20261014..handoff/20261015 -- <路径>` 为准；文档清单说明功能，Git diff 给出全部文本变化。

# Handoff（2026-10-14）：G1 携杯导航加避障 —— 交给笔记本端 Codex 继续

本文件是给在 `xjtlu-SuperServer` 上运行的 Codex 的交接说明。读完本文再读 `docs/field_test_runbook.md`（现场逐步手册）。机器人是智元 精灵 G1（GDK `a2d_sdk` 1.5.0）；**不是 G2，不要引用 `/home/yirong/g2_gdk` 或任何 G2 文档**。

## 1. 目标与现状
- 流程：识别杯子 → 抓取 → A 桌导航到 B 桌 → 放杯。本轮要加：场地重布后**重建地图**，并让导航**自动绕开地面纸盒（高 ≥20 cm）**。
- 已完成（全部在仓库 `/home/yirong/g1_cup_grasp_v2`，这是代码真源；Mac 只是 rsync 副本）：
  - 环境：`source scripts/env_laptop.sh`（venv + GDK + 有线 10.42.0.102）。`python -m pytest tests -q` 当前 90 通过。
  - 导航看门狗已容忍底盘 `agv_status=5 Avoid`（预算：单次 30 s、累计 60 s、≤5 次、偏离 ≤1 m/累计 2 m；Avoid 与其后 5 s 宽限期内冻结"接近后远离/无进展"规则并重置基线）。实现在 `tools/navigate_station.py`，配置在 `config/transport.yaml` 的 `navigation:` 块。
  - 底盘避障接入：`agv_collision_level: 2`（HAL 实测 2→1.0× 全尺寸避障区；1/3→0.8×）；`set_agv_col_ctrl` 只发 True，**永不发 False**（False 会清零避障区约 2 s）。
  - 新工具：`tools/map_manager.py --list`（只读地图清单/三方 map_id 核对）、`tools/teach_station.py --station A|B`（只读示教停靠点候选）、`tools/chassis_pause.py --release --confirm-path-clear`（释放暂停，唯一会发底盘命令的新工具）。工具的审查修复任务 `phase2b_fixes` 可能仍在跑（`tmux ls` 看 `phase2b_fixes`；报告 `/tmp/codex_tasks/phase2b_fixes.report.md`）。
- 取证结论（证据在 `outputs/navigation_contract/g1_nav_api_report.md` 与 `robot_probe/`）：底盘控制器有原生"长时间避障绕行"开关 `scheduleParam.isCanAvoidDetour`（当前 false，等待 10 s 后绕行，最大 1.8 m）——放盒子测试前由现场人员在底盘 HMI `http://10.42.0.153:8080` 参数页打开。自主导航遇障到底"绕"还是"停"只能实测。

## 2. 现场流程（现场人员执行，Codex 辅助判读）
按 `docs/field_test_runbook.md` 第 0–5 节。Codex 的职责：
1. 运行只读命令并判读报告（手册 0.3 的 `chk` / `summ` 函数），把结论写回 `outputs/field_notes/<日期>.md`（新建目录；每次运行一条：命令、report_path、status、判读）。
2. 示教完成后，按手册 2.4 **一次性**改 `config/transport.yaml`（`map_id`、`map_name`、`stations.A/B`、`place` 整块；`place.base_pose.map_id` 必须等于 `map_id`），改前备份到 `outputs/config_switch/`，改后跑手册 2.5 校验。
3. 手册 2.6 更新 README 坐标（不要动被测试断言的句子）。

## 3. 待做代码任务（阶段 3 实机前必须完成；TDD，离线，禁止实例化 Slam()/RobotDds()）
手册作者在读代码时发现以下阻塞点，需要 Codex 先改代码：
1. **`tools/navigate_station.py` 独立入口无法通过 preflight**：它只用 GDK 反馈（`collect_samples(slam)`），GDK 1.5 的 `Position` 不带 `map_id`，`runtime_map_check` 永远不通过。改法：加 `--native-feedback`（与 `transport_cup.py` 一样用 `NativeChassisFeedback` 作为 `collect_samples` 与 `execute_navigation` 的反馈源；`get_map_list` 仍用 Slam），`--dispatch` 时强制要求该标志。
2. **空载运行的夹爪/手臂检查**：`navigate_station.main` 硬性要求右夹爪反馈在 5–90（闭合夹杯），`execute_navigation` 还检查运输姿态漂移 ≤20 mm/7°。加 `--no-cup`：跳过夹爪区间与运输姿态漂移检查，但保留 SERVO/故障/碰撞检查；报告里记 `no_cup: true`。
3. **测试与真实配置解耦**：部分测试把夹具里的 `map_id: 6`/旧坐标与真实 `config/transport.yaml` 比较，换图后会失败。改为测试自带配置副本（`tests/fixtures/transport_config_test.yaml` 或在测试里构造 dict），所有期望由该副本派生。
4. **两个 10 s 计时器冲突**：底盘绕行要先等 `minDetourWaitAvoidTime=10 s`，看门狗 `no_progress_timeout_s` 也是 10 s；若底盘停障时仍上报状态 7，看门狗会先中止。把 `config/transport.yaml` 的 `navigation.no_progress_timeout_s` 改为 25.0（注释写明原因），并确认 `treat_halted_walking_as_avoid` 仍默认 false。
5. `navigate_station.py` 加 `--label <str>`，报告文件名加后缀（如 `_B_navigation_obstacle.json`），手册 3.2.4 不再需要手动 `cp`。
6. 以上每项先写失败测试再实现；完成后 `python -m pytest tests -q` 全绿，并在手册 3.1.1/3.1.2/3.2.4 对应处把"待确认/代码改动"字样更新为实际用法。

## 4. 红线
- 任何向机器人发送运动/导航/建图/重定位/模式切换/暂停释放的命令，只能由现场人员在急停在手时按手册执行；Codex 不得自行发起。
- 不改 `agv_collision_level`、不发 `set_agv_col_ctrl(False)`、不开启 `navigation.detour.enabled`（未实现，代码会拒绝）。
- 出错后工具只会 `pause_walk(True)`；释放暂停只用 `chassis_pause.py --release --confirm-path-clear`，且先清场。
- 不动 `/home/yirong/g2_gdk`；不改 `~/.bashrc`；pip 用清华镜像；不要装 CUDA 版 torch（磁盘只剩约 25 GB）。

## 5. 关键文件
- 方案：Mac `~/.claude/plans/review-5090-1-2-ethereal-whisper.md`（笔记本上无副本；要点已在本文件）。
- 手册：`docs/field_test_runbook.md`。
- 配置：`config/transport.yaml`、`config/grasp_v2.yaml`。
- 入口：`tools/transport_cup.py`（完整携杯）、`tools/navigate_station.py`（独立导航）、`tools/check_navigation.py`（只读）。
- 取证：`outputs/navigation_contract/g1_nav_api_report.md`、`g1_probe_phase1/report.md`、`genie_msgs_pb_0.8.0_notes.md`。
- Codex 任务书与日志：`/tmp/codex_tasks/`（重启即丢，重要结论请落到 `outputs/`）。

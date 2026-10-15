# G1 现场测试 Runbook：新图 cup_new 与地面盒子避障

> 2026-10-15 交接入口：[当前状态、全部代码改动和 Git 基线](handoff_codex_20261015.md)。下文现场命令需结合该文档的未完成项阅读。

适用于 2026-10-14 场地重布后的验证。依据：方案文件、`README.md`、`config/transport.yaml`、`tools/*.py` 源码、`outputs/navigation_contract/g1_nav_api_report.md` §1/§2/§4。标"待确认"处没有文件证据，以现场实测为准。

## 当前现场进度（2026-10-14）
- 新图为 `7/cup_new`，A/B 与放杯姿态已录入并切换到正式配置；独立导航开关已为用户现场空载基线启用，但最近一次派发因 A 点 `Avoid` 状态在发运动命令前被拒绝。
- 实际 A 已由用户确认改为当前位置，采用 `outputs/navigation_check/20261014_103008_033002_A.json` 的五次稳定采样，坐标 `(0.049, 0.072) m, -0.185039 rad`，置信度 99、自动 Idle；替换此前纠正标签后的 A。旧报告按新 A 重新执行 preflight 判据通过，尚非新的实时采样。
- 实际 B 使用 `outputs/station_teach/station_teach_20261014_100550_490773_B.json`，坐标 `(-1.570, 0.396) m, 2.976621 rad`，五次采样 Idle、静止、置信度 82，HMI/控制器/原生 map_id 均为 7。
- 放杯使用 `outputs/place_teach/place_teach_20261014_180011_909900.json`，`place.base_pose.map_id=7`。B 最终位置与放杯底盘参考一致，朝向差约 0.18°；离线路径已通过关节余量检查，尚未做新图实机运输验收。
- 配置备份、来源与离线结果见 `outputs/config_switch/map7_cup_new_pending_place_20261014_173353/switch_record.json`；A 点更新记录见 `outputs/config_switch/A_redefine_20261014_183321/update_record.json`。下一步核对 2b 的避障绑定及手臂/通道净空，再由现场人员按 3.0–3.1 准备空载基线。
- 新增 `plan_avoidance.py` 与 `plan_docking.py` 为纯离线提案工具；近桌直线进出、驻车锁止和阶段保护切换尚未验证，计划不能实机执行。控制器参数保留核验功能已加入入口，但当前 `transport.yaml` 未选择 `preserve_verified`，也未存在可用的现场审查 manifest；不要把设计图当作已生效保护区。
- 调参前可由现场人员执行 `.venv/bin/python tools/check_avoidance_profile.py --live --output outputs/avoidance_design_20261014/controller_snapshot_current.json` 只读采集完整配置。先审查几何、引用关系、传感器覆盖和刹停，再决定可用参数；不能把候选的 `review_required` 直接当批准。字段/模式与限制详见 README 的保护区设计章节。

## 约定
- 笔记本 `xjtlu-SuperServer`，所有命令先执行：`cd /home/yirong/g1_cup_grasp_v2 && source scripts/env_laptop.sh`。
- 带 `--native-feedback`、`teach_*`、`transport_cup.py` 的命令会提示"机器人 SSH/sudo 密码（不保存）"，由现场人员手输，不要写进命令行或文件。
- 一人专职急停，不兼操作键盘。退出码：0 成功，2 前置/判据未通过（有报告），1 异常。
- **回报** = 停止一切运动命令，保留 `outputs/` 报告，把终端末尾 JSON 与 `report_path` 发回 Claude；不改代码，不改 `navigation.*` / `agv_collision_level`，不重试运动命令。
- 每次出发前机器人必须在 A 点（preflight 要求距 A ≤0.08 m、航向 ≤12°，即使目标是 A 也一样），所以返程只能手柄/HMI 手动复位。

## 0. 前置检查
**0.1 机器人与现场**（目视 + HMI `http://10.42.0.153:8080`）
- 不在充电（preflight 报 `robot is charging` 即此项）；HMI 自动模式（`isHardAuto`、`isSoftAuto` 均 true）；`isNormal` true，`isPause/isScram/isEdged` 均 false；急停在手且未按下。
- 机械臂 SERVO，头部俯仰与腰部在手眼标定位姿（程序不会自动切换，`assert_execution_ready` 会拒绝）。
- 失败：按 HMI 提示恢复；机械臂用官方方式恢复 SERVO；恢复不了 → 回报。

**0.2 网络**
```bash
ping -c2 -W2 10.42.0.101 && ping -c2 -W2 10.42.0.153
curl -s -m 5 -o /dev/null -w 'HMI %{http_code}\n' http://10.42.0.153:8080
nc -z -w2 10.42.0.153 6002 && echo "controller 6002 ok"; nc -z -w2 10.42.0.101 2379 && echo "aorta 2379 ok"
```
预期：全部通，HMI 200。失败：查网线（enp5s0）和 `ip -4 addr show dev enp5s0` 是否有 10.42.0.x/24；仍不通 → 回报，不自行改网络。

**0.3 定义两个判读函数**（每个新终端粘贴一次；`chk` 读最新的 check_navigation 报告，`summ` 读导航/运输报告）
```bash
chk() { python - <<'EOF'
import glob, json, os
ds = (json.load(open(f)) for f in sorted(glob.glob("outputs/navigation_check/*_[AB].json"), key=os.path.getmtime, reverse=True))
d = next(x for x in ds if "samples" in x and "station" in x)
h, r = d.get("hmi", {}), d.get("runtime_map") or {}
st = h.get("state", {})
print(d.get("status"), d.get("error"), "| 地图", d.get("map_list", {}).get("map_id"), d.get("map_list", {}).get("map_name"))
print("mapId HMI/控制器/原生:", h.get("robot_config_map_id"), d.get("controller", {}).get("driver_selected_map_id"), r.get("map_ids"), "available/verified:", r.get("available"), r.get("verified"))
print({k: st.get(k) for k in ("isHardAuto", "isSoftAuto", "isNormal", "isPause", "isScram", "isEdged", "confidence")}, "充电:", [b.get("isCharging") for b in st.get("batteryStateList") or []])
print("agv_status:", [x["agv_status"] for x in d.get("samples", [])], "summary:", d.get("summary"))
EOF
}
summ() { python - "$1" <<'EOF'
import itertools, json, math, sys, yaml
d = json.load(open(sys.argv[1])); s = yaml.safe_load(open("config/transport.yaml"))["stations"]
a, b = s["A"], s["B"]; rows = d.get("navigation_feedback", []); av = d.get("avoidance") or {}
def off(r):  # 到 A→B 线段的距离
    dx, dy = b["x_m"] - a["x_m"], b["y_m"] - a["y_m"]
    t = max(0, min(1, ((r["agv_pos_x"] - a["x_m"]) * dx + (r["agv_pos_y"] - a["y_m"]) * dy) / (dx * dx + dy * dy)))
    return math.hypot(r["agv_pos_x"] - a["x_m"] - t * dx, r["agv_pos_y"] - a["y_m"] - t * dy)
print("status:", d.get("status"), "| error:", d.get("error"), "| stop_confirmed:", (d.get("stop") or {}).get("stop_confirmed"))
print("agv_status 序列(状态,样本数):", [(k, len(list(g))) for k, g in itertools.groupby(r["agv_status"] for r in rows)])
print("最大横向偏移 m: %.3f | 最大线速度 m/s: %.3f" % (max(map(off, rows), default=0), max((abs(r["linear_speed"]) for r in rows), default=0)))
print("avoidance:", {k: av.get(k) for k in ("episode_count", "total_avoid_s", "longest_episode_s")}, "| 碰撞等级:", (d.get("chassis_collision_avoidance") or {}).get("level"))
for e in av.get("episodes", []): print("  episode", e["kind"], "时长", e["duration_s"], "距目标", e["start_distance_m"], "->", e["end_distance_m"], "峰值", e["max_distance_m"])
print("arrival:", d.get("arrival"))
EOF
}
```
状态码：1 EdgedTouch、2 Scram、3 Mapping、4 NeedRelocate、5 Avoid、6 Charging、7 Walking、8 Idle、9 NotReady。

**0.4 只读检查（旧图 6 基线）**
```bash
python tools/check_navigation.py --station A --native-feedback; echo "exit=$?"; chk
```
预期：exit 0、`INSPECTION_RECORDED`；地图列表含 6/CUP（取证时为 2/ssd、5/444、6/CUP），HMI/控制器/原生 mapId 均为 6，`available/verified` True；`confidence` ≥50；`agv_status` 全 8。`summary.position_error_m` 相对旧 A 点无意义。
失败：exit 2 `MAP_NOT_CONFIRMED` → 看三处 mapId 谁不是 6；exit 1 看 `error`（SSH 密码错、`native chassis feedback unavailable`）。只读，可重试；仍失败 → 回报。

**0.5 地图基线** `python tools/map_manager.py --list --native-feedback; echo "exit=$?"`
预期：exit 0、`MAP_LIST_RECORDED`。记下 `maps` 作建图前基线。失败同 0.4。

**0.6 测试基线** `python -m pytest tests -q 2>&1 | tail -5` —— 记录通过/失败数，供 2.5 对照。

## 1. 建图（HMI/手柄，不用 SDK）
**1.1 布场**：桌椅道具摆到最终位置并此后不再动（建图后移动会使地图失效）。**纸盒先不放**（盒子要作为地图里没有的障碍）；地面清线缆杂物，建图时人员不要在车前走动。
**1.2 新建地图并扫描**：在 HMI 的系统设计 → 地图管理创建目标地图，并在机器人组 `test` → 基本参数把运行地图关联到目标图。机器人静止、现场急停在手时，由操作员在系统监控 → 424 → 手柄区切到软手动，再到目标地图 → 编辑 → 激光工具 → 地图构建，右侧选择机器人 424，点击"开始扫图"。预期：进入 Mapping（`agv_status` 3）。提示"当前 AGV 非手动模式"时核对软手动；选择机器人为空时先核对组的运行地图。失败：查充电、急停、模式；不行回报。
**1.3 手柄慢速绕场**：速度 ≤0.1 m/s（手动最大速度配置 200 mm/s），依次覆盖 A 桌、B 桌、通道，最后回到起点附近闭环。预期：HMI 实时地图连续无断墙。失败：漂移/重影 → 回到 1.2 重建，不删除旧图（2/ssd、5/444、6/CUP；删图不在本 runbook 内）。
**1.4 结束并保存**，目标地图名为 `cup_new`。到主页操作 → 地图上传，将地图同步至服务；确认提示说明会同步所有 AMR、影响运行，须由现场人员在机器人静止时执行。遇到服务器版本更新提示时先保留扫描成果，明确要采用的版本再确认。预期：地图列表出现 cup_new，预览无重影。
**1.5 切换并重定位**：HMI 把当前地图切到 cup_new，在 HMI 重定位，直到置信度 ≥50；确认自动模式（`isHardAuto/isSoftAuto` true）、机器人 Idle。失败：置信度上不去 → 移到特征丰富处再重定位；仍不行回报。
**1.6 记录新 map_id 并核对三方一致**
```bash
python tools/map_manager.py --list --native-feedback; echo "exit=$?"
python - <<'EOF'
import glob, json
d = json.load(open(sorted(glob.glob("outputs/map_manager/*_list.json"))[-1]))
print("maps:", d["maps"]); print("HMI:", d["hmi_robot_config_map_id"], "| 控制器:", d["controller_selected_map_id"], "| 原生:", (d["runtime_map"] or {}).get("map_ids"))
EOF
export NEW_MAP_ID=7   # 示例值，换成三者一致的实际 id
```
预期：**此时 exit 2 / `MAP_INCONSISTENT` 是正常的**（config 仍是旧图 6，工具以它为对照）；`maps` 含 cup_new；HMI、控制器、原生（5 个采样）三个 ID 相同。
失败：`maps` 缺 cup_new（保存失败或 GDK 列表未刷新）→ 回报，不重启服务；三者不一致 → HMI 重做"切换当前地图"后重跑，仍不一致 → 回报；原生 `map_ids` 为 None → 回报。

## 2. 示教 A/B 与放杯
config 在 2.4 之前保持旧值（内部一致，各工具才能启动）；**不要分批改 config**。
**2.1 示教 A**：手柄把机器人开到 A 桌停靠位（手臂够得到杯位；用胶带标出底盘位置，供复位与量距），确认自动模式、Idle，采样期间（约 3 s）不动。
```bash
python tools/teach_station.py --station A --samples 5 --expect-map-id "$NEW_MAP_ID"; echo "exit=$?"
```
预期：exit 0、`STATION_TEACH_RECORDED`，`station_candidate.map_id == NEW_MAP_ID`，报告 `outputs/station_teach/station_teach_<ts>_A.json`。读候选：
```bash
python - <<'EOF'
import glob, json
for s in "AB":
    fs = sorted(glob.glob(f"outputs/station_teach/*_{s}.json"))
    if fs:
        d = json.load(open(fs[-1])); print(fs[-1]); print(" ", d["status"], d.get("rejection_reasons"), d["station_candidate"], d["planned_request_preview"])
EOF
```
失败：`STATION_TEACH_REJECTED` 看 `rejection_reasons`——不稳定/超速：等静止重跑；非 Idle：查自动模式与暂停；置信度 <50：HMI 重定位；map 不可用或不符：回 1.6。每次重跑新建文件，以最后一个为准。
**2.2 示教 B**：手柄开到 B 桌停靠位（正对 B 桌、手臂够得到放杯位，同样贴胶带），同上命令改 `--station B`。此后底盘不要再动，直接做 2.3。
**2.3 示教放杯**：B 桌放杯落点放标记；按官方示教方式把右夹爪摆到放杯姿态并保持不动（同 2026-10-13）。
```bash
python tools/teach_place_pose.py; echo "exit=$?"
python -c "import glob,json; p=sorted(glob.glob('outputs/place_teach/place_teach_*.json'))[-1]; d=json.load(open(p)); print(p, d['status']); print(json.dumps(d['place_config_candidate']))"
```
预期：exit 0、`PLACE_TEACH_CAPTURED`，`base_pose.map_id == NEW_MAP_ID`（取自原生反馈）。工具不动机械臂/夹爪、不改配置。失败：`taught pose does not agree with the calibrated URDF` / `too close to a joint limit` → 调整姿态重示教；`map_id` 不是新 id → 回 1.6。
**2.4 备份并手改 `config/transport.yaml`（一次改完）**
```bash
cp config/transport.yaml outputs/config_switch/transport_before_cup_new_$(date +%Y%m%d_%H%M%S).yaml
```
只改这些项（值来自 2.1/2.2/2.3 的候选）：`map_id: <NEW_MAP_ID>`、`map_name: cup_new`、`stations.A/B` 的 `x_m/y_m/yaw_rad`、`place.position_m`、`place.rpy_rad`、`place.base_pose.{map_id,agv_pos_x,agv_pos_y,agv_angle}`（`map_id` 必须等于顶层 `map_id`，否则 teach_station/map_manager/chassis_pause/navigate_station/transport_cup 启动即拒绝，报 `place base_pose was taught on another map`）。顶部 "HMI map 6 (CUP)" 注释顺手改。**不改**：`agv_collision_level: 2`、`enable_real_navigation: false`（3.1 前才开）、`navigation:`、`carry:`、`native_feedback:`。
**2.5 校验**
```bash
for s in A B; do python tools/navigate_station.py --station $s --preview | grep -E '"status"|"error"|"x_mm"|"y_mm"|"yaw_deg"'; done
python tools/map_manager.py --list --native-feedback; echo "exit=$?"
python -m pytest tests -q 2>&1 | tail -15
```
预期：两次均 `OFFLINE_REQUEST_PREVIEW`，`x_mm/y_mm` = 候选×1000 取整，`yaw_deg` = 弧度转度 mod 360（朝向 π 应为 180）；map_manager exit 0、`MAP_LIST_RECORDED`；pytest 对照 0.6。
失败：preview 报 `ERROR` 且信息含 `must be` / `unknown navigation keys` / `place base_pose…` = yaml 写错，回头改；map_manager 仍 exit 2 → 看 `config_name_matches` 与三个 ID。导航与运输测试已使用自带配置夹具，换图后仍应全绿；若测试失败，列出失败用例名回报，不把它忽略为预期的旧坐标问题。
**2.6 更新 README 坐标**：`grep -n '地图 6\|1\.542\|0\.646\|(1542\|(646' README.md`。把描述"当前配置"的句子（导航步骤 1 的 A/B 坐标与"地图 6（CUP）"、"地图 6 的导航入口"、"与地图 6 一致"、携杯 A→B 段的 B 目标）改为新图/新坐标；保留 2026-10-13 事故复盘中的旧数值；不要动被测试断言的句子（"`config/transport.yaml` 当前设为 2"、"1→0.8×、2→1.0×、3→0.8×"、"`g1_nav_api_report.md` §1.1"）。

## 2b. 底盘 HMI 避障策略绑定（任何自主导航之前做一次；建图不需要）
取证快照（2026-10-14）显示"全局避障策略"里 前进/后退/左旋/右旋/所有情形 的区域列表全部为空，"前停车"引用了不存在的区域 2——即前进时没有任何避障区域生效。在 HMI（管理员登录）机器人组设置 → 避障策略 中改为：

| 情形 | 绑定区域 |
|---|---|
| 前进 | 1 前激光避障、6 前视觉避障 |
| 后退 | 7 后视觉避障 |
| 左旋 / 右旋 | 3 旋转前激光避障、5 旋转侧前激光避障 |
| 左移 / 右移 / 所有情形 | 留空 |
| 前停车 | 1 |
| 后停车 | 7 |

局部避障策略"左关"不动。保存后只读核对（预期 `前进` 为 `[1, 6]` 等，`mapId` 仍为 `NEW_MAP_ID`）：
```bash
curl -s -m 8 -X POST http://10.42.0.153:6002/api/Config/Agv/GetParam -H 'Content-Type: application/json' -d '{"id":424}' | python3 -c "import json,sys; c=json.load(sys.stdin)['Content']; print('mapId',c['mapId']); print([(s['name'],s['rangeList']) for s in c['avoidStrategyParam']['globalStrategy']]); print({k:c['scheduleParam'][k] for k in ('navigationMode','isCanAvoidDetour','minDetourWaitAvoidTime')})"
```
与脚本的关系：策略决定"哪些区域在什么运动下生效"；脚本的 `agv_collision_level: 2` 决定区域尺寸（1.0×）；看门狗负责容忍 Avoid 状态。三者互补，脚本不改策略。

## 3. 实机验证三级（用户在场，急停在手）
**3.0 每级出发前**（2b 已完成）：机器人在 A 胶带位，自动模式、Idle、未暂停、未充电；A→B 通道及两侧无人。
```bash
python tools/check_navigation.py --station A --native-feedback; echo "exit=$?"; chk
```
预期：exit 0；`summary.position_error_m` <0.08、`|yaw_error_deg|` <12、`confidence_range` ≥50；`verified` True。失败：偏差超限 → 手柄挪回 A 重测；置信度低 → HMI 重定位。

### 3.1 空载新图 A→B（HMI 绕行开关保持关闭，作基线）
**3.1.1 原生地图反馈只读检查**：`navigate_station.py` 用 GDK `Slam.get_map_list()` 检查地图清单，并可用原生底盘反馈采样 `map_id`；GDK `Position` 本身不带 `map_id`。
```bash
python tools/navigate_station.py --station A --native-feedback; echo "exit=$?"
```
预期：`runtime_map.available` 和 `runtime_map.verified` 均为 True，HMI/控制器地图匹配。任一为 False → 回报，不要尝试绕过 preflight。
**3.1.2 机械臂**摆到携杯时的运输姿态但不夹杯，确保路线净空检查覆盖实际手臂外伸包络；机械臂保持 SERVO。空载命令使用 `--no-cup`，它只跳过右夹爪 5–90 区间和导航期间运输姿态漂移检查；SERVO、故障、碰撞与其余 preflight 门槛仍生效。操作员仍须按 3.0 确认路线和手臂包络净空，`--carry-clearance-verified` 仍为必需项。
**3.1.3 开真实导航**：`config/transport.yaml` 设 `enable_real_navigation: true`（3.2 结束后改回 `false`；完整携杯入口不依赖此项）。
**3.1.4 派发并判读**
```bash
python tools/navigate_station.py --station B --dispatch --native-feedback --no-cup --carry-clearance-verified --label baseline; echo "exit=$?"
summ "$(ls -t outputs/navigation_check/*_B_navigation_baseline.json | head -1)"
```
预期：exit 0、`status: ARRIVED`，`arrival` 位置 ≤0.05 m、航向 ≤5°；序列约 `[(7,n),(8,m)]`；横向偏移≈0.02 m、最大速度≲0.2 m/s（`autoMoveParam.maxSpeed` 200 mm/s；参照 10-13 成功记录 `outputs/transport/20261013_224901_962261.json`：0.019 m / 0.107 m/s）；`avoidance.episode_count` 为 0；碰撞等级 2；HMI 位置与 B 桌一致、手臂够得到放杯位（目视）。
失败：preflight 类 `error`（`robot is charging`、`chassis is not Idle`、`robot is not localized near station A`…）未发生运动，修正后可重试；运动中 `status: ERROR` → 看 `stop.stop_confirmed`，true 则已暂停（HMI `isPause`=true）转第 4 节，false 或 `STOP_UNCONFIRMED` → 立即急停并回报。报告：`outputs/navigation_check/<ts>_B_navigation_baseline.json`。

### 3.2 空载 + 盒子（先开底盘原生绕行）
**3.2.1 HMI 打开"长时间避障绕行"**（`scheduleParam.isCanAvoidDetour`；参数页路径、保存按钮名待确认）。改前、改后各读回一次：
```bash
curl -s -m 8 -X POST http://10.42.0.153:6002/api/Config/Agv/GetParam -H 'Content-Type: application/json' -d '{"id":424}' | python -c "import json,sys; c=json.load(sys.stdin)['Content']; print(c['mapId'], {k: c['scheduleParam'][k] for k in ('navigationMode','isCanAvoidDetour','minDetourWaitAvoidTime','maxDetourDistacne','maxDetourBackLength','avoidRange','safeDistance')})"
```
取证值（2026-10-14）：`6 {navigationMode 1, isCanAvoidDetour False, 10000, 1800, 2500, 200, 100}`。预期：只有 `isCanAvoidDetour` 变 True，`mapId` 仍是 `NEW_MAP_ID`；再跑 `map_manager.py --list --native-feedback` 应 exit 0。失败：读回仍 False、其它值或 mapId 变了 → 回报，勿自行再改。最大绕行 1800 mm、最大后退 2500 mm（含义待确认）→ 路径两侧与后方 ≥2 m 内无人无物。
**3.2.2 摆盒子**（地面纸盒，高 ≥20 cm；记录长×宽×高）。先算位置：
```bash
python - <<'EOF'
import math, yaml
s = yaml.safe_load(open("config/transport.yaml"))["stations"]; a, b = s["A"], s["B"]
d = math.hypot(b["x_m"] - a["x_m"], b["y_m"] - a["y_m"]); db = max(d / 2, 0.5); t = db / d
print("|AB|=%.3f m；盒心距 B %.2f m、距 A %.2f m；盒心地图坐标 x=%.3f y=%.3f" % (d, db, d - db, b["x_m"] + t * (a["x_m"] - b["x_m"]), b["y_m"] + t * (a["y_m"] - b["y_m"])))
EOF
```
盒心放在 A→B 连线上，距 B ≥0.5 m（|AB|<1.0 m 时中点不满足，取距 B 0.5 m 处）。算出距 A <0.5 m，说明障碍离起点过近（阈值待确认）→ 回报。用卷尺从 A 胶带位沿连线量；盒子至少一侧留 ≥0.45 m（车宽）加手臂余量的通道，否则无处可绕。
**3.2.3 派发**：机器人回 A，过 3.0，机械臂姿态同 3.1.2。按 3.1.4 命令派发，并把 `--label baseline` 改为 `--label obstacle`。操作员盯 HMI 状态页"避障"栏与触发传感器 id，记录首次减速/停车时车头离盒的距离。**车头离盒约 0.3 m 仍未减速或停车 → 立即急停**（建议值，非文件数据）。
**3.2.4 判读**：工具会直接生成 `outputs/navigation_check/<ts>_B_navigation_obstacle.json`，无需手动复制或改名：
```bash
summ "$(ls -t outputs/navigation_check/*_B_navigation_obstacle.json | head -1)"
```

| 结果 | 报告判据 | 下一步 |
|---|---|---|
| 绕障 | `ARRIVED`；盒子未被碰动；横向偏移明显（量级 ≥ 盒子半宽 + 车体半宽 0.225 m）；常伴 `avoidance.episode_count`≥1（状态 5），也可能全程 7 但轨迹侧移 | 记录序列与 episodes；同布置再复测 1 次（建议），两次均绕障 → 3.3 |
| 停障 | 盒前静止、无接触；`ERROR`，`error` 含 `chassis avoidance`（单次/累计超时、次数、偏离过大）或 `made no verified progress for 25 seconds`；`stop.stop_confirmed` true | 不重试、不改配置；移走盒子 → 第 4 节释放暂停 → 手动回 A；回报并附状态序列与停车距离。原生绕行先等 10 s（`minDetourWaitAvoidTime`）；`no_progress_timeout_s` 设为 25 s，给状态 7 的停障等待绕行留出时间。`treat_halted_walking_as_avoid` 仍为 false，`navigation.detour.enabled` 仍未实现且代码会拒绝开启。 |
| 碰撞 | 任何接触（车体/手臂/盒子被推动）、序列含状态 1 或 2、`error` 含 `chassis protection/state` | 急停；停止后续所有级别，不进 3.3；保持现场、拍照、量盒子位移；回报 |

其它 `error`（定位、反馈中断、机械臂、夹爪）与避障无关 → 第 4 节。

### 3.3 携杯 + 盒子
**3.3.1 前提**：3.2 判为"绕障"；盒子布置不变；HMI 绕行开关仍开；A 桌放好杯子，机器人在 A，过 3.0。
**3.3.2 无运动演练**：`python tools/transport_cup.py --device cpu; echo "exit=$?"`。预期：exit 0、`PLAN_READY_NO_MOTION`；`runtime_map.verified` true；`planned_request` 为新 B 的 mm/度；`planned_place_at_B` 存在；`model_validation.passed` true。失败：`does not confirm map N` → 回 1.6；`no feasible grasp+carry route` → 挪杯重试。
**3.3.3 执行**（抓杯 → 携杯导航 → 放杯）：
```bash
python tools/transport_cup.py --device cpu --execute --enable-real; echo "exit=$?"
summ "$(ls -t outputs/transport/2*.json | head -1)"
```
可选分步：先加 `--stop-after carry`，十分钟内用 `--resume-navigation --execute --enable-real` 续导航（不再抓取）。原生反馈 helper 默认寿命 600 s，整次运行是否会超时待确认。
预期：exit 0、`ARRIVED_B_CUP_RELEASED`（只表示放杯动作已发出，杯子是否稳定落桌须目视）；报告含 `avoidance`、`chassis_collision_avoidance`；盒子未碰。报告：`outputs/transport/<ts>.json`。
失败：`ERROR` → 看 `stop.stop_confirmed`；`STOP_UNCONFIRMED` → 立即急停。机械臂已 `arm.stop()`（见 `arm_stop_response`），杯子可能仍在夹爪里；不要重跑 `--execute`（会重新识别抓取）；回报后由 Claude 决定是否 `--resume-navigation`。

## 4. 异常处理
**4.1 任何中止后的固定顺序**：①不再发运动命令，车还在动就急停；②读终端末尾 `status`、`error`、`stop`、`report_path`；③目视盒子/杯子/手臂/人；④出错后工具只会 `pause_walk(True)` 且从不自动释放，HMI `isPause=true` 是预期的安全终态，它会使下一次 preflight 报 `HMI normal/emergency/touch/pause state is not confirmed safe`；⑤需要继续时按 4.2 释放，再手柄复位回 A、恢复自动模式，过 3.0。
**4.2 `chassis_pause.py --release` 的使用边界**
```bash
python tools/chassis_pause.py --release --confirm-path-clear; echo "exit=$?"
```
- 只在 HMI `isPause=true`、`isNormal=true`、无急停/触边、5 个样本静止、置信度 ≥50 时执行（不要求 Idle），否则 `PAUSE_RELEASE_REJECTED`（原因见 `rejection_reason`，exit 2）。
- 先清场：盒子移走、人退开、急停在手。`pause_walk(False)` 可能让底盘续跑未完成的导航任务（待确认）；工具监视 5 s，速度非零即再次暂停并返回 `RESUMED_MOTION_REPAUSED`（exit 2）→ 视为故障，回报。
- 不用于：碰撞后、急停/触边未复位、定位丢失、HMI 上有人故意暂停、想"让它继续走到 B"。
- `PAUSE_RELEASED`（exit 0）只表示 `isPause` 已 false 且静止，不代表位置正确；`PAUSE_NOT_RELEASED`（5 s 后 HMI 仍暂停）→ 回报，不连续重试。仅需暂停：`python tools/chassis_pause.py --pause`。
**4.3 定位丢失**（`agv_status` 4、`position_conf` <50、`localization confidence is below 50`）：停试验 → HMI 重定位 → 置信度 ≥50 → `check_navigation.py --station A --native-feedback; chk`。反复丢失 → 回报（地图质量问题可能需重建）。
**4.4 报告位置**（`ls -t` 取最新；`--preview` 也会写 `_navigation.json`）

| 工具 | 报告 |
|---|---|
| check_navigation | `outputs/navigation_check/<ts>_<A或B>.json` |
| navigate_station | `outputs/navigation_check/<ts>_<A或B>_navigation[_<label>].json` |
| chassis_pause | `outputs/navigation_check/<ts>_pause_release.json` / `_pause.json` |
| map_manager | `outputs/map_manager/<ts>_list.json` |
| teach_station | `outputs/station_teach/station_teach_<ts>_<A或B>.json` |
| teach_place_pose | `outputs/place_teach/place_teach_<ts>.json` |
| transport_cup | `outputs/transport/<ts>.json`（`latest_hold.json` 是携杯回执，不是报告） |

**4.5 必须停止并回报**：①任何物理接触或盒子被碰动；状态 1/2；`STOP_UNCONFIRMED`；`RESUMED_MOTION_REPAUSED`；②三方 map_id 不一致或 `runtime_map.verified` 为 False 且 HMI 重切图/重定位无效；③3.2 的"停障"结果，或 `NavigationAvoidTimeout` 类中止；④看不懂的 preflight 拒绝，或 `robot HAL version changed`；⑤运动后报告缺 `avoidance` / `chassis_collision_avoidance` 键；⑥置信度反复 <50；⑦需要改代码、`navigation.*`、`agv_collision_level` 或开启 detour；⑧SSH/网络反复失败。

## 5. 本次现场记录清单
- [ ] 新 `map_id` ___、`map_name` cup_new、建图时间 ___、HMI 预览有无重影 ___
- [ ] 三方 ID（HMI/控制器/原生）___ / ___ / ___；`maps` 列表 ___
- [ ] A：`x_m` ___ `y_m` ___ `yaw_rad` ___（报告 ___）；B：`x_m` ___ `y_m` ___ `yaw_rad` ___（报告 ___）
- [ ] 放杯 `position_m` ___ `rpy_rad` ___ `base_pose` (map_id ___, x ___, y ___, angle ___)（报告 ___）
- [ ] `config/transport.yaml` 备份文件名 ___；pytest 前后结果 ___ → ___；`|AB|` ___ m
- [ ] HMI 绕行开关改前/改后读回值 ___ / ___
- [ ] 盒子：尺寸 ___×___×___ cm、盒心地图坐标 ___、距 A/B ___/___ m、朝向 ___、两侧通道净宽 ___
- [ ] 每次运行：时间 ___、命令 ___、报告文件名 ___、`status` ___、结果（绕障/停障/碰撞/无关错误）___
- [ ] 每次运行的 `agv_status` 序列 ___、`avoidance.episode_count/total_avoid_s/longest_episode_s` ___、最大横向偏移 ___、首次减速/停车时车头离盒 ___ m、HMI"避障"栏与触发传感器 id ___
- [ ] 3.3：`ARRIVED_B_CUP_RELEASED` 与否 ___，杯子落点目视 ___
- [ ] 4.2 若使用：释放前状态、命令、结果 `status` ___、报告 ___

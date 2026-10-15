# 精灵 G1 导航、建图与避障只读取证报告

取证日期：2026-10-14；本机起始时间 13:02:49（UTC+8）。SSH 对象 `agi@192.168.3.30`，查询底盘 `10.42.0.153:6002` 和 HMI `:8080`。本文证据路径相对 `outputs/navigation_contract/`；原样响应为 `.json`，`.pretty.json` 仅在本机解析排版，数值未改。查询响应时间戳约 13:16，不能与本机、HAL 日志时间默认视为同步。`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:1369`

**核心结论：** 当前分配地图为 `6/CUP`；自主导航（`navigationMode=1`）的长时间避障绕行开关为 false。当前避障区域吻合基准的 0.8 缩放，但前进等全局策略未关联区域，不能证明行驶时会触发停障。已安装 HAL 的等级实现为 1→0.8、2→1.0、3→0.8，与“越高越灵敏”注释冲突；False 是临时等级 0 后约 2 秒恢复等级 2，True 分支直接返回。`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:827`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:939`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:413`；`robot_probe/hal_static/avoid_level_table.txt:5`；`robot_probe/hal_static/CollisionCtrl.disasm.txt:13`；`robot_probe/hal_static/AvoidTimerCallback.disasm.txt:31`

## 1. 避障等级、区域、开关与传感器

### 1.1 等级 1/2/3 不是 stop/warn2/warn1 三个触发带

服务注释原文：`避障等级1，2，3 ， 等级越高，越灵敏`。这是 SDK 灵敏度参数；HMI 中另有触发带：`stop` 为一级停车，`warn2` 为二级减速，`warn1` 为三级减速，两套“等级”不可混为一谈。`robot_probe/protos/agv/AGVCollisionLevel.srv:3`；`robot_probe/hmi/labels_context.txt:1`；`robot_probe/hmi/polygon_frame_context.txt:14`

| SDK level | 本版 HAL 缩放因子 | 对本机基准 JSON 的处理 | 证据 |
|---|---:|---|---|
| 1 | 0.8 | 同时处理所有传感器的 stop/warn1/warn2；x=50 保持，其余 x 与全部 y 乘因子后向零取整；zRange 不缩放 | `robot_probe/hal_static/avoid_level_table.txt:3`；`robot_probe/hal_static/process_obstacle_ranges.disasm.txt:301`；`robot_probe/hal_static/process_obstacle_ranges.disasm.txt:201` |
| 2 | 1.0 | 同时处理所有传感器的 stop/warn1/warn2；x=50 保持，其余 x 与全部 y 乘因子后向零取整；zRange 不缩放 | `robot_probe/hal_static/avoid_level_table.txt:4`；`robot_probe/hal_static/process_obstacle_ranges.disasm.txt:301`；`robot_probe/hal_static/process_obstacle_ranges.disasm.txt:201` |
| 3 | 0.8 | 同时处理所有传感器的 stop/warn1/warn2；x=50 保持，其余 x 与全部 y 乘因子后向零取整；zRange 不缩放 | `robot_probe/hal_static/avoid_level_table.txt:5`；`robot_probe/hal_static/process_obstacle_ranges.disasm.txt:301`；`robot_probe/hal_static/process_obstacle_ranges.disasm.txt:201` |

这是**安装文件的静态实现证据**，未调用等级设置。构造函数将四个 `int32 + double` 常量插入等级表；`HandleAGVAvoidLevel` 经虚表进入 `SetAvoidLevel`，再调用 `SetAvoidRange` 与 `adjust_obstacle_ranges`。`SetAvoidRange` 使用启动时读入的基准 JSON，而不是对已缩小的在线 JSON 继续累乘。两处 HAL 文件 SHA-256 完全相同；PID 3800 的 cmdline 指向 `/home/agi/app/bin/hal`，但无权限读取 `/proc/3800/exe` 链接，无法证明进程内存未发生替换。`robot_probe/hal_static/linkhouWrapper.disasm.txt:353`；`robot_probe/hal_static/vtable_selected.txt:3`；`robot_probe/hal_static/SetAvoidLevel.disasm.txt:38`；`robot_probe/hal_static/SetAvoidRange.disasm.txt:173`；`robot_probe/hal_static/adjust_obstacle_ranges.disasm.txt:95`；`robot_probe/hal_static/binary_hashes.txt:1`；`robot_probe/hal_static/runtime_identity.txt:1`

### 1.2 多边形原文与距离

以下都是四顶点矩形，完整顶点顺序保留在原始文件；表用 x/y 边界表示同一多边形。坐标单位按 HMI 距离/车体参数为 mm；HMI 把传感器坐标先旋转再加传感器安装偏移，因此以下边界应按传感器局部坐标解读，不能直接当作车体前沿净空距离。`robot_probe/hmi/units_strategy_context.txt:12`；`robot_probe/hmi/polygon_frame_context.txt:2`

| 区域 id/name（sensorId；shape） | 触发带 | HAL 基准 x；y | 当前控制器 x；y（亦为 level 1/3 静态结果） | zRange | 证据 |
|---|---|---|---|---|---|
| 1/前激光避障（6；0） | stop | [50,277]；[-340,325] | [50,221]；[-272,260] | [0,0] | `robot_probe/hal_conf/linkhou_avoid_range.json:5`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:10` |
| 1/前激光避障（6；0） | warn2 | [50,1050]；[-425,425] | [50,840]；[-340,340] | [0,0] | `robot_probe/hal_conf/linkhou_avoid_range.json:5`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:10` |
| 1/前激光避障（6；0） | warn1 | [50,1550]；[-525,525] | [50,1240]；[-420,420] | [0,0] | `robot_probe/hal_conf/linkhou_avoid_range.json:5`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:10` |
| 3/旋转前激光避障（6；0） | stop | [50,310]；[-325,325] | [50,248]；[-260,260] | [0,0] | `robot_probe/hal_conf/linkhou_avoid_range.json:85`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:90` |
| 3/旋转前激光避障（6；0） | warn2 | [50,310]；[-425,425] | [50,248]；[-340,340] | [0,0] | `robot_probe/hal_conf/linkhou_avoid_range.json:85`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:90` |
| 3/旋转前激光避障（6；0） | warn1 | [50,310]；[-525,525] | [50,248]；[-420,420] | [0,0] | `robot_probe/hal_conf/linkhou_avoid_range.json:85`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:90` |
| 5/旋转侧前激光避障（6；0） | stop | [50,310]；[-325,325] | [50,248]；[-260,260] | [0,0] | `robot_probe/hal_conf/linkhou_avoid_range.json:165`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:170` |
| 5/旋转侧前激光避障（6；0） | warn2 | [50,310]；[-425,425] | [50,248]；[-340,340] | [0,0] | `robot_probe/hal_conf/linkhou_avoid_range.json:165`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:170` |
| 5/旋转侧前激光避障（6；0） | warn1 | [50,310]；[-525,525] | [50,248]；[-420,420] | [0,0] | `robot_probe/hal_conf/linkhou_avoid_range.json:165`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:170` |
| 6/前视觉避障（12；2） | stop | [50,550]；[-325,325] | [50,440]；[-260,260] | [-180,180] | `robot_probe/hal_conf/linkhou_avoid_range.json:245`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:250` |
| 6/前视觉避障（12；2） | warn2 | [50,1050]；[-425,425] | [50,840]；[-340,340] | [-180,180] | `robot_probe/hal_conf/linkhou_avoid_range.json:245`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:250` |
| 6/前视觉避障（12；2） | warn1 | [50,1550]；[-525,525] | [50,1240]；[-420,420] | [-180,180] | `robot_probe/hal_conf/linkhou_avoid_range.json:245`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:250` |
| 7/后视觉避障（13；2） | stop | [50,550]；[-325,325] | [50,440]；[-260,260] | [-180,180] | `robot_probe/hal_conf/linkhou_avoid_range.json:325`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:330` |
| 7/后视觉避障（13；2） | warn2 | [50,1050]；[-425,425] | [50,840]；[-340,340] | [-180,180] | `robot_probe/hal_conf/linkhou_avoid_range.json:325`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:330` |
| 7/后视觉避障（13；2） | warn1 | [50,1550]；[-525,525] | [50,1240]；[-420,420] | [-180,180] | `robot_probe/hal_conf/linkhou_avoid_range.json:325`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:330` |

基准前激光 stop 顶点原文为 `(50,-340),(50,325),(277,325),(277,-340)`；在线参数为 `(50,-272),(50,260),(221,260),(221,-272)`。视觉 stop 基准远边 x=550，在线 x=440；warn2 远边 1050→840；warn1 远边 1550→1240。**当前区域与 0.8 因子完全匹配，只能定位为 level 1 或 level 3 的同形结果，未找到可区分两者的当前等级字段。**`robot_probe/hal_conf/linkhou_avoid_range.json:14`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:17`；`robot_probe/hal_static/avoid_level_table.txt:3`

完整区域列表已在本机按该缩放规则逐字段核验，相等比较为 True。`robot_probe/hal_static/geometry_crosscheck.txt:3`

### 1.3 `set_agv_col_ctrl(True/False)` 的实际实现

GDK 将 `ctrl` 原样写入 SetBool.data，发给 `/hal/nav/collision_override`。已安装 HAL 的实现：连接有效时，**True 直接成功返回，不执行恢复/启用动作；False 调用 SetAvoidRange(0)，将可缩放 x/y 清零（x=50 仍保留），随后定时约 2 秒，回调执行 SetAvoidRange(2) 恢复基准区域**。定时常量 `37888 + (30517<<16) = 2000000000 ns`。因此 False 不能当作永久关闭，True 不能当作立即恢复；连续 False 会取消并重新安排定时。未通过写接口实测物理效果，也不能据此认为触边或急停被屏蔽。`robot_probe/docs/slam_core.py:44`；`robot_probe/docs/slam_core.py:192`；`robot_probe/hal_static/vtable_selected.txt:4`；`robot_probe/hal_static/CollisionCtrl.disasm.txt:13`；`robot_probe/hal_static/CollisionCtrl.disasm.txt:28`；`robot_probe/hal_static/CollisionCtrl.disasm.txt:45`；`robot_probe/hal_static/CollisionCtrl.disasm.txt:46`；`robot_probe/hal_static/AvoidTimerCallback.disasm.txt:31`；`robot_probe/hal_static/avoid_level_table.txt:2`

### 1.4 传感器、启用关系与采样状态

| 配置传感器 | id/IP | 安装 x/y/z；theta | 区域引用 | 证据 |
|---|---|---|---|---|
| 前激光 | 6 / 10.42.0.122 | 223/0/0；0 | 区域 1/3/5 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:1163` |
| 前视觉避障相机 | 12 / 未填 | 300/0/272；0 | 区域 6 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:1059` |
| 后视觉避障相机 | 13 / 未填 | -300/0/62；180 | 区域 7 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:1092` |

前激光扫描角为 [-135,135]、频率 15、检测距离 [10,50000]、resolution=0.333333；配置 z=0 是坐标参数，**未找到离地安装高度的可信测量/标定值**。前视觉 z=272、后视觉 z=62；视觉区域 zRange=[-180,180] 不是“相机离地 ±180”。导航列表还引用“后激光” sensorId=7，但 sensorsParam.laserList 中只有 id=6，不能把该引用当作已安装的第二激光。ultrasonicList 为空。`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:1148`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:1059`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:1092`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:906`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:1254`

当前 globalStrategy 的前进/后退/左旋/右旋/左移/右移/所有情形 `rangeList=[]`；前停车 `[2]`、后停车 `[1]`；localStrtegy（响应拼写）“左关”为空。区域 id=2 不在现存 avoidRangeList 的 1/3/5/6/7 中。故存在**区域定义但运动策略未关联，且停车引用缺失区域**，需区分几何配置与实际激活关系。`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:413`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:474`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:6`

状态快照：`isAvoid=false`、`avoidStatus=0`、`avoidSensorList=[]`、`linearSpeed=0`、充电 true。HMI 用 isAvoid 显示“避障：是/否”，用 avoidSensorList 显示本次触发传感器；这不是传感器硬件清单，也不证明避障功能被全局关闭。`laserBeamList` 仅在 HMI 的 LaserState 数据处理代码中找到，所有本次查询 JSON 均未返回该字段；未调用 OpenLaserState/KeepLaserState，也未连接 WS。`robot_probe/hmi/api_AMR_GetState_id424.json.pretty.json:54`；`robot_probe/hmi/api_AMR_GetState_id424.json.pretty.json:6`；`robot_probe/hmi/api_AMR_GetState_id424.json.pretty.json:63`；`robot_probe/hmi/parameter_context.txt:1`；`robot_probe/hmi/parameter_context.txt:12`

## 2. 导航遇障：绕障、重规划还是停障

HMI 枚举 `0=路径导航、1=自主导航`；当前值 1。`isCanAvoidDetour` 的前端名称为“长时间避障绕行”开关，当前 false。因此可证实“自主导航已配置，长时间绕障绕行未开启”；不能把“自主导航”本身等同于遇障自动绕行。未找到独立 bypass/replan 开关或可证明在线局部重规划行为的日志。`robot_probe/hmi/parameter_context.txt:9`；`robot_probe/hmi/labels_context.txt:7`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:960`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:939`；`robot_probe/logs/query_summary.txt:3`

| 参数完整名 | 当前值 | 解读/单位 | 证据 |
|---|---|---|---|
| scheduleParam.navigationMode | `1` | 1=自主导航 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:960` |
| scheduleParam.isCanAvoidDetour | `false` | 绕行关闭 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:939` |
| scheduleParam.minDetourWaitAvoidTime | `10000` | 10000 ms=10秒；开关关闭时不能当作会触发绕行 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:956` |
| scheduleParam.minDetourBackLength | `0` | mm | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:955` |
| scheduleParam.maxDetourBackLength | `2500` | mm | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:946` |
| scheduleParam.maxDetourDistacne | `1800` | mm；原键拼写如此 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:947` |
| scheduleParam.avoidRange | `200` | mm；不是SDK等级 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:924` |
| scheduleParam.safeDistance | `100` | 原值，实际算法用途未验证 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:966` |
| scheduleParam.isCanBackward | `false` | 原始配置值；不等于现场行为验证 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:940` |
| scheduleParam.isCanShift | `false` | 原始配置值；不等于现场行为验证 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:941` |
| scheduleParam.angleTolerance | `2` | 原值；角度容差 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:923` |
| scheduleParam.maxSpeed | `1000` | 调度层 mm/s | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:953` |
| scheduleParam.maxCurveSpeed | `200` | 调度层 mm/s | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:945` |
| scheduleParam.maxEmptyVectorSpeed | `20` | 调度层原值，HMI不同页面单位有冲突 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:948` |
| scheduleParam.maxFullVectorSpeed | `20` | 调度层原值，HMI不同页面单位有冲突 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:949` |
| scheduleParam.maxTimeOut | `5000` | 原始配置值；不等于现场行为验证 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:954` |
| scheduleParam.minExplorDistance | `2000` | 原始配置值；不等于现场行为验证 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:957` |
| scheduleParam.minExplorNodeNum | `10` | 原始配置值；不等于现场行为验证 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:958` |
| scheduleParam.maxSendNodeNum | `10` | 原始配置值；不等于现场行为验证 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:952` |
| scheduleParam.length | `600` | mm | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:943` |
| scheduleParam.width | `450` | mm | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:970` |
| scheduleParam.rotateRadius | `375` | 原值 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:965` |
| scheduleParam.isHasCoordinate | `true` | 原始配置值；不等于现场行为验证 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:942` |
| scheduleParam.queueingStrategy | `0` | 原始配置值；不等于现场行为验证 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:963` |
| scheduleParam.acceleration | `150` | 原始配置值；不等于现场行为验证 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:922` |
| scheduleParam.deceleration | `150` | 原始配置值；不等于现场行为验证 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:929` |

| 运动控制/车体参数 | 当前值 | 证据 |
|---|---|---|
| moveControlParam.autoMoveParam.acc | `600` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:841` |
| moveControlParam.autoMoveParam.dec | `600` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:842` |
| moveControlParam.autoMoveParam.maxRotateSpeed | `200` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:844` |
| moveControlParam.autoMoveParam.maxSpeed | `200` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:845` |
| moveControlParam.autoMoveParam.rotateAcc | `200` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:847` |
| moveControlParam.autoMoveParam.rotateDec | `200` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:848` |
| moveControlParam.manualMoveParam.acc | `600` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:867` |
| moveControlParam.manualMoveParam.dec | `600` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:868` |
| moveControlParam.manualMoveParam.maxRotateSpeed | `200` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:870` |
| moveControlParam.manualMoveParam.maxSpeed | `200` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:871` |
| moveControlParam.manualMoveParam.rotateAcc | `200` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:873` |
| moveControlParam.manualMoveParam.rotateDec | `200` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:874` |
| moveControlParam.abutMoveParam.acc | `100` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:830` |
| moveControlParam.abutMoveParam.dec | `100` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:831` |
| moveControlParam.abutMoveParam.maxRotateSpeed | `200` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:833` |
| moveControlParam.abutMoveParam.maxSpeed | `200` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:834` |
| moveControlParam.abutMoveParam.rotateAcc | `100` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:836` |
| moveControlParam.abutMoveParam.rotateDec | `100` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:837` |
| moveControlParam.controlParam.accuracySpeed | `1` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:852` |
| moveControlParam.controlParam.accuracyTheta | `1` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:853` |
| moveControlParam.controlParam.accuracyX | `5` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:854` |
| moveControlParam.controlParam.accuracyY | `5` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:855` |
| moveControlParam.controlParam.angularCof | `1` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:856` |
| moveControlParam.controlParam.brakeDistance | `0` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:857` |
| moveControlParam.controlParam.horizontalCof | `1` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:858` |
| moveControlParam.controlParam.resolution | `0.02` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:861` |
| moveControlParam.defaultMode | `1`；与 navigationMode、GDK mode 为不同参数 | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:864` |
| chassisParam.head | `300` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:486` |
| chassisParam.mode | `0` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:488` |
| chassisParam.tail | `300` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:490` |
| chassisParam.wheelAngle | `24.3` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:492` |
| chassisParam.wheelDiameter | `152` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:493` |
| chassisParam.wheelDistance | `375` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:494` |
| chassisParam.wheelSpace | `375` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:495` |
| chassisParam.width | `450` | `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:496` |

当前 autoMoveParam.maxSpeed=200（HMI 线速度显示 mm/s，即 0.2 m/s）；maxRotateSpeed=200，HMI 手动滑杆代码先除以 1000 再 radToDeg，按该逻辑是 0.2 rad/s。HMI 参数页同时标 rad/s，状态页又把 angularSpeed 标为 mm/s，存在前端单位标注不一致，保留原始值，不把 200 直接解释成 200 rad/s。停车精度 accuracyX=5 mm、accuracyY=5 mm、accuracyTheta=1 度。车体 head/tail 各 300、width=450，对应配置外形长 600×宽450 mm；不包含机械臂或杯子的外伸包络。`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:840`；`robot_probe/hmi/parameter_context.txt:28`；`robot_probe/hmi/parameter_context.txt:16`；`robot_probe/hmi/parameter_context.txt:1`；`robot_probe/hmi/parameter_context.txt:18`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:485`

**停障结论：** HMI 与 IO 配置确有 stop=停车、warn=减速语义，支持停障/减速机制的存在；但本次前进等策略 rangeList 为空，又没有碰撞时的状态/命令日志，**未找到“当前携杯导航遇障一定停下”的证据**。同样未找到“当前会绕障重规划”的实测证据。`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:678`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:413`；`robot_probe/logs/query_summary.txt:2`

HAL 配置中 `config.yaml` 仅以 Genie.devices 的 chassis/linkhou 段选择底盘，没有独立 agv/nav/slam 配置段；linkhou_chassis_config.yaml 仅 6 行：125 Hz、host=10.42.0.153、ws=6001、http=6002、amr_id=102、avoid_range 文件名，未含导航模式/限速/到位容差/绕障键。**102 是该静态文件值，控制器实测设备序列号/Agv id 是 424**。备用配置 diff 显示 60→200 Hz、设备结构从旧的 Jaka/ctek 字典改为当前 Realman/多手型列表并新增底盘等，不应视为同结构导航备份。`robot_probe/hal_conf/config.yaml:40`；`robot_probe/hal_conf/linkhou_chassis_config.yaml:1`；`robot_probe/controller/api_AMR_GetDeviceInfo_id424.json.pretty.json:4`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:556`；`robot_probe/hal_conf/config_vs_backup.diff:5`

## 3. HMI 与控制器 API 路径表（写接口仅列出）

范围：已下载首页、4 个直接 JS 包及 manifest 列出的 47 个分包，共 51 个 JS。所有 `/api/` 原始出现项见 all_api_grep.tsv，字面路径去重见 all_api_paths.txt；动态路径保留模板。此表是**本次可读静态资源、HAL 二进制及成功查询中发现的完整相关路径集合**，不声称枚举了控制器所有未暴露接口。HTTP HMI 基址 `http://10.42.0.153:8080`；HAL HTTP 基址 `http://10.42.0.153:6002`；WS 基址 `ws://10.42.0.153:6001`，未连接 WS。`robot_probe/hmi/asset_manifest.json:1`；`robot_probe/hmi/all_api_grep.tsv:1`；`robot_probe/hal_conf/linkhou_chassis_config.yaml:3`

| 分类 | 完整路径/动态模板 | 类型与发现侧 | 本次调用 | 证据 |
|---|---|---|---|---|
| 建图 | `/api/AMR/StartMapping` | POST；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:48` |
| 建图 | `/api/AMR/StopMapping` | POST；HAL，HMI另有 StopSlam 旧通信调用 | 否 | `robot_probe/hal_static/controller_api_paths.txt:35` |
| 建图 | `/api/AMR/MappingData` | WS；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:37` |
| 地图 | `/api/Map/GetFloorMap` | POST；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:39` |
| 地图 | `/api/Map/UpdateFloorMap` | POST；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:38` |
| 地图 | `/api/Config/Map/GetParam` | POST；控制器查询+HAL | 是，仅 id424/{} | `robot_probe/hal_static/controller_api_paths.txt:32` |
| 地图 | `/api/Config/Map/AddParam` | POST；HAL+HMI配置模板 | 否 | `robot_probe/hal_static/controller_api_paths.txt:31` |
| 地图 | `/api/Config/Map/UpdateParam` | POST；HMI配置动态模板 | 否 | `robot_probe/hmi/all_api_occurrences.tsv:28` |
| 地图 | `/api/Config/Map/RemoveParam` | POST；HAL+HMI配置模板 | 否 | `robot_probe/hal_static/controller_api_paths.txt:37` |
| 重定位 | `/api/AMR/Relocation` | POST；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:47` |
| 避障/配置 | `/api/Config/Agv/GetParam` | POST；控制器查询+HAL | 是，仅 id424/{} | `robot_probe/hal_static/controller_api_paths.txt:10` |
| 避障/配置 | `/api/Config/AgvType/GetParam` | POST；控制器查询+HAL | 是，仅 id424/{} | `robot_probe/hal_static/controller_api_paths.txt:9` |
| 避障/配置 | `/api/Config/Agv/UpdateParam` | POST；HMI配置动态模板 | 否 | `robot_probe/hmi/all_api_occurrences.tsv:28` |
| 避障/配置 | `/api/Config/AgvType/UpdateParam` | POST；HAL+HMI配置模板 | 否 | `robot_probe/hal_static/controller_api_paths.txt:39` |
| 避障状态 | `/api/AMR/GetState` | POST；HMI | 是，8080 id424 | `robot_probe/hmi/all_api_occurrences.tsv:58` |
| 避障/激光 | `/api/AMR/LaserState` | WS；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:40` |
| 避障/激光 | `/api/AMR/OpenLaserState` | POST；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:44` |
| 避障/激光 | `/api/AMR/CloseLaserState` | POST；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:45` |
| 避障/激光 | `/api/AMR/KeepLaserState` | POST；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:46` |
| 导航模式 | `/api/AMR/SetAutoMode` | POST；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:42` |
| 导航模式 | `/api/AMR/SetManualMode` | POST；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:43` |
| 导航运动 | `/api/AMR/MoveToTargetPose` | POST；HAL | 否 | `robot_probe/hal_static/controller_api_paths.txt:23` |
| 导航运动 | `/api/AMR/MoveRelative` | POST；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:59` |
| 导航运动 | `/api/AMR/ManualMove` | POST；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:12` |
| 导航控制 | `/api/AMR/PauseWalk` | POST；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:56` |
| 导航控制 | `/api/AMR/ContinueWalk` | POST；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:57` |
| 导航回退 | `/api/AMR/EnterFallbackMode` | HAL 路径 | 否 | `robot_probe/hal_static/controller_api_paths.txt:40` |
| 导航回退 | `/api/AMR/ExitFallbackMode` | HAL 路径 | 否 | `robot_probe/hal_static/controller_api_paths.txt:41` |
| 导航回退 | `/api/AMR/Fallback` | HAL WS字符串 | 否 | `robot_probe/hal_static/controller_api_paths.txt:8` |
| 导航路径 | `/api/AMR/WalkPath` | WS；HMI | 否 | `robot_probe/hmi/all_api_occurrences.tsv:69` |
| 导航路径 | `/api/AMR/SendPath` | WS；HMI | 否 | `robot_probe/hmi/all_api_occurrences.tsv:70` |
| 导航路径 | `/api/AMR/PlanPath` | WS；HMI | 否 | `robot_probe/hmi/all_api_occurrences.tsv:71` |
| 导航状态 | `/api/AMR/State` | WS；HMI+HAL | 否 | `robot_probe/hmi/all_api_occurrences.tsv:9` |
| 导航目标 | `/api/Station/Navigation/GetAllStation` | POST；HMI | 否 | `robot_probe/hmi/all_api_occurrences.tsv:21` |
| 导航任务 | `/api/Task/CreateTask` | POST；HMI | 否 | `robot_probe/hmi/all_api_occurrences.tsv:4` |
| 导航任务 | `/api/Task/CancelTask` | POST；HMI | 否 | `robot_probe/hmi/all_api_occurrences.tsv:5` |
| 导航任务 | `/api/Task/DeleteTask` | POST；HMI | 否 | `robot_probe/hmi/all_api_occurrences.tsv:6` |
| 导航任务查询 | `/api/Task/GetAllTask` | POST；HMI | 否 | `robot_probe/hmi/all_api_occurrences.tsv:2` |
| 导航任务查询 | `/api/Task/GetTaskRecord` | POST；HMI | 否 | `robot_probe/hmi/all_api_occurrences.tsv:3` |
| 任务状态 | `/api/Task` | WS；HMI | 否 | `robot_probe/hmi/all_api_occurrences.tsv:15` |
| 总配置 | `/api/Config/GetAllParam` | POST；HMI | 是，8080 {} | `robot_probe/hmi/all_api_occurrences.tsv:27` |
| 系统配置 | `/api/Config/System/GetAllParam` | POST；HAL+控制器查询 | 是，仅 id424/{} | `robot_probe/hal_static/controller_api_paths.txt:12` |
| 设备信息 | `/api/AMR/GetDeviceInfo` | POST；HMI+HAL | 是，仅6002 id424/{} | `robot_probe/hmi/all_api_occurrences.tsv:53` |
| 配置广播 | `/api/Config` | WS；HMI | 否 | `robot_probe/hmi/all_api_occurrences.tsv:60` |
| 导航状态/路径 | `/api/AMR/AMR-{id}/State` | HMI动态 WS；WlakPath 为原包拼写 | 否 | `robot_probe/hmi/all_api_occurrences.tsv:9` |
| 导航状态/路径 | `/api/AMR/AMR-{id}/WalkPath` | HMI动态 WS；WlakPath 为原包拼写 | 否 | `robot_probe/hmi/all_api_occurrences.tsv:66` |
| 导航状态/路径 | `/api/AMR/AMR-{id}/WlakPath` | HMI动态 WS；WlakPath 为原包拼写 | 否 | `robot_probe/hmi/all_api_occurrences.tsv:76` |
| 导航状态/路径 | `/api/AMR/AMR-{id}/PlanPath` | HMI动态 WS；WlakPath 为原包拼写 | 否 | `robot_probe/hmi/all_api_occurrences.tsv:69` |
| 导航状态/路径 | `/api/AMR/AMR-{id}/SendPath` | HMI动态 WS；WlakPath 为原包拼写 | 否 | `robot_probe/hmi/all_api_occurrences.tsv:67` |

HMI 通用配置模板为 `/api/Config/{section}/AddParam`、`/api/Config/{section}/UpdateParam`、`/api/Config/{section}/RemoveParam`，其中 Agv/AgvType/Map 可由配置区名称实例化；未对实例化的写接口做可用性测试。未发现单独 `/api/.../SetAvoidRange` HTTP 路径：SetAvoidRange 是 HAL 内部方法，实际修改区域走 AgvType/UpdateParam。未发现专用切换地图 HTTP 路径，HAL 静态字符串显示地图操作与配置操作，SDK 则有 switch_map 服务。`robot_probe/hmi/all_api_occurrences.tsv:27`；`robot_probe/hal_static/SetAvoidRange.disasm.txt:305`；`robot_probe/docs/slam_core.py:154`

HMI 的 StopSlam 还通过 window.sendRequestToServer 的 moduleType/Operation 旧通信机制发起；其存在不等于在 JS 中找到了对应 HTTP 字面路径。WS 的 PlanPath 仅说明可显示规划路径，不能证明当前配置启用了动态避障重规划。`robot_probe/hmi/dynamic_api_context.txt:6`；`robot_probe/hmi/all_api_occurrences.tsv:71`

### 查询结果与业务错误

| 端口 | POST路径 | 请求体 | HTTP / 业务码 | 原始响应证据 |
|---|---|---|---|---|
| 6002 | `/api/AMR/GetDeviceInfo` | `{"id":424}` | 200 / 0（Success） | `robot_probe/controller/api_AMR_GetDeviceInfo_id424.json:1` |
| 6002 | `/api/AMR/GetDeviceInfo` | `{}` | 200 / 0（Success） | `robot_probe/controller/api_AMR_GetDeviceInfo_empty.json:1` |
| 6002 | `/api/Config/Agv/GetParam` | `{"id":424}` | 200 / 0（Success） | `robot_probe/controller/api_Config_Agv_GetParam_id424.json:1` |
| 6002 | `/api/Config/Agv/GetParam` | `{}` | 200 / 0（Success） | `robot_probe/controller/api_Config_Agv_GetParam_empty.json:1` |
| 6002 | `/api/Config/AgvType/GetParam` | `{"id":424}` | 200 / 2（Invalid Argument: Id Not Exist） | `robot_probe/controller/api_Config_AgvType_GetParam_id424.json:1` |
| 6002 | `/api/Config/AgvType/GetParam` | `{}` | 200 / 0（Success） | `robot_probe/controller/api_Config_AgvType_GetParam_empty.json:1` |
| 6002 | `/api/Config/Map/GetParam` | `{"id":424}` | 200 / 2（Invalid Argument: Id Not Exist） | `robot_probe/controller/api_Config_Map_GetParam_id424.json:1` |
| 6002 | `/api/Config/Map/GetParam` | `{}` | 200 / 0（Success） | `robot_probe/controller/api_Config_Map_GetParam_empty.json:1` |
| 6002 | `/api/Config/System/GetAllParam` | `{"id":424}` | 200 / 0（Success） | `robot_probe/controller/api_Config_System_GetAllParam_id424.json:1` |
| 6002 | `/api/Config/System/GetAllParam` | `{}` | 200 / 0（Success） | `robot_probe/controller/api_Config_System_GetAllParam_empty.json:1` |
| 8080 | `/api/AMR/GetState` | `{"id":424}` | 200 / 0（Success） | `robot_probe/hmi/api_AMR_GetState_id424.json:1` |
| 8080 | `/api/Config/GetAllParam` | `{}` | 200 / 0（Success） | `robot_probe/hmi/api_Config_GetAllParam_empty.json:1` |

AgvType 的 id424 是不存在的类型 ID，Map 的 id424 也不是地图 ID；这两次响应均为 `ErrorCode=2, Invalid Argument: Id Not Exist`，HTTP 200 不能视为查询成功。空请求分别返回类型字典与地图字典；AMR 实际 agvTypeId=1。`robot_probe/controller/api_Config_AgvType_GetParam_id424.json.pretty.json:4`；`robot_probe/controller/api_Config_Map_GetParam_id424.json.pretty.json:4`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:4`

控制器设备信息：serialNumber=424、softwareVersion=7.2.0.23、firmwareVersion=2.0.0.7；System/GetAllParam返回appVersion=5.1.0.11 Beta、serverVersion=633。`robot_probe/controller/api_AMR_GetDeviceInfo_empty.json.pretty.json:5`；`robot_probe/controller/api_Config_System_GetAllParam_empty.json.pretty.json:9`

## 4. 地图列表、HMI mapId 与控制器 mapId

| id/name | areaId | floorId | floorParams.mapId | 证据 |
|---|---:|---:|---:|---|
| 2/ssd | 2 | 1 | 2 | `robot_probe/controller/api_Config_Map_GetParam_empty.json.pretty.json:53` |
| 5/444 | 3 | 1 | 5 | `robot_probe/controller/api_Config_Map_GetParam_empty.json.pretty.json:106` |
| 6/CUP | 4 | 1 | 6 | `robot_probe/controller/api_Config_Map_GetParam_empty.json.pretty.json:159` |

HMI 总配置 Agv[0] 为 `id=424,name=AMR-424,agvTypeId=1,mapId=6`；类型 1 的 mapId 同样为6。6002 的 Agv/GetParam(id424) 也返回 mapId=6。HMI 与控制器地图列表均为 2/ssd、5/444、6/CUP。状态 GetState 不含独立 mapId 字段，仅有 nodeId=`6-1-1000003` 与 pose.z=1，因此“当前加载为地图6”有配置和节点前缀支持，未获得 Position.map_id 的运行时订阅快照。`robot_probe/hmi/current_config_extract.txt:2`；`robot_probe/hmi/current_config_extract.txt:3`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:827`；`robot_probe/hmi/current_config_extract.txt:6`；`robot_probe/hmi/api_AMR_GetState_id424.json.pretty.json:79`

## 5. proto/消息注释、SDK 服务路径与单位

`/home/agi/app` 是 `/app/ota/a2d/latest/app` 的符号链接。首次 find 未跟随链接为空；find -L 后只有 header/image_preprocess/mode_switch/joint_state 四个 `.proto`。追加搜索 home、备份目录及两个 GDK 安装包，也未找到可读的 AGV/Nav/Slam 原始 `.proto` 或独立 GDK PDF/HTML/Markdown 手册。已拉取 ROS `.msg/.srv`（含注释）、20 个 generated pb2 与安装 GDK Python 源码；pb2 的序列化描述只用本机 AST 解析，descriptor.txt **不含原始注释**，不冒称原始 proto。`robot_probe/inventory/app_location.txt:1`；`robot_probe/inventory/proto_paths_follow.txt:1`；`robot_probe/inventory/extra_proto_docs.txt:1`；`robot_probe/inventory/installer_members.txt:1`；`robot_probe/inventory/server_installer_members.txt:1`；`robot_probe/docs/METADATA:4`；`robot_probe/protos/generated_pb2/AGVCollisionLevel_pb2.descriptor.txt:1`

| 请求项 | 实际注释摘录/定义 | 单位与边界结论 | 证据 |
|---|---|---|---|
| AGVMappingControl.control | `1 开始建图 2 停止建图`；control=2 时 save_map=true 保存，false丢弃；map_name为地图名称 | 注释为 int32 数字约定，并非 proto enum 类型 | `robot_probe/protos/agv/AGVMappingControl.srv:3` |
| AGVCollisionLevel.collision_level | `避障等级1，2，3 ， 等级越高，越灵敏` | 注释与本版静态缩放表冲突，见第1节 | `robot_probe/protos/agv/AGVCollisionLevel.srv:3` |
| AGVModeControl.mode | `1: 手动, 2: 自动`；error_code=0成功 | 与控制器 navigationMode 0/1不是同一枚举 | `robot_probe/protos/agv/AGVModeControl.srv:3` |
| SetAGVInfo | float32 set_max_linear_speed / set_max_angular_speed / set_min_collision_distance | 消息定义没有单位注释。线/角速度按SDK周边代码及持久配置推断 m/s、rad/s；最小避障距离单位未找到明确证据 | `robot_probe/protos/agv/SetAGVInfo.msg:3` |
| ReLocalization | relocalization_mode：`暂时只支持手动重定位 1:手动重定位`；float32 x/y/theta | srv无单位注释；SDK原样透传；本版HAL未见1000或180/π转换，控制器侧坐标按HMI为mm/度，故本版请求使用mm/度是静态链路推断，未实测 | `robot_probe/protos/agv/ReLocalization.srv:3` |
| Position.map_id | `当前地图ID` | int32；当前位置 agv_pos_x/y/z为m、agv_angle为弧度；线速度m/s、角速度弧度/s | `robot_probe/protos/agv/Position.msg:23` |
| AGVTaskState.finish_state | `0: 未工作 1: 工作中` | uint32数字约定；未定义“2=成功”等完成状态，不能自行补枚举 | `robot_probe/protos/agv/AGVTaskState.msg:5` |
| NavigatePose | `/sdk/nav/navigate_to_pose #全局路径规划`；`/sdk/nav/move_relative #相对位置移动` | x/y/theta无单位注释；与Position状态的m/rad不能混用，需看当前HAL实现 | `robot_probe/protos/agv/NavigatePose.srv:1` |

Position 的定位/速度单位逐字段原文见 `robot_probe/protos/agv/Position.msg:4`；`robot_probe/protos/agv/Position.msg:7`；`robot_probe/protos/agv/Position.msg:12`；`robot_probe/protos/agv/Position.msg:13`。

单位的补充证据：SDK map_builder CLI 明示线速度 m/s、角速度 rad/s；持久配置 `/data/.chassis/config.yaml` 的 max_linear_speed 与拼写 max_angular_spped 都约0.2；HAL SetChassisConfig 两条限速链把输入乘1000后向零取整写入maxSpeed/maxRotateSpeed。该函数没有发现将最小避障距离写入避障多边形的实现，不能用 `set_min_collision_distance` 替代等级/多边形设置。ReLocalization 的SDK函数原样赋值，HAL回调原样传 float，linkhou Relocation 对 x/y/z/theta直接转整数，再POST /api/AMR/Relocation；没有证据证明会把m/rad自动转换成mm/度。`robot_probe/docs/map_builder.py:120`；`robot_probe/docs/map_builder.py:121`；`robot_probe/hal_conf/persisted_chassis_config.yaml:1`；`robot_probe/hal_static/SetChassisConfig.disasm.txt:395`；`robot_probe/hal_static/SetChassisConfig.disasm.txt:429`；`robot_probe/docs/slam_core.py:176`；`robot_probe/hal_static/HandleReLocalization.disasm.txt:22`；`robot_probe/hal_static/Relocation.disasm.txt:62`；`robot_probe/hal_static/Relocation.disasm.txt:116`

SDK 当前实际服务路径（只读源码，不调用）：`robot_probe/docs/slam_core.py:34`

| SDK能力 | 服务/主题路径 | 证据 |
|---|---|---|
| switch_nav_mode_topic | `/hal/switch_nav_mode` | `robot_probe/docs/slam_core.py:33` |
| mapping_control_topic | `/hal/slam/mapping_control` | `robot_probe/docs/slam_core.py:34` |
| pub_map_topic | `/hal/map` | `robot_probe/docs/slam_core.py:35` |
| get_map_topic | `/hal/map/get_map` | `robot_probe/docs/slam_core.py:36` |
| map_list_topic | `/hal/map/map_list` | `robot_probe/docs/slam_core.py:37` |
| delete_map_topic | `/hal/map/remove_map` | `robot_probe/docs/slam_core.py:38` |
| switch_map_topic | `/hal/map/switch_map` | `robot_probe/docs/slam_core.py:39` |
| navigate_topic | `/hal/nav/navigate_to_pose` | `robot_probe/docs/slam_core.py:40` |
| move_to_relative_topic | `/hal/nav/move_relative` | `robot_probe/docs/slam_core.py:41` |
| reloc_topic | `/hal/loc/relocalize` | `robot_probe/docs/slam_core.py:42` |
| set_agv_col_level_topic | `/hal/nav/collision` | `robot_probe/docs/slam_core.py:43` |
| set_agv_col_ctrl_topic | `/hal/nav/collision_override` | `robot_probe/docs/slam_core.py:44` |
| pause_walk_topic | `/hal/nav/pause_walk` | `robot_probe/docs/slam_core.py:47` |
| set_chassis_info_topic | `/hal/set_chassis_info` | `robot_probe/docs/slam_core.py:48` |
| get_chassis_info_topic | `/hal/chassis_info` | `robot_probe/docs/slam_core.py:49` |
| chassis_position_topic | `/hal/position` | `robot_probe/docs/slam_core.py:50` |
| chassis_battery_status_topic | `/hal/batt_state` | `robot_probe/docs/slam_core.py:51` |

注释一致性限制：Position 的 IMU 加速度/角速度注释写“单位(m)”，显然不足以形成可信物理单位契约；本报告只确认其中明确且合理的定位/线角速度单位，不按这两类错误注释推导传感器物理量。`robot_probe/protos/agv/Position.msg:14`；`robot_probe/protos/agv/Position.msg:17`

## 6. HAL 真实行为、网络与未知项

HAL 日志实际在 `/data/logs`，而 `/home/agi/app/log` 不存在；app/log.log 只有谐波驱动初始化。最新 HAL symlink 指向 hal.log.INFO.20261013-222454.3800.0，但文件内当前内容含 2026-10-14 11:07 服务创建与11:46 detail_info；文件名日期不能当作每行日期。当前 detail_info 中的 0.8 区域、空策略列表、mapId=6 与查询快照一致，属于真实HAL配置读取日志。`robot_probe/inventory/log_listing.txt:1`；`robot_probe/inventory/sdk_logs_locations.txt:75`；`robot_probe/logs/recent_hal.txt:1`；`robot_probe/logs/recent_hal.txt:42`

已对全部可见 HAL 文件筛选 2026-10-13 09:2x 与任务关键词，返回0行；显式 MoveToTargetPose/Relocation/Start/StopMapping 与避障设置失败日志也返回0行。另有 2026-10-13 15:0x 的 mapping_data_ 流（旧日志）；这是历史数据读取证据，不证明本次仍在建图。无法从现有日志重建“携杯碰撞”的导航命令、避障状态或停止原因。`robot_probe/logs/query_summary.txt:2`；`robot_probe/logs/query_summary.txt:3`；`robot_probe/logs/behavior_hal_tail.txt:5`

网络只读证据：wlan0=192.168.3.30/24、xfi0.10=10.42.0.101/24，10.42.0.0/24 经xfi0.10直连；默认路由经192.168.3.1/wlan0。与HAL host=10.42.0.153相符。`robot_probe/inventory/network.txt:2`；`robot_probe/inventory/network.txt:5`；`robot_probe/hal_conf/linkhou_chassis_config.yaml:2`

未知/证据不足项：

- 当前level到底是1还是3；两者0.8区域同形，未查询ROS服务。 `robot_probe/hal_static/avoid_level_table.txt:3`；`robot_probe/hal_static/avoid_level_table.txt:5`
- 等级3与注释冲突的供应商设计意图、是否为实现错误；静态文件与内存运行版本完全一致性。 `robot_probe/protos/agv/AGVCollisionLevel.srv:3`；`robot_probe/hal_static/runtime_identity.txt:1`
- 空rangeList与缺失id2引用是否导致所有对应方向不触发避障；控制器如何处理无效引用。 `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:413`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:6`
- 当前遇障是否实际减速/停车、是否有未暴露的局部重规划；未做动作/障碍实验。 `robot_probe/logs/query_summary.txt:3`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:939`
- 携杯碰撞事件09:2x的具体日志、传感器触发、目标姿态与停止原因。 `robot_probe/logs/query_summary.txt:2`
- 前激光物理离地高度、后激光id7是否真实存在、视觉避障实时可用性。 `robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:1148`；`robot_probe/controller/api_Config_Agv_GetParam_id424.json.pretty.json:906`
- 实时laserBeamList点云、控制器原生当前mapId独立状态字段、Position.map_id快照。 `robot_probe/hmi/parameter_context.txt:12`；`robot_probe/hmi/api_AMR_GetState_id424.json.pretty.json:79`
- 原始AGV/Nav/Slam.proto的注释及完整GDK手册；finish_state超过0/1的定义。 `robot_probe/inventory/proto_paths_follow.txt:1`；`robot_probe/protos/agv/AGVTaskState.msg:5`
- SetAGVInfo最小避障距离的单位/是否生效、速度字段具体ABI绑定与所有控制器角速度字段单位。 `robot_probe/protos/agv/SetAGVInfo.msg:1`；`robot_probe/hal_static/SetChassisConfig.disasm.txt:1`；`robot_probe/hmi/parameter_context.txt:1`
- 重定位/导航请求单位的供应商书面契约；本报告mm/度结论为当前静态调用链推断，未调用写接口实测。 `robot_probe/protos/agv/ReLocalization.srv:1`；`robot_probe/hal_static/Relocation.disasm.txt:62`
- 未出现在静态资源中的控制器导航/地图接口及其参数约束；HMI旧通信module/operation与HTTP接口的全部对应关系。 `robot_probe/hmi/dynamic_api_context.txt:6`

取证操作边界：所有远端命令仅目录/文件读取、grep/tail、Python标准库归档解析，以及ip信息和curl；12个POST全部是Get开头，请求体仅id424或{}，每次curl均-m8；其余请求是HMI静态JS/HTML GET。无sudo、远端文件写入、服务重启、ROS/GDK发布、机器人可执行程序运行。未访问 `/home/yirong/g2_gdk`。原始文件总清单与哈希另存 robot_probe/file_sha256.txt。


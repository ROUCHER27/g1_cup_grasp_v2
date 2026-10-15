# genie_msgs_pb 0.8.0 消息/服务字段表（2026-10-14，由 *_pb2 descriptor 导出；原始 .proto 注释见 outputs/navigation_contract/robot_probe/protos/agv/*.srv|*.msg）

包路径：/home/yirong/g1_cup_grasp_v2/.venv/lib/python3.10/site-packages/genie_msgs_pb

## AGVMappingControl  (`srv/AGVMappingControl_pb2.py`)
### message AGVMappingControl_Request
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| header | 242399333 | .std_msgs.msg.pb.Header | optional |
| control | 232475539 | sfixed32 | optional |
| save_map | 420968672 | bool | optional |
| map_name | 218887410 | string | optional |

### message AGVMappingControl_Response
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| error_code | 325256566 | sfixed32 | optional |

注释：pb2 不含原始注释（见 robot_probe/protos/agv/ 的 .srv/.msg）。

## AGVCollisionLevel  (`srv/AGVCollisionLevel_pb2.py`)
### message AGVCollisionLevel_Request
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| header | 242399333 | .std_msgs.msg.pb.Header | optional |
| collision_level | 314440847 | sfixed32 | optional |

### message AGVCollisionLevel_Response
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| error_code | 325256566 | sfixed32 | optional |

注释：pb2 不含原始注释（见 robot_probe/protos/agv/ 的 .srv/.msg）。

## AGVModeControl  (`srv/AGVModeControl_pb2.py`)
### message AGVModeControl_Request
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| header | 242399333 | .std_msgs.msg.pb.Header | optional |
| mode | 399137592 | sfixed32 | optional |

### message AGVModeControl_Response
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| error_code | 325256566 | sfixed32 | optional |

注释：pb2 不含原始注释（见 robot_probe/protos/agv/ 的 .srv/.msg）。

## SetAGVInfo  (`msg/SetAGVInfo_pb2.py`)
### message SetAGVInfo
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| header | 242399333 | .std_msgs.msg.pb.Header | optional |
| set_max_linear_speed | 77512538 | float | optional |
| set_max_angular_speed | 421687719 | float | optional |
| set_min_collision_distance | 514486519 | float | optional |

注释：pb2 不含原始注释（见 robot_probe/protos/agv/ 的 .srv/.msg）。

## AGVInfo  (`msg/AGVInfo_pb2.py`)
### message AGVInfo
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| header | 242399333 | .std_msgs.msg.pb.Header | optional |
| max_linear_speed | 482937344 | float | optional |
| max_angular_speed | 358851820 | float | optional |
| min_collision_distance | 59749543 | float | optional |

注释：pb2 不含原始注释（见 robot_probe/protos/agv/ 的 .srv/.msg）。

## ReLocalization  (`srv/ReLocalization_pb2.py`)
### message ReLocalization_Request
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| header | 242399333 | .std_msgs.msg.pb.Header | optional |
| relocalization_mode | 79222855 | sfixed32 | optional |
| x | 215755280 | float | optional |
| y | 467354973 | float | optional |
| theta | 186158578 | float | optional |

### message ReLocalization_Response
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| error_code | 325256566 | sfixed32 | optional |

注释：pb2 不含原始注释（见 robot_probe/protos/agv/ 的 .srv/.msg）。

## NavigatePose  (`srv/NavigatePose_pb2.py`)
### message NavigatePose_Request
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| header | 242399333 | .std_msgs.msg.pb.Header | optional |
| x | 215755280 | float | optional |
| y | 467354973 | float | optional |
| theta | 186158578 | float | optional |

### message NavigatePose_Response
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| error_code | 325256566 | sfixed32 | optional |

注释：pb2 不含原始注释（见 robot_probe/protos/agv/ 的 .srv/.msg）。

## Position  (`msg/Position_pb2.py`)
### message Position
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| header | 242399333 | .std_msgs.msg.pb.Header | optional |
| agv_status | 281887280 | sfixed32 | optional |
| position_conf | 124496863 | float | optional |
| agv_pos_x | 176305748 | float | optional |
| agv_pos_y | 495274407 | float | optional |
| agv_pos_z | 76319661 | float | optional |
| agv_angle | 66904363 | float | optional |
| odom_x | 73783734 | float | optional |
| odom_y | 325254765 | float | optional |
| odom_z | 174833355 | float | optional |
| odom_angle | 439907344 | float | optional |
| linear_speed | 396659941 | float | optional |
| angular_speed | 499165955 | float | optional |
| acc_x | 42945669 | float | optional |
| acc_y | 361238624 | float | optional |
| acc_z | 209809754 | float | optional |
| gyro_x | 296835030 | float | optional |
| gyro_y | 112626773 | float | optional |
| gyro_z | 532622599 | float | optional |
| roll | 246758048 | float | optional |
| pitch | 127911595 | float | optional |
| yaw | 384186525 | float | optional |
| motor_states | 160732956 | .genie_msgs.msg.pb.MotorState | repeated |
| agv_task_state | 342191022 | .genie_msgs.msg.pb.AGVTaskState | optional |

注释：pb2 不含原始注释（见 robot_probe/protos/agv/ 的 .srv/.msg）。

## AGVTaskState  (`msg/AGVTaskState_pb2.py`)
### message AGVTaskState
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| header | 242399333 | .std_msgs.msg.pb.Header | optional |
| task_uuid | 46980179 | fixed32 | optional |
| task_reqid | 96417597 | string | optional |
| curr_station_idx | 124179179 | sfixed32 | optional |
| finish_state | 170051241 | fixed32 | optional |

注释：pb2 不含原始注释（见 robot_probe/protos/agv/ 的 .srv/.msg）。

## AGVMapList  (`srv/AGVMapList_pb2.py`)
### message AGVMapList_Request
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| header | 242399333 | .std_msgs.msg.pb.Header | optional |

### message AGVMapList_Response
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| map_id | 331705119 | sfixed32 | repeated |
| map_name | 218887410 | string | repeated |
| error_code | 325256566 | sfixed32 | optional |

注释：pb2 不含原始注释（见 robot_probe/protos/agv/ 的 .srv/.msg）。

## AGVSwitchMap  (`srv/AGVSwitchMap_pb2.py`)
### message AGVSwitchMap_Request
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| header | 242399333 | .std_msgs.msg.pb.Header | optional |
| map_id | 331705119 | sfixed32 | optional |

### message AGVSwitchMap_Response
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| error_code | 325256566 | sfixed32 | optional |

注释：pb2 不含原始注释（见 robot_probe/protos/agv/ 的 .srv/.msg）。

## AGVGetMap  (`srv/AGVGetMap_pb2.py`)
### message AGVGetMap_Request
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| header | 242399333 | .std_msgs.msg.pb.Header | optional |
| map_id | 331705119 | sfixed32 | optional |

### message AGVGetMap_Response
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| x | 215755280 | sfixed32 | optional |
| y | 467354973 | sfixed32 | optional |
| width | 203053244 | sfixed32 | optional |
| height | 357434455 | sfixed32 | optional |
| name | 505645505 | string | optional |
| data | 234097369 | string | optional |
| error_code | 325256566 | sfixed32 | optional |

注释：pb2 不含原始注释（见 robot_probe/protos/agv/ 的 .srv/.msg）。

## BatteryStatus  (`msg/BatteryStatus_pb2.py`)
### message BatteryStatus
| 字段 | 编号 | 类型 | label |
|---|---|---|---|
| header | 242399333 | .std_msgs.msg.pb.Header | optional |
| energy | 387026206 | float | optional |
| voltage | 228124592 | float | optional |
| current | 485707626 | float | optional |
| capacity | 367577322 | float | optional |
| temperature | 508445154 | float | optional |
| status | 453014720 | uint32 | optional |

注释：pb2 不含原始注释（见 robot_probe/protos/agv/ 的 .srv/.msg）。

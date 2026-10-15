# source 此文件以在 5090 笔记本上运行 G1 工具
source /home/yirong/g1_gdk/a2d_sdk/env.sh
source /home/yirong/g1_cup_grasp_v2/.venv/bin/activate
export PYTHONPATH=/home/yirong/g1_cup_grasp_v2/src
# 本工作站 enp5s0 的 G1 有线地址；不能沿用其他采集机的 10.42.0.10。
export LOCATOR_IP=10.42.0.102
export AORTA_DISCOVERY_URI=http://10.42.0.101:2379
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
export DYLOG_DEFAULT_LEVEL=FATAL
# env.sh 会把 ROS humble 的 site-packages 加入 PYTHONPATH，其 launch_testing pytest 插件缺 lark；禁用插件自动加载
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

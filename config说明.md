# Configuration 说明 (config.json)

本说明文档对应项目根目录下的 `config.json`，列出所有字段及含义，确保 `config.json` 保持纯净 JSON 格式，所有注释信息集中在此文档中。

---

## serial

- **port**: 串口设备路径，字符串。例如：`"/dev/ttyUSB0"`
- **baudrate**: 波特率，整数。例如：`115200`

## loop_rate

- 主循环频率（Hz），数字。例如：`10`

## transforms

### lidar_to_vehicle

- **translation**: 平移向量 `[x, y, z]`，单位米
- **rotation**: 旋转向量 `[roll, pitch, yaw]`，单位弧度
- **parent_frame**: 父坐标系名称，字符串
- **child_frame**: 子坐标系名称，字符串

> 注意：在 `config.json` 中，此处需填入完整的 Transform 对象字典，或填 `null` 表示暂不使用。

## map

- **grid_size**: 栅格地图分辨率（米），数字
- **size**: 地图尺寸 `[width, height]`，栅格数量

## tracker

- **max_age**: 目标跟踪中保留未匹配目标的最大时间（秒），数字

## planning

- **goal**: 规划目标点坐标 `[x, y]`，数字数组
- **num_points**: 平滑路径点数，整数

## control

- **method**: 控制方式，字符串，支持：
  - `"pure_pursuit"`：纯追踪算法
  - `"mpc"`：模型预测控制
- **lookahead_distance**: 前瞻距离（米），数字，仅 `pure_pursuit` 有效
- **desired_speed**: 期望速度（米/秒），数字，仅 `pure_pursuit` 有效
- **mpc_horizon**: MPC 预测步数，整数，仅 `mpc` 有效
- **mpc_dt**: MPC 时间步长（秒），数字，仅 `mpc` 有效

---

**使用说明**

- 请保持 `config.json` 为纯净的 JSON 文件，不要在其中添加注释。
- 如需调整参数，请直接编辑 `config.json` 或使用命令行 `--config` 指定其他配置文件。
- `main.py` 中使用标准 `json.load` 解析配置，确保加载过程无须额外依赖。

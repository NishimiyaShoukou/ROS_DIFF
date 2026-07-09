# URDF 参数编辑器

给初学者用的可视化网页，直接编辑 `my_urdf/robot.urdf.xacro` 里的几何参数，实时看俯视 / 侧视预览，撤销未保存修改、备份保存，并同步控制与导航参数。

## 打开

直接双击：

```text
TOOLS\urdf_editor\start_urdf_editor.bat
```

或在终端里：

```bash
python urdf_server.py --port 8766
```

浏览器访问 <http://127.0.0.1:8766/>。本工具与 `stm32f1/TOOLS/pid_tuner`（端口 8765）独立，可同时运行。

## 可编辑参数（12 项）

| 分组 | 参数 | 默认值 | 说明 |
|---|---|---|---|
| 车身尺寸 | `body_length` | 0.146 m | 板前后总长 |
| 车身尺寸 | `body_width` | 0.110 m | 板左右宽 |
| 车身尺寸 | `body_height` | 0.030 m | 板厚 |
| 前后位置 | `front_extent` | 0.122 m | 驱动轴到车前端 (+X) |
| 前后位置 | `rear_extent` | 0.024 m | 驱动轴到车后端 (-X) |
| 驱动轮 | `wheel_track` | 0.129 m | 两轮中心距 |
| 驱动轮 | `wheel_radius` | 0.024 m | 影响里程计算 |
| 驱动轮 | `wheel_width` | 0.019 m | 轮胎厚度 |
| 万向轮 | `caster_x` | 0.098 m | 驱动轴前方偏移 |
| 万向轮 | `caster_radius` | 0.010 m | 球体半径 |
| 雷达 | `laser_x` | 0.080 m | 驱动轴前方偏移 |
| 雷达 | `laser_z_from_axle` | 0.130 m | 相对驱动轴高度 |

## 联动

修改 `wheel_track` 或 `wheel_radius` 后，右侧 “联动 base_controller.yaml” 面板会实时显示推导值：

- `wheel_separation = wheel_track`
- `wheel_circumference = 2π × wheel_radius × 已有地面标定比例`

同步时会从当前 `base_controller.yaml` 与已加载的轮子半径计算并保留地面
标定比例。这样修改机器人模型尺寸不会意外丢失实车里程标定。

点 **同步控制与 footprint** 会修改：

- `config/base_controller.yaml` 中的上述两个字段
- `config/costmap_common.yaml` 中根据前后外廓、轮距和轮宽推导出的 `footprint`

其它字段不动；两个文件都会先备份。

## 备份策略

每次写入前，服务器把当前文件复制到 `<原文件名>.bak`，并轮换保留最近 3 份历史备份。例如：

```
my_urdf/robot.urdf.xacro
my_urdf/robot.urdf.xacro.bak
my_urdf/robot.urdf.xacro.bak.1
my_urdf/robot.urdf.xacro.bak.2
```

其中 `.bak` 是保存前的最近版本。如果改坏了想恢复：

```bash
copy my_urdf\robot.urdf.xacro.bak my_urdf\robot.urdf.xacro
```

## 坐标系

REP-103：

- `+X` 前
- `+Y` 左
- `+Z` 上

所有数值单位为米。输入框步长 0.001 m（即 1 mm）。超过合理区间会标红。

## 安全

`urdf_server.py` 默认只监听本机，并且只允许读写 `myrobot/` 包内、已存在且属于白名单后缀（`.xacro` / `.urdf` / `.yaml` / `.yml`）的文件。写入前会校验 XML/Xacro、拒绝跨站浏览器请求，并使用原子替换避免半写文件。

红框参数不能保存。`body_length` 必须等于 `front_extent + rear_extent`。

## 验证外观

修改并保存后，重新启动 rviz 看效果：

```bash
roslaunch myrobot remote_rviz.launch
```

或在已有 ROS 环境下：

```bash
rosrun xacro xacro.py $(rospack find myrobot)/my_urdf/robot.urdf.xacro
```

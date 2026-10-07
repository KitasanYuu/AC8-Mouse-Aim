# 能力矩阵（Capability Matrix）

本文件列出模组的能力及其实现状态。状态标签的定义见[文档总览](../README.md#状态标签)。

## 飞行

| 能力 | 状态 | 说明 |
|---|---|---|
| 鼠标指向飞控 | `Implemented` | 瞄准点保存在世界坐标；翻滚使升力对准瞄准点再拉杆；规则见[飞控规范](../02-architecture/flight-control.md) |
| 到位制动与改平 | `Implemented` | 按实测机体模型预测并提前收杆，接近瞄准点时改平机翼 |
| 精细选杆 | `Implemented` | 瞄准点 20° 内用机体模型向前模拟几组候选杆量，取代价最低者 |
| 移动瞄准点补正 | `Implemented` | 瞄准点自身的角速度作为前馈 |
| 按机型在线辨识 | `Implemented` | 默认关闭（`ident_memory=0`），按标定模型飞行 |
| 低空下沉时的拉起方式 | `Implemented` | 只改变到达方式，不改变瞄准点 |
| 失速机动辅助 | `Implemented` | 按住专用键时，高G力回转开始的约 0.04 秒俯仰回中再拉满；是否能进入取决于机型 |
| 防撞地、防失速、过载限制 | `Out-of-Scope` | 见[原则与边界](../00-overview/principles-and-boundaries.md) |
| 修改飞行模型、推力、伤害 | `Out-of-Scope` | 只写入玩家操纵输入 |

## 输入与让出

| 能力 | 状态 | 说明 |
|---|---|---|
| 鼠标原始输入捕获 | `Implemented` | 挂接游戏的 `GetRawInputData`；不可用时退回 DirectInput |
| 手动接管（键盘） | `Implemented` | 读取游戏的俯仰、翻滚、平移输入，≥ 0.5 视为玩家操作；改键后同样有效 |
| 手动接管（手柄） | `Experimental` | 规则相同，未实测 |
| 自动驾驶让出 | `Implemented` | 两个平移输入同时按住即为自动驾驶（游戏的专用键也如此）；松开后瞄准点回到机首 |
| 注视与剧情演出让出 | `Implemented` | 长按锁定约 0.35 秒后让出；已验证 `ImpactCamera` 与剧情镜头路径，不保证覆盖全部演出 |
| 暂停处理 | `Implemented` | 暂停时冻结瞄准点，恢复后对准画面中心（离机首过远时对准机首） |
| 开场与重新放置 | `Implemented` | 游戏镜头接管超过 1 秒、或机体被放置（瞬移超过 1 km）时瞄准点回到机首 |

## 镜头与视角

| 能力 | 状态 | 说明 |
|---|---|---|
| 追尾镜头 | `Implemented` | 距离、高度、跟随速度可调；距离 0 使用游戏自身位置 |
| 座舱／机首视角 | `Implemented` | 保留游戏的镜头位置；四种视角模式；鼠标摇杆 |
| 自由视角 | `Implemented` | 按住期间鼠标只转动镜头 |
| 自由视角后保持朝向 | `Experimental` | 可选：自由视角期间瞄准圈跟随镜头，松开时瞄准点移到镜头方向 |
| 视野角度（FOV）增减 | `Implemented` | 在游戏的视野角度上增减，保留游戏随速度的变化；追尾与座舱分别设置 |
| 视角交接过渡 | `Implemented` | 与游戏镜头互相交接时 0.5 秒过渡；相差过大时直接切换 |
| 设置变化过渡 | `Implemented` | 视野角度、镜头距离与高度等在约 1 秒内过渡 |
| 修改游戏保存的视角模式 | `Out-of-Scope` | 开局视角由游戏的存档决定 |

## 界面

| 能力 | 状态 | 说明 |
|---|---|---|
| 指示层 | `Implemented` | 桌面透明覆盖层；独占全屏下可能不可见 |
| 调参面板 | `Implemented` | Settings 与 Key Config 两页；写回 `config.ini`，下一帧生效 |
| 按键配置 | `Implemented` | 模组的全部按键在 `[keys]`，可在面板中录制 |
| 多语言 | `Implemented` | 英文、简体中文；默认跟随游戏语言；新增语言只需添加文件 |
| 状态行 | `Implemented` | 默认隐藏 |

## 开发与诊断

| 能力 | 状态 | 说明 |
|---|---|---|
| 配置热重载 | `Implemented` | 保存 `config.ini` 后下一次检查（0.5 秒内）生效 |
| 飞控逻辑热加载 | `Implemented` | 开发用 `ac8_flight_logic.dll`，不在正式安装中 |
| 飞行重建（遥测） | `Implemented` | 见[飞行重建](../04-development/flight-reconstruction.md) |
| 飞控对比台 | `Implemented` | 见[飞控对比台](../04-development/flight-bench.md) |
| 性能统计 | `Implemented` | 见[性能问题反馈](../03-usage/performance-reporting.md) |
| 运行时只读探测 | `Implemented` | 机库参数表、机体几何、镜头状态；仅在需要时运行 |
| 联机模式 | `Out-of-Scope` | 见[原则与边界](../00-overview/principles-and-boundaries.md#3-离线与安全) |

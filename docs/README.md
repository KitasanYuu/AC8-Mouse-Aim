# AC8 MouseFlight 文档

文档按职责分区。能力与实现状态的权威清单是[能力矩阵](01-capabilities/capability-matrix.md)，飞控行为以[飞控规范](02-architecture/flight-control.md)为准。

| 分区 | 主题 | 内容 |
|---|---|---|
| [00-overview](00-overview/) | 概览、原则与现状 | project-overview、principles-and-boundaries、current-status |
| [01-capabilities](01-capabilities/) | 能力清单与限制 | capability-matrix、limitations |
| [02-architecture](02-architecture/) | 实现方式 | architecture、flight-control、camera-and-views、input-and-takeover |
| [03-usage](03-usage/) | 安装与使用 | installation、controls、configuration、localization、performance-reporting |
| [04-development](04-development/) | 开发工具与流程 | build-and-deploy、flight-reconstruction、flight-bench |

## 状态标签

| 标签 | 含义 |
|---|---|
| `Implemented` | 仓库中存在可运行实现，并已在游戏中使用验证 |
| `Experimental` | 已实现，但游戏内验证不足，或只在部分条件下验证 |
| `Out-of-Scope` | 当前版本明确不做 |

“计划”“理论可行”不得写成 `Implemented`。自动检查（编译、数学检查、闭环仿真）不等于游戏内验证。

## 术语

文档、界面文字与日志说明统一使用以下用语。玩家可见的中文用语与游戏自身设置菜单保持一致。

| 用语 | 含义 | 不使用 |
|---|---|---|
| 瞄准点 | 鼠标指定的世界方向，飞控把机首带向它 | 目标方向、指针、准星 |
| 瞄准圈 | 画面上标出瞄准点的圆圈 | 鼠标圈、光标 |
| 机首标记 | 画面上标出机首实际朝向的小圆圈 | 机头十字 |
| 鼠标辅助 | 本模组的鼠标指向飞控整体，可用开关键关闭 | 鼠标模式（旧称） |
| 追尾镜头 | 游戏的第三人称追尾视角，及模组接管后的同一视角 | 尾追相机、跟随相机 |
| 座舱／机首视角 | 游戏的座舱视角与机首视角（无机身） | 近景视角、机头视角 |
| 鼠标摇杆 | 座舱／机首视角下，瞄准圈停在鼠标移到的屏幕位置、按偏离量持续转向的操作方式 | — |
| 视野角度（FOV） | 镜头的水平视野角度 | 视野、FOV（单独使用） |
| 高G力回转 | 游戏的高G转弯（加速与减速同时，或单独的高G键） | 高G、High-G |
| 失速机动 | 游戏的过失速机动（低于 500 km/h 时由高G力回转进入） | PSM、过失速 |
| 自动驾驶 | 游戏把机体回到水平的功能（同时按住左右平移键，或其专用键） | AutoPilot、autopilot |
| 手动接管 | 玩家按游戏的俯仰／翻滚／平移键时，该轴暂由玩家直接操控 | 键盘超控 |
| 注视 | 长按锁定键时游戏把镜头转向目标或剧情点 | 凝视 |
| 调参面板 | 游戏内的设置面板（Settings 与 Key Config 两页） | 调试面板 |
| 让出 | 模组暂停操控和镜头修正，交还给游戏 | 释放 |

# AC8 MouseFlight 文档

文档按职责分区。能力与实现状态的权威清单是[能力矩阵](01-capabilities/capability-matrix.md)，飞控行为以[飞控规范](02-architecture/flight-control.md)为准。

| 分区 | 主题 | 文档 |
|---|---|---|
| 00-overview | 概览、原则与现状 | [项目概览](00-overview/project-overview.md) · [原则与边界](00-overview/principles-and-boundaries.md) · [当前状态](00-overview/current-status.md) |
| 01-capabilities | 能力清单与限制 | [能力矩阵](01-capabilities/capability-matrix.md) · [已知限制与排查](01-capabilities/limitations.md) |
| 02-architecture | 实现方式 | [总体架构](02-architecture/architecture.md) · [飞控规范](02-architecture/flight-control.md) · [镜头与视角](02-architecture/camera-and-views.md) · [输入与接管](02-architecture/input-and-takeover.md) |
| 03-usage | 安装与使用 | [安装与卸载](03-usage/installation.md) · [操作](03-usage/controls.md) · [配置与调参面板](03-usage/configuration.md) · [多语言](03-usage/localization.md) · [问题反馈](03-usage/performance-reporting.md) |
| 04-development | 开发工具与流程 | [构建与调试部署](04-development/build-and-deploy.md) · [飞行重建](04-development/flight-reconstruction.md) · [飞控对比台](04-development/flight-bench.md) |

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

# AC8 MouseFlight

[English](README_EN.md) | 简体中文

**在线飞控对比：[Flight Bench 网页](https://kitasanyuu.github.io/AC8-Mouse-Aim/)**（浏览器打开，可回放三维画面）

为 **ACE COMBAT 8 离线单人模式**提供 War Thunder 式的鼠标指向飞控：用鼠标指定瞄准点，飞机按升力矢量规则把机首带过去。

> 仅供离线单人使用。不支持多人模式，不保证与其他输入、镜头或加载器模组兼容。游戏更新后，旧版本模组可能不再适用。
> 本项目为非官方模组，与游戏开发商、发行商及 War Thunder 无关联。

## 简介

模组只替换玩家的操纵输入并修正镜头，不修改飞行模型、推力、武器或存档：

- **鼠标辅助**：瞄准点保存在世界坐标中；飞控先翻滚使升力对准瞄准点，再拉杆，按实测机体模型提前收杆并改平。不设防撞地、防失速等限制，完全执行玩家的指向。
- **镜头**：追尾镜头平滑转向瞄准点；座舱／机首视角保留游戏的镜头，以鼠标摇杆操作并可朝瞄准圈转头；可增减视野角度（FOV）。
- **与游戏协作**：注视、剧情演出、自动驾驶期间让出操控与镜头；按游戏的键位识别手动接管。
- **调参面板**：游戏内调整常用设置与全部按键，立即生效；界面跟随游戏语言（英文、简体中文）。

## 当前状态

- 当前版本：`0.2.30+Alf.1.0.0`。`+` 前是分支所基于的上游版本，`Alf.` 后是本项目自己的版本号；变更见 [CHANGELOG](CHANGELOG.md)。
- 目标环境：Windows x64，ACE COMBAT 8 Steam Build 25201480。原生模块校验游戏与加载器签名，不匹配时拒绝启用。
- 游戏内验证只在开发者本机完成。权威状态见[当前状态](docs/00-overview/current-status.md)与[能力矩阵](docs/01-capabilities/capability-matrix.md)。

## 飞控对比

在线网页：**<https://kitasanyuu.github.io/AC8-Mouse-Aim/>**

网页中，本项目、上游 FletcherMiya 0.2.30 与 xsd467 pw.11 的飞控在同一个机体模型上，由同一个模拟飞行员飞同一组场景（通用机动、降落、多机缠斗、头目战），逐场景比较偏差、击落与综合代价，可回放三维画面。这是模拟结果，不等于游戏内体验；方法与局限见[飞控对比台](docs/04-development/flight-bench.md)。

## 快速开始

1. 完全退出游戏，下载（或 `git clone`）本仓库。
2. 运行 `AC8MouseFlight.cmd`，选择 **1** 安装。
3. 首次安装后，在 Steam 启动选项中填写（安装结束时会自动复制到剪贴板）：

   ```text
   cmd /d /c "set EOS_USE_ANTICHEATCLIENTNULL=1&& %command% -anticheat_settings=AC8MouseAim_Offline.json"
   ```

4. 启动游戏，选择 EXPERT 操作类型，进入单人任务。建议使用无边框窗口。

| 默认按键 | 功能 |
|---|---|
| 鼠标 | 移动瞄准点 |
| F（按住） | 自由视角 |
| F3 | 调参面板 |
| F8 | 鼠标辅助 ON/OFF |
| F9 | 瞄准点回到机首 |

全部操作见[操作](docs/03-usage/controls.md)；更新、切换到联机原版模式与卸载见[安装与卸载](docs/03-usage/installation.md)。**进入多人模式前，必须先切换到联机原版模式并清空启动参数。**

## 项目结构

```text
AC8MouseFlight.cmd   安装／更新、模式切换、启动参数、卸载
tools/               安装工具的实现
Payload/             装入游戏目录的文件（Lua 脚本、语言文件、配置、DLL、UE4SS）
src/                 原生模块：输入、飞控、镜头修正、指示层、调参面板、本地化
tests/               飞控数学检查、闭环仿真、Lua 检查
dev/                 构建、检查、调试部署、飞行重建、飞控对比台
docs/                文档
```

构建与调试见[构建与调试部署](docs/04-development/build-and-deploy.md)。

## 文档

- [文档总览](docs/README.md)（分区、状态标签、术语）
- [项目概览](docs/00-overview/project-overview.md) · [原则与边界](docs/00-overview/principles-and-boundaries.md) · [当前状态](docs/00-overview/current-status.md)
- [能力矩阵](docs/01-capabilities/capability-matrix.md) · [已知限制与排查](docs/01-capabilities/limitations.md)
- [总体架构](docs/02-architecture/architecture.md) · [飞控规范](docs/02-architecture/flight-control.md) · [镜头与视角](docs/02-architecture/camera-and-views.md) · [输入与接管](docs/02-architecture/input-and-takeover.md)
- [安装与卸载](docs/03-usage/installation.md) · [操作](docs/03-usage/controls.md) · [配置与调参面板](docs/03-usage/configuration.md) · [多语言](docs/03-usage/localization.md) · [问题反馈](docs/03-usage/performance-reporting.md)
- [构建与调试部署](docs/04-development/build-and-deploy.md) · [飞行重建](docs/04-development/flight-reconstruction.md) · [飞控对比台](docs/04-development/flight-bench.md)

## 致谢与许可

本项目分支自 [FletcherMiya/AC8-Mouse-Aim](https://github.com/FletcherMiya/AC8-Mouse-Aim)（原作者 Fletcher Gong，CC0 1.0）。原仓库提供了模组的基础：UE4SS 加载与离线启动方式、原生输入挂接、镜头修正与指示层；本项目在此之上重写了飞控，并加入了座舱／机首视角、调参面板、多语言等功能。

- [xsd467/AC8-Mouse-Aim](https://github.com/xsd467/AC8-Mouse-Aim)：pw.11（05d7f48）与 pw5 飞控，CC0 1.0，作为参考飞控收录于飞控对比台。
- [RE-UE4SS](https://github.com/UE4SS-RE/RE-UE4SS)：模组加载、Lua 运行环境与接口，MIT 许可。
- [MinHook](https://github.com/TsudaKageyu/minhook)：原生函数挂接，BSD 2-Clause 许可。

项目原创代码的许可见 `LICENSE.txt`，第三方组件见 `THIRD_PARTY_NOTICES.txt`。本项目不包含游戏资产或游戏可执行文件。

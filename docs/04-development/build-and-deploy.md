# 构建与调试部署（Build & Deploy）

## 环境

- Visual Studio 2022 的 MSVC x64 C++ 工具链、Windows SDK、Git、网络连接（首次构建获取依赖）。
- `dev\build.cmd` 先运行 `dev\Setup-Dependencies.ps1`，把 RE-UE4SS 固定在提交 `e3ba1016562d6c0868c410d0a71e88bfcdbf691b`，源码保存在不提交的 `deps/ue4ss-source/`，之后可离线复用。构建不编译、不替换随附的 `UE4SS.dll`。

## 构建与检查

```bat
dev\build.cmd
dev\check.cmd
dev\check_lua.cmd
```

| 命令 | 作用 |
|---|---|
| `build.cmd` | 生成 `build/ac8_mouse_aim_010.dll` 并复制到 `Payload/.../Scripts/`；同时生成开发用 `build/ac8_flight_logic.dll`。DLL 文件名为历史命名，不代表版本 |
| `check.cmd` | C++ 飞控数学检查与闭环仿真（`build/flight_sim.exe`）；标称机体下所有标准机动须收敛且过冲 < 3° |
| `check_lua.cmd` | 用固定的 LuaRaw 源码编译 Lua，检查全部 Lua 脚本的语法与镜头数学 |

上述检查不能替代游戏内验证。闭环仿真可单独试参数：`build\flight_sim.exe pitch_gain=3 roll_brake=0.3`，扫参：`build\flight_sim.exe sweep`。

## 版本号

格式为 `<上游版本>+Alf.<本项目版本>`，例如 `0.2.30+Alf.1.0.0`，定义在 `src/version.h`（`MOUSEFLIGHT_UPSTREAM`、`MOUSEFLIGHT_RELEASE`），状态行与日志从这里读取。发布时提高本项目版本，在 [CHANGELOG](../../CHANGELOG.md) 记录变更并打标签 `alf-v<本项目版本>`；并入新的上游版本时改上游部分。

## 调试部署

退出游戏后运行一次：

```bat
dev\Dev-Deploy.cmd -GamePath "D:\SteamLibrary\steamapps\common\ACE COMBAT 8"
```

- 目标目录尚未安装时，构建后完成首次安装；已有其他来源的 UE4SS 或加载器时拒绝覆盖。首次安装后仍需设置 Steam 离线启动参数（见[安装与卸载](../03-usage/installation.md)）。
- 之后直接运行 `dev\Dev-Deploy.cmd`：记住游戏目录，构建后只同步内容有变化的 DLL 与 Lua 脚本。
- `-IncludeConfig`：同时同步仓库的 `config.ini`（默认保留游戏中的配置）。
- `-NoBuild`：只改 Lua 时跳过构建。`-Preview`：列出将更新的文件，不构建。
- `-Rollback`：恢复最近一次同步前的版本（备份在不提交的 `build/dev-backups/`）。
- 调试部署要求游戏已退出，并校验已安装的 UE4SS 和加载器；不重装框架，不修改 Steam 启动选项。

DLL 与 `main.lua` 之间按固定的参数个数交换数据，两者必须同时更新；只更新其中之一时，原生模块会拒绝每一帧的数据。

## 运行中调试

- **调参数**：修改游戏中 `config.ini` 并保存，0.5 秒内生效；或使用调参面板。`[tuning]` 即开发配置，确定的值要收进仓库的 `config.ini`。
- **改飞控逻辑**（`src/maneuver.h`、`src/flight_logic.h`）：游戏运行中执行 `dev\Dev-Deploy.cmd -Live`，替换开发用 `ac8_flight_logic.dll`，约 1 秒内热加载，日志记录 `flight logic: hot-loaded generation N`；接口不兼容时拒绝并保留当前逻辑。修改挂接、桥接、Lua 或 `LogicInput`/`LogicOutput` 结构仍需退出游戏后正常同步。
- 开发用 `ac8_flight_logic.dll` 留在游戏目录时，每次启动都会替换内置逻辑；正式使用前应删除（日志中出现 `hot-loaded generation 1` 即说明存在）。

## 目录

```text
AC8MouseFlight.cmd   玩家入口：安装／更新、模式切换、启动参数、卸载
tools/               AC8MouseFlight.cmd 的实现
Payload/             装入游戏目录的文件（Lua 脚本、语言文件、配置、DLL、UE4SS）
src/                 原生模块：输入、飞控、镜头修正、指示层、调参面板、本地化、诊断
tests/               飞控数学检查、闭环仿真、Lua 检查
dev/                 构建、检查、调试部署、飞行重建（telemetry/）、飞控对比台（compare/）
docs/                文档
recordings/          遥测录像（不提交）
backups/             安装工具的备份（不提交）
```

## 约定

- 不通过移除校验来适配未知游戏或加载器版本。
- 自上游 0.2.30 起通过 UE4SS 导出的 LuaMadeSimple 接口交换数值，不直接操作 Lua 虚拟机内部结构。
- 提交说明简短，一两句概括；集成使用 rebase 与快进合并。
- 游戏数据（`dev/compare/aircraft/aircraft.txt`、探测输出、设置菜单文字）不提交。

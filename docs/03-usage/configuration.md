# 配置与调参面板（Configuration）

安装后的配置文件：

```text
Game/Binaries/Win64/UE4SS/Mods/AC8MouseAim/config.ini
```

保存后在 0.5 秒内生效，不需要重启游戏；调参面板的修改在下一帧生效。常用设置建议直接在调参面板中调整。

## 分段

| 分段 | 内容 |
|---|---|
| `[control]` | 鼠标灵敏度、追尾镜头、座舱／机首视角、自由视角、视野角度、语言、状态行、遥测端口、版本适配项 |
| `[keys]` | 模组的全部按键 |
| `[tuning]` | 飞控参数，见[飞控规范](../02-architecture/flight-control.md#调参入口) |

`[control]` 中的 `roll_gain`、`pitch_gain`、`yaw_gain`、`turn_pull`、`max_bank`、`smoothing`、`*_damping`、`dead_zone`、`horizontal_fov`、`vertical_fov` 为旧版本的兼容项，当前不起作用。输入槽位（`pitch_slot`、`roll_slot`）和轴符号（`*_sign`）属于版本适配设置，正常情况下不应修改。

## 调参面板

飞行中按调参面板键（默认 F3）打开。面板只在指示层显示时出现；暂停、注视、自动驾驶期间随指示层隐藏，按键交还游戏。面板打开并显示时，面板用到的按键不会传给游戏；录制按键期间所有按键都不会传给游戏。

### 设置页

| 分组 | 项目 | 对应键名 |
|---|---|---|
| 鼠标与追尾镜头 | 鼠标灵敏度、镜头追踪速度、镜头距离、镜头高度、自由视角后保持朝向、FOV 修正 | `sensitivity`、`camera_follow`、`camera_distance`、`camera_height`、`free_look_keep`、`camera_fov_add` |
| 座舱／机首视角 | 视角模式、转头比例、转头上限、鼠标倍率、瞄准圈曲线、视角延迟、视角水平修正、FOV 修正 | `near_view_camera`、`near_view_follow`、`near_view_hud`、`near_view_mouse`、`near_view_expo`、`near_view_inertia`、`near_view_level`、`near_view_fov_add` |
| 飞行控制 | 俯仰增益、翻滚减速、水平恢复、移动瞄准点补正、补正平滑、高G力满拉角度 | `pitch_gain`、`roll_brake`、`level_per_deg`、`lead_gain`、`lead_filter`、`highg_full_from` |
| 显示 | 语言、状态显示 | `language`、`status_line` |

飞行控制组只影响接近瞄准点的最后阶段：大机动中杆量常常已满，转动快慢由机体本身决定，差别不易察觉。测试时宜做 5–20° 的小幅修正与跟踪。

### 按键配置页

列出 `[keys]` 中的全部按键。选中一项按录键键（默认 Enter），再按想用的新键：Esc 取消，Delete 清除（不绑定）。新键与其他项相同时，底部提示与哪些项重复。

## 按键

`[keys]` 的写法：`F1`–`F24`、单个字母或数字、`Up`、`Down`、`Left`、`Right`、`Backspace`、`Enter`、`Space`、`Tab`、`Shift`、`Ctrl`、`Alt`、`CapsLock`、`Insert`、`Delete`、`Home`、`End`、`PageUp`、`PageDown`、`Numpad0`–`Numpad9`、`MButton`、`XButton1` / `XButton2`（鼠标后侧／前侧键），或虚拟键码（如 `0x26`）；`none` 表示不绑定。

| 键名 | 默认 | 功能 |
|---|---|---|
| `toggle` | F8 | 鼠标辅助 ON/OFF |
| `recenter` | F9 | 瞄准点回到机首 |
| `free_look` | F | 按住：自由视角 |
| `hud` | F7 | 指示层 ON/OFF |
| `post_stall` | XButton1 | 按住：失速机动辅助 |
| `tuning` | F3 | 调参面板 |
| `tuning_up` / `tuning_down` | Up / Down | 面板中选择上一项／下一项 |
| `tuning_less` / `tuning_more` | Left / Right | 面板中减小／增大数值 |
| `tuning_fine` | Shift | 按住：1/5 步长 |
| `tuning_undo` | Backspace | 还原为打开面板时的值 |
| `tuning_bind` | Enter | 在按键配置页录制新按键 |
| `reload` | F10 | 重新读取 `config.ini` |
| `trace` | F4 | 逐帧飞行记录 |
| `perf` | F5 | 性能统计 |
| `camera_probe` | F6 | 镜头状态采集 |
| `hangar_specs` | F7 | 机库中：读取机体参数表 |
| `hangar_geometry` | F8 | 机库中：读取机体几何 |

手动接管不使用 `[keys]`，按游戏自己的键位识别。

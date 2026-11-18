# tools —— 烧录脚本（J-Link）

两根板子的 MCU 完全相同，**同一时刻只有一根探针**：烧之前先确认 J-Link 插在哪块板上。

```powershell
cd D:\Robomaster\omni_skeleton

# 烧底盘板
& "D:\Robomaster\J-Link\JLink_V976\JLink.exe" -NoGui 1 -ExitOnError 1 -CommanderScript tools\flash_chassis.jlink

# 烧云台板（静默版固件）
& "D:\Robomaster\J-Link\JLink_V976\JLink.exe" -NoGui 1 -ExitOnError 1 -CommanderScript tools\flash_gimbal.jlink

# 把云台板整片擦空（不烧任何固件）
& "D:\Robomaster\J-Link\JLink_V976\JLink.exe" -NoGui 1 -ExitOnError 1 -CommanderScript tools\erase_gimbal.jlink
```

| 脚本 | 作用 |
| --- | --- |
| `flash_chassis.jlink` | 擦 + 烧 `Chassis/build/omni_chassis.elf` |
| `flash_gimbal.jlink` | 擦 + 烧 `Gimbal/build/omni_gimbal.elf`（静默版） |
| `erase_gimbal.jlink` | 整片擦除云台板，不留固件 |

## 注意事项

1. **SWD 速度用 1000 kHz**。手上的 J-Link CE（S/N 69409796）在 4000 kHz 下会
   `Failed to initialize DAP`，插上能识别但连不上。
2. 若出现 `Failed to initialize DAP` 且 `VTref` 有 3.3 V：VCC/GND 通了但 SWDIO/SWCLK
   没通，或目标固件把调试口锁了（PA13/PA14 被改作普通 IO / 进了低功耗 / 开了 RDP）。
3. `loadfile` 里是绝对路径，换电脑/换盘符要改。
4. 烧完会自动 `r`（复位）+ `g`（运行）。

# 连续采样底盘调试快照 debug::g_snapshot 并解码成表格
# 用法（在仓库根目录）：
#   .\tools\capture_snapshot.ps1 -Samples 100 -IntervalMs 200
#   加 -Csv out.csv 可导出
#
# ⚠ 如果提示「在此系统上禁止运行脚本」（ExecutionPolicy），改用：
#   powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\capture_snapshot.ps1
#
# ⚠ 地址 0x20000938 是当前这版固件里 debug::g_snapshot 的地址；
#   重新编译后要用下面这条命令查新地址：
#   arm-none-eabi-nm -C --defined-only Chassis/build/omni_chassis.elf | findstr g_snapshot
param(
    [int]$Samples = 60,
    [int]$IntervalMs = 200,
    [string]$Csv = ""
)

$ErrorActionPreference = 'Stop'
$jlink = 'D:\Robomaster\J-Link\JLink_V976\JLink.exe'
$addr = '0x20000938'   # debug::g_snapshot，改代码后要按新 map 更新
$words = 22            # sizeof(Snapshot) = 88 字节

$script = New-Object System.Collections.Generic.List[string]
$script.Add('device STM32F407VG')
$script.Add('si SWD')
$script.Add('speed 1000')
$script.Add('connect')
$script.Add('g')
for ($i = 0; $i -lt $Samples; $i++) {
    $script.Add("sleep $IntervalMs")
    $script.Add("mem32 $addr, $words")
}
$script.Add('qc')

$tmpScript = Join-Path $env:TEMP 'jl_capture.jlink'
Set-Content -Path $tmpScript -Value $script -Encoding ascii

$raw = & $jlink -NoGui 1 -ExitOnError 0 -CommanderScript $tmpScript 2>&1

# 收集所有 "XXXXXXXX = a b c d" 行里的十六进制字
$hexWords = New-Object System.Collections.Generic.List[uint32]
foreach ($line in $raw) {
    if ($line -match '^[0-9A-Fa-f]{8}\s*=\s*(.*)$') {
        foreach ($tok in ($matches[1] -split '\s+')) {
            if ($tok -match '^[0-9A-Fa-f]{8}$') {
                $hexWords.Add([Convert]::ToUInt32($tok, 16))
            }
        }
    }
}

function U32ToFloat([uint32]$v) {
    return [BitConverter]::ToSingle([BitConverter]::GetBytes($v), 0)
}

$count = [math]::Floor($hexWords.Count / $words)
Write-Host "采集到 $count 个样本（每 $IntervalMs ms 一个）`n"

$rows = @()
for ($s = 0; $s -lt $count; $s++) {
    $b = $s * $words
    $w = $hexWords.GetRange($b, $words)
    $row = [pscustomobject]@{
        tick      = $w[0]
        life      = ($w[1] -band 0xFF)
        mode      = (($w[1] -shr 8) -band 0xFF)
        rc_online = (($w[1] -shr 16) -band 0xFF)
        run       = (($w[1] -shr 24) -band 0xFF)
        fb_ok     = (($w[2] -shr 8) -band 0xFF)
        rc_wheel  = [math]::Round((U32ToFloat $w[6]), 3)
        delta     = [math]::Round((U32ToFloat $w[7]), 3)
        cmd_w     = [math]::Round((U32ToFloat $w[9]), 3)
        t_fl      = [math]::Round((U32ToFloat $w[10]), 2)
        t_fr      = [math]::Round((U32ToFloat $w[11]), 2)
        t_rl      = [math]::Round((U32ToFloat $w[12]), 2)
        t_rr      = [math]::Round((U32ToFloat $w[13]), 2)
        a_fl      = [math]::Round((U32ToFloat $w[14]), 2)
        a_fr      = [math]::Round((U32ToFloat $w[15]), 2)
        a_rl      = [math]::Round((U32ToFloat $w[16]), 2)
        a_rr      = [math]::Round((U32ToFloat $w[17]), 2)
        o_fl      = [math]::Round((U32ToFloat $w[18]), 2)
        o_fr      = [math]::Round((U32ToFloat $w[19]), 2)
        o_rl      = [math]::Round((U32ToFloat $w[20]), 2)
        o_rr      = [math]::Round((U32ToFloat $w[21]), 2)
    }
    $rows += $row
}

$rows | Format-Table -AutoSize | Out-String -Width 200 | Write-Host

if ($Csv -ne "") {
    $rows | Export-Csv -Path $Csv -NoTypeInformation -Encoding UTF8
    Write-Host "已导出：$Csv"
}

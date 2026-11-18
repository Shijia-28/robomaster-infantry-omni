# Equivalence test for the gimbal module refactor.
# Compiles the SAME trace_main.cpp against the OLD implementation and the NEW one,
# then compares the two traces line by line. Identical output => behaviour preserved.
#
# Usage:  powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\equiv_test\run_equiv_test.ps1
param(
    # Both default to this repository: the test compares the current gimbal module
    # against the reference trace produced by the same tree, so pass an older checkout
    # via -OldTree when you want to compare two revisions.
    [string]$OldTree = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path,
    [string]$NewTree = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path,
    [string]$Gxx = ""
)

$ErrorActionPreference = 'Stop'
# Host compiler: override with -Gxx if yours lives elsewhere.
$gxx  = if ($Gxx) { $Gxx } else { "D:\x86_64-16.2.0-release-win32-seh-ucrt-rt_v14-rev1\mingw64\bin\g++.exe" }
$here = $PSScriptRoot
$work = Join-Path $env:TEMP "equiv_test"
New-Item -ItemType Directory -Force -Path $work | Out-Null

function Build-Trace([string]$tree, [string]$outExe) {
    $gimbalCpp = Join-Path $tree "Gimbal\RobotModules\component\gimbal.cpp"
    if (-not (Test-Path $gimbalCpp)) { throw "not found: $gimbalCpp" }
    & $gxx -std=c++20 -O2 -w `
        -I (Join-Path $here "mock") `
        -I (Join-Path $tree "RobotComponents\component") `
        -I (Join-Path $tree "Gimbal\RobotModules\component") `
        (Join-Path $here "trace_main.cpp") $gimbalCpp -o $outExe
    if ($LASTEXITCODE -ne 0) { throw "compile failed: $tree" }
}

Write-Host "=== compile OLD: $OldTree"
Build-Trace $OldTree (Join-Path $work "trace_old.exe")
Write-Host "=== compile NEW: $NewTree"
Build-Trace $NewTree (Join-Path $work "trace_new.exe")

$oldOut = Join-Path $work "old.csv"
$newOut = Join-Path $work "new.csv"
& (Join-Path $work "trace_old.exe") | Set-Content -Path $oldOut -Encoding ascii
& (Join-Path $work "trace_new.exe") | Set-Content -Path $newOut -Encoding ascii

$oldLines = Get-Content $oldOut
$newLines = Get-Content $newOut
Write-Host ("old cycles = {0}, new cycles = {1}" -f $oldLines.Count, $newLines.Count)

if ($oldLines.Count -ne $newLines.Count) {
    Write-Host "*** FAIL: line count differs" -ForegroundColor Red
    exit 1
}

$diff = 0
$firstDiff = -1
for ($i = 0; $i -lt $oldLines.Count; $i++) {
    if ($oldLines[$i] -ne $newLines[$i]) {
        if ($firstDiff -lt 0) { $firstDiff = $i }
        $diff++
    }
}

if ($diff -eq 0) {
    Write-Host ("PASS: {0} cycles identical" -f $oldLines.Count) -ForegroundColor Green
    exit 0
} else {
    Write-Host ("FAIL: {0} cycles differ, first at {1}" -f $diff, $firstDiff) -ForegroundColor Red
    Write-Host ("OLD: " + $oldLines[$firstDiff])
    Write-Host ("NEW: " + $newLines[$firstDiff])
    exit 1
}

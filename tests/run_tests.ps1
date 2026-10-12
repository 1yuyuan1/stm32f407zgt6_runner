$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$out = Join-Path ([System.IO.Path]::GetTempPath()) ('f407-line-tests-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $out | Out-Null
$includes = @("-I$PSScriptRoot/stubs", "-I$repo/Libs/inc", "-I$repo/Mod/inc")
& gcc -std=c11 @includes -c "$repo/Libs/src/bsp_uart.c" -o "$out/bsp.o"
if ($LASTEXITCODE -ne 0) { throw 'BSP compile failed' }
& gcc -std=c11 @includes -c "$repo/Libs/src/Emm_V5.c" -o "$out/emm.o"
if ($LASTEXITCODE -ne 0) { throw 'Emm compile failed' }
& g++ -std=c++17 -Wall -Wextra -Werror @includes "$PSScriptRoot/line_follow_test.cpp" "$repo/Mod/src/gray16.cpp" "$repo/Mod/src/line_follow.cpp" "$repo/Mod/src/chassis_control.cpp" "$repo/Mod/src/step_motor.cpp" "$out/bsp.o" "$out/emm.o" -o "$out/line_follow_test.exe"
if ($LASTEXITCODE -ne 0) { throw 'Test compile failed' }
& "$out/line_follow_test.exe"
if ($LASTEXITCODE -ne 0) { throw 'Tests failed' }
& g++ -std=c++17 -Wall -Wextra -Werror @includes "$PSScriptRoot/chassis_app_test.cpp" "$repo/Mod/src/chassis_app.cpp" "$repo/Mod/src/chassis_control.cpp" "$repo/Mod/src/gray16.cpp" "$repo/Mod/src/line_follow.cpp" "$repo/Mod/src/step_motor.cpp" "$out/bsp.o" "$out/emm.o" -o "$out/chassis_app_test.exe"
if ($LASTEXITCODE -ne 0) { throw 'Chassis test compile failed' }
& "$out/chassis_app_test.exe"
if ($LASTEXITCODE -ne 0) { throw 'Chassis task tests failed' }

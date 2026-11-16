@echo off
setlocal enabledelayedexpansion

echo ========================================================
echo       Flashing Air Mouse Firmware to Nice!Nano
echo ========================================================

set UF2_FILE=build\zephyr\zephyr.uf2

if not exist "%UF2_FILE%" (
    echo [ERROR] Cannot find: %UF2_FILE%
    pause
    exit /b 1
)

set TARGET_DRIVE=
for %%d in (D E F G H I J K L M N O P Q R S T U V W X Y Z) do (
    if exist "%%d:\INFO_UF2.TXT" (
        set TARGET_DRIVE=%%d:
        goto :found
    )
)

echo [WARNING] Nice!Nano drive not detected!
echo Please double click Reset button to enter Bootloader mode.
pause
exit /b 1

:found
echo Found Nice!Nano at: %TARGET_DRIVE%
echo Copying %UF2_FILE% to %TARGET_DRIVE%\ ...
copy /Y "%UF2_FILE%" "%TARGET_DRIVE%\" >nul

echo [SUCCESS] Firmware flashed! The board is now rebooting.
timeout /t 3 >nul
exit /b 0

@echo off
setlocal

set "PROJECT_DIR=%~dp0"
set "CONFIG=%~1"
set "TARGET_NAME=%~2"
set "TOOLCHAIN_DIR=%~3"

if "%CONFIG%"=="" set "CONFIG=Debug"
if "%TARGET_NAME%"=="" set "TARGET_NAME=ChargeStation"

set "BUILD_DIR=%PROJECT_DIR%VisualGDB\%CONFIG%"
set "HEXBIN_DIR=%PROJECT_DIR%hexbin"
set "ELF_FILE=%BUILD_DIR%\%TARGET_NAME%"
set "HEX_FILE=%HEXBIN_DIR%\%TARGET_NAME%.hex"
set "UP_FILE=%HEXBIN_DIR%\%TARGET_NAME%.up"
set "OBJCOPY=%TOOLCHAIN_DIR%\bin\arm-none-eabi-objcopy.exe"

if not exist "%HEXBIN_DIR%" mkdir "%HEXBIN_DIR%"

if not exist "%ELF_FILE%" (
    echo [PostBuild] ERROR: ELF not found: %ELF_FILE%
    goto :fail
)

if not exist "%OBJCOPY%" set "OBJCOPY=%ProgramFiles(x86)%\Sysprogs\VisualGDB\Keil\arm-none-eabi-objcopy.exe"
if not exist "%OBJCOPY%" (
    echo [PostBuild] ERROR: arm-none-eabi-objcopy.exe not found.
    goto :fail
)

"%OBJCOPY%" -O ihex "%ELF_FILE%" "%HEX_FILE%"
if errorlevel 1 (
    echo [PostBuild] ERROR: Failed to generate HEX file.
    goto :fail
)

if not exist "%HEXBIN_DIR%\HexCrypt.exe" (
    echo [PostBuild] ERROR: HexCrypt.exe not found.
    goto :fail
)

if exist "%UP_FILE%" del /f /q "%UP_FILE%"

pushd "%HEXBIN_DIR%"
HexCrypt.exe %TARGET_NAME%.hex
popd

if not exist "%UP_FILE%" (
    echo [PostBuild] ERROR: HexCrypt did not generate %TARGET_NAME%.up.
    goto :fail
)

echo [PostBuild] Generated: hexbin\%TARGET_NAME%.hex
echo [PostBuild] Generated: hexbin\%TARGET_NAME%.up
exit /b 0

:fail
exit /b 1

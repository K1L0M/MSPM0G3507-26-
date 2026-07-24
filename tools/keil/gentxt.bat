@echo off

set SYSCFG_PATH="C:\ti\ccstheia151\ccs\tools\compiler\ti-cgt-armllvm_4.0.0.LTS\bin\tiarmhex.exe"

if not exist "%SYSCFG_PATH%" (
    echo.
    echo Couldn't find ARM Hex Tools: %SYSCFG_PATH%
    echo.
    exit
)

echo Using ARM Hex Tool from %SYSCFG_PATH%

set PROJ_DIR=%~1
set PROJ_DIR=%PROJ_DIR:'=%

set SYSCFG_FILE=%~2
set SYSCFG_FILE=%SYSCFG_FILE:'=%

%SYSCFG_PATH%  --diag_wrap=off --ti_txt -o "%PROJ_DIR%\Objects\%SYSCFG_FILE%.txt"  "%PROJ_DIR%\Objects\%SYSCFG_FILE%.axf"

@echo off
rem Loads the MSVC x64 environment (found with vswhere) and runs the given command in it.
rem Usage:  scripts\msvc.cmd cmake --preset msvc
rem From Git Bash:  cmd //c "scripts\msvc.cmd cmake --preset msvc"
setlocal
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
  echo msvc.cmd: vswhere.exe not found. Install Visual Studio with the C++ desktop workload. 1>&2
  exit /b 1
)
set "VSPATH="
for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -prerelease -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSPATH=%%i"
if not defined VSPATH (
  echo msvc.cmd: no Visual Studio installation with MSVC x64 tools found. 1>&2
  exit /b 1
)
rem vcvars looks up vswhere.exe through PATH.
for %%d in ("%VSWHERE%") do set "PATH=%%~dpd;%PATH%"
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
if "%~1"=="" (
  echo msvc.cmd: MSVC environment loaded from "%VSPATH%". Pass a command to run. 1>&2
  exit /b 0
)
%*
exit /b %ERRORLEVEL%

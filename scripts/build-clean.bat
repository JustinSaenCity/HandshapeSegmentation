@ECHO OFF

if exist build rmdir /s /q build

cmake --preset default
if errorlevel 1 exit /b 1

cmake --build --preset release
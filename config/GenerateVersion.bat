@echo off
 
cd C:\Users\idea\Desktop\BLE_51822_Controller\config

set VER_FILE=version.h

for /f %%i in ('"c:\Program Files\Git\bin\git.exe" rev-list HEAD -n 1') do (
  set version_hash=%%i
)

for /f %%i in ('"c:\Program Files\Git\bin\git.exe" rev-list HEAD --count') do (
  set version_number=%%i
)

set "build_date=%date:~,4%-%date:~5,2%-%date:~8,2%"
set "build_time=%time:~0,2%:%time:~3,2%:%time:~6,2%"
 
set "verstr=r%version_number% %version_hash:~0,8% %build_date% %build_time%"
echo HEAD VERSION : %verstr%

echo #ifndef PROJECT_VERSION_H > %VER_FILE%
echo #define PROJECT_VERSION_H >> %VER_FILE%
echo #define GIT_VERSION "%verstr%" >> %VER_FILE%
echo #define GIT_VERSION_NUM "%version_number%" >> %VER_FILE%
echo #define GIT_VERSION_HASH "%version_hash:~0,8%" >> %VER_FILE%
echo #define BUILD_DATE "%build_date%" >> %VER_FILE%
echo #define BUILD_TIME "%build_time%" >> %VER_FILE%
echo #endif >> %VER_FILE%

echo "Job done!!"


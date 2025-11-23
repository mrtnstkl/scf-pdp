@echo off
if not exist "build\debug" mkdir build\debug
cd build\debug
cmake ..\.. -G "Unix Makefiles" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug
copy /y compile_commands.json ..\..
make
cd ..\..

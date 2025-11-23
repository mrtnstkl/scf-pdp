@echo off
if not exist "build\release" mkdir build\release
cd build\release
cmake ..\.. -G "Unix Makefiles" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
copy /y compile_commands.json ..\..
make
cd ..\..

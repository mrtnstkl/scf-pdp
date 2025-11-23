mkdir -p build/debug
cd build/debug
cmake ../.. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug && cp compile_commands.json ../..
make
cd ../..

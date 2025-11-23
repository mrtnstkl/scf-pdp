mkdir -p build/release
cd build/release
cmake ../.. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release && cp compile_commands.json ../..
make
cd ../..

:: Deletes All files in the Build and Rebuilds 
::(Hidden, System, and Read-Only Files are Not Affected) 
::
@ECHO OFF 
rd /s /q Build
mkdir Build
cd .\Build\
cmake -G "MinGW Makefiles" --toolchain ../arm-none-eabi-gcc.cmake -DCMAKE_BUILD_TYPE=Debug -DNDEBUG=0 .. 
cmake --build . -- -j16
cd ..
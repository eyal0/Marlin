set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86)
set(CMAKE_CXX_COMPILER i686-w64-mingw32-g++)
# x87 math rounds differently from the single-precision float Marlin uses.
set(CMAKE_CXX_FLAGS_INIT "-msse2 -mfpmath=sse")

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR i686)
set(CMAKE_CXX_COMPILER g++)
# x87 math rounds differently from the single-precision float Marlin uses.
set(CMAKE_CXX_FLAGS_INIT "-m32 -msse2 -mfpmath=sse")

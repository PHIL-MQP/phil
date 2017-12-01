set(CMAKE_SYSTEM_NAME Generic)

if (WIN32)
    set(TOOLCHAIN_ROOT "/c/frc")
else ()
    set(TOOLCHAIN_ROOT "/usr/")
endif ()

set(TRIPLE "arm-linux-gnueabihf")
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)

set(CMAKE_C_COMPILER "${TRIPLE}-gcc" CACHE PATH "gcc" FORCE)
set(CMAKE_CXX_COMPILER "${TRIPLE}-g++" CACHE PATH "g++" FORCE)
set(CMAKE_AR "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-gcc-ar" CACHE PATH "archive" FORCE)
set(CMAKE_LINKER "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-ld" CACHE PATH "linker" FORCE)
set(CMAKE_RANLIB "${TOOLCHAIN_ROOT}/bin/${TRIPLE}-gcc-ranlib" CACHE PATH "ranlib" FORCE)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --sysroot=$ENV{SYSROOT} -mtune=cortex-a15 -march=armv7-a -mfpu=neon-vfpv4")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --sysroot=$ENV{SYSROOT} -mtune=cortex-a15 -march=armv7-a -mfpu=neon-vfpv4")
set(CMAKE_EXE_LINKER_FLAGS "--sysroot=$ENV{SYSROOT} -Wl,-rpath-link,$ENV{SYSROOT}/usr/lib/arm-linux-gnueabihf/tegra" CACHE STRING "executable linker flags" FORCE)
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --sysroot=$ENV{SYSROOT}")
set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} --sysroot=$ENV{SYSROOT}")


set(CMAKE_FIND_ROOT_PATH "$ENV{SYSROOT}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

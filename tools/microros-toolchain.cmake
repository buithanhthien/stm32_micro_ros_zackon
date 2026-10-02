set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(CMAKE_C_COMPILER "${PROJECT_ROOT}/.tools/root/usr/bin/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "${PROJECT_ROOT}/.tools/root/usr/bin/arm-none-eabi-g++")
set(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
set(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")
set(FLAGS "-mcpu=cortex-m7 -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -Os -ffunction-sections -fdata-sections --specs=nano.specs -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='")
set(CMAKE_C_FLAGS_INIT "-std=c11 ${FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "-std=c++14 ${FLAGS} -fno-rtti -fno-exceptions")
set(__BIG_ENDIAN__ 0)

# Do not find host ROS x86 libraries via PATH or CMake system prefixes.
set(CMAKE_IGNORE_PREFIX_PATH "/opt/ros/jazzy")

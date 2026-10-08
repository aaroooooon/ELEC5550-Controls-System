# ESP-IDF component wrapping the Edge Impulse "Weed Detection" C++ library.
#
# The library itself stays where it was exported to, in lib/. It cannot be a
# component on its own: its CMakeLists.txt targets Zephyr / a plain `app`
# executable, and the '#' in its folder name is not legal in a CMake target
# name. To drop in a retrained model, replace that folder and update EI_LIB_DIR
# if the name changed.
#
# platformio.ini lists the folder under lib_ignore, so PlatformIO's library
# finder leaves it to this component instead of trying to compile it again.

set(EI_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../../lib/weed-detection-cpp-mcu-v5-impulse-#1")
get_filename_component(EI_LIB_DIR "${EI_LIB_DIR}" ABSOLUTE)
set(EI_SDK_DIR "${EI_LIB_DIR}/edge-impulse-sdk")

if(NOT EXISTS "${EI_SDK_DIR}")
    message(FATAL_ERROR "Edge Impulse library not found at ${EI_LIB_DIR}")
endif()

# Same source selection as the SDK's own ESP32 recipe
# (edge-impulse-sdk/cmake/zephyr/CMakeLists.txt): everything except CMSIS, which
# is Arm-only, plus the Xtensa assembly kernels from ESP-NN and esp-dsp. Sources
# for other platforms are compiled but empty, as they are guarded by
# EI_PORTING_* checks.
file(GLOB_RECURSE EI_SOURCES
    "${EI_SDK_DIR}/*.cpp"
    "${EI_SDK_DIR}/*.cc"
    "${EI_SDK_DIR}/*.c"
    "${EI_SDK_DIR}/porting/espressif/*.S"
    "${EI_LIB_DIR}/tflite-model/*.cpp"
)
list(FILTER EI_SOURCES EXCLUDE REGEX "/edge-impulse-sdk/CMSIS/")
list(APPEND EI_SOURCES "${EI_SDK_DIR}/CMSIS/DSP/Source/StatisticsFunctions/arm_rms_f32.c")

idf_component_register(
    SRCS ${EI_SOURCES}
    # All public: the SDK's headers reach into ESP-NN and esp-dsp, and main.cpp
    # includes those headers too.
    INCLUDE_DIRS
        "${EI_LIB_DIR}"
        "${EI_SDK_DIR}"
        "${EI_SDK_DIR}/porting"
        "${EI_SDK_DIR}/porting/espressif/ESP-NN"
        "${EI_SDK_DIR}/porting/espressif/ESP-NN/include"
        "${EI_SDK_DIR}/porting/espressif/ESP-NN/src/common"
        "${EI_SDK_DIR}/porting/espressif/esp-dsp/modules/common/include"
        "${EI_SDK_DIR}/porting/espressif/esp-dsp/modules/fft/include"
    REQUIRES esp_timer
)

# The SDK picks its ESP32 port and turns on the ESP-NN kernels by testing
# CONFIG_IDF_TARGET_*, but it tests before it includes sdkconfig.h. Defining the
# macro up front gives every file the same answer; the value matches
# sdkconfig.h's own definition, so there is no redefinition clash. PUBLIC so
# main.cpp, which includes the same headers, is configured identically.
string(TOUPPER "${IDF_TARGET}" EI_IDF_TARGET_UPPER)
target_compile_definitions(${COMPONENT_LIB} PUBLIC
    CONFIG_IDF_TARGET_${EI_IDF_TARGET_UPPER}=1
    SILENCE_EI_CLASSFIER_OBJECT_DETECTION_COUNT_WARNING=1
)

# Vendor code: silence its warnings (the project builds with -Werror=all) and
# optimise it regardless of the project's debug optimisation level, since this
# is where nearly all of the inference time goes.
target_compile_options(${COMPONENT_LIB} PRIVATE -w -O2)

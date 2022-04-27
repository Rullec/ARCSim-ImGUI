add_subdirectory(./json)

add_library(
    utils_lib

    FileUtil.cpp
    JsonUtil.cpp
    LogUtil.cpp
    MathUtil.cpp
    MathUtil2.cpp
    Rand.cpp
    TimeUtil.cpp
    SysUtil.cpp
    # ObjUtil.cpp
    StringUtil.cpp
)

target_link_libraries(utils_lib JSON)

if(WIN32)
    target_link_libraries(utils_lib spdlog)
endif()
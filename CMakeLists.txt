cmake_minimum_required(VERSION 3.0.0)
project(main VERSION 0.1.0 LANGUAGES CXX)
add_definitions(-D _USE_MATH_DEFINES)
add_definitions(-D _CRT_NO_VA_START_VALIDATION)
include_directories(include)
include_directories(include/imgui)
include_directories(.)
include_directories(./src)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4061 /wd4244 /wd4246 /wd4305 /wd4267 /wd4711 /wd4710 /wd4514 /wd4477 /wd4819 /wd4018")

set(CMAKE_CXX_STANDARD 17)
include_directories(Third-Party/include)
include_directories(utils)
include_directories(Third-Party/include/png)
include_directories(Third-Party/include/taucs)
# include_directories(Third-Party/include/json)
link_directories(./Third-Party/lib)

add_subdirectory(AdaptiveCloth)
add_subdirectory(./Third-Party/include/alglib)
# add_subdirectory(./Third-Party/include/json)
add_subdirectory(utils)
add_subdirectory(include/imgui)

set(THIRD_PARTY_LIB blas clapack freeglut libmetis libtaucs vcf2c alglib_lib utils_lib)

add_executable(main ./AdaptiveCloth/Main.cpp)
# add_executable(new_main ./AdaptiveCloth/new_main.cpp)

target_link_libraries(main adaptive_cloth_lib ${THIRD_PARTY_LIB} legacy_stdio_definitions imgui_lib)
# target_link_libraries(new_main adaptive_cloth_lib ${THIRD_PARTY_LIB} legacy_stdio_definitions imgui_lib)

if(WIN32)
    if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
        add_custom_command(
            TARGET main POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E rename
                ${CMAKE_CURRENT_BINARY_DIR}/Debug/main.exe
                ${CMAKE_SOURCE_DIR}/main.exe
        )
    else()
        add_custom_command(
            TARGET main POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E rename
                ${CMAKE_CURRENT_BINARY_DIR}/Release/main.exe
                ${CMAKE_SOURCE_DIR}/main.exe
        )
    endif()
else()
    add_custom_command(
        TARGET main POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E rename
            ${CMAKE_CURRENT_BINARY_DIR}/main
            ${CMAKE_SOURCE_DIR}/main
    )
endif()


# if(WIN32)
#     if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
#         add_custom_command(
#             TARGET new_main POST_BUILD
#             COMMAND ${CMAKE_COMMAND} -E rename
#                 ${CMAKE_CURRENT_BINARY_DIR}/Debug/new_main.exe
#                 ${CMAKE_SOURCE_DIR}/new_main.exe
#         )
#     else()
#         add_custom_command(
#             TARGET new_main POST_BUILD
#             COMMAND ${CMAKE_COMMAND} -E rename
#                 ${CMAKE_CURRENT_BINARY_DIR}/Release/new_main.exe
#                 ${CMAKE_SOURCE_DIR}/new_main.exe
#         )
#     endif()
# else()
#     add_custom_command(
#         TARGET new_main POST_BUILD
#         COMMAND ${CMAKE_COMMAND} -E rename
#             ${CMAKE_CURRENT_BINARY_DIR}/new_main
#             ${CMAKE_SOURCE_DIR}/new_main
#     )
# endif()
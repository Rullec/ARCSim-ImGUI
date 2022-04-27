cmake_minimum_required(VERSION 3.5)

# add_definitions("-O3 -Wall -fPIC")
include_directories(..)
set(JSON_SRC
json_reader.cpp
json_value.cpp
json_writer.cpp
)
add_library(JSON ${JSON_SRC})
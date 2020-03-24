
add_library(MahonyAHRS MahonyAHRS.c)

target_include_directories(
    MahonyAHRS
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
)

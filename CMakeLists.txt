set(module_name "mpu6886")
message("Module: ${module_name}")

add_subdirectory( MahonyAHRS )

# afr_module()

afr_module_sources(
# target_sources(
    ${module_name} 
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/src/mpu6886.c"
)

afr_module_include_dirs(
# target_include_directories(
    ${module_name}
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/include"
)

afr_module_dependencies(
    ${module_name} 
    INTERFACE
        AFR::common
        AFR::common_io
    PUBLIC
        MahonyAHRS

)

add_library(${module_name} INTERFACE)
add_library(sensor::${module_name} ALIAS ${module_name})

set(module_name "mpu6886")
message("Module: ${module_name}")

add_subdirectory( afr-common-io-helpers )
add_subdirectory( MahonyAHRS )

add_library(${module_name})

target_sources(
    ${module_name} 
    PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/src/mpu6886.c"
)

target_include_directories(
    ${module_name}
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/include"
)

target_link_libraries(
    ${module_name} 
    PRIVATE
        AFR::common
        AFR::common_io        
        common-io-helpers
        MahonyAHRS
)

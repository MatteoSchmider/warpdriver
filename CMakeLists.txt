project (warpdriver)

add_subdirectory(pigpio)

add_library(${PROJECT_NAME} 
    RaspiSPI.cpp
    WarpDriver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} 
    pigpio
)

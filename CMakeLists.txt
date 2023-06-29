project (warpdriver)

add_library(${PROJECT_NAME} 
    src/RaspiSPI.cpp
    src/WarpDriver.cpp
)

target_include_directories(${PROJECT_NAME}
    INTERFACE
    ${PROJECT_SOURCE_DIR}/src
)
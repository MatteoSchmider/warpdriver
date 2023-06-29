project (warpdriver)

add_library(${PROJECT_NAME} 
    src/RaspiSPI.cpp
    src/WarpDriver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src
)

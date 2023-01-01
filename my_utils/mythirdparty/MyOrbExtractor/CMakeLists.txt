

find_package(OpenCV 3.4.9 REQUIRED)

add_library(OrbExtractor src/OrbExtractor.cc)

target_link_libraries(OrbExtractor PUBLIC
    ${OpenCV_LIBS}
)
target_include_directories(OrbExtractor PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIR}
)
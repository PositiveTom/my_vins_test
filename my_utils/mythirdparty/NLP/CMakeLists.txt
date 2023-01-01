
#   [0]. 寻找外部库
find_package(Eigen3 REQUIRED)
#   [1]. 添加库文件
set(NLP_FILE 
    src/problem.cc 
    src/vertex.cc 
    src/edge.cc
    src/loss_function.cc
)
#   [2]. 创建库
add_library(NLP_LIB SHARED ${NLP_FILE})
#   [3]. 添加库目录
target_include_directories(NLP_LIB PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

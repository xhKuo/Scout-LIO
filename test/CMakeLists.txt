find_package( OpenCV REQUIRED )

# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
#set( G2O_ROOT /usr/local/include/g2o )
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

include(${PROJECT_SOURCE_DIR}/cmake/FindCSparse.cmake)
include(${PROJECT_SOURCE_DIR}/cmake/FindG2O.cmake)
# 添加头文件
include_directories( "/usr/include/eigen3"
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR}
        )

add_executable( test_BA test_BA-3Dto2D.cpp )

target_link_libraries( test_BA
        ${OpenCV_LIBS}
        ${CSPARSE_LIBRARY}
        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension
        )
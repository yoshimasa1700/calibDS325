# CMakeのバージョン
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CMAKE_CXX_COMPILER g++)

# リリースビルド用オプション
SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O2 -fpermissive -fPIC")
# デバッグビルド用オプション
SET(CMAKE_CXX_FLAGS_DEBUG "-g -fpermissive -fPIC")
# ビルドのタイプはリリース
#SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

#set(CMAKE_CXX_FLAGS "$(CMAKE_CXX_FLAGS) -fpermissive")

SET(CMAKE_INSTALL_PREFIX=/usr/local)
# プロジェクト名
PROJECT(calibDS325)

FIND_PACKAGE(OpenCV)
#FIND_PACKAGE(Boost)
#find_package(OpenMP)
#find_package(OpenGL)
#find_package(GLUT)
# if (OPENMP_FOUND)
#     set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# endif()

# バージョン番号
SET(serial "1.0.0")
# 共有ライブラリのバージョン番号
SET(soserial "1")
# ビルドするためにインクルードするヘッダーファイルの場所
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}")

# 共有ライブラリとしてビルド
#ADD_LIBRARY(HFMD_core SHARED src/CConfig.cpp src/CRForest.cpp src/HoG.cpp src/CDataset.cpp src/CRTree.cpp src/util.cpp)

ADD_EXECUTABLE(calibDS325 main.cpp)

TARGET_LINK_LIBRARIES(calibDS325 ${OpenCV_LIBRARIES} gflags)

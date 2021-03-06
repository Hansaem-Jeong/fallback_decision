cmake_minimum_required(VERSION 3.0.2)
project(fallback_decision)

# Find CUDA
find_package(CUDA QUIET)
if (CUDA_FOUND)
  find_package(CUDA REQUIRED)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_30,code=sm_30
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=sm_61
    -gencode arch=compute_62,code=sm_62
    -gencode arch=compute_72,code=sm_72
  )
  add_definitions(-DGPU)
  add_definitions(-DCUDNN)
else()
  list(APPEND LIBRARIES "m")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  aes_data.msg
  Chassis.msg
  Sensor.msg
  Traffic.msg
  Line.msg
  AEB.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
    include
    src
  LIBRARIES 
    ${PROJECT_NAME}_lib
  CATKIN_DEPENDS 
    roscpp 
    rospy
    std_msgs
  DEPENDS 
    system_lib
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)


set(PROJECT_LIB_FILES
#bevimage
  src/MWCUSOLVERUtils.cpp
#predic
  src/MWCNNLayer.cpp
  src/MWCustomLayerForCuDNN.cpp
  src/MWElementwiseAffineLayer.cpp
  src/MWFCLayer.cpp
  src/MWFusedConvReLULayer.cpp
  src/MWInputLayer.cpp
  src/MWMaxPoolingLayer.cpp
  src/MWOutputLayer.cpp
  src/MWSoftmaxLayer.cpp
  src/MWTensorBase.cpp
#decision
  src/P_result.cpp
)

set(PROJECT_CUDA_FILES
#predic
  src/Decision_Predict.cu
  src/Decision_Predict_data.cu
  src/Decision_Predict_initialize.cu
  src/Decision_Predict_terminate.cu
  src/DeepLearningNetwork.cu
  src/MWCNNLayerImpl.cu
  src/MWElementwiseAffineLayerImpl.cu
  src/MWElementwiseAffineLayerImplKernel.cu
  src/MWFCLayerImpl.cu
  src/MWFusedConvReLULayerImpl.cu
  src/MWMaxPoolingLayerImpl.cu
  src/MWOutputLayerImpl.cu
  src/MWSoftmaxLayerImpl.cu
  src/MWTargetNetworkImpl.cu
  src/predict.cu
#bevimage
  src/BEV_image.cu
  src/BEV_image_data.cu
  src/BEV_image_initialize.cu
  src/BEV_image_terminate.cu
  src/rtGetInf.cu
  src/rtGetNaN.cu
  src/rt_nonfinite.cu
)

if (CUDA_FOUND)

  link_directories(
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
  )

  cuda_add_library(${PROJECT_NAME}_lib
    ${PROJECT_LIB_FILES}
    ${PROJECT_CUDA_FILES}
  )

  target_link_libraries(${PROJECT_NAME}_lib
    cuda
    cudart
    cublas
    curand
    cudnn
    cusolver
  )

  cuda_add_executable(${PROJECT_NAME}
    node/aes_node.cpp
  )

else()

  add_library(${PROJECT_NAME}_lib
    ${PROJECT_LIB_FILES} 
    ${PROJECT_CORE_FILES}
  )

  add_executable(${PROJECT_NAME}
    node/aes_node.cpp
  )

endif()
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)


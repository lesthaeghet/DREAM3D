project(ImageProcessingTest)


## YOU MUST INCLUDE THIS FILE TO USE ITK !!!
## Then do not forget to add "${ITK_LIBRARIES}" to the target_link_libraries below
include( ${DREAM3D_SUPPORT_DIR}/ITKSupport/IncludeITK.cmake)
include_directories(${ITK_INCLUDE_DIRS})


find_package(ITK REQUIRED)
include(${ITK_USE_FILE})

add_executable(BinaryWatershedTest  BinaryWatershedTest.cpp)
 
if( "${ITK_VERSION_MAJOR}" LESS 4 )
  target_link_libraries(BinaryWatershedTest ITKReview ${ITK_LIBRARIES})
else( "${ITK_VERSION_MAJOR}" LESS 4 )
  target_link_libraries(BinaryWatershedTest ${ITK_LIBRARIES})
endif( "${ITK_VERSION_MAJOR}" LESS 4 )


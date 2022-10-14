


cmake_minimum_required( VERSION 2.8 )
# Create Project
project( AER_TER_REgistration )
add_executable( AT_Registration ./src/AT_Registration.cpp
 )

include_directories(include/)

file(GLOB HEADERS
    include/*.hpp
    
    

)
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "project" )

# Find Packages
# Find PCL
find_package( PCL 1.8 REQUIRED )

# Find OpenCV

find_package( OpenCV REQUIRED )

if( PCL_FOUND AND OpenCV_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )
  include_directories( ${OpenCV_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )

  # For Use Not PreCompiled Features 
  #add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_directories( ${OpenCV_LIB_DIR} )



  # [Linker]>[Input]>[Additional Dependencies]
  target_link_libraries(  AT_Registration ${PCL_LIBRARIES} )
  target_link_libraries(  AT_Registration ${OpenCV_LIBS} )
endif()


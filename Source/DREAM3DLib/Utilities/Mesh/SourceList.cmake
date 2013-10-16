#--////////////////////////////////////////////////////////////////////////////
#--
#--  Copyright (c) 2013, William Lenthe, University of California Santa Barbara
#--  All rights reserved.
#--  BSD License: http://www.opensource.org/licenses/bsd-license.html
#--
#--////////////////////////////////////////////////////////////////////////////


set(DREAM3DLib_Utilities_IO_HDRS
  ${DREAM3DLib_SOURCE_DIR}/Utilities/Mesh/triangle.h
  ${DREAM3DLib_SOURCE_DIR}/Utilities/Mesh/assert.hpp
  ${DREAM3DLib_SOURCE_DIR}/Utilities/Mesh/del_interface.hpp
  ${DREAM3DLib_SOURCE_DIR}/Utilities/Mesh/dpoint.hpp
  ${DREAM3DLib_SOURCE_DIR}/Utilities/Mesh/triangle_impl.hpp
)

set(DREAM3DLib_Utilities_IO_SRCS
  ${DREAM3DLib_SOURCE_DIR}/Utilities/Mesh/assert.cpp
  ${DREAM3DLib_SOURCE_DIR}/Utilities/Mesh/del_impl.cpp
  ${DREAM3DLib_SOURCE_DIR}/Utilities/Mesh/triangle.c
)

cmp_IDE_SOURCE_PROPERTIES( "DREAM3DLib/Utilities/Mesh" "${DREAM3DLib_Utilities_IO_HDRS}" "${DREAM3DLib_Utilities_IO_SRCS}" "0")
if( ${PROJECT_INSTALL_HEADERS} EQUAL 1 )
    INSTALL (FILES ${DREAM3DLib_Utilities_IO_HDRS}
            DESTINATION include/DREAM3D/Utilities/IO
            COMPONENT Headers   )
endif()

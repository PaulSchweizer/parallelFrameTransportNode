CMAKE_MINIMUM_REQUIRED(VERSION 3.12)

PROJECT ( FrameTransportNode )

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_EXTENSIONS OFF)


FIND_PACKAGE ( Maya REQUIRED )

ADD_DEFINITIONS ( ${MAYA_DEFINITIONS} )

INCLUDE_DIRECTORIES ( ${MAYA_INCLUDE_DIR} )

ADD_LIBRARY ( parallelFrameTransportNode SHARED
  parallelFrameTransportNode.cpp
  )

TARGET_LINK_LIBRARIES ( parallelFrameTransportNode
  ${MAYA_Foundation_LIBRARY}
  ${MAYA_OpenMayaUI_LIBRARY}
  ${MAYA_OpenMaya_LIBRARY}
  )

MAYA_SET_LIBRARY_PROPERTIES ( parallelFrameTransportNode )

INSTALL ( TARGETS
  parallelFrameTransportNode
  DESTINATION
  ./maya${MAYA_VERSION_NUMBER}/plug-ins
  )

INSTALL ( FILES
  AEparallelFrameTransportTemplate.mel
  DESTINATION
  ./maya${MAYA_VERSION_NUMBER}/scripts
  )

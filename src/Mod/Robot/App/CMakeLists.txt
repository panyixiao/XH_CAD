if(MSVC)
    add_definitions(-DFCAppRobot -DHAVE_ACOSH -DHAVE_ASINH -DHAVE_ATANH)
else(MSVC)
    add_definitions(-DHAVE_LIMITS_H -DHAVE_CONFIG_H)
endif(MSVC)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)
find_package(Boost REQUIRED COMPONENTS date_time)

# urdf parser
find_package(urdfdom REQUIRED)
pkg_check_modules(pkg_tinyxml REQUIRED)
pkg_check_modules(pkg_tinyxml2 REQUIRED)
find_package(urdf QUIET)
if(urdf_FOUND)
  add_definitions(-DHAS_URDF)
  include_directories(${urdf_INCLUDE_DIRS})
endif()
include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS})

include_directories(
    ${CMAKE_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_BINARY_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}
    ${Boost_INCLUDE_DIRS}
    ${OCC_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIR}
    ${QT_INCLUDE_DIR}
    ${XercesC_INCLUDE_DIRS}
    ${urdfdom_INCLUDE_DIRS}
)
link_directories(${OCC_LIBRARY_DIR})

set(Robot_LIBS
    FreeCADApp
    Mesh
    Part
    MeshPart
    ${OCC_LIBRARIES}
    ${OCC_DEBUG_LIBRARIES}
    ${QT_QTCORE_LIBRARY}
)

generate_from_xml(Robot6AxisPy)
generate_from_xml(TrajectoryPy)
generate_from_xml(WaypointPy)
generate_from_xml(RobotObjectPy)


SET(Python_SRCS
#    Robot6AxisPy.xml
#    Robot6AxisPyImp.cpp
#    TrajectoryPy.xml
#    TrajectoryPyImp.cpp
#    RobotObjectPy.xml
#    RobotObjectPyImp.cpp
#    WaypointPy.xml
#    WaypointPyImp.cpp
)

SET(Mod_SRCS
    AppRobot.cpp
    PreCompiled.cpp
    PreCompiled.h
)

set(Database_SRC
    Database/MechanicDatabase.cpp
    Database/MechanicDatabase.h
    Database/ToolDatabase.cpp
    Database/ToolDatabase.h
    Database/FileOperator.cpp
    Database/FileOperator.h
    )

set(MechanicObj_SRC
    Mechanics/MechanicBase.cpp
    Mechanics/MechanicBase.h
    Mechanics/MechanicPoser.cpp
    Mechanics/MechanicPoser.h
    Mechanics/MechanicRobot.cpp
    Mechanics/MechanicRobot.h
    Mechanics/MechanicGroup.h
    Mechanics/MechanicGroup.cpp
    Mechanics/KinematicModel.h
    Mechanics/KinematicModel.cpp
    Mechanics/RobotAlgos.h
    Mechanics/RobotAlgos.cpp
    Mechanics/Robot6AxisObject.h
    Mechanics/Robot6AxisObject.cpp
    Mechanics/MechanicDevice.h
    Mechanics/MechanicDevice.cpp)

set(PlanningObj_SRC
    PlanningObj/PlanningObject.cpp
    PlanningObj/PlanningObject.h)

set(Tool_SRCS
    Tool/ToolObject.cpp
    Tool/ToolObject.h
    Tool/TorchObject.cpp
    Tool/TorchObject.h
    Tool/ScannerObject.cpp
    Tool/ScannerObject.h)

set(Utilites_SRC
    Utilites/EnumUtility.h
    Utilites/CAD_Utility.h
    Utilites/CAD_Utility.cpp
    Utilites/PartUtility.cpp
    Utilites/PartUtility.h
    Utilites/MeshUtility.cpp
    Utilites/MeshUtility.h
    Utilites/DS_Utility.cpp
    Utilites/DS_Utility.h
    Utilites/Export_Utility.cpp
    Utilites/Export_Utility.h
    Utilites/FileIO_Utility.h
    Utilites/FileIO_Utility.cpp
    Utilites/FrameObject.cpp
    Utilites/FrameObject.h)

set(TaskManage_SRC
    TaskManage/Action.cpp
    TaskManage/Action.h
    TaskManage/TaskObject.cpp
    TaskManage/TaskObject.h
    TaskManage/ActionObject.cpp
    TaskManage/ActionObject.h
    TaskManage/SimulationManager.cpp
    TaskManage/SimulationManager.h
    TaskManage/TracSimulatior.cpp
    TaskManage/TracSimulatior.h)

set(RobotTrac_SRCS
    Trac/RobotProgram.cpp
    Trac/RobotProgram.h
    Trac/RobotWaypoint.h
    Trac/RobotWaypoint.cpp
    Trac/RobotTracObject.cpp
    Trac/RobotTracObject.h
    Trac/RobotCommand.cpp
    Trac/RobotCommand.h
    Trac/MoveCommand.cpp
    Trac/MoveCommand.h
    Trac/CoordCommand.cpp
    Trac/CoordCommand.h
    Trac/ToolCommand.cpp
    Trac/ToolCommand.h
    Trac/EdgebasedTracObject.cpp
    Trac/EdgebasedTracObject.h)

SET(Robot_SRCS
    ${Database_SRC}
    ${MechanicObj_SRC}
    ${PlanningObj_SRC}
    ${Utilites_SRC}
    ${TaskManage_SRC}
    ${Mod_SRCS}
    ${Python_SRCS}
    ${RobotTrac_SRCS}
    ${Tool_SRCS}
)

SET(TRAC_IK_SRC
    trac_ik_cp/dual_quaternion.h
    trac_ik_cp/kdl_tl.cpp
    trac_ik_cp/kdl_tl.hpp
    trac_ik_cp/math3d.h
    trac_ik_cp/nlopt_ik.cpp
    trac_ik_cp/nlopt_ik.hpp
    trac_ik_cp/trac_ik.hpp
    trac_ik_cp/trac_ik.cpp
)

set(KDL_PARSER_SRC
    kdl_parser_cp/kdl_parser.hpp
    kdl_parser_cp/check_kdl_parser.cpp
    kdl_parser_cp/kdl_parser.cpp
    kdl_parser_cp/visibility_control.hpp
)

FILE( GLOB KDL_SRCS kdl_cp/[^.]*.cpp )
FILE( GLOB KDL_HPPS kdl_cp/[^.]*.hpp kdl_cp/[^.]*.inl)

FILE( GLOB UTIL_SRCS kdl_cp/utilities/[^.]*.cpp kdl_cp/utilities/[^.]*.cxx)
FILE( GLOB UTIL_HPPS kdl_cp/utilities/[^.]*.h kdl_cp/utilities/[^.]*.hpp)


SET(Robot_SRCS
  ${Robot_SRCS}
  ${KDL_SRCS}
  ${KDL_HPPS}
  ${UTIL_SRCS}
  ${UTIL_HPPS}
  ${TRAC_IK_SRC}
  ${KDL_PARSER_SRC}
)

SOURCE_GROUP("Python" FILES ${Python_SRCS})
SOURCE_GROUP("Module" FILES ${Mod_SRCS})

SET(Robot_Scripts
    Init.py
    KukaExporter.py
    RobotExample.py
    RobotExampleTrajectoryOutOfShapes.py
)

if (EXISTS ${CMAKE_SOURCE_DIR}/src/Mod/Robot/Lib/Kuka)
SET(Robot_Resources
)
endif ()

add_library(Robot SHARED ${Robot_SRCS})
target_link_libraries(Robot ${Robot_LIBS}
    ${Boost_LIBRARIES}
    ${pkg_nlopt_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${pkg_tinyxml_LIBRARIES}
    ${pkg_tinyxml2_LIBRARIES}
)
if(urdf_FOUND)
  target_link_libraries(Robot ${urdf_LIBRARIES})
endif()

fc_target_copy_resource(Robot
    ${CMAKE_SOURCE_DIR}/src/Mod/Robot
    ${CMAKE_BINARY_DIR}/Mod/Robot
    ${Robot_Scripts})

SET_BIN_DIR(Robot Robot /Mod/Robot)
SET_PYTHON_PREFIX_SUFFIX(Robot)

INSTALL(TARGETS Robot DESTINATION ${CMAKE_INSTALL_LIBDIR})

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
    Mechanics/MechanicExtAx.h
    Mechanics/MechanicExtAx.cpp
    Mechanics/MechanicRobot.cpp
    Mechanics/MechanicRobot.h
    Mechanics/KinematicModel.h
    Mechanics/KinematicModel.cpp
    Mechanics/RobotAlgos.h
    Mechanics/RobotAlgos.cpp
#    Mechanics/Robot6AxisObject.h
#    Mechanics/Robot6AxisObject.cpp
#    Mechanics/MechanicDevice.h
#    Mechanics/MechanicDevice.cpp
    )

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
    TaskManage/TaskObject.cpp
    TaskManage/TaskObject.h
    TaskManage/ProgramSimulator.cpp
    TaskManage/ProgramSimulator.h
    TaskManage/TargetPoint.h
    TaskManage/TargetPoint.cpp
    TaskManage/RobotProgram.cpp
    TaskManage/RobotProgram.h)

set(Command_SRCS
    TaskManage/Command/CommandBase.h
    TaskManage/Command/CommandBase.cpp
    TaskManage/Command/MoveCommand.h
    TaskManage/Command/MoveCommand.cpp
    TaskManage/Command/CoordCommand.h
    TaskManage/Command/CoordCommand.cpp
    TaskManage/Command/ToolCommand.h
    TaskManage/Command/ToolCommand.cpp)

set(RobotTrac_SRCS
#    Trac/RobotTracObject.cpp
#    Trac/RobotTracObject.h
#    Trac/EdgebasedTracObject.cpp
#    Trac/EdgebasedTracObject.h
    )

SET(RobotMod_SRCS
    ${Database_SRC}
    ${MechanicObj_SRC}
    ${PlanningObj_SRC}
    ${Utilites_SRC}
    ${TaskManage_SRC}
    ${Mod_SRCS}
    ${Python_SRCS}
    ${RobotTrac_SRCS}
    ${Tool_SRCS}
    ${Command_SRCS}
)

SET(TRAC_IK_SRC
    Kinematics/trac_ik_cp/dual_quaternion.h
    Kinematics/trac_ik_cp/kdl_tl.cpp
    Kinematics/trac_ik_cp/kdl_tl.hpp
    Kinematics/trac_ik_cp/math3d.h
    Kinematics/trac_ik_cp/nlopt_ik.cpp
    Kinematics/trac_ik_cp/nlopt_ik.hpp
    Kinematics/trac_ik_cp/trac_ik.hpp
    Kinematics/trac_ik_cp/trac_ik.cpp
)

set(KDL_PARSER_SRC
    Kinematics/kdl_parser_cp/kdl_parser.hpp
    Kinematics/kdl_parser_cp/check_kdl_parser.cpp
    Kinematics/kdl_parser_cp/kdl_parser.cpp
    Kinematics/kdl_parser_cp/visibility_control.hpp
)

FILE( GLOB KDL_SRCS Kinematics/kdl_cp/[^.]*.cpp )
FILE( GLOB KDL_HPPS Kinematics/kdl_cp/[^.]*.hpp Kinematics/kdl_cp/[^.]*.h Kinematics/kdl_cp/[^.]*.inl)

FILE( GLOB UTIL_SRCS Kinematics/kdl_cp/utilities/[^.]*.cpp Kinematics/kdl_cp/utilities/[^.]*.cxx)
FILE( GLOB UTIL_HPPS Kinematics/kdl_cp/utilities/[^.]*.h Kinematics/kdl_cp/utilities/[^.]*.hpp)

SET(Kinematics_SRC
    ${TRAC_IK_SRC}
    ${KDL_SRCS}
    ${KDL_HPPS}
    ${KDL_PARSER_SRC}
    ${UTIL_SRCS}
    ${UTIL_HPPS}
    )

SET(Robot_SRCS
  ${RobotMod_SRCS}
  ${Kinematics_SRC}
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

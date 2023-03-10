
add_subdirectory(App)
if(BUILD_GUI)
    add_subdirectory(Gui)
endif(BUILD_GUI)

set(Robot_Scripts
    Init.py
     KukaExporter.py
     RobotExample.py
     RobotExampleTrajectoryOutOfShapes.py
)

if(BUILD_GUI)
    list (APPEND Robot_Scripts
        InitGui.py
        MovieTool.py
    )
endif(BUILD_GUI)

SET(Robot_Resources
    Lib/Model/Robot/MOKA/MR07S-930/MR07S_930.urdf
    Lib/Model/Robot/MOKA/MR07S-930/meshes/base_link.STL
    Lib/Model/Robot/MOKA/MR07S-930/meshes/Link1.STL
    Lib/Model/Robot/MOKA/MR07S-930/meshes/Link2.STL
    Lib/Model/Robot/MOKA/MR07S-930/meshes/Link3.STL
    Lib/Model/Robot/MOKA/MR07S-930/meshes/Link4.STL
    Lib/Model/Robot/MOKA/MR07S-930/meshes/Link5.STL
    Lib/Model/Robot/MOKA/MR07S-930/meshes/Link6.STL
    Lib/Model/Robot/Kawasaki/BA006N/kawasaki_ba006n.urdf
    Lib/Model/Robot/Kawasaki/BA006N/meshes/base_link.STL
    Lib/Model/Robot/Kawasaki/BA006N/meshes/Link1.STL
    Lib/Model/Robot/Kawasaki/BA006N/meshes/Link2.STL
    Lib/Model/Robot/Kawasaki/BA006N/meshes/Link3.STL
    Lib/Model/Robot/Kawasaki/BA006N/meshes/Link4.STL
    Lib/Model/Robot/Kawasaki/BA006N/meshes/Link5.STL
    Lib/Model/Robot/Kawasaki/BA006N/meshes/Link6.STL
    Lib/Model/Robot/Kawasaki/RS007L/RS007L.urdf
    Lib/Model/Robot/Kawasaki/RS007L/meshes/base_link.STL
    Lib/Model/Robot/Kawasaki/RS007L/meshes/Link1.STL
    Lib/Model/Robot/Kawasaki/RS007L/meshes/Link2.STL
    Lib/Model/Robot/Kawasaki/RS007L/meshes/Link3.STL
    Lib/Model/Robot/Kawasaki/RS007L/meshes/Link4.STL
    Lib/Model/Robot/Kawasaki/RS007L/meshes/Link5.STL
    Lib/Model/Robot/Kawasaki/RS007L/meshes/Link6.STL

    Lib/Model/Poser/2Axis/L-Type/200kg/2AxisPositioner/2AxisPositioner.urdf
    Lib/Model/Poser/2Axis/L-Type/200kg/2AxisPositioner/meshes/base_link.STL
    Lib/Model/Poser/2Axis/L-Type/200kg/2AxisPositioner/meshes/link1.STL
    Lib/Model/Poser/2Axis/L-Type/200kg/2AxisPositioner/meshes/link2.STL
    Lib/Model/Poser/2Axis/L-Type/500kg/2AxisPositioner_NoGripper/2AxisPositioner_NoGripper.urdf
    Lib/Model/Poser/2Axis/L-Type/500kg/2AxisPositioner_NoGripper/meshes/base_link.STL
    Lib/Model/Poser/2Axis/L-Type/500kg/2AxisPositioner_NoGripper/meshes/Link1.STL
    Lib/Model/Poser/2Axis/L-Type/500kg/2AxisPositioner_NoGripper/meshes/Link2.STL
    Lib/Model/Poser/2Axis/L-Type/500kg/2AxisPositioner_WithGripper/2AxisPositioner_WithGripper.urdf
    Lib/Model/Poser/2Axis/L-Type/500kg/2AxisPositioner_WithGripper/meshes/base_link.STL
    Lib/Model/Poser/2Axis/L-Type/500kg/2AxisPositioner_WithGripper/meshes/Link1.STL
    Lib/Model/Poser/2Axis/L-Type/500kg/2AxisPositioner_WithGripper/meshes/Link2.STL

    Lib/Model/ExtAx/RotateBase/RotateBase.urdf
    Lib/Model/ExtAx/RotateBase/meshes/base_link.STL
    Lib/Model/ExtAx/RotateBase/meshes/Link1.STL

    )

add_custom_target(RobotScripts ALL
    SOURCES ${Robot_Scripts} ${Robot_Resources}
)

fc_target_copy_resource(RobotScripts
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/Mod/Robot
    ${Robot_Scripts})

fc_target_copy_resource(RobotScripts
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_DATADIR}/Mod/Robot
    ${Robot_Resources})

INSTALL(
    FILES
        ${Robot_Scripts}
    DESTINATION
        Mod/Robot
)

INSTALL(
    DIRECTORY
        Lib
    DESTINATION
        ${CMAKE_INSTALL_DATADIR}/Mod/Robot
    PATTERN "Makefile*" EXCLUDE
    PATTERN "*.pdf" EXCLUDE
)


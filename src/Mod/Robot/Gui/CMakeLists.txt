if(MSVC)
    add_definitions(-DFCGuiRobot -DHAVE_ACOSH -DHAVE_ASINH -DHAVE_ATANH)
else(MSVC)
    add_definitions(-DHAVE_LIMITS_H -DHAVE_CONFIG_H)
endif(MSVC)

include_directories(
    ${CMAKE_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}
    ${Boost_INCLUDE_DIRS}
    ${COIN3D_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIR}
    ${OCC_INCLUDE_DIR}
    ${QT_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIRS}
    ${XercesC_INCLUDE_DIRS}
)
link_directories(${OCC_LIBRARY_DIR})

set(RobotGui_LIBS
    Robot
    PartGui
    FreeCADGui
)

SET(RobotGui_SRCS_Module
    AppRobotGui.cpp
    Resources/Robot.qrc
    PreCompiled.cpp
    PreCompiled.h
    Workbench.cpp
    Workbench.h
)
SOURCE_GROUP("Module" FILES ${RobotGui_SRCS_Module})

if(BUILD_QT5)
    qt5_add_resources(RobotResource_SRCS Resources/Robot.qrc)
else()
    qt4_add_resources(RobotResource_SRCS Resources/Robot.qrc)
endif()

SOURCE_GROUP("Resources" FILES ${RobotResource_SRCS})


SET(Communication_SRCS
    Comm/MechanicGroupCommManager.cpp
    Comm/MechanicGroupCommManager.h)
#SOURCE_GROUP("Comm" FILES ${Communication_SRCS})

SET(RobotGui_SRC_Module
    AppRobotGui.cpp
    Resources/Robot.qrc
    PreCompiled.cpp
    PreCompiled.h
    Workbench.cpp
    Workbench.h
)
SOURCE_GROUP("Module" FILES ${RobotGui_SRC_Module})


SET(RobotGui_SRC_Commands
    Commands/CommandTools.cpp
    Commands/CommandModels.cpp
    Commands/CommandMechs.cpp
    Commands/CommandTasks.cpp
)
SOURCE_GROUP("Commands" FILES ${RobotGui_SRC_Commands})

SET(RobotGui_SRC_Utilities
    Utilites/DraggerUtility.cpp
    Utilites/DraggerUtility.h
    Utilites/DocUtility.cpp
    Utilites/DocUtility.h
    Utilites/WidgetUtility.cpp
    Utilites/WidgetUtility.h
    Utilites/ViewProviderFrameObject.h
    Utilites/ViewProviderFrameObject.cpp
    ${Communication_SRCS})
SOURCE_GROUP("Utilites" FILES ${RobotGui_SRC_Utilities})

# Tool Object
SET(RobotGui_SRC_ToolViewProvider
    Tool/ViewProviderToolObject.h
    Tool/ViewProviderToolObject.cpp
    Tool/ViewProviderTorchObject.h
    Tool/ViewProviderTorchObject.cpp
    Tool/ViewProviderScannerObject.cpp
    Tool/ViewProviderScannerObject.h)

set(RobotGui_SRC_ToolTaskDlg
    Tool/TaskDlgToolSetupPanel.h
    Tool/TaskDlgToolSetupPanel.cpp
    Tool/ToolOperationPanel.cpp
    Tool/ToolOperationPanel.h)

set(RobotGui_SRC_ToolTaskBox
    Tool/TaskBoxTorchToolSetupPanel.cpp
    Tool/TaskBoxTorchToolSetupPanel.h
    Tool/TaskBoxLaserScannerSetupPanel.cpp
    Tool/TaskBoxLaserScannerSetupPanel.h)

set(RobotGui_UIC_Tool
    Tool/ToolOperationPanel.ui
    Tool/TaskDlgToolSetupPanel.ui
    Tool/TaskBoxLaserScannerSetupPanel.ui
    Tool/TaskBoxTorchToolSetupPanel.ui)


SOURCE_GROUP("Tool" FILES ${RobotGui_SRC_ToolViewProvider}
                          ${RobotGui_SRC_ToolTaskDlg}
                          ${RobotGui_SRC_ToolTaskBox}
                          ${RobotGui_UIC_Tool})

# Planning Objects

set(RobotGui_SRC_PlanningObjectViewProvider
    PlanningObj/ViewProviderPlanningObj.cpp
    PlanningObj/ViewProviderPlanningObj.h)

set(RobotGui_SRC_PlanningObjectTaskDlg
    PlanningObj/TaskDlgPlanningObject.h
    PlanningObj/TaskDlgPlanningObject.cpp)

set(RobotGui_UIC_PlanningObj
    PlanningObj/TaskDlgPlanningObject.ui)


SOURCE_GROUP("PlanningObj" FILES ${RobotGui_SRC_PlanningObjectViewProvider}
                                 ${RobotGui_SRC_PlanningObjectTaskDlg}
                                 ${RobotGui_UIC_PlanningObj})


# Mechanics
set(RobotGui_SRC_MechanicViewProvider
#    Mechanics/ViewProviderRobot6AxisObject.cpp
#    Mechanics/ViewProviderRobot6AxisObject.h
#    Mechanics/ViewProviderMechanicDevice.cpp
#    Mechanics/ViewProviderMechanicDevice.h
    Mechanics/ViewProviderMechanicBase.cpp
    Mechanics/ViewProviderMechanicBase.h
    Mechanics/ViewProviderMechanicRobot.cpp
    Mechanics/ViewProviderMechanicRobot.h
    Mechanics/ViewProviderMechanicGroup.cpp
    Mechanics/ViewProviderMechanicGroup.h)

set(RobotGui_SRC_MechanicsTaskBox
    Mechanics/TaskBoxMechanicGroupPanel.cpp
    Mechanics/TaskBoxMechanicGroupPanel.h
    Mechanics/TaskBoxRobotTeachPanel.cpp
    Mechanics/TaskBoxRobotTeachPanel.h
    Mechanics/TaskBoxPoserSetupPanel.cpp
    Mechanics/TaskBoxPoserSetupPanel.h
    )

set(RobotGui_SRC_MechanicsTaskDlg
    Mechanics/TaskDlgMechanicControl.cpp
    Mechanics/TaskDlgMechanicControl.h
    Mechanics/QDialogCalibrationPanel.cpp
    Mechanics/QDialogCalibrationPanel.h
    Mechanics/MechanicSelectionPanel.cpp
    Mechanics/MechanicSelectionPanel.h
    )

set(RobotGui_UIC_Mechanics
    Mechanics/TaskBoxMechanicGroupPanel.ui
    Mechanics/TaskBoxRobotTeachPanel.ui
    Mechanics/TaskBoxPoserSetupPanel.ui
    Mechanics/MechanicSelectionPanel.ui
    Mechanics/QDialogCalibrationPanel.ui
    )

SOURCE_GROUP("Mechanics" FILES ${RobotGui_SRC_MechanicViewProvider}
                               ${RobotGui_SRC_MechanicsTaskBox}
                               ${RobotGui_SRC_MechanicsTaskDlg}
                               ${RobotGui_UIC_Mechanics})
# TracObj Related
set(RobotGui_SRC_TracObjectViewProvider
    Trac/ViewProviderEdgebasedTracObject.h
    Trac/ViewProviderEdgebasedTracObject.cpp
    Trac/ViewProviderRobotTrajectory.h
    Trac/ViewProviderRobotTrajectory.cpp)

SOURCE_GROUP("Trac" FILES ${RobotGui_SRC_TracObjectViewProvider})

# TaskManage Objects
set(RobotGui_SRC_TaskManageViewProvider
    TaskManage/ViewProviderTaskObject.cpp
    TaskManage/ViewProviderTaskObject.h)

set(RobotGui_SRC_TaskManageTaskBox
    TaskManage/TaskBox_TaskManager.cpp
    TaskManage/TaskBox_TaskManager.h
#    TaskManage/TaskBox_ActionManager.cpp
#    TaskManage/TaskBox_ActionManager.h
    TaskManage/TaskBox_EdgeBasedTracEditor.cpp
    TaskManage/TaskBox_EdgeBasedTracEditor.h
    TaskManage/TaskBox_FreerunTracEditor.cpp
    TaskManage/TaskBox_FreerunTracEditor.h
    TaskManage/TaskBox_TracSimulator.cpp
    TaskManage/TaskBox_TracSimulator.h)

set(RobotGui_SRC_TaskManageTaskDlg
    TaskManage/TaskDlg_TaskManage.h
    TaskManage/TaskDlg_TaskManage.cpp)

set(RobotGui_UIC_TaskManage
#    TaskManage/TaskBox_ActionManager.ui
    TaskManage/TaskBox_TaskManager.ui
    TaskManage/TaskBox_FreerunTracEditor.ui
#    TaskManage/TaskBox_EdgeTracGenerator.ui
    TaskManage/TaskBox_EdgeBasedTracEditor.ui
    TaskManage/TaskBox_TracSimulator.ui
    )


SOURCE_GROUP("TaskManage" FILES ${RobotGui_SRC_TaskManageViewProvider}
                                ${RobotGui_SRC_TaskManageTaskBox}
                                ${RobotGui_SRC_TaskManageTaskDlg}
                                ${RobotGui_UIC_TaskManage})
# ViewProvider Compact
SET(RobotGui_SRC_ViewProvider
    ${RobotGui_SRC_ToolViewProvider}
    ${RobotGui_SRC_PlanningObjectViewProvider}
    ${RobotGui_SRC_MechanicViewProvider}
    ${RobotGui_SRC_TracObjectViewProvider}
    ${RobotGui_SRC_TaskManageViewProvider})

#SOURCE_GROUP("ViewProvider" FILES ${RobotGui_SRC_ViewProvider})

SET(RobotGui_SRC_TaskBoxes
    ${RobotGui_SRC_ToolTaskBox}
    ${RobotGui_SRC_MechanicsTaskBox}
    ${RobotGui_SRC_TracObjectTaskBox}
    ${RobotGui_SRC_TaskManageTaskBox})

#SOURCE_GROUP("Task_Boxes" FILES ${RobotGui_SRC_TaskBoxes})

SET(RobotGui_SRC_TaskDlg
    ${RobotGui_SRC_ToolTaskDlg}
    ${RobotGui_SRC_PlanningObjectTaskDlg}
    ${RobotGui_SRC_MechanicsTaskDlg}
    ${RobotGui_SRC_TaskManageTaskDlg})

#SOURCE_GROUP("Task_Dialogs" FILES ${RobotGui_SRC_TaskDlg})
set(RobotGui_UIC_SRCS
    ${RobotGui_UIC_Tool}
    ${RobotGui_UIC_PlanningObj}
    ${RobotGui_UIC_Mechanics}
    ${RobotGui_UIC_TaskManage}
)

if(BUILD_QT5)
    qt5_wrap_ui(RobotGui_UIC_HDRS ${RobotGui_UIC_SRCS})
else()
    qt4_wrap_ui(RobotGui_UIC_HDRS ${RobotGui_UIC_SRCS})
endif()


SET(RobotGui_SRC
    ${RobotGui_UIC_HDRS}
    ${RobotResource_SRCS}
    ${RobotGui_SRC_Module}
    ${RobotGui_SRC_ViewProvider}
    ${RobotGui_SRC_Commands}
    ${RobotGui_SRC_TaskBoxes}
    ${RobotGui_SRC_TaskDlg}
    ${RobotGui_SRC_Utilities}
)

SET(RobotGuiIcon_SVG
    Resources/icons/RobotWorkbench.svg
)

add_library(RobotGui SHARED ${RobotGui_SRC})
target_link_libraries(RobotGui ${RobotGui_LIBS})

SET_BIN_DIR(RobotGui RobotGui /Mod/Robot)
SET_PYTHON_PREFIX_SUFFIX(RobotGui)

fc_copy_sources(RobotGui "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_DATADIR}/Mod/Robot" ${RobotGuiIcon_SVG})

#fc_target_copy_resource(RobotGui
#    ${CMAKE_SOURCE_DIR}/src/Mod/Robot
#    ${CMAKE_BINARY_DIR}/Mod/Robot
#    InitGui.py MovieTool.py)

#SET_BIN_DIR(RobotGui RobotGui /Mod/Robot)
#SET_PYTHON_PREFIX_SUFFIX(RobotGui)

INSTALL(TARGETS RobotGui DESTINATION ${CMAKE_INSTALL_LIBDIR})

fc_copy_sources(RobotGui "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_DATADIR}/Mod/Robot" ${RobotGuiIcon_SVG})

INSTALL(TARGETS RobotGui DESTINATION ${CMAKE_INSTALL_LIBDIR})
INSTALL(FILES ${RobotGuiIcon_SVG} DESTINATION "${CMAKE_INSTALL_DATADIR}/Mod/Robot/Resources/icons")

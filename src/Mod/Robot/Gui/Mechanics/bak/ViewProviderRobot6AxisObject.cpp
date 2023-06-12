
#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/SbVec3f.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/draggers/SoJackDragger.h>
#include <Inventor/draggers/SoTrackballDragger.h>
#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <QFile>
#include <QFileInfo>
#endif

#include <Mod/Robot/App/Mechanics/Robot6AxisObject.h>
#include <Mod/Part/App/PartFeature.h>
#include <Mod/Part/Gui/ViewProvider.h>
#include <App/Document.h>
#include <App/VRMLObject.h>
#include <Gui/Application.h>
#include <Gui/Control.h>
#include <Gui/Command.h>
#include <Base/FileInfo.h>
#include <Base/Stream.h>
#include <Base/Console.h>
#include <sstream>
#include <Mod/Robot/App/Utilites/DS_Utility.h>

#include "ViewProviderRobot6AxisObject.h"
#include "TaskDlgMechanicControl.h"

using namespace Gui;
using namespace RobotGui;

PROPERTY_SOURCE(RobotGui::ViewProviderRobot6AxisObject, Gui::ViewProviderGeometryObject)

ViewProviderRobot6AxisObject::ViewProviderRobot6AxisObject()
  : pcDragger(0),toolShapeVP(0)
{
//    ADD_PROPERTY(InteractiveTeach,(0));

	pcRobotRoot = new Gui::SoFCSelection();
    pcRobotRoot->highlightMode = Gui::SoFCSelection::OFF;
    pcRobotRoot->ref();

    m_LinkMeshGroup = new SoSeparator();
    m_LinkMeshGroup->setName(SbName("Robot_LinkGroup"));
    m_LinkMeshGroup->ref();

    // set nodes for the manipulator outfit
    m_TcpRoot = new SoGroup();
    m_TcpRoot->ref();

}

bool ViewProviderRobot6AxisObject::generateLinkMeshNodes_fromURDF(const char *urdf_FilePath)
{
    Robot::Robot6AxisObject* robotPtr = static_cast<Robot::Robot6AxisObject*>(pcObject);
    auto urdfFileInfo = QFileInfo(QString::fromLocal8Bit(urdf_FilePath));

    int linkID = 0;
    for(auto linkName : robotPtr->getKinematicModelRef().getLinkNames()){
        std::string meshPath;
        auto path_1 = urdfFileInfo.absolutePath().toStdString() + "/meshes/" + linkName +".stl";
        auto path_2 = urdfFileInfo.absolutePath().toStdString() + "/meshes/" + linkName +".STL";
        if(QFile(QString::fromStdString(path_1)).exists())
            meshPath = path_1;
        else if(QFile(QString::fromStdString(path_2)).exists())
            meshPath = path_2;
        if(meshPath.empty()){
            std::string warning = "Can't find stl/STL file for link " + linkName +"\n";
            Base::Console().Warning(warning.c_str());
            continue;
        }
        std::string msg = string("Loading mesh file from ") + meshPath + string("\n");
        Base::Console().Message(msg.c_str());
        SoTransform *linkTrans = new SoTransform();
//        std::string jointName;
        if(linkID != 0){
            Robot::DS_Utility::PlacementToTransform(linkTrans,
                                                    robotPtr->getJointTransformation(linkID-1));
        }
        // Generate MeshNode
        auto meshNodeName = std::string(robotPtr->getNameInDocument())+"_"+linkName;
        auto meshNodePtr = Robot::MeshUtility::generateMeshNode(meshNodeName.c_str(),
                                                                meshPath.c_str(),
                                                                linkTrans,
                                                                mesh_sf);
        m_LinkNames.push_back(meshNodeName);
        if (nullptr != meshNodePtr) {
          m_LinkMeshGroup->addChild(meshNodePtr);
        };
        linkID++;
    }

//    int linkID = 0;
//    // Generate Link Group
//    for(const auto& linkMeshInfo : linkMeshPath){
//        auto& linkName = linkMeshInfo.first;
//        auto& meshPath = linkMeshInfo.second;
//        std::string msg = string("Loading ") + linkName + string(" mesh file from ") + meshPath + string("\n");
//        Base::Console().Message(msg.c_str());
//        SoTransform *linkTrans = new SoTransform();
////        std::string jointName;
//        if(linkID != 0){
//            Robot::DS_Utility::PlacementToTransform(linkTrans,
//                                                    robotPtr->getJointTransformation(linkID-1));
//        }
//        // Generate MeshNode
//        auto meshNodeName = std::string(robotPtr->getNameInDocument())+"_"+linkName;
//        auto meshNodePtr = Robot::MeshUtility::generateMeshNode(meshNodeName.c_str(),
//                                                                meshPath.c_str(),
//                                                                linkTrans,
//                                                                mesh_sf);
//        m_LinkNames.push_back(meshNodeName);
//        if (nullptr != meshNodePtr) {
//          m_LinkMeshGroup->addChild(meshNodePtr);
//        };
//        linkID++;
//    }

    return m_LinkMeshGroup->getChildren()->getLength() != 0;
}

bool ViewProviderRobot6AxisObject::updatelinkmeshPoses(const Base::Placement& basePose)
{
    Robot::Robot6AxisObject* robObj = static_cast<Robot::Robot6AxisObject*>(pcObject);
    if(m_LinkNames.size()-1 != robObj->MainAxisValues.getSize())
        return false;
    // Update BaseLink Mesh Pose
    auto nodeName = std::string("TransitionOf")+m_LinkNames[0];
    auto transNode = static_cast<SoTransform *>(m_LinkMeshGroup->getByName(nodeName.c_str()));
    if(transNode!=nullptr){
        SbMatrix M;
        M.setTransform(SbVec3f(basePose.getPosition().x,
                               basePose.getPosition().y,
                               basePose.getPosition().z),
                       SbRotation(basePose.getRotation()[0], basePose.getRotation()[1],
                                  basePose.getRotation()[2], basePose.getRotation()[3]),
                       SbVec3f(1, 1, 1));
        transNode->setMatrix(M);
    }
    // Update Link1-6 MeshPose
    for(int i = 0; i<robObj->MainAxisValues.getSize(); i++){
        auto nodeName = std::string("TransitionOf")+m_LinkNames[i+1];
        auto transNode = static_cast<SoTransform *>(m_LinkMeshGroup->getByName(nodeName.c_str()));
        if(transNode!=nullptr){
            auto newPos = robObj->getJointTransformation(i);
//            newPos = Base::Placement(newPos.toMatrix());
            SbMatrix M;
            M.setTransform(
                SbVec3f(newPos.getPosition().x,
                        newPos.getPosition().y,
                        newPos.getPosition().z),
                SbRotation(newPos.getRotation()[0], newPos.getRotation()[1],
                           newPos.getRotation()[2], newPos.getRotation()[3]),
                SbVec3f(1, 1, 1));
            transNode->setMatrix(M);
            if(toolShapeVP){
                toolShapeVP->setTransformation(robObj->getCurrentFlanPose(Robot::CoordOrigin::World).toMatrix());
            }
        }
    }
    return true;
}

bool ViewProviderRobot6AxisObject::callbackRegistered()
{
    return callBack_UpdatePanelWidgets!=nullptr;
}

ViewProviderRobot6AxisObject::~ViewProviderRobot6AxisObject()
{
    pcRobotRoot->unref();
    m_LinkMeshGroup->unref();
    m_TcpRoot->unref();
}

void ViewProviderRobot6AxisObject::attach(App::DocumentObject *pcObj)
{
    ViewProviderGeometryObject::attach(pcObj);

    addDisplayMaskMode(pcRobotRoot, "On");
    pcRobotRoot->objectName = pcObj->getNameInDocument();
    pcRobotRoot->documentName = pcObj->getDocument()->getName();
    pcRobotRoot->subElementName = "Main";
    pcRobotRoot->addChild(m_LinkMeshGroup);
    pcRobotRoot->addChild(m_TcpRoot);

    addDisplayMaskMode(m_LinkMeshGroup, "Off");
    m_LinkMeshGroup->addChild(m_TcpRoot);
}

void ViewProviderRobot6AxisObject::setDisplayMode(const char* ModeName)
{
    if ( strcmp("On",ModeName)==0 )
        setDisplayMaskMode("On");

    if ( strcmp("Off",ModeName)==0 )
        setDisplayMaskMode("Off");
    ViewProviderGeometryObject::setDisplayMode( ModeName );
}

std::vector<std::string> ViewProviderRobot6AxisObject::getDisplayModes(void) const
{
    std::vector<std::string> StrList;
    StrList.push_back("On");
    StrList.push_back("Off");
    return StrList;
}

void ViewProviderRobot6AxisObject::onChanged(const App::Property* prop)
{
    Robot::Robot6AxisObject* robObj = static_cast<Robot::Robot6AxisObject*>(pcObject);

    ViewProviderGeometryObject::onChanged(prop);
}

bool ViewProviderRobot6AxisObject::setTipPosition(const Base::Placement &new_Pose)
{
    Robot::Robot6AxisObject* robObj = static_cast<Robot::Robot6AxisObject*>(pcObject);
    return robObj->setRobotTipPose(new_Pose);
}

void ViewProviderRobot6AxisObject::setTipPoseByTeach(const Base::Placement &new_Pose)
{
    Robot::Robot6AxisObject* robObj = static_cast<Robot::Robot6AxisObject*>(pcObject);
    robObj->setTipPoseByDragger(new_Pose);
}

bool ViewProviderRobot6AxisObject::doubleClicked()
{
    std::string Msg("Edit target Robot");
    Msg += this->pcObject->Label.getValue();
    try {
      Gui::Command::openCommand(Msg.c_str());
      Gui::Command::doCommand(Gui::Command::Gui,
                              "Gui.ActiveDocument.setEdit('%s',%d)",
                              this->pcObject->getNameInDocument(),
                              Gui::ViewProvider::EditMode::Default);
      Gui::Command::commitCommand();
      return true;
    } catch (const Base::Exception &e) {
      Base::Console().Error("%s\n", e.what());
      return false;
    }

}

void ViewProviderRobot6AxisObject::updateData(const App::Property* prop)
{
    Robot::Robot6AxisObject* robObj = static_cast<Robot::Robot6AxisObject*>(pcObject);
    if (prop == &robObj->FilePath_URDF) {
        generateLinkMeshNodes_fromURDF(robObj->FilePath_URDF.getValue());
        robObj->updateAxisValues();
    }

    else if(prop == &robObj->MainAxisValues ||
            prop == &robObj->Pose_Flan ||
            prop == &robObj->Pose_Reference ||
            prop == &robObj->Pose_Ref2Base){
        updatelinkmeshPoses(robObj->getCurrentBasePose());
        if(callbackRegistered()){
            callBack_UpdatePanelWidgets();
        }
        if(pcDragger!=nullptr){
            pcDragger->setDraggerPosition(robObj->getTeachDraggerPose());
        }
    }

    else if(prop == &robObj->TeachCoordIndex){
        if(pcDragger!=nullptr){
            pcDragger->setDraggerPosition(robObj->getTeachDraggerPose());
        }
    }

    else if(prop == &robObj->Visible){
        setVisible(robObj->Visible.getValue());
    }

    else if (prop == &robObj->CurrentToolIndex){
        claimChildren();
        if(pcDragger!=nullptr){
            pcDragger->setDraggerPosition(robObj->getTeachDraggerPose());
        }
    }

    else if(prop == &robObj->Activated){
        if(robObj->Activated.getValue() && !robObj->isEditing()){
            setEdit(0);
        }
        else{
            unsetEdit(0);
        }
        robObj->InteractiveTeach.setValue(robObj->Activated.getValue());
    }

    else if (prop == &robObj->InteractiveTeach) {
        if (robObj->InteractiveTeach.getValue()) {
            if (pcDragger == nullptr){
                pcDragger = new InteractiveDragger(this->getRoot(),
                                                   robObj->getTeachDraggerPose(),
                                                   DraggerUsage::Interaction);
                pcDragger->setup_incCallback(std::bind(&ViewProviderRobot6AxisObject::DraggerMotionCallback,
                                                       this, std::placeholders::_1));
                pcDragger->setup_finishCallback(std::bind(&ViewProviderRobot6AxisObject::DraggerFinishCallback,
                                                          this, std::placeholders::_1));
                pcDragger->setAttachingViewProvider(this);
            }
        }
        else {
            if(pcDragger!=nullptr){
                pcDragger->destroyDragger();
                pcDragger = nullptr;
            }
        }
    }
}

bool ViewProviderRobot6AxisObject::setEdit(int ModNum){
    auto t_Robot = static_cast<Robot::Robot6AxisObject *>(pcObject);
    t_Robot->InteractiveTeach.setValue(true);
    auto dlg = new TaskDlgMechanicControl(t_Robot);
    if (dlg == nullptr)
      return false;
    callBack_UpdatePanelWidgets = std::bind(&TaskDlgMechanicControl::signal_updatePanelWidgets, dlg);
    Gui::Control().showDialog(dlg);
    t_Robot->setEditingStatus(true);
    return true;
}

void ViewProviderRobot6AxisObject::unsetEdit(int ModeNum)
{
    Robot::Robot6AxisObject* t_Robot = static_cast<Robot::Robot6AxisObject*>(pcObject);
    t_Robot->InteractiveTeach.setValue(false);
    callBack_UpdatePanelWidgets = nullptr;
    Gui::Control().closeDialog();
    t_Robot->setEditingStatus(false);
}

std::vector<App::DocumentObject *> ViewProviderRobot6AxisObject::claimChildren() const
{
    Robot::Robot6AxisObject* t_Robot = static_cast<Robot::Robot6AxisObject*>(pcObject);
    return t_Robot->getChildrenList();
}

void ViewProviderRobot6AxisObject::DraggerMotionCallback(InteractiveDragger *t_dragger)
{
    Robot::Robot6AxisObject* robObj = static_cast<Robot::Robot6AxisObject*>(pcObject);
    auto diff = t_dragger->getLastPose().inverse() * t_dragger->getCurrentPose();
    robObj->setTipPoseByDiff(diff);
    //    t_dragger->setDraggerTrans(robObj->getCurrentTipPose().getPosition());
}

void ViewProviderRobot6AxisObject::DraggerFinishCallback(InteractiveDragger *t_dragger)
{
    Robot::Robot6AxisObject* robObj = static_cast<Robot::Robot6AxisObject*>(pcObject);
    t_dragger->setDraggerPosition(robObj->getTeachDraggerPose());
}


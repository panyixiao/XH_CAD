
#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
# include <Inventor/SoDB.h>
# include <Inventor/SoInput.h>
# include <Inventor/SbVec3f.h>
# include <Inventor/nodes/SoSeparator.h>
# include <Inventor/nodes/SoTransform.h>
# include <Inventor/nodes/SoSphere.h>
# include <Inventor/nodes/SoRotation.h>
# include <Inventor/actions/SoSearchAction.h>
# include <Inventor/draggers/SoJackDragger.h>
# include <Inventor/draggers/SoTrackballDragger.h>
# include <Inventor/VRMLnodes/SoVRMLTransform.h>
# include <QFile>
#endif

#include <Mod/Robot/App/Mechanics/MechanicGroup.h>
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

#include "ViewProviderMechanicGroup.h"
#include "TaskDlgMechanicControl.h"

using namespace Gui;
using namespace RobotGui;

PROPERTY_SOURCE(RobotGui::ViewProviderMechanicGroup, Gui::ViewProviderGeometryObject)

ViewProviderMechanicGroup::ViewProviderMechanicGroup()
  : pcDragger(0),toolShapeVP(0)
{
    ADD_PROPERTY(EnableSelection,(true));

    pcRoot = new Gui::SoFCSelection();
    pcRoot->highlightMode = Gui::SoFCSelection::OFF;
    pcRoot->ref();

    m_LinkMeshGroup = new SoSeparator();
    m_LinkMeshGroup->setName(SbName("Robot_LinkGroup"));
    m_LinkMeshGroup->ref();

    // set nodes for the manipulator outfit
    m_TcpRoot = new SoGroup();
    m_TcpRoot->ref();

}

bool ViewProviderMechanicGroup::generateLinkMeshNodes(const char *FileName)
{
    // Read Link Mesh File Path
    std::ifstream in(FileName);
    if(!in)return false;
    std::vector<Base::Placement> initMeshPoses;
    std::vector<std::string> split_1, split_2,split_3;
    // over read the header
    std::string tmp_line;

    std::map<std::string, std::string> linkMeshPath;

    // Readin mesh file path and initial Poses
    while(std::getline(in, tmp_line)){
        Robot::DS_Utility::split(tmp_line,',',split_1);
        if(split_1.size() == 2){
            Robot::DS_Utility::split(split_1[1], ';', split_2);
            if(split_2.size() == 2){
                linkMeshPath.insert(std::make_pair(split_1[0],split_2[0]));
                Robot::DS_Utility::split(split_2[1],' ', split_3);
                if(split_3.size() == 6){
                    Base::Placement t_Pose;
                    t_Pose.setPosition(Base::Vector3d(std::atof(split_3[0].c_str()),
                                                      std::atof(split_3[1].c_str()),
                                                      std::atof(split_3[2].c_str())));
                    float rad_r, rad_p, rad_y;
                    rad_r = std::atof(split_3[3].c_str());
                    rad_p = std::atof(split_3[4].c_str());
                    rad_y = std::atof(split_3[5].c_str());
                    t_Pose.setRotation(Robot::DS_Utility::convertFromEulerAngle(Robot::DS_Utility::convert_Rad2Deg(rad_r),
                                                                                Robot::DS_Utility::convert_Rad2Deg(rad_p),
                                                                                Robot::DS_Utility::convert_Rad2Deg(rad_y)));
                    initMeshPoses.push_back(t_Pose);
                }
            }
        }
        split_1.clear();
        split_2.clear();
        split_3.clear();
    }

    Robot::MechanicGroup* t_Group = static_cast<Robot::MechanicGroup*>(pcObject);
    if(!t_Group->setupJointChain(initMeshPoses))
        return false;

    int linkID = 0;

    // Generate Link Group
    for(const auto& linkMeshInfo : linkMeshPath){
        auto& linkName = linkMeshInfo.first;
        auto& meshPath = linkMeshInfo.second;
        std::string msg = string("Loading ") + linkName + string(" mesh file from ") +
                         meshPath + string("\n");
        Base::Console().Message(msg.c_str());
        SoTransform *linkTrans = new SoTransform();
//        std::string jointName;
        if(linkID != 0){
            Robot::DS_Utility::PlacementToTransform(linkTrans,
                                                    t_Group->getJointTransformation(linkID-1));
        }
        // Generate MeshNode
        auto meshNodePtr = Robot::MeshUtility::generateMeshNode(linkName.c_str(),
                                                                meshPath.c_str(),
                                                                linkTrans,
                                                                mesh_sf);
        m_LinkNames.push_back(linkName);
        if (nullptr != meshNodePtr) {
          m_LinkMeshGroup->addChild(meshNodePtr);
        };
        linkID++;
    }

    return m_LinkMeshGroup->getChildren()->getLength() != 0;
}

bool ViewProviderMechanicGroup::generateLinkMeshNodes_fromURDF(const char *FileName)
{
    // Read Link Mesh File Path
    std::ifstream in(FileName);
    if(!in)
      return false;
    std::vector<std::string> split_1, split_2;
    // over read the header
    std::string tmp_line;

    std::map<std::string, std::string> linkMeshPath;

    // Readin mesh file path and initial Poses
    while(std::getline(in, tmp_line)){
        Robot::DS_Utility::split(tmp_line,',',split_1);
        if(split_1.size() == 2){
            Robot::DS_Utility::split(split_1[1], ';', split_2);
            if(split_2.size() == 2){
                linkMeshPath.insert(std::make_pair(split_1[0],split_2[0]));
            }
        }
        split_1.clear();
        split_2.clear();
    }

    Robot::MechanicGroup* t_Group = static_cast<Robot::MechanicGroup*>(pcObject);

    int linkID = 0;
    // Generate Link Group
    for(const auto& linkMeshInfo : linkMeshPath){
        auto& linkName = linkMeshInfo.first;
        auto& meshPath = linkMeshInfo.second;
        std::string msg = string("Loading ") + linkName + string(" mesh file from ") +
                         meshPath + string("\n");
        Base::Console().Message(msg.c_str());
        SoTransform *linkTrans = new SoTransform();
//        std::string jointName;
        if(linkID != 0){
            Robot::DS_Utility::PlacementToTransform(linkTrans,
                                                    t_Group->getJointTransformation(linkID-1));
        }
        // Generate MeshNode
        auto meshNodeName = std::string(t_Group->getNameInDocument())+"_"+linkName;
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

    return m_LinkMeshGroup->getChildren()->getLength() != 0;
}

bool ViewProviderMechanicGroup::updatelinkmeshPoses(const Base::Placement& basePose)
{
    Robot::MechanicGroup *t_Group = static_cast<Robot::MechanicGroup*>(pcObject);
    if(m_LinkNames.size()-1 != t_Group->AxisValues.getSize())
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
    for(int i = 0; i<t_Group->AxisValues.getSize(); i++){
        auto nodeName = std::string("TransitionOf")+m_LinkNames[i+1];
        auto transNode = static_cast<SoTransform *>(m_LinkMeshGroup->getByName(nodeName.c_str()));
        if(transNode!=nullptr){
            auto newPos = t_Group->getJointTransformation(i);
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
                toolShapeVP->setTransformation(t_Group->getGroupFlanPose(Robot::CoordOrigin::World).toMatrix());
            }
        }
    }
    return true;
}

bool ViewProviderMechanicGroup::updateTipPanelCallbackRegistered()
{
    return callBack_UpdatePanelWidgets!=nullptr;
}

ViewProviderMechanicGroup::~ViewProviderMechanicGroup()
{
    pcRoot->unref();
    m_LinkMeshGroup->unref();
    m_TcpRoot->unref();
}

void ViewProviderMechanicGroup::attach(App::DocumentObject *pcObj)
{
    ViewProviderGeometryObject::attach(pcObj);

    addDisplayMaskMode(pcRoot, "On");
    pcRoot->objectName = pcObj->getNameInDocument();
    pcRoot->documentName = pcObj->getDocument()->getName();
    pcRoot->subElementName = "Main";
    pcRoot->addChild(m_LinkMeshGroup);
    pcRoot->addChild(m_TcpRoot);

    addDisplayMaskMode(m_LinkMeshGroup, "Off");
    m_LinkMeshGroup->addChild(m_TcpRoot);
}

void ViewProviderMechanicGroup::setDisplayMode(const char* ModeName)
{
    if ( strcmp("On",ModeName)==0 )
        setDisplayMaskMode("On");

    if ( strcmp("Off",ModeName)==0 )
        setDisplayMaskMode("Off");
    ViewProviderGeometryObject::setDisplayMode( ModeName );
}

std::vector<std::string> ViewProviderMechanicGroup::getDisplayModes(void) const
{
    std::vector<std::string> StrList;
    StrList.push_back("On");
    StrList.push_back("Off");
    return StrList;
}

void ViewProviderMechanicGroup::onChanged(const App::Property* prop)
{
//    Robot::MechanicGroup* t_Group = static_cast<Robot::MechanicGroup*>(pcObject);
    if(prop == &EnableSelection){
        if(pcDragger!=nullptr){
            pcDragger->enableSceneGraphSelection(EnableSelection.getValue());
        }
    }

    ViewProviderGeometryObject::onChanged(prop);
}

bool ViewProviderMechanicGroup::setTipPosition(const Base::Placement &new_Pose)
{
    Robot::MechanicGroup* t_Group = static_cast<Robot::MechanicGroup*>(pcObject);
    return t_Group->setTipPose(new_Pose);
}

void ViewProviderMechanicGroup::setTipPoseByTeach(const Base::Placement &new_Pose)
{
    Robot::MechanicGroup* t_Group = static_cast<Robot::MechanicGroup*>(pcObject);
    t_Group->setTipPoseByDraggerPose(new_Pose);
}

bool ViewProviderMechanicGroup::doubleClicked()
{
    std::string Msg("Edit target MechanicGroup");
    Msg += this->pcObject->Label.getValue();
    try {
      Gui::Command::openCommand(Msg.c_str());
      Gui::Command::doCommand(Gui::Command::Gui,
                              "Gui.ActiveDocument.setEdit('%s',%d)",
                              this->pcObject->getNameInDocument(),
                              Gui::ViewProvider::EditMode::Default);
      Gui::Command::commitCommand();
      return true;
    }
    catch (const Base::Exception &e) {
      Base::Console().Error("%s\n", e.what());
      return false;
    }
}

void ViewProviderMechanicGroup::updateData(const App::Property* prop)
{
    Robot::MechanicGroup* t_Group = static_cast<Robot::MechanicGroup*>(pcObject);
    if(t_Group->isRestoring())
        return;
    if (prop == &t_Group->File_Mesh) {
        generateLinkMeshNodes_fromURDF(t_Group->File_Mesh.getValue());
        t_Group->updateAxisValues();
    }

    else if(prop == &t_Group->NetworkConnected){
        if(callBack_ConnectedToStation!=nullptr)
            callBack_ConnectedToStation(t_Group->NetworkConnected.getValue());
    }

    else if(prop == &t_Group->AxisValues){
        if(callBack_UpdatePanelWidgets!=nullptr){
            callBack_UpdatePanelWidgets();
        }
        if(pcDragger!=nullptr){
            pcDragger->setDraggerPosition(t_Group->getTeachDraggerPose());
        }
    }

    else if(prop == &t_Group->ActiveToolIndex){
        if(pcDragger!=nullptr){
            pcDragger->setDraggerPosition(t_Group->getTeachDraggerPose());
        }
    }

    else if(prop == &t_Group->TeachCoordIndex){
        if(pcDragger!=nullptr){
            pcDragger->setDraggerPosition(t_Group->getTeachDraggerPose());
        }
    }

    else if(prop == &t_Group->Visiable){
        setVisible(t_Group->Visiable.getValue());
    }

    else if(prop == &t_Group->Activated){
        if(t_Group->Activated.getValue() && !t_Group->isEditing()){
            setEdit(0);
        }
        else{
            unsetEdit(0);
        }
        t_Group->InteractiveTeach.setValue(t_Group->Activated.getValue());
    }

    else if (prop == &t_Group->InteractiveTeach) {
        if (t_Group->InteractiveTeach.getValue()) {
            if (pcDragger == nullptr){
                pcDragger = new InteractiveDragger(this->getRoot(),
                                                   t_Group->getTeachDraggerPose(),
                                                   DraggerUsage::Interaction);
                pcDragger->setup_incCallback(std::bind(&ViewProviderMechanicGroup::DraggerMotionCallback,
                                                       this, std::placeholders::_1));
                pcDragger->setup_finishCallback(std::bind(&ViewProviderMechanicGroup::DraggerFinishCallback,
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

bool ViewProviderMechanicGroup::setEdit(int ModNum){
    auto t_Group = static_cast<Robot::MechanicGroup *>(pcObject);
    t_Group->InteractiveTeach.setValue(true);
    auto dlg = new TaskDlgMechanicControl(t_Group);
    if (dlg == nullptr)
      return false;
    callBack_UpdatePanelWidgets = std::bind(&TaskDlgMechanicControl::signal_updatePanelWidgets,
                                            dlg);
    callBack_ConnectedToStation = std::bind(&TaskDlgMechanicControl::signal_stationConnected,
                                            dlg,std::placeholders::_1);
    Gui::Control().showDialog(dlg);
    t_Group->setEditingStatus(true);
    return true;
}

void ViewProviderMechanicGroup::unsetEdit(int ModeNum)
{
    Robot::MechanicGroup* t_Group = static_cast<Robot::MechanicGroup*>(pcObject);
    t_Group->InteractiveTeach.setValue(false);
    callBack_UpdatePanelWidgets = nullptr;
    callBack_ConnectedToStation = nullptr;
    Gui::Control().closeDialog();
    t_Group->setEditingStatus(false);
}

std::vector<App::DocumentObject *> ViewProviderMechanicGroup::claimChildren() const
{
    Robot::MechanicGroup* t_Group = static_cast<Robot::MechanicGroup*>(pcObject);
    return t_Group->getChildrenList();
}

void ViewProviderMechanicGroup::DraggerMotionCallback(InteractiveDragger *t_dragger)
{
    Robot::MechanicGroup* t_Group = static_cast<Robot::MechanicGroup*>(pcObject);
    auto diff = t_dragger->getLastPose().inverse() * t_dragger->getCurrentPose();
    t_Group->setTipPoseByDiff(diff);
}

void ViewProviderMechanicGroup::DraggerFinishCallback(InteractiveDragger *t_dragger)
{
    Robot::MechanicGroup* t_Group = static_cast<Robot::MechanicGroup*>(pcObject);
    t_dragger->setDraggerPosition(t_Group->getTeachDraggerPose());
}


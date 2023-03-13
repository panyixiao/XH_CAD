
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
#include <QFile>
#include <QFileInfo>
#endif

#include <Mod/Robot/App/Mechanics/MechanicBase.h>
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

#include "ViewProviderMechanicBase.h"
#include "TaskDlgMechanicControl.h"

using namespace Gui;
using namespace RobotGui;

PROPERTY_SOURCE(RobotGui::ViewProviderMechanicBase,
                Gui::ViewProviderGeometryObject)

ViewProviderMechanicBase::ViewProviderMechanicBase()
  : pcDragger(0),toolShapeVP(0)
{
    pcRoot = new Gui::SoFCSelection();
    pcRoot->highlightMode = Gui::SoFCSelection::OFF;
    pcRoot->ref();

    m_LinkMeshs = new SoSeparator();
    m_LinkMeshs->setName(SbName("Mechanics"));
    m_LinkMeshs->ref();

    // set nodes for the manipulator outfit
    m_TcpRoot = new SoGroup();
    m_TcpRoot->ref();
}

bool ViewProviderMechanicBase::generateLinkMeshNodes_fromURDF(const string &filePath_urdf)
{
    if(filePath_urdf.empty())
        return false;
    Robot::MechanicBase* t_MechanicObj = static_cast<Robot::MechanicBase*>(pcObject);
    auto urdfFileInfo = QFileInfo(QString::fromStdString(filePath_urdf));

    int linkID = 0;
    // Generate Link Group
    for(const auto& linkName : t_MechanicObj->getKinematicModelRef().getLinkNames()){
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
        std::string msg = string("Loading ") + linkName + string(" mesh file from ") +
                         meshPath + string("\n");
        Base::Console().Message(msg.c_str());
        SoTransform *linkTrans = new SoTransform();
//        std::string jointName;
        if(linkID != 0){
            Robot::DS_Utility::PlacementToTransform(linkTrans,
                                                    t_MechanicObj->getJointTransformation(linkID-1));
        }
        // Generate MeshNode
        auto meshNodeName = std::string(t_MechanicObj->getNameInDocument())+"_"+linkName;
        auto meshNodePtr = Robot::MeshUtility::generateMeshNode(meshNodeName.c_str(),
                                                                meshPath.c_str(),
                                                                linkTrans,
                                                                mesh_sf);
        m_LinkNames.push_back(meshNodeName);
        if (nullptr != meshNodePtr) {
          m_LinkMeshs->addChild(meshNodePtr);
        };
        linkID++;
    }

    return m_LinkMeshs->getChildren()->getLength() != 0;
}

bool ViewProviderMechanicBase::updatelinkmeshPoses()
{
    Robot::MechanicBase* t_MechanicObj = static_cast<Robot::MechanicBase*>(pcObject);
    if(m_LinkNames.size()-1 != t_MechanicObj->AxisValues.getSize())
        return false;
    Base::Placement basePose = t_MechanicObj->getOriginPose();
    // Update BaseLink Mesh Pose
    auto nodeName = std::string("TransitionOf")+m_LinkNames[0];
    auto transNode = static_cast<SoTransform *>(m_LinkMeshs->getByName(nodeName.c_str()));
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
    // Update MeshPoses
    for(int i = 0; i<t_MechanicObj->AxisValues.getSize(); i++){
        auto nodeName = std::string("TransitionOf")+m_LinkNames[i+1];
        auto transNode = static_cast<SoTransform *>(m_LinkMeshs->getByName(nodeName.c_str()));
        if(transNode!=nullptr){
            auto newPos = t_MechanicObj->getJointTransformation(i);
//            newPos = Base::Placement(newPos.toMatrix());
            SbMatrix M;
            M.setTransform(SbVec3f(newPos.getPosition().x,
                                   newPos.getPosition().y,
                                   newPos.getPosition().z),
                            SbRotation(newPos.getRotation()[0], newPos.getRotation()[1],
                                       newPos.getRotation()[2], newPos.getRotation()[3]),
                            SbVec3f(1, 1, 1));
            transNode->setMatrix(M);
            if(toolShapeVP){
                toolShapeVP->setTransformation(t_MechanicObj->getCurrentTipPose().toMatrix());
            }
        }
    }
    return true;
}

bool ViewProviderMechanicBase::callbackRegistered()
{
    return callBack_UpdatePanelWidgets!=nullptr;
}

ViewProviderMechanicBase::~ViewProviderMechanicBase()
{
    pcRoot->unref();
    m_LinkMeshs->unref();
    m_TcpRoot->unref();
}

void ViewProviderMechanicBase::attach(App::DocumentObject *pcObj)
{
    ViewProviderGeometryObject::attach(pcObj);

    addDisplayMaskMode(pcRoot, "On");
    pcRoot->objectName = pcObj->getNameInDocument();
    pcRoot->documentName = pcObj->getDocument()->getName();
    pcRoot->subElementName = "Main";
    pcRoot->addChild(m_LinkMeshs);
    pcRoot->addChild(m_TcpRoot);

    addDisplayMaskMode(m_LinkMeshs, "Off");
    m_LinkMeshs->addChild(m_TcpRoot);
}

void ViewProviderMechanicBase::setDisplayMode(const char* ModeName)
{
    if ( strcmp("On",ModeName)==0 )
        setDisplayMaskMode("On");

    if ( strcmp("Off",ModeName)==0 )
        setDisplayMaskMode("Off");
    ViewProviderGeometryObject::setDisplayMode( ModeName );
}

std::vector<std::string> ViewProviderMechanicBase::getDisplayModes(void) const
{
    std::vector<std::string> StrList;
    StrList.push_back("On");
    StrList.push_back("Off");
    return StrList;
}

void ViewProviderMechanicBase::onChanged(const App::Property* prop)
{
    ViewProviderGeometryObject::onChanged(prop);
}

bool ViewProviderMechanicBase::setTipPosition(const Base::Placement &new_Pose)
{
    Robot::MechanicBase* t_MechanicObj = static_cast<Robot::MechanicBase*>(pcObject);
//    return t_MechanicObj->setCurrentTip(new_Pose);
    return false;
}

void ViewProviderMechanicBase::setTipPoseByTeach(const Base::Placement &new_Pose)
{
    Robot::MechanicBase* t_MechanicObj = static_cast<Robot::MechanicBase*>(pcObject);
//    t_MechanicObj->setTipPoseByDraggerPose(new_Pose);
}

bool ViewProviderMechanicBase::doubleClicked()
{
    std::string Msg("Edit target Positioner");
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

void ViewProviderMechanicBase::updateData(const App::Property* prop)
{
    Robot::MechanicBase* t_Mechanics = static_cast<Robot::MechanicBase*>(pcObject);
    if (prop == &t_Mechanics->FilePath_URDF) {
        if(generateLinkMeshNodes_fromURDF(t_Mechanics->FilePath_URDF.getStrValue()))
            t_Mechanics->updateAxisValues();
    }

    else if(prop == &t_Mechanics->AxisValues){
        updatelinkmeshPoses();
        if(callbackRegistered()){
            callBack_UpdatePanelWidgets();
        }
        if(pcDragger!=nullptr){
            pcDragger->setDraggerPosition(t_Mechanics->getTeachDraggerPose());
        }
    }

//    else if(prop == &t_Mechanics->TeachCoordIndex){
//        if(pcDragger!=nullptr){
//            pcDragger->setDraggerPosition(t_Mechanics->getTeachDraggerPose());
//        }
//    }

    else if(prop == &t_Mechanics->Visiable){
        setVisible(t_Mechanics->Visiable.getValue());
    }

    // Move RobotBase
    else if(prop == &t_Mechanics->Trans_Ref2Base || prop == &t_Mechanics->Pose_RefOrigin){
        updatelinkmeshPoses();
        if(callbackRegistered()){
            callBack_UpdatePanelWidgets();
        }
        if(pcDragger!=nullptr){
            pcDragger->setDraggerPosition(t_Mechanics->getTeachDraggerPose());
        }
    }

    else if(prop == &t_Mechanics->Activated){
        if(t_Mechanics->Activated.getValue() && !t_Mechanics->isEditing.getValue()){
            setEdit(0);
        }
        else{
            unsetEdit(0);
        }
        t_Mechanics->InteractiveTeach.setValue(t_Mechanics->Activated.getValue());
    }

    else if (prop == &t_Mechanics->InteractiveTeach) {
        if (t_Mechanics->InteractiveTeach.getValue()) {
            if (pcDragger == nullptr){
                pcDragger = new InteractiveDragger(this->getRoot(),
                                                   t_Mechanics->getTeachDraggerPose(),
                                                   DraggerUsage::Interaction);
                pcDragger->setup_incCallback(std::bind(&ViewProviderMechanicBase::DraggerMotionCallback,
                                                       this, std::placeholders::_1));
                pcDragger->setup_finishCallback(std::bind(&ViewProviderMechanicBase::DraggerFinishCallback,
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

bool ViewProviderMechanicBase::setEdit(int ModNum){
    auto t_Mechanics = static_cast<Robot::MechanicBase *>(pcObject);
    t_Mechanics->InteractiveTeach.setValue(true);
    auto dlg = new TaskDlgMechanicControl(t_Mechanics);
    if (dlg == nullptr)
      return false;
    callBack_UpdatePanelWidgets = std::bind(&TaskDlgMechanicControl::signal_updatePanelWidgets, dlg);
    Gui::Control().showDialog(dlg);
    t_Mechanics->isEditing.setValue(true);
    return true;
}

void ViewProviderMechanicBase::unsetEdit(int ModeNum)
{
    Robot::MechanicBase* t_Mechanics = static_cast<Robot::MechanicBase*>(pcObject);
    t_Mechanics->InteractiveTeach.setValue(false);
    callBack_UpdatePanelWidgets = nullptr;
    Gui::Control().closeDialog();
    t_Mechanics->isEditing.setValue(false);
}

std::vector<App::DocumentObject *> ViewProviderMechanicBase::claimChildren() const
{
    Robot::MechanicBase* t_Mechanics = static_cast<Robot::MechanicBase*>(pcObject);
    return std::vector<App::DocumentObject *>();
}

void ViewProviderMechanicBase::DraggerMotionCallback(InteractiveDragger *t_dragger)
{
    Robot::MechanicBase* t_Mechanic = static_cast<Robot::MechanicBase*>(pcObject);
    auto diff = t_dragger->getLastPose().inverse() * t_dragger->getCurrentPose();
    if(t_Mechanic->isDerivedFrom(Robot::MechanicRobot::getClassTypeId())){
        auto t_RobotPtr = static_cast<Robot::MechanicRobot*>(t_Mechanic);
        t_RobotPtr->setTipPoseByDiff(diff);
    }
}

void ViewProviderMechanicBase::DraggerFinishCallback(InteractiveDragger *t_dragger)
{
    Robot::MechanicBase* t_Mechanics = static_cast<Robot::MechanicBase*>(pcObject);
    t_dragger->setDraggerPosition(t_Mechanics->getTeachDraggerPose());
}


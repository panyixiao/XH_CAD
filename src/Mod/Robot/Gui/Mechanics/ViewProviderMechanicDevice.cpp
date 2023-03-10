
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

#include <Mod/Robot/App/Mechanics/MechanicDevice.h>
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

#include "ViewProviderMechanicDevice.h"
#include "TaskDlgMechanicControl.h"

using namespace Gui;
using namespace RobotGui;

PROPERTY_SOURCE(RobotGui::ViewProviderMechanicDevice,
                Gui::ViewProviderGeometryObject)

ViewProviderMechanicDevice::ViewProviderMechanicDevice()
  : pcDragger(0),toolShapeVP(0)
{
//    ADD_PROPERTY(InteractiveTeach,(0));

    pcRoot = new Gui::SoFCSelection();
    pcRoot->highlightMode = Gui::SoFCSelection::OFF;
    pcRoot->ref();

    m_LinkMeshGroup = new SoSeparator();
    m_LinkMeshGroup->setName(SbName("Positioner_LinkGroup"));
    m_LinkMeshGroup->ref();

    // set nodes for the manipulator outfit
    m_TcpRoot = new SoGroup();
    m_TcpRoot->ref();
}

bool ViewProviderMechanicDevice::generateLinkMeshNodes_fromURDF(const char *urdf_FilePath)
{
//    // Read Link Mesh File Path
//    std::ifstream in(urdf_FilePath);
//    if(!in)
//      return false;
//    std::vector<std::string> split_1, split_2;
//    // over read the header
//    std::string tmp_line;

//    std::map<std::string, std::string> linkMeshPath;

//    // Readin mesh file path and initial Poses
//    while(std::getline(in, tmp_line)){
//        Robot::DS_Utility::split(tmp_line,',',split_1);
//        if(split_1.size() == 2){
//            Robot::DS_Utility::split(split_1[1], ';', split_2);
//            if(split_2.size() == 2){
//                linkMeshPath.insert(std::make_pair(split_1[0],split_2[0]));
//            }
//        }
//        split_1.clear();
//        split_2.clear();
//    }

    Robot::MechanicDevice* t_Positioner = static_cast<Robot::MechanicDevice*>(pcObject);
    auto urdfFileInfo = QFileInfo(QString::fromLocal8Bit(urdf_FilePath));

    int linkID = 0;
    // Generate Link Group
    for(const auto& linkName : t_Positioner->getKinematicModelRef().getLinkNames()){
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
                                                    t_Positioner->getJointTransformation(linkID-1));
        }
        // Generate MeshNode
        auto meshNodeName = std::string(t_Positioner->getNameInDocument())+"_"+linkName;
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

bool ViewProviderMechanicDevice::updatelinkmeshPoses()
{
    Robot::MechanicDevice* t_Object = static_cast<Robot::MechanicDevice*>(pcObject);
    if(m_LinkNames.size()-1 != t_Object->AxisValues.getSize())
        return false;
    Base::Placement basePose = t_Object->getOriginPose();
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
    // Update MeshPoses
    for(int i = 0; i<t_Object->AxisValues.getSize(); i++){
        auto nodeName = std::string("TransitionOf")+m_LinkNames[i+1];
        auto transNode = static_cast<SoTransform *>(m_LinkMeshGroup->getByName(nodeName.c_str()));
        if(transNode!=nullptr){
            auto newPos = t_Object->getJointTransformation(i);
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
                toolShapeVP->setTransformation(t_Object->getCurrentFlanPose(Robot::CoordOrigin::World).toMatrix());
            }
        }
    }
    return true;
}

bool ViewProviderMechanicDevice::callbackRegistered()
{
    return callBack_UpdatePanelWidgets!=nullptr;
}

ViewProviderMechanicDevice::~ViewProviderMechanicDevice()
{
    pcRoot->unref();
    m_LinkMeshGroup->unref();
    m_TcpRoot->unref();
}

void ViewProviderMechanicDevice::attach(App::DocumentObject *pcObj)
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

void ViewProviderMechanicDevice::setDisplayMode(const char* ModeName)
{
    if ( strcmp("On",ModeName)==0 )
        setDisplayMaskMode("On");

    if ( strcmp("Off",ModeName)==0 )
        setDisplayMaskMode("Off");
    ViewProviderGeometryObject::setDisplayMode( ModeName );
}

std::vector<std::string> ViewProviderMechanicDevice::getDisplayModes(void) const
{
    std::vector<std::string> StrList;
    StrList.push_back("On");
    StrList.push_back("Off");
    return StrList;
}

void ViewProviderMechanicDevice::onChanged(const App::Property* prop)
{
    Robot::MechanicDevice* t_Positioner = static_cast<Robot::MechanicDevice*>(pcObject);

    ViewProviderGeometryObject::onChanged(prop);
}

bool ViewProviderMechanicDevice::setTipPosition(const Base::Placement &new_Pose)
{
    Robot::MechanicDevice* t_Positioner = static_cast<Robot::MechanicDevice*>(pcObject);
    return t_Positioner->setTipPose(new_Pose);
}

void ViewProviderMechanicDevice::setTipPoseByTeach(const Base::Placement &new_Pose)
{
    Robot::MechanicDevice* t_Positioner = static_cast<Robot::MechanicDevice*>(pcObject);
    t_Positioner->setTipPoseByDraggerPose(new_Pose);
}

bool ViewProviderMechanicDevice::doubleClicked()
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

void ViewProviderMechanicDevice::updateData(const App::Property* prop)
{
    Robot::MechanicDevice* t_Positioner = static_cast<Robot::MechanicDevice*>(pcObject);
    if (prop == &t_Positioner->File_URDF) {
        generateLinkMeshNodes_fromURDF(t_Positioner->File_URDF.getValue());
        t_Positioner->updateAxisValues();
    }

    else if(prop == &t_Positioner->AxisValues){
        updatelinkmeshPoses();
        if(callbackRegistered()){
            callBack_UpdatePanelWidgets();
        }
        if(pcDragger!=nullptr){
            pcDragger->setDraggerPosition(t_Positioner->getTeachDraggerPose());
        }
    }

    else if(prop == &t_Positioner->TeachCoordIndex){
        if(pcDragger!=nullptr){
            pcDragger->setDraggerPosition(t_Positioner->getTeachDraggerPose());
        }
    }

    else if(prop == &t_Positioner->Visiable){
        setVisible(t_Positioner->Visiable.getValue());
    }

    // Move RobotBase
    else if(prop == &t_Positioner->Pose_Ref2Base || prop == &t_Positioner->Pose_Reference){
        updatelinkmeshPoses();
        if(callbackRegistered()){
            callBack_UpdatePanelWidgets();
        }
        if(pcDragger!=nullptr){
            pcDragger->setDraggerPosition(t_Positioner->getTeachDraggerPose());
        }
    }

    else if(prop == &t_Positioner->Activated){
        if(t_Positioner->Activated.getValue() && !t_Positioner->Editing.getValue()){
            setEdit(0);
        }
        else{
            unsetEdit(0);
        }
        t_Positioner->InteractiveTeach.setValue(t_Positioner->Activated.getValue());
    }

    else if (prop == &t_Positioner->InteractiveTeach) {
        if (t_Positioner->InteractiveTeach.getValue()) {
            if (pcDragger == nullptr){
                pcDragger = new InteractiveDragger(this->getRoot(),
                                                   t_Positioner->getTeachDraggerPose(),
                                                   DraggerUsage::Interaction);
                pcDragger->setup_incCallback(std::bind(&ViewProviderMechanicDevice::DraggerMotionCallback,
                                                       this, std::placeholders::_1));
                pcDragger->setup_finishCallback(std::bind(&ViewProviderMechanicDevice::DraggerFinishCallback,
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

bool ViewProviderMechanicDevice::setEdit(int ModNum){
    auto t_Positioner = static_cast<Robot::MechanicDevice *>(pcObject);
    t_Positioner->InteractiveTeach.setValue(true);
    auto dlg = new TaskDlgMechanicControl(t_Positioner);
    if (dlg == nullptr)
      return false;
    callBack_UpdatePanelWidgets = std::bind(&TaskDlgMechanicControl::signal_updatePanelWidgets, dlg);
    Gui::Control().showDialog(dlg);
    t_Positioner->Editing.setValue(true);
    return true;
}

void ViewProviderMechanicDevice::unsetEdit(int ModeNum)
{
    Robot::MechanicDevice* t_Positioner = static_cast<Robot::MechanicDevice*>(pcObject);
    t_Positioner->InteractiveTeach.setValue(false);
    callBack_UpdatePanelWidgets = nullptr;
    Gui::Control().closeDialog();
    t_Positioner->Editing.setValue(false);
}

std::vector<App::DocumentObject *> ViewProviderMechanicDevice::claimChildren() const
{
    Robot::MechanicDevice* t_Positioner = static_cast<Robot::MechanicDevice*>(pcObject);
    return t_Positioner->getChildrenList();
}

void ViewProviderMechanicDevice::DraggerMotionCallback(InteractiveDragger *t_dragger)
{
}

void ViewProviderMechanicDevice::DraggerFinishCallback(InteractiveDragger *t_dragger)
{
    Robot::MechanicDevice* t_Positioner = static_cast<Robot::MechanicDevice*>(pcObject);
    t_dragger->setDraggerPosition(t_Positioner->getTeachDraggerPose());
}


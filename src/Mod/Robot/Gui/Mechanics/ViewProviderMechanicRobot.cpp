
#include <App/Document.h>
#include <App/DocumentObject.h>
#include <Mod/Robot/Gui/Mechanics/ViewProviderMechanicRobot.h>
#include <Mod/Robot/App/Mechanics/MechanicBase.h>
#include <Mod/Robot/App/Mechanics/MechanicRobot.h>

using namespace Gui;
using namespace RobotGui;

PROPERTY_SOURCE(RobotGui::ViewProviderMechanicRobot,
                RobotGui::ViewProviderMechanicBase)

ViewProviderMechanicRobot::ViewProviderMechanicRobot()
  : toolShapeVP(0)
{

}

ViewProviderMechanicRobot::~ViewProviderMechanicRobot()
{
}

void ViewProviderMechanicRobot::onChanged(const App::Property* prop)
{
    ViewProviderGeometryObject::onChanged(prop);
}

bool ViewProviderMechanicRobot::setTipPosition(const Base::Placement &new_Pose)
{
    Robot::MechanicRobot* t_RobotPtr = static_cast<Robot::MechanicRobot*>(pcObject);
//    return t_MechanicObj->setCurrentTip(new_Pose);
    return false;
}

void ViewProviderMechanicRobot::setTipPoseByTeach(const Base::Placement &new_Pose)
{
    Robot::MechanicRobot* t_RobotPtr = static_cast<Robot::MechanicRobot*>(pcObject);
}

void ViewProviderMechanicRobot::updateData(const App::Property* prop)
{
    Robot::MechanicRobot* t_MechanicRobotPtr = static_cast<Robot::MechanicRobot*>(pcObject);
    if(prop == &t_MechanicRobotPtr->TeachCoordIndex){
        if(pcDragger!=nullptr){
            pcDragger->setDraggerPosition(t_MechanicRobotPtr->getTeachDraggerPose());
        }
    }
    else if (prop == &t_MechanicRobotPtr->InteractiveDraggerOn) {
        if (t_MechanicRobotPtr->InteractiveDraggerOn.getValue()) {
            if (pcDragger == nullptr){
                pcDragger = new InteractiveDragger(this->getRoot(),
                                                   t_MechanicRobotPtr->getTeachDraggerPose(),
                                                   DraggerUsage::Interaction);
                pcDragger->setup_incCallback(std::bind(&ViewProviderMechanicRobot::DraggerMotionCallback,
                                                       this, std::placeholders::_1));
                pcDragger->setup_finishCallback(std::bind(&ViewProviderMechanicRobot::DraggerFinishCallback,
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
    ViewProviderMechanicBase::updateData(prop);
}

bool ViewProviderMechanicRobot::setEdit(int ModNum){
    auto t_MechanicRobotPtr = static_cast<Robot::MechanicRobot *>(pcObject);
    auto dlg = new TaskDlgMechanicControl(t_MechanicRobotPtr);
    if (dlg == nullptr)
      return false;
    callBack_UpdatePanelWidgets = std::bind(&TaskDlgMechanicControl::signal_updatePanelWidgets, dlg);
    Gui::Control().showDialog(dlg);
    t_MechanicRobotPtr->InteractiveDraggerOn.setValue(true);
    t_MechanicRobotPtr->isEditing.setValue(true);
    return true;
}

void ViewProviderMechanicRobot::unsetEdit(int ModeNum)
{
    auto t_MechanicRobotPtr = static_cast<Robot::MechanicRobot*>(pcObject);
    callBack_UpdatePanelWidgets = nullptr;
    Gui::Control().closeDialog();
    t_MechanicRobotPtr->InteractiveDraggerOn.setValue(false);
    t_MechanicRobotPtr->isEditing.setValue(false);
}

bool ViewProviderMechanicRobot::updatelinkmeshPoses()
{
    ViewProviderMechanicBase::updatelinkmeshPoses();
    Robot::MechanicRobot* t_RobotPtr = static_cast<Robot::MechanicRobot*>(pcObject);
    if(toolShapeVP){
        toolShapeVP->setTransformation(t_RobotPtr->getCurrentTipPose(Robot::CoordOrigin::World).toMatrix());
    }
}

std::vector<App::DocumentObject *> ViewProviderMechanicRobot::claimChildren() const
{
    Robot::MechanicRobot* t_RobotPtr = static_cast<Robot::MechanicRobot*>(pcObject);
    return t_RobotPtr->getChildrenList();
}

void ViewProviderMechanicRobot::DraggerMotionCallback(InteractiveDragger *t_dragger)
{
    Robot::MechanicRobot* t_RobotPtr = static_cast<Robot::MechanicRobot*>(pcObject);
    auto diff = t_dragger->getLastPose().inverse() * t_dragger->getCurrentPose();
    t_RobotPtr->setTipPoseByDiff(diff);
}

void ViewProviderMechanicRobot::DraggerFinishCallback(InteractiveDragger *t_dragger)
{
    Robot::MechanicRobot* t_RobotPtr = static_cast<Robot::MechanicRobot*>(pcObject);
    t_dragger->setDraggerPosition(t_RobotPtr->getTeachDraggerPose());
}


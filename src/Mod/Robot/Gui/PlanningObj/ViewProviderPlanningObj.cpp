#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <App/Document.h>

//#include "Mod/RD_Setup/App/rd_RobotLib_Interface.h"
//#include "Mod/RD_Setup/Gui/TaskDialog_RD_PlanningObject.h"
#include <Base/Console.h>
#include <Gui/Command.h>
#include <Gui/Control.h>
#include <Inventor/nodes/SoTransform.h>
#include <functional>
//#include <g3log/rlog.h>
#include "Mod/Robot/Gui/PlanningObj/TaskDlgPlanningObject.h"
#include "ViewProviderPlanningObj.h"

using namespace PartGui;
using namespace RobotGui;
PROPERTY_SOURCE(RobotGui::ViewProviderPlanningObj,
                PartGui::ViewProviderPartExt)

ViewProviderPlanningObj::ViewProviderPlanningObj() {}
ViewProviderPlanningObj::~ViewProviderPlanningObj() {}

void ViewProviderPlanningObj::attach(App::DocumentObject *pcObj) {
    m_DocPtr = pcObj->getDocument();
//  m_PlanningObj = dynamic_cast<Robot::PlanningObject *>(pcObj);
  ViewProviderPartExt::attach(pcObj);
}

void ViewProviderPlanningObj::setDisplayMode(const char *displayModeName) {
  ViewProviderPartExt::setDisplayMode(displayModeName);
}

std::vector<std::string> ViewProviderPlanningObj::getDisplayModes() const {
  return ViewProviderPartExt::getDisplayModes();
}

void ViewProviderPlanningObj::updateData(const App::Property *prop) {

  Robot::PlanningObject* t_PlanningObjPtr = static_cast<Robot::PlanningObject*>(pcObject);

  if (prop == &t_PlanningObjPtr->Placement ) {
//      m_PlanningObj->updateAttachedObj();
  }
  else if(prop == &t_PlanningObjPtr->isEditing){
      if(t_PlanningObjPtr->isEditing.getValue()){
          setEdit(0);
      }else{
          unsetEdit(0);
      }
  }
  else if(prop == &t_PlanningObjPtr->Trans_O2M ||
          prop == &t_PlanningObjPtr->Pose_Mount){
      t_PlanningObjPtr->Placement.setValue(t_PlanningObjPtr->Pose_Mount.getValue() *
                                           t_PlanningObjPtr->Trans_O2M.getValue().inverse());;
  }
  ViewProviderPartExt::updateData(prop);
}

void ViewProviderPlanningObj::onChanged(const App::Property *prop) {
  ViewProviderPartExt::onChanged(prop);
}

bool ViewProviderPlanningObj::doubleClicked() {
    Robot::PlanningObject* t_PlanningObjPtr = static_cast<Robot::PlanningObject*>(pcObject);
    t_PlanningObjPtr->isEditing.setValue(true);
    return true;
}

void ViewProviderPlanningObj::updateCenterPlacement(const Base::Placement &newPlacement) {
    Robot::PlanningObject* t_PlanningObjPtr = static_cast<Robot::PlanningObject*>(pcObject);
    t_PlanningObjPtr->Trans_O2M.setValue(newPlacement);
}

bool ViewProviderPlanningObj::setEdit(int ModNum) {
  if (ModNum != ViewProvider::EditMode::Transform) {
      Robot::PlanningObject* t_PlanningObjPtr = static_cast<Robot::PlanningObject*>(pcObject);
      auto dlg = new RobotGui::TaskDlgPlanningObject(t_PlanningObjPtr);
      Gui::Control().showDialog(dlg);
  }
  return true;
}

void ViewProviderPlanningObj::unsetEdit(int ModNum) {
}

bool ViewProviderPlanningObj::onDelete(const std::vector<std::string> &subNames){
    return ViewProviderPartExt::onDelete(subNames);
}

std::vector<App::DocumentObject *> ViewProviderPlanningObj::claimChildren() const
{
    std::vector<App::DocumentObject*> childrenList;
//    if(!m_PlanningObj->CurrentFrame.getStrValue().empty())
//        childrenList.push_back(m_DocPtr->getObject(m_PlanningObj->CurrentFrame.getValue()));
    return childrenList;
}

void ViewProviderPlanningObj::updateRenderStatus(bool colliding) {
  if (colliding) {
    DiffuseColor.setValue(App::Color(1.0, 0.0, 0.0, 0.0));
  } else {
    DiffuseColor.setValue(App::Color(.5, 0.5, 0.5, 0.0));
  }
}

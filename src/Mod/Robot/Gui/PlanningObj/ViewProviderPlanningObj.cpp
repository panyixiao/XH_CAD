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
  m_PlanningObj = dynamic_cast<Robot::PlanningObject *>(pcObj);
  ViewProviderPartExt::attach(pcObj);
}

void ViewProviderPlanningObj::setDisplayMode(const char *displayModeName) {
  ViewProviderPartExt::setDisplayMode(displayModeName);
}

std::vector<std::string> ViewProviderPlanningObj::getDisplayModes() const {
  return ViewProviderPartExt::getDisplayModes();
}

void ViewProviderPlanningObj::updateData(const App::Property *prop) {

  if (prop == &m_PlanningObj->Placement ) {
//      m_PlanningObj->updateAttachedObj();
  }
  else if(prop == &m_PlanningObj->Translation_O2M ||
          prop == &m_PlanningObj->Pose_Mount){
      m_PlanningObj->Placement.setValue(m_PlanningObj->Pose_Mount.getValue() *
                                        m_PlanningObj->Translation_O2M.getValue().inverse());;
  }
  ViewProviderPartExt::updateData(prop);
}

void ViewProviderPlanningObj::onChanged(const App::Property *prop) {
  ViewProviderPartExt::onChanged(prop);
}

bool ViewProviderPlanningObj::doubleClicked() {
  std::string Msg("Edit ");
  Msg += this->pcObject->Label.getValue();
  try {
    Gui::Command::openCommand(Msg.c_str());
    Gui::Command::doCommand(Gui::Command::Gui,
                            "Gui.ActiveDocument.setEdit('%s',%d)",
                            this->pcObject->getNameInDocument(),
                            Gui::ViewProvider::EditMode::Default);
    return true;
  }
  catch (const Base::Exception &e) {
    Base::Console().Error("%s\n", e.what());
    return false;
  }
}

void ViewProviderPlanningObj::updateCenterPlacement(const Base::Placement &newPlacement) {
  if (m_PlanningObj == nullptr)
    return;
  m_PlanningObj->updateTranslation_Origin2Mount(newPlacement);
}

bool ViewProviderPlanningObj::setEdit(int ModNum) {
  if (ModNum != ViewProvider::EditMode::Transform) {
    auto dlg = new RobotGui::TaskDlgPlanningObject(m_PlanningObj);
    Gui::Control().showDialog(dlg);
  }
  setupDragger(true);
  return true;
}

void ViewProviderPlanningObj::unsetEdit(int ModNum) {
    setupDragger(false);
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

void ViewProviderPlanningObj::setupDragger(bool enable) {
  if (enable) {
    if (m_dragger == nullptr) {
      m_dragger = new InteractiveDragger();
      m_dragger->createDragger(this->getRoot(),
                               m_PlanningObj->getCurrentMountPose(),
                               RobotGui::DraggerUsage::Interaction);
      m_dragger->setAttachingViewProvider(this);
      m_dragger->setup_poseCallback(std::bind(&ViewProviderPlanningObj::updateCenterPlacement,
                                          this,
                                          std::placeholders::_1));
    }
  } else {
    if (m_dragger) {
      m_dragger->destroyDragger();
      m_dragger = nullptr;
    }
  }
}

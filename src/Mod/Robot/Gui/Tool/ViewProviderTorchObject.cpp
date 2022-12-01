// Created by Yixiao 2022-06-15

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _ProComp_
#endif

#include "ViewProviderTorchObject.h"
#include <App/Application.h>
#include <App/Document.h>
#include <Base/Console.h>
#include <Gui/Command.h>
#include <Gui/Control.h>
#include <functional>

#include "TaskDlgToolSetupPanel.h"

using namespace std::placeholders;

using namespace Robot;
using namespace RobotGui;

PROPERTY_SOURCE(RobotGui::ViewProviderTorchObject, RobotGui::ViewProviderToolObject)

ViewProviderTorchObject::ViewProviderTorchObject() {}

ViewProviderTorchObject::~ViewProviderTorchObject() {}

void ViewProviderTorchObject::onChanged(const App::Property *prop)
{
    ViewProviderToolObject::onChanged(prop);
}

void ViewProviderTorchObject::attach(App::DocumentObject *obj) {
  ViewProviderToolObject::attach(obj);
}

void ViewProviderTorchObject::updateData(const App::Property *prop) {

    Robot::TorchObject * torchPtr = static_cast<Robot::TorchObject *>(pcObject);
    if (prop == &torchPtr->Shape) {
    }
    ViewProviderToolObject::updateData(prop);
}

bool ViewProviderTorchObject::setEdit(int ModNum) {
  if (ModNum == Gui::ViewProvider::EditMode::Default) {
    auto m_tool = static_cast<Robot::TorchObject *>(pcObject);
    Gui::TaskView::TaskDialog *dlg = new TaskDlgToolObject(m_tool);
    if(dlg == nullptr)
        return false;
    Gui::Control().showDialog(dlg);
  }
  return true;
}

void ViewProviderTorchObject::setDisplayMode(const char *displayModeName) {
  ViewProviderToolObject::setDisplayMode(displayModeName);
}

std::vector<std::string> ViewProviderTorchObject::getDisplayModes() const {
  return ViewProviderToolObject::getDisplayModes();
}

bool ViewProviderTorchObject::doubleClicked() {
  return this->setEdit(Gui::ViewProvider::EditMode::Default);
}

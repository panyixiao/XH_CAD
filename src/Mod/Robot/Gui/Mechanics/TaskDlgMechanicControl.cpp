// Created By Yixiao 2022-04-24

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <map>

#include <Base/Console.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Document.h>

#include "QGroupBox"
#include "QLineEdit"
#include "QSignalMapper"
#include "TaskDlgMechanicControl.h"

#define MaxiumTaskBoxWidth 350

using namespace RobotGui;

TaskDlgMechanicControl::TaskDlgMechanicControl(Robot::MechanicBase *t_Mechanics, QWidget *parent)
{
    if(t_Mechanics == nullptr)
        return;
    m_DocPtr = t_Mechanics->getDocument();
    if(t_Mechanics->isDerivedFrom(Robot::MechanicRobot::getClassTypeId())){
        auto targetRobot = static_cast<Robot::MechanicRobot*>(t_Mechanics);
        // Face Selection
        auto m_FaceSelection = new Gui::TaskView::TaskSelectLinkProperty("SELECT Part::Feature SUBELEMENT Face COUNT 1",
                                                                    &(targetRobot->LinkedFaceFeature));
        m_FaceSelection->hide();
        // Edge Selection
        auto m_EdgeSelection = new Gui::TaskView::TaskSelectLinkProperty("SELECT Part::Feature SUBELEMENT Edge COUNT 1",
                                                                    &(targetRobot->LinkedEdgeFeature));
        m_EdgeSelection->hide();

        Content.push_back(m_FaceSelection);
        Content.push_back(m_EdgeSelection);

        auto t_RobotPanel = new TaskBoxRobotTeachPanel(targetRobot,
                                                       m_FaceSelection,
                                                       m_EdgeSelection);
        QObject::connect(this, SIGNAL(signal_updatePanelWidgets()),
                         t_RobotPanel, SLOT(slot_updatePanelWidgets()));

        t_RobotPanel->setMaximumWidth(MaxiumTaskBoxWidth);

        // Contents
        Content.push_back(t_RobotPanel);
    }
    else if(t_Mechanics->isDerivedFrom(Robot::MechanicPoser::getClassTypeId())){
        auto t_PoserPtr = static_cast<Robot::MechanicPoser*>(t_Mechanics);
        auto t_PoserPanel = new TaskBoxPoserSetupPanel(t_PoserPtr);
        QObject::connect(this, SIGNAL(signal_updatePanelWidgets()),
                         t_PoserPanel, SLOT(slot_updateSliderPosition()));
        t_PoserPanel->setMaximumWidth(MaxiumTaskBoxWidth);
        // Contents
        Content.push_back(t_PoserPanel);
    }
    else if(t_Mechanics->isDerivedFrom(Robot::MechanicExtAx::getClassTypeId())){
        auto t_ExtAxPtr = static_cast<Robot::MechanicExtAx*>(t_Mechanics);
        auto t_ExtAxPanel = new TaskBoxExtAxSetupPanel(t_ExtAxPtr);
        QObject::connect(this, SIGNAL(signal_updatePanelWidgets()),
                         t_ExtAxPanel, SLOT(slot_updateSliderPosition()));
        t_ExtAxPanel->setMaximumWidth(MaxiumTaskBoxWidth);
        // Contents
        Content.push_back(t_ExtAxPanel);
    }
    // Buttons
    setButtonPosition(ButtonPosition::South);
    setDlgWdith(MaxiumTaskBoxWidth+5);
}

TaskDlgMechanicControl::~TaskDlgMechanicControl()
{
}

bool TaskDlgMechanicControl::accept() {
  Gui::Document *document = Gui::Application::Instance->getDocument(m_DocPtr);
  if (document != nullptr) {
    document->commitCommand();
    document->resetEdit();
  }

  return true;
}

bool TaskDlgMechanicControl::reject() {
  Gui::Document *document = Gui::Application::Instance->getDocument(m_DocPtr);
  if (document != nullptr) {
    document->commitCommand();
    document->resetEdit();
  }

  return true;
}

void TaskDlgMechanicControl::closeDlg()
{
    Q_EMIT destroyed();
}

#include "moc_TaskDlgMechanicControl.cpp"

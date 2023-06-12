// Created by Yixiao 2023/06/06

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "TaskBox_TracEditPanel.h"
#include <App/Document.h>
#include <Base/Console.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Command.h>
#include <Gui/Control.h>
#include <Gui/Document.h>
#include "Mod/Robot/Gui/ui_TaskBox_TracEditPanel.h"

using namespace RobotGui;

TaskBox_TracEditPanel::TaskBox_TracEditPanel(Robot::TaskObject *t_taskPtr,
                                             QWidget *parent)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"),
              tr("Task Manager"), true, parent) {
  if (t_taskPtr == nullptr)
    return;
  m_currentDoc = t_taskPtr->getDocument();
  m_targetTaskPtr = t_taskPtr;
  initUi();
}

bool TaskBox_TracEditPanel::accept() {

}

bool TaskBox_TracEditPanel::reject() {

}


void TaskBox_TracEditPanel::initUi() {
    m_proxy = new QWidget();
    m_ui = new Ui_TaskBox_TracEditPanel;
    m_ui->setupUi(m_proxy);
    this->groupLayout()->addWidget(m_proxy, Qt::AlignTop);
}

#include "moc_TaskBox_TracEditPanel.cpp"

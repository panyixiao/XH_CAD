// Created by Yixiao 2022/05/29

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "TaskBox_TaskEditor.h"
#include <App/Document.h>
#include <Base/Console.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Command.h>
#include <Gui/Control.h>
#include <Gui/Document.h>

#include <Mod/Robot/Gui/ui_TaskBox_TaskEditor.h>


using namespace RobotGui;

TaskBox_TaskEditor::TaskBox_TaskEditor(Robot::TaskObject *taskObjectPtr,
                                         QWidget *parent)
    : TaskBox(Gui::BitmapFactory().pixmap("document-new"),
              tr("Task Manager"), true, parent) {
  if (taskObjectPtr == nullptr)
    return;

  m_currentDoc = taskObjectPtr->getDocument();
  m_targetTaskPtr = taskObjectPtr;
  initUi();
}

bool TaskBox_TaskEditor::accept() {

}

bool TaskBox_TaskEditor::reject() {

}


void TaskBox_TaskEditor::initUi() {
    m_proxy = new QWidget();

    m_ui_New = new Ui_TaskBox_TaskEditor;
    m_ui_New->setupUi(m_proxy);

  this->groupLayout()->addWidget(m_proxy, Qt::AlignTop);


}




#include "TaskManage/moc_TaskBox_TaskEditor.cpp"

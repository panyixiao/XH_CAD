// Created by Yixiao 2022/05/12

#include "PreCompiled.h"
#ifndef _PreComp_
#endif

#include "TaskManageWidget.h"

using namespace Gui::TaskManage;

TaskManageWidget::TaskManageWidget(QWidget *parent) : QWidget(parent){
}

TaskManageWidget::~TaskManageWidget(){
}


// Button Group box
TaskManageDlg_Ctrl::TaskManageDlg_Ctrl(QWidget *parent) : TaskManageWidget(parent)
{
    hboxLayout = new QHBoxLayout(this);
    buttonBox = new QDialogButtonBox(this);
    buttonBox->setStandardButtons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    buttonBox->setCenterButtons(true);
    hboxLayout->addWidget(buttonBox);
}

TaskManageDlg_Ctrl::~TaskManageDlg_Ctrl(){

}

QDialogButtonBox *TaskManageDlg_Ctrl::standardButtons() const{
    return buttonBox;
}

void TaskManageDlg_Ctrl::showButtons(bool _flag)
{
    if(_flag)
        buttonBox->show();
    else
        buttonBox->hide();
}

#include "TaskManagePanel/moc_TaskManageWidget.cpp"

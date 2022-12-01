// Created by Yixiao 2022/05/12

#include "PreCompiled.h"
#ifndef _PreComp_
#endif

#include "TaskManageDialog.h"
#include "Document.h"
#include "Application.h"

using namespace Gui::TaskManage;

TaskManageDialog::TaskManageDialog(QWidget *parent, QString dlg_name, App::Document *pDoc) : QDialog(parent){
    setModal(false);
    setWindowTitle(dlg_name);
    m_Layout = std::shared_ptr<QVBoxLayout>(new QVBoxLayout);
    m_DlgCtrl = new TaskManageDlg_Ctrl;
    connect(m_DlgCtrl->standardButtons(),SIGNAL(accepted()),this,SLOT(accept()));
    connect(m_DlgCtrl->standardButtons(),SIGNAL(rejected()),this,SLOT(reject()));
    std::string docName;
    if(pDoc)
        docName = std::string(pDoc->getName());
    setDocumentName(docName);
}

TaskManageDialog::~TaskManageDialog(){
    for(auto cont : m_Content)
        m_Layout->removeWidget(cont);
    for (auto it=m_Content.begin();
         it!=m_Content.end();++it) {
        if(*it!=nullptr){
            delete *it;
            *it = 0;
        }
    }
}

void TaskManageDialog::dlg_finished(int command){
    Q_EMIT finished(command);
}

void TaskManageDialog::resetEdit(){
    if(documentName.empty())
        return;
    Gui::Document *document
        = Gui::Application::Instance->getDocument(documentName.c_str());
    if(document == nullptr) return;
    document->commitCommand();
    document->resetEdit();
}

void TaskManageDialog::addWidget(QWidget *newItem){
    m_Layout->setMargin(0);
    m_Layout->setSpacing(0);
    if(newItem!=nullptr){
        // Clear all old widgets
        for(auto cont : m_Content){
            m_Layout->removeWidget(cont);
        }
        // Add
        m_Content.push_back(newItem);
        // Refresh
        for(auto cont : m_Content)
            m_Layout->addWidget(cont,0,Qt::AlignTop);
    }
    m_Layout->addWidget(m_DlgCtrl);
    this->setLayout(m_Layout.get());
}

#include "TaskManagePanel/moc_TaskManageDialog.cpp"

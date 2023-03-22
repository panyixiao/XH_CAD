// Created by Yixiao 2022/05/22
#ifndef TASKMANAGEDLG_H
#define TASKMANAGEDLG_H

#include <QObject>
#include <QWidget>
#include <QDialog>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <App/Document.h>
#include "TaskManageWidget.h"

namespace Gui{

namespace  TaskManage {
class TaskManageDialog :  public QDialog
{
    Q_OBJECT
public:
  TaskManageDialog(QWidget *parent = 0,QString dlg_name = QObject::tr("Task Manager"),App::Document* pDoc = 0);
  ~TaskManageDialog();

  void showButtonBox(bool flag);

  const std::vector<QWidget*> &getDialogContent(void) const{
      return m_Content;
  }

  const std::string& getDocumentName() const{
      return documentName;
  }

  void setDocumentName(const std::string& doc){
      documentName = doc;
  }

  virtual const std::string getEditingObject() const{
      return std::string();
  }

  virtual const std::string getDialogType() const = 0;

public:
    /// is called by the framework when the dialog is opened
    virtual void open(){}
    /// is called by the framework if a button is clicked which has no accept or reject role
    virtual void clicked(int){}
    /// is called by the framework if the dialog is accepted (Ok)
    virtual void accept(){}
    /// is called by the framework if the dialog is rejected (Cancel)
    virtual void reject(){}
    /// is called by the framework if the user press the help button
    virtual void helpRequested(){
    }

    void dlg_finished(int command);
    void resetEdit();
    void emitDestructionSignal() {
        Q_EMIT aboutToBeDestroyed();
    }
    void addWidget(QWidget* newItem);

Q_SIGNALS:
    void aboutToBeDestroyed();

protected:
    /// List of TaskBoxes of that dialog
    std::vector<QWidget*> m_Content;
    std::shared_ptr<QBoxLayout> m_Layout;
    TaskManageDlg_Ctrl* m_DlgCtrl = nullptr;

private:
    std::string documentName = "";
};

}

}
#endif

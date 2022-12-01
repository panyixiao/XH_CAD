// Created by Yixiao 2022/05/12

#ifndef TASKMANAGEPANELWIDGET_H
#define TASKMANAGEPANELWIDGET_H

#include <QObject>
#include <QWidget>
#include <QDialogButtonBox>
#include <QHBoxLayout>

namespace Gui{

namespace TaskManage {
class TaskManageWidget : public QWidget
{
    Q_OBJECT
public:
  TaskManageWidget(QWidget* parent = 0);
  ~TaskManageWidget();
};


class TaskManageDlg_Ctrl : public TaskManageWidget
{
    Q_OBJECT
public:
    TaskManageDlg_Ctrl(QWidget* parent = 0);
    ~TaskManageDlg_Ctrl();
    QDialogButtonBox* standardButtons() const;
    void showButtons(bool _flag);
protected:
    QHBoxLayout *hboxLayout;
    QDialogButtonBox *buttonBox;
};
}

}


#endif

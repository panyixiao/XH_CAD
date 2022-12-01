// Created by Yixiao 2022/05/12

#ifndef TASKMANAGEDOCKPANEL_H
#define TASKMANAGEDOCKPANEL_H

#include <Gui/Selection.h>
#include <Gui/DockWindow.h>
#include <memory>
#include "TaskManageDialog.h"

class QTabWidget;
class DockWindow;

namespace Gui{

class ControlSingleton;

typedef std::shared_ptr<TaskManage::TaskManageDialog> TaskManageDlg_sptr;

namespace DockWnd{
    // A combination of dockWnd and taskView
    class TaskManageDockPanel : public DockWindow/*, public Gui::SelectionSingleton::ObserverType*/
    {
        Q_OBJECT
    public:
      TaskManageDockPanel(Gui::Document* pcDoc, QWidget* parent = 0);
      virtual ~TaskManageDockPanel();
      friend class Gui::ControlSingleton;
    protected:
      bool showDialog( TaskManageDlg_sptr dlg_Ptr);
      void closeCurrentDialog();
      void setActiveDialog(TaskManageDlg_sptr dlgPtr);
      void addNewDialog(TaskManageDlg_sptr t_DlgPtr);
      bool connectSignals();
      void removeTab();

    protected Q_SLOTS:
      bool activeDlg_accepted();
      bool activeDlg_rejected();
      void helpRequest();
      void clicked (QAbstractButton * button);
      void tabChanged(int index);

    Q_SIGNALS:
      void dlg_Accepted(std::string msg);
      void dlg_Rejected(std::string msg);

    private:
      QTabWidget* m_tab = nullptr;
      std::vector<TaskManageDlg_sptr> m_DialogContainer;
      TaskManageDlg_sptr m_ActiveDlg = nullptr;
    };
}
}


#endif


#ifndef ROBOTGUI_TOOLOPERATIONPANEL_H
#define ROBOTGUI_TOOLOPERATIONPANEL_H

#include "Mod/Robot/Gui/PreCompiled.h"
#include <App/Document.h>
#include <QStringList>
#include <QStringListModel>
#include <QItemSelectionModel>
#include <Gui/TaskManagePanel/TaskManageDialog.h>
#include <Gui/FileDialog.h>
#include <Gui/MainWindow.h>
#include <Gui/Document.h>
#include "Mod/Robot/App/Tool/ToolObject.h"
#include "Mod/Robot/App/Database/ToolDatabase.h"

class Ui_ToolOperationPanel;

namespace RobotGui{

class ToolOperationPanel : public Gui::TaskManage::TaskManageDialog
{
    Q_OBJECT
public:
    ToolOperationPanel(App::Document* pDoc, Robot::ToolType t_Type, QWidget *parent = 0);

    const std::string getDialogType() const{
        return std::string("Tool Operation Dialog");
    }

private Q_SLOTS:
    void slot_brandSelected();
    void slot_torchTubeTypeSelected();
    void slot_torchTubeLengthSelected();
    void slot_2DScannerMountSelected();
    void slot_2DScannerRangeSelected();
    void slot_CreateNewTool();
    void slot_ImportSelectedTool();
    void slot_CancelOperation();
    void slot_SelectionChanged(QItemSelection selection);

protected:
    virtual void accept();
    virtual void reject();
    void initUi_framework();
    void connectSignals();
    void initUi_WeldTorchPanel();
    void initUi_2DScannerPanel();
    void initUi_3DCameraPanel();
    void initUi_GripperPanel();

    void update_TorchTubeType();
    void update_TorchTubeLength();
    void update_2DScannerMount();
    void update_2DScannerRange();
    void update_ToolItemList();


private:
    Ui_ToolOperationPanel* m_ui = nullptr;
    QWidget * m_proxy = nullptr;
    App::Document* m_DocPtr = nullptr;
    QStringList *m_stringList = nullptr;
    QStringListModel *m_stringListModel = nullptr;
    QItemSelectionModel *m_SelectionModel = nullptr;
    Robot::ToolType m_ToolType;
    std::shared_ptr<Robot::ToolDatabase> m_DatabasePtr = nullptr;
    std::string m_selectedToolBrand;
    std::string m_selectedToolName;
};

}


#endif // RD_DIALOG_ROBOTSELECTIONPANEL_H

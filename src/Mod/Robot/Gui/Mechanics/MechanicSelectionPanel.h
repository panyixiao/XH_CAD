#ifndef MECHANIC_SELECTIONPANEL_H
#define MECHANIC_SELECTIONPANEL_H

#include <Mod/Robot/App/Database/MechanicDatabase.h>
#include <Mod/Robot/App/Mechanics/MechanicDevice.h>
#include <Mod/Robot/App/Mechanics/Robot6AxisObject.h>
#include <Gui/TaskManagePanel/TaskManageDialog.h>
#include <App/Document.h>
#include <QStringList>
#include <QStringListModel>
#include <QItemSelectionModel>

class Ui_MechanicSelectionPanel;

namespace RobotGui{

class MechanicSelectionPanel : public Gui::TaskManage::TaskManageDialog
{
    Q_OBJECT
public:
    MechanicSelectionPanel(App::Document* pDoc,
                           Robot::MechanicDatabase* t_dbPtr,
                           Robot::MechanicType t_Type,
                           QWidget *parent = 0);

    const string getDialogType() const{
        return string("Mechanic Selection Dialog");
    }
    virtual void accept();
    virtual void reject();

private Q_SLOTS:
    void slot_brandSelected();
    void slot_dofSelected();
    void slot_typeSelected();
    void slot_payloadSelected();
    void slot_robotSelectionChanged(QItemSelection selection);

protected:
    void initUi_framework();
    void initUi_RobotSelection();
    void initUi_ExtAxisSelection();
    void initUi_Positioner();

    void connectSignals();

    void update_MechTypeList();
    void update_MechPayloadList();
    void update_MechDofList();
    void update_MechNameList();

    bool insertMechanics();

private:
    Robot::MechanicDatabase* m_DatabasePtr = nullptr;

    Ui_MechanicSelectionPanel* m_ui = nullptr;
    QWidget * m_proxy = nullptr;
    App::Document* m_DocPtr = nullptr;
    QStringList *m_stringList = nullptr;
    QStringListModel *m_stringListModel = nullptr;
    QItemSelectionModel *m_SelectionModel = nullptr;
    Robot::MechanicType m_Type;
    std::string m_selectedModelBrand;
    std::string m_selectedModelName;
};

}


#endif // RD_DIALOG_ROBOTSELECTIONPANEL_H

// Created by Yixiao 2022-05-010

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <Gui/BitmapFactory.h>
#include "TaskBox_FreerunTracEditor.h"
#include <Base/Console.h>
#include <QMessageBox>
#include <QButtonGroup>
#include "Mod/Robot/App/Utilites/FrameObject.h"
#include "Mod/Robot/App/Trac/RobotWaypoint.h"
#include "Mod/Robot/Gui/ui_TaskBox_FreerunTracEditor.h"

using namespace RobotGui;

TaskBox_FreerunTracEditor::TaskBox_FreerunTracEditor(Robot::RobotTracObject *t_TracObj,
                                                     QWidget *parent):
    TaskBox(Gui::BitmapFactory().pixmap("document-new"), tr("动作轨迹编辑"), true, parent){
  if (t_TracObj == nullptr)
    return;
  setTargetTracObject(t_TracObj);
}

bool TaskBox_FreerunTracEditor::setTargetTracObject(Robot::RobotTracObject *t_TracObj)
{
    if(t_TracObj == nullptr)
        return false;

    m_FreeTrac = t_TracObj;
    m_DocPtr = m_FreeTrac->getDocument();

    auto t_OperatorPtr = m_DocPtr->getObject(t_TracObj->getOperatorName().c_str());
    if(t_OperatorPtr->isDerivedFrom(Robot::MechanicGroup::getClassTypeId())){
        m_MechGroup = static_cast<Robot::MechanicGroup*>(t_OperatorPtr);
        initUi_PanelWidget();
        m_MechGroup->Activated.setValue(true);
        m_MechGroup->setActiveRobot(1);
    }
    else
        Base::Console().Message("TaskBox_FreerunTracEditor: Failed to Find operator pointer for this Trac");
    return false;
}


void TaskBox_FreerunTracEditor::initUi_PanelWidget()
{
  m_proxy = new QWidget();
  m_ui = new Ui_TaskBox_FreerunTracEditor();
  m_ui->setupUi(m_proxy);
  this->groupLayout()->addWidget(m_proxy,Qt::AlignTop);

  QString content;
  content = QObject::tr("Belong To: ")+QString::fromStdString(m_FreeTrac->TracManagerName.getStrValue());
  m_ui->label_TaskName->setText(content);
  updateOperatorName();
  QObject::connect(m_ui->pushButton_changeOperator, SIGNAL(clicked(bool)),
                   this, SLOT(slot_changeGroupOperator()));

  // Move Command
  QObject::connect(m_ui->pushButton_MoveCommand, SIGNAL(clicked()), this,
                   SLOT(slot_insertCommand_Move()));
  QObject::connect(m_ui->pushButton_DeleteCommand, SIGNAL(clicked()), this,
                   SLOT(slot_deleteCommand()));
  QObject::connect(m_ui->comboBox_PoseType,SIGNAL(currentIndexChanged(int)),
                   this, SLOT(slot_pointTypeChanged()));
  slot_pointTypeChanged();

  initUi_CoordBox();
  initUi_ToolBox();

  m_btnGroup = new QButtonGroup;
  m_btnGroup->addButton(m_ui->radioButton_FINE,1);
  m_btnGroup->addButton(m_ui->radioButton_CNT, 0);

  m_stringList = new QStringList();
  m_stringListModel = new QStringListModel(*m_stringList, NULL);
  m_ui->listView_PointList->setModel(m_stringListModel);
  QItemSelectionModel *selectionModelPtr = m_ui->listView_PointList->selectionModel();
  QObject::connect(selectionModelPtr, SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
                   this, SLOT(slot_selectionItemChanged(QItemSelection)));
  updateCommandWindow();

  Q_EMIT Signal_updateTracObject(m_FreeTrac->getRobotProgramSptr());
}

void TaskBox_FreerunTracEditor::initUi_CoordBox()
{
    // Coord Command
    QObject::connect(m_ui->pushButton_ChangeCoord, SIGNAL(clicked()),
                     this, SLOT(slot_insertCommand_Coord()));
    QObject::connect(m_ui->comboBox_CoordSys,SIGNAL(currentIndexChanged(int)),
                     this, SLOT(slot_changeCoord()));
    QObject::connect(m_ui->Button_FinishEdit, SIGNAL(clicked()), this,
                     SLOT(slot_finishEdit()));
}

void TaskBox_FreerunTracEditor::initUi_ToolBox()
{
    m_ui->radioButton_useTorch->setEnabled(m_MechGroup->hasTorch());
    m_ui->radioButton_useScanner->setEnabled(m_MechGroup->hasScanner());
    switch(m_MechGroup->getCurrentTool()){
    case Robot::ToolType::Undefined:
        m_ui->radioButton_NoTool->setChecked(true);
        break;
    case Robot::ToolType::WeldTorch:
        m_ui->radioButton_useTorch->setChecked(true);
        break;
    case Robot::ToolType::_2DScanner:
        m_ui->radioButton_useScanner->setChecked(true);
        break;
    case Robot::ToolType::_3DCamera:
        m_ui->radioButton_useCamera->setChecked(true);
        break;
    default:
        m_ui->radioButton_NoTool->setChecked(true);
        break;
    }
    QObject::connect(m_ui->radioButton_NoTool, SIGNAL(clicked()),
                     this, SLOT(slot_changeOperatorTool()));
    QObject::connect(m_ui->radioButton_useTorch, SIGNAL(clicked()),
                     this, SLOT(slot_changeOperatorTool()));
    QObject::connect(m_ui->radioButton_useScanner, SIGNAL(clicked()),
                     this, SLOT(slot_changeOperatorTool()));
    QObject::connect(m_ui->radioButton_useCamera, SIGNAL(clicked()),
                     this, SLOT(slot_changeOperatorTool()));
    QObject::connect(m_ui->pushButton_ToolSwitch, SIGNAL(clicked()),
                     this, SLOT(slot_insertCommand_OperateTool()));
}

void TaskBox_FreerunTracEditor::slot_changeOperatorTool()
{
    if(m_ui->radioButton_NoTool->isChecked()){
        m_MechGroup->setCurrentToolType(Robot::ToolType::Undefined);
        m_ui->pushButton_ToolSwitch->setEnabled(false);
    }
    else{
        m_ui->pushButton_ToolSwitch->setEnabled(true);
        if(m_ui->radioButton_useTorch->isChecked())
            m_MechGroup->setCurrentToolType(Robot::ToolType::WeldTorch);

        else if(m_ui->radioButton_useScanner->isChecked())
            m_MechGroup->setCurrentToolType(Robot::ToolType::_2DScanner);

        else if(m_ui->radioButton_useCamera->isChecked())
            m_MechGroup->setCurrentToolType(Robot::ToolType::_3DCamera);
    }
}

void TaskBox_FreerunTracEditor::slot_insertCommand_OperateTool()
{
    Robot::ToolType t_Type;
    if(m_ui->radioButton_NoTool->isChecked())
        t_Type = Robot::ToolType::Undefined;
    if(m_ui->radioButton_useTorch->isChecked())
        t_Type = Robot::ToolType::WeldTorch;
    if(m_ui->radioButton_useScanner->isChecked())
        t_Type = Robot::ToolType::_2DScanner;

    if(t_Type!= Robot::ToolType::Undefined){
        if(!switchOn){
            slot_insertCommand_ChangeTool();
            slot_insertCommand_Move();
            m_FreeTrac->insertCMD_SetToolActivate(std::string(m_MechGroup->getNameInDocument()),
                                                  t_Type);
            switchOn = true;
            m_ui->pushButton_ToolSwitch->setText(tr("Switch Off"));
            switch(t_Type){
            case Robot::ToolType::WeldTorch:
                m_ui->radioButton_NoTool->setEnabled(false);
                m_ui->radioButton_useScanner->setEnabled(false);
                break;
            case Robot::ToolType::_2DScanner:
                m_ui->radioButton_NoTool->setEnabled(false);
                m_ui->radioButton_useTorch->setEnabled(false);
                break;
            }

        }else{
            slot_insertCommand_Move();
            m_FreeTrac->insertCMD_SetToolDeActivate(std::string(m_MechGroup->getNameInDocument()),
                                                    t_Type);
            switchOn = false;
            m_ui->pushButton_ToolSwitch->setText(tr("Switch On"));
            switch(t_Type){
            case Robot::ToolType::WeldTorch:
                m_ui->radioButton_NoTool->setEnabled(true);
                m_ui->radioButton_useScanner->setEnabled(true);
                break;
            case Robot::ToolType::_2DScanner:
                m_ui->radioButton_NoTool->setEnabled(true);
                m_ui->radioButton_useTorch->setEnabled(true);
                break;
            }
        }
        m_MechGroup->setCurrentToolActive(switchOn);
    }

    Q_EMIT Signal_updateTracObject(m_FreeTrac->getRobotProgramSptr());
    updateCommandWindow();
}


const int TaskBox_FreerunTracEditor::getSelectedItemID() const {
  return m_ui->listView_PointList->selectionModel()->currentIndex().row();
}

void TaskBox_FreerunTracEditor::updateCommandWindow() {
  m_stringList->clear();
  for (auto cmdPtr : m_FreeTrac->getRobotProgramSptr()->getCmmdData()) {
      auto cmdName = m_FreeTrac->getRobotProgramSptr()->generateCommandStr(cmdPtr);
      m_stringList->append(QString::fromStdString(cmdName));
  };
  m_stringListModel->setStringList(*m_stringList);
}

void TaskBox_FreerunTracEditor::updatePoseLabel(const TargetPoint_sptr t_wpnt) {
    m_ui->label_PoseInfo->setText(tr(t_wpnt->getWP_Name().c_str()));
}

void TaskBox_FreerunTracEditor::resetOperator(bool activated) {
    m_MechGroup->Activated.setValue(activated);
//    m_MechGroup->isDriven.setValue(false);
}

void TaskBox_FreerunTracEditor::updateOperatorName()
{
    if(m_MechGroup == nullptr)
        return;
    auto content = QObject::tr("Operator: ")+QString::fromStdString(m_MechGroup->getActiveOperatorName());
    m_ui->label_OperatorName->setText(content);
}

bool TaskBox_FreerunTracEditor::slot_insertCommand_Move() {

    Robot::CoordOrigin t_coord = Robot::CoordOrigin::World;

    Robot::PoseType t_type;
    switch(m_ui->comboBox_PoseType->currentIndex()){
    case 0:
        t_type = Robot::PoseType::CART;
        break;
    case 1:
        t_type = Robot::PoseType::JNTS;
        break;
    };

    double pointSpeed = m_ui->doubleSpinBox_pointSpeed->value();

    slot_insertCommand_ChangeTool();

    if(t_type == Robot::PoseType::JNTS){
        m_FreeTrac->insertCMD_MOVE(std::string(m_MechGroup->getNameInDocument()),
                                   Robot::TargetPoint(m_MechGroup->getCurrentGroupPose(Robot::CordType::ACS)),
                                   Robot::MoveType::MOVJ,
                                   Robot::MovePrec::FINE,
                                   pointSpeed);
    }
    else{
        Robot::MovePrec t_Prec;
        if(m_ui->radioButton_CNT->isChecked()){
            t_Prec = Robot::MovePrec::CNT;
        }else{
            t_Prec = Robot::MovePrec::FINE;
        }
        if(m_ui->radioButton_NoTool->isChecked()){
            m_FreeTrac->insertCMD_MOVE(std::string(m_MechGroup->getNameInDocument()),
                                       Robot::TargetPoint(m_MechGroup->getCurrentGroupPose(Robot::CordType::WCS)),
                                       Robot::MoveType::MOVL,
                                       t_Prec,
                                       pointSpeed);
        }
        else{
            m_FreeTrac->insertCMD_MOVE(std::string(m_MechGroup->getNameInDocument()),
                                       Robot::TargetPoint(m_MechGroup->getCurrentGroupPose(Robot::CordType::TCS)),
                                       Robot::MoveType::MOVL,
                                       t_Prec,
                                       pointSpeed);
        }

    }

    updateCommandWindow();
    Q_EMIT Signal_updateTracObject(m_FreeTrac->getRobotProgramSptr());
    return true;
}

void TaskBox_FreerunTracEditor::slot_insertCommand_ChangeTool()
{
    Robot::ToolType t_Type;
    if(m_ui->radioButton_NoTool->isChecked())
        t_Type = Robot::ToolType::Undefined;
    if(m_ui->radioButton_useTorch->isChecked())
        t_Type = Robot::ToolType::WeldTorch;
    if(m_ui->radioButton_useScanner->isChecked())
        t_Type = Robot::ToolType::_2DScanner;

    if(m_TeachTool!=t_Type){
        m_FreeTrac->insertCMD_SwitchTool(std::string(m_MechGroup->getNameInDocument()),
                                         t_Type);
        m_TeachTool = t_Type;
    }
    updateCommandWindow();
    Q_EMIT Signal_updateTracObject(m_FreeTrac->getRobotProgramSptr());
}

void TaskBox_FreerunTracEditor::slot_insertCommand_Coord()
{
    if(m_ui->comboBox_CoordSys->currentIndex()==1){
        m_FreeTrac->insertCMD_ChangeCord(std::string(m_MechGroup->getNameInDocument()),
                                         Robot::CordType::ACS);
    }
    else if(m_ui->comboBox_CoordSys->currentIndex()==2){
        m_FreeTrac->insertCMD_ChangeCord(std::string(m_MechGroup->getNameInDocument()),
                                         Robot::CordType::PCS,
                                         m_ui->spinBox_CoordID->value());
    }

    updateCommandWindow();
    Q_EMIT Signal_updateTracObject(m_FreeTrac->getRobotProgramSptr());
}

void TaskBox_FreerunTracEditor::slot_deleteCommand() {
    if(m_FreeTrac->removeTargetCommand(getSelectedItemID())){
        updateCommandWindow();
        Q_EMIT Signal_updateTracObject(m_FreeTrac->getRobotProgramSptr());
    }else{
        Base::Console().Message("Failed to Delete Target Point");
    }
}

void TaskBox_FreerunTracEditor::slot_changeGroupOperator()
{
    if(m_MechGroup!=nullptr)
        m_MechGroup->changeOperator();
    updateOperatorName();
}

void TaskBox_FreerunTracEditor::slot_selectionItemChanged(const QItemSelection &selection) {
    auto selectedCmd = m_FreeTrac->getRobotProgramSptr()->getCMD_byPosition(getSelectedItemID());
    if(selectedCmd->getType() == Robot::CommandType::SetMove){
        auto t_waypointPtr = m_FreeTrac->getRobotProgramSptr()->getWaypoint_byCommand(selectedCmd);
        updatePoseLabel(t_waypointPtr);
        updateGroupPose(t_waypointPtr);
    }
}

void TaskBox_FreerunTracEditor::updateGroupPose(const TargetPoint_sptr t_wpntPtr)
{
    if(m_MechGroup != nullptr){
        m_MechGroup->setGroupPose(t_wpntPtr->getWPPoseData());
    }
}

void TaskBox_FreerunTracEditor::slot_pointTypeChanged()
{
    bool flag_enableCont = false;
    bool flag_percentSpeed = false;
    // CART
    if(m_ui->comboBox_PoseType->currentIndex() == 0){
        flag_enableCont = true;
        flag_percentSpeed = false;
    }
    // JNTS
    else{
        flag_enableCont = false;
        flag_percentSpeed = true;
    }

//    m_ui->radioButton_FINE->setChecked(flag_enableCont);
//    m_ui->radioButton_CNT->setEnabled(flag_enableCont);
//    m_ui->radioButton_CNT->setChecked(!flag_enableCont);

    m_ui->doubleSpinBox_cntPercent->setEnabled(flag_enableCont);

    if(flag_percentSpeed){
        m_ui->doubleSpinBox_pointSpeed->setMaximum(100);
        m_ui->label_speedunit->setText(tr("%"));
    }
    else{
        m_ui->doubleSpinBox_pointSpeed->setMaximum(10000);
        m_ui->doubleSpinBox_pointSpeed->setValue(200);
        m_ui->label_speedunit->setText(tr("mm/s"));
    }

}

void TaskBox_FreerunTracEditor::slot_changeCoord()
{
    if(m_ui->comboBox_CoordSys->currentIndex() == 2){
        m_ui->spinBox_CoordID->setEnabled(true);
    }
    else{
        m_ui->spinBox_CoordID->setEnabled(false);
    }
}

void TaskBox_FreerunTracEditor::slot_finishEdit() {
  if (m_FreeTrac->getWaypointData().empty()) {
    auto constrName = m_FreeTrac->getNameInDocument();
    if (constrName != nullptr)
        m_DocPtr->removeObject(constrName);
  }
  resetOperator(false);
  this->hide();
  Q_EMIT Signal_TracEditDone(m_FreeTrac);
}

#include "moc_TaskBox_FreerunTracEditor.cpp"

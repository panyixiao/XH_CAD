#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "MechanicSelectionPanel.h"
#include "ui_MechanicSelectionPanel.h"
#include <Gui/Command.h>
#include <QMessageBox>

using namespace RobotGui;

MechanicSelectionPanel::MechanicSelectionPanel(App::Document *pDoc,
                                               Robot::MechanicDatabase *t_dbPtr,
                                               Robot::MechanicType t_Type,
                                               QWidget *parent):
    TaskManageDialog(parent)
{
    if(pDoc == nullptr || t_dbPtr == nullptr)
        return;
    initUi_framework();
    m_DocPtr = pDoc;
    m_DatabasePtr = t_dbPtr;
    m_Type = t_Type;
    switch (m_Type) {
    case Robot::MechanicType::M_Robot:
      initUi_RobotSelection();
      break;
    case Robot::MechanicType::M_ExtAxis:
      initUi_ExtAxisSelection();
      break;
    case Robot::MechanicType::M_Positioner:
      initUi_Positioner();
      break;
    default:
      break;
    }
}

void MechanicSelectionPanel::initUi_framework() {
  setModal(true);
  m_proxy = new QWidget();
  m_ui = new Ui_MechanicSelectionPanel();
  m_ui->setupUi(m_proxy);
  this->addWidget(m_proxy);

  m_stringList = new QStringList();
  m_stringListModel = new QStringListModel(*m_stringList, NULL);
  m_ui->listView_MechanicList->setModel(m_stringListModel);
  QItemSelectionModel *selectionModelPtr = m_ui->listView_MechanicList->selectionModel();
  QObject::connect(selectionModelPtr, SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
                   this, SLOT(slot_robotSelectionChanged(QItemSelection)));
  connectSignals();
}

void MechanicSelectionPanel::initUi_RobotSelection()
{
    QString dlg_name = tr("Robot Selection");
    setWindowTitle(dlg_name);
    auto brands = m_DatabasePtr->getAvailableRobotBrands();
    for(auto brand : brands){
        m_ui->comboBox_brand->addItem(tr(Robot::RobotBrand_str[(int)brand]));
    }
    slot_brandSelected();
}

void MechanicSelectionPanel::initUi_ExtAxisSelection()
{
    QString dlg_name = tr("Ext-Axis Selection");
    setWindowTitle(dlg_name);
    auto brands = m_DatabasePtr->getAvailableExtAxBrands();
    for(auto brand : brands){
        m_ui->comboBox_brand->addItem(tr(Robot::ExtAxBrand_str[(int)brand]));
    }
    slot_brandSelected();
}

void MechanicSelectionPanel::initUi_Positioner()
{
    QString dlg_name = tr("Positioner Selection");
    setWindowTitle(dlg_name);
    auto brands = m_DatabasePtr->getAvailablePoserBrands();
    for(auto brand : brands){
        m_ui->comboBox_brand->addItem(tr(Robot::PoserBrand_str[(int)brand]));
    }
    slot_brandSelected();
}

void MechanicSelectionPanel::connectSignals()
{
    QObject::connect(m_ui->comboBox_brand, SIGNAL(currentIndexChanged(QString)),
                     this, SLOT(slot_brandSelected()));
    QObject::connect(m_ui->comboBox_payload, SIGNAL(currentIndexChanged(QString)),
                     this, SLOT(slot_payloadSelected()));
    QObject::connect(m_ui->comboBox_MechType, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(slot_typeSelected()));
    QObject::connect(m_ui->comboBox_MechDof, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(slot_dofSelected()));
}

void MechanicSelectionPanel::slot_brandSelected()
{
    switch(m_Type){
    case Robot::MechanicType::M_Robot:
        m_selectedModelBrand = std::string(Robot::RobotBrand_str[m_ui->comboBox_brand->currentIndex()]);
        break;
    case Robot::MechanicType::M_Positioner:
        m_selectedModelBrand = std::string(Robot::PoserBrand_str[m_ui->comboBox_brand->currentIndex()]);
        break;
    case Robot::MechanicType::M_ExtAxis:
        m_selectedModelBrand = std::string(Robot::ExtAxBrand_str[m_ui->comboBox_brand->currentIndex()]);
        break;
    default:
        break;
    }
    update_MechTypeList();
    update_MechDofList();
    update_MechPayloadList();
    update_MechNameList();
}
void MechanicSelectionPanel::slot_typeSelected()
{
    update_MechDofList();
    update_MechPayloadList();
    update_MechNameList();
}

void MechanicSelectionPanel::slot_dofSelected()
{
    update_MechPayloadList();
    update_MechNameList();
}


void MechanicSelectionPanel::slot_payloadSelected()
{
    update_MechNameList();
}

void MechanicSelectionPanel::slot_robotSelectionChanged(QItemSelection selection)
{
    if (selection.indexes().isEmpty())
        return;
    auto itemName_QString = selection.indexes().first().data(Qt::DisplayRole).toString();
    m_selectedModelName = std::string((const char*)itemName_QString.toLocal8Bit().constData());
    // TODO: Change Display Figure
}

void MechanicSelectionPanel::update_MechTypeList()
{
    m_ui->comboBox_MechType->clear();
    switch(m_Type){
    case Robot::MechanicType::M_Robot:{
        auto t_types = m_DatabasePtr->getAvailableRobotType((Robot::RobotBrand)m_ui->comboBox_brand->currentIndex());
        for(auto t_type : t_types){
            m_ui->comboBox_MechType->addItem(tr(Robot::RobotType_str[(int)t_type]));
        }
    }
        break;
    case Robot::MechanicType::M_Positioner:{
        auto t_types = m_DatabasePtr->getAvailablePoserType((Robot::PoserBrand)m_ui->comboBox_brand->currentIndex());
        for(auto t_type : t_types){
            m_ui->comboBox_MechType->addItem(tr(Robot::PoserType_str[(int)t_type]));
        }
    }
        break;
    case Robot::MechanicType::M_ExtAxis:{
        auto t_types = m_DatabasePtr->getAvailableExtAxType((Robot::ExtAxBrand)m_ui->comboBox_brand->currentIndex());
        for(auto t_type : t_types){
            m_ui->comboBox_MechType->addItem(tr(Robot::ExtAxBrand_str[(int)t_type]));
        }
    }
        break;
    default:
        break;
    }
}

void MechanicSelectionPanel::update_MechPayloadList()
{
    m_ui->comboBox_payload->clear();
    m_ui->comboBox_payload->addItem(QString::number(0));
    switch(m_Type){
    case Robot::MechanicType::M_Robot:{
        auto t_payloads = m_DatabasePtr->getAvailableRobot_Payload((Robot::RobotBrand)m_ui->comboBox_brand->currentIndex(),
                                                                   (Robot::RobotType)m_ui->comboBox_MechType->currentIndex(),
                                                                   (uint)m_ui->comboBox_MechDof->currentText().toInt());
        for(auto t_payload : t_payloads){
            m_ui->comboBox_payload->addItem(QString::number(t_payload));
        }
    }
        break;
    case Robot::MechanicType::M_Positioner:{
        auto t_payloads = m_DatabasePtr->getAvailablePoser_Payload((Robot::PoserBrand)m_ui->comboBox_brand->currentIndex(),
                                                                   (Robot::PoserType)m_ui->comboBox_MechType->currentIndex(),
                                                                   (uint)m_ui->comboBox_MechDof->currentText().toInt());
        for(auto t_payload : t_payloads){
            m_ui->comboBox_payload->addItem(QString::number(t_payload));
        }
    }
        break;
    case Robot::MechanicType::M_ExtAxis:{
        auto t_payloads = m_DatabasePtr->getAvailableExtAx_Payload((Robot::ExtAxBrand)m_ui->comboBox_brand->currentIndex(),
                                                                   (Robot::ExtAxType)m_ui->comboBox_MechType->currentIndex(),
                                                                   (uint)m_ui->comboBox_MechDof->currentText().toInt());
        for(auto t_payload : t_payloads){
            m_ui->comboBox_payload->addItem(QString::number(t_payload));
        }
    }
        break;
    default:
        break;
    }
}

void MechanicSelectionPanel::update_MechDofList()
{
    m_ui->comboBox_MechDof->clear();
    m_ui->comboBox_MechDof->addItem(QString::number(0));
    switch(m_Type){
    case Robot::MechanicType::M_Robot:{
        auto t_dofs = m_DatabasePtr->getAvailableRobot_DOF((Robot::RobotBrand)m_ui->comboBox_brand->currentIndex(),
                                                           (Robot::RobotType)m_ui->comboBox_MechType->currentIndex());
        for(auto t_dof : t_dofs){
            m_ui->comboBox_MechDof->addItem(QString::number(t_dof));
        }
    }
        break;
    case Robot::MechanicType::M_Positioner:{
        auto t_dofs = m_DatabasePtr->getAvailablePoser_DOF((Robot::PoserBrand)m_ui->comboBox_brand->currentIndex(),
                                                           (Robot::PoserType)m_ui->comboBox_MechType->currentIndex());
        for(auto t_dof : t_dofs){
            m_ui->comboBox_MechDof->addItem(QString::number(t_dof));
        }
    }
        break;
    case Robot::MechanicType::M_ExtAxis:{
        auto t_dofs = m_DatabasePtr->getAvailableExtAx_DOF((Robot::ExtAxBrand)m_ui->comboBox_brand->currentIndex(),
                                                           (Robot::ExtAxType)m_ui->comboBox_MechType->currentIndex());
        for(auto t_dof : t_dofs){
            m_ui->comboBox_MechDof->addItem(QString::number(t_dof));
        }
    }
        break;
    default:
        break;
    }
}

void MechanicSelectionPanel::update_MechNameList()
{
    m_stringList->clear();
    switch(m_Type){
    case Robot::MechanicType::M_Robot:{
        auto t_items = m_DatabasePtr->getParaMatched_Robots((Robot::RobotBrand)m_ui->comboBox_brand->currentIndex(),
                                                            (Robot::RobotType)m_ui->comboBox_MechType->currentIndex(),
                                                            (uint)m_ui->comboBox_MechDof->currentText().toInt(),
                                                            (float)m_ui->comboBox_payload->currentText().toFloat());
        for(auto& t_item : t_items){
            m_stringList->append(QString::fromStdString(t_item.model_Name));
        }
    }
        break;
    case Robot::MechanicType::M_Positioner:{
        auto t_items = m_DatabasePtr->getParaMatched_Posers((Robot::PoserBrand)m_ui->comboBox_brand->currentIndex(),
                                                            (Robot::PoserType)m_ui->comboBox_MechType->currentIndex(),
                                                            (uint)m_ui->comboBox_MechDof->currentText().toInt(),
                                                            (float)m_ui->comboBox_payload->currentText().toFloat());
        for(auto& t_item : t_items){
            m_stringList->append(QString::fromStdString(t_item.model_Name));
        }
    }
        break;
    case Robot::MechanicType::M_ExtAxis:{
        auto t_items = m_DatabasePtr->getParaMatched_ExtAxs((Robot::ExtAxBrand)m_ui->comboBox_brand->currentIndex(),
                                                            (Robot::ExtAxType)m_ui->comboBox_MechType->currentIndex(),
                                                            (uint)m_ui->comboBox_MechDof->currentText().toInt(),
                                                            (float)m_ui->comboBox_payload->currentText().toFloat());
        for(auto& t_item : t_items){
            m_stringList->append(QString::fromStdString(t_item.model_Name));
        }
    }
        break;
    default:
        break;
    }
    m_stringListModel->setStringList(*m_stringList);
}

void MechanicSelectionPanel::accept() {
  if (insertMechanics())
      return this->done(QDialog::Accepted);
}

void MechanicSelectionPanel::reject() {
  return this->done(QDialog::Rejected);
}


bool MechanicSelectionPanel::insertMechanics() {
  if (m_selectedModelBrand.empty() ||
      m_selectedModelName.empty()) {
    QMessageBox message(QMessageBox::Warning, tr("Alert"),
                        tr("Target Device has not been SELECTED!"),
                        QMessageBox::Yes | QMessageBox::No, NULL);
    message.exec();
    return false;
  }
  std::string model_info = m_selectedModelBrand + "_" + m_selectedModelName;
  Gui::Command::openCommand("Insert Mechanic");
  Gui::Command::doCommand(Gui::Command::Doc, "import Robot");
  switch(m_Type){
  case Robot::MechanicType::M_Robot:
      Gui::Command::doCommand(Gui::Command::Doc, "Robot.insert_ROBOT(\"%s\",\"%s\")",
                              model_info.c_str(),
                              m_DocPtr->getName());
      break;
  case Robot::MechanicType::M_Positioner:
      Gui::Command::doCommand(Gui::Command::Doc, "Robot.insert_POSER(\"%s\",\"%s\")",
                              model_info.c_str(),
                              m_DocPtr->getName());
      break;
  case Robot::MechanicType::M_ExtAxis:
      Gui::Command::doCommand(Gui::Command::Doc, "Robot.insert_EXTAX(\"%s\",\"%s\")",
                              model_info.c_str(),
                              m_DocPtr->getName());
      break;
  default:
      break;
  }

  Gui::Command::doCommand(Gui::Command::Gui, "Gui.SendMsgToActiveView(\"ViewFit\")");
  if (m_DocPtr->getObjects().size() == 1) {
      Gui::Command::doCommand(Gui::Command::Gui,"Gui.activeDocument().activeView().viewAxonometric()");
  }
  Gui::Command::commitCommand();
  return true;
}


#include "moc_MechanicSelectionPanel.cpp"

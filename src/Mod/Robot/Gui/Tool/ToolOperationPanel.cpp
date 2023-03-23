
#include "ToolOperationPanel.h"
#include "ui_ToolOperationPanel.h"
#include <Gui/Command.h>
#include <QMessageBox>
#include <Mod/Robot/App/Mechanics/MechanicRobot.h>
#include <Mod/Robot/App/Mechanics/MechanicPoser.h>

using namespace RobotGui;

ToolOperationPanel::ToolOperationPanel(App::Document *pDoc, Robot::ToolType t_Type,
                                       QWidget *parent): TaskManageDialog(parent)
{

    if(pDoc == nullptr)
        return;
    showButtonBox(false);
    m_DatabasePtr = std::make_shared<Robot::ToolDatabase>();
    m_DocPtr = pDoc;
    m_ToolType = t_Type;
    initUi_framework();
    QString panel_Name = tr("ToolSetup Panel");
    setWindowTitle(panel_Name);
}

void ToolOperationPanel::initUi_framework() {
  setModal(true);
  m_proxy = new QWidget();
  m_ui = new Ui_ToolOperationPanel();
  m_ui->setupUi(m_proxy);
  this->addWidget(m_proxy);

  m_stringList = new QStringList();
  m_stringListModel = new QStringListModel(*m_stringList, NULL);

  switch(m_ToolType){
  case Robot::ToolType::WeldTorch:
      initUi_WeldTorchPanel();
      break;
  case Robot::ToolType::_2DScanner:
      initUi_2DScannerPanel();
      break;
  case Robot::ToolType::_3DCamera:
      initUi_3DCameraPanel();
      break;
  case Robot::ToolType::Gripper:
      initUi_GripperPanel();
      break;
  default:
      break;
  }
//  update_AssembleTargetList();
  connectSignals();
}

void ToolOperationPanel::initUi_WeldTorchPanel()
{
    m_ui->tabWidget->setCurrentIndex(0);
    m_ui->tab_2DScanner->setEnabled(false);
    m_ui->tab_3DCamera->setEnabled(false);
    m_ui->tab_Gripper->setEnabled(false);
    auto torchBrands = m_DatabasePtr->getAvailableTorchBrands();
    for(auto t_brand : torchBrands){
        m_ui->comboBox_TorchBrand->addItem(tr(Robot::StrList_WeldTorchBrands[(int)t_brand]));
    }
    m_ui->listView_TorchList->setModel(m_stringListModel);
    m_SelectionModel = m_ui->listView_TorchList->selectionModel();
    m_ui->pushButton_CreateTool->setText(tr("Create Torch"));
    slot_brandSelected();
}

void ToolOperationPanel::initUi_2DScannerPanel()
{
    m_ui->tabWidget->setCurrentIndex(1);
    m_ui->tab_weldTorch->setEnabled(false);
    m_ui->tab_3DCamera->setEnabled(false);
    m_ui->tab_Gripper->setEnabled(false);
    auto scannerBrand = m_DatabasePtr->getAvailableScannerBrands();
    for(auto t_brand : scannerBrand){
        m_ui->comboBox_2DScannerBrand->addItem(tr(Robot::StrList_2DScannerBrands[(int)t_brand]));
    }
    m_ui->listView_2DScannerList->setModel(m_stringListModel);
    m_SelectionModel = m_ui->listView_2DScannerList->selectionModel();
    m_ui->pushButton_CreateTool->setText(tr("Create Scanner"));
    slot_brandSelected();
}

void ToolOperationPanel::initUi_3DCameraPanel()
{
    m_ui->tabWidget->setCurrentIndex(2);
    m_ui->tab_weldTorch->setEnabled(false);
    m_ui->tab_2DScanner->setEnabled(false);
    m_ui->tab_Gripper->setEnabled(false);
//    auto depthCameraBrands = m_DatabasePtr->getA();
//    for(auto t_brand : depthCameraBrands){
//        m_ui->comboBox_2DScannerBrand->addItem(tr(Robot::StrList_2DScannerBrands[(int)t_brand]));
//    }
    m_ui->listView_3DCameraList->setModel(m_stringListModel);
    m_ui->pushButton_CreateTool->setText(tr("Create Camera"));
    m_ui->pushButton_LoadTool->setText(tr("Insert Camera"));
    slot_brandSelected();
}

void ToolOperationPanel::initUi_GripperPanel()
{
    m_ui->tabWidget->setCurrentIndex(3);
    m_ui->tab_weldTorch->setEnabled(false);
    m_ui->tab_3DCamera->setEnabled(false);
    m_ui->tab_2DScanner->setEnabled(false);
//    auto scannerBrand = m_DatabasePtr->getAvailableScannerBrands();
//    for(auto t_brand : scannerBrand){
//        m_ui->comboBox_2DScannerBrand->addItem(tr(Robot::StrList_2DScannerBrands[(int)t_brand]));
//    }
    m_ui->listView_GripperList->setModel(m_stringListModel);
    slot_brandSelected();
}


void ToolOperationPanel::connectSignals()
{
    QObject::connect(m_ui->comboBox_TorchBrand, SIGNAL(currentIndexChanged(QString)),
                     this, SLOT(slot_brandSelected()));
    QObject::connect(m_ui->comboBox_2DScannerBrand, SIGNAL(currentIndexChanged(QString)),
                     this, SLOT(slot_brandSelected()));
    QObject::connect(m_ui->comboBox_3DCameraBrand, SIGNAL(currentIndexChanged(QString)),
                     this, SLOT(slot_brandSelected()));

    QObject::connect(m_ui->comboBox_TorchTubeType, SIGNAL(currentIndexChanged(QString)),
                     this, SLOT(slot_torchTubeTypeSelected()));
    QObject::connect(m_ui->comboBox_TorchTubeLength, SIGNAL(currentIndexChanged(QString)),
                     this, SLOT(slot_torchTubeLengthSelected()));
    QObject::connect(m_ui->comboBox_2DScannerMount, SIGNAL(currentIndexChanged(QString)),
                     this, SLOT(slot_2DScannerMountSelected()));
    QObject::connect(m_ui->comboBox_2DScannerRange, SIGNAL(currentIndexChanged(QString)),
                     this, SLOT(slot_2DScannerRangeSelected()));

    QObject::connect(m_ui->pushButton_LoadTool, SIGNAL(clicked()),
                     this, SLOT(slot_ImportSelectedTool()));
    QObject::connect(m_ui->pushButton_CreateTool, SIGNAL(clicked()),
                     this, SLOT(slot_CreateNewTool()));
    QObject::connect(m_ui->pushButton_cancel, SIGNAL(clicked()),
                     this, SLOT(slot_CancelOperation()));

    if(m_SelectionModel!=nullptr)
        QObject::connect(m_SelectionModel, SIGNAL(selectionChanged(QItemSelection, QItemSelection)),
                         this, SLOT(slot_SelectionChanged(QItemSelection)));
}

void ToolOperationPanel::slot_brandSelected()
{
    switch (m_ToolType) {
    case Robot::ToolType::WeldTorch:{
        m_selectedToolBrand = std::string(Robot::StrList_WeldTorchBrands[m_ui->comboBox_TorchBrand->currentIndex()]);
        update_TorchTubeType();
        update_TorchTubeLength();
    }
        break;
    case Robot::ToolType::_2DScanner:{
        m_selectedToolBrand = std::string(Robot::StrList_2DScannerBrands[m_ui->comboBox_2DScannerBrand->currentIndex()]);
        update_2DScannerMount();
        update_2DScannerRange();
    }
        break;
    case Robot::ToolType::_3DCamera:
        break;
    case Robot::ToolType::Gripper:
        break;
    default:
        break;
    }
    update_ToolItemList();
}

void ToolOperationPanel::slot_torchTubeTypeSelected()
{
    update_TorchTubeLength();
    update_ToolItemList();
}

void ToolOperationPanel::slot_torchTubeLengthSelected()
{
    update_ToolItemList();
}

void ToolOperationPanel::slot_2DScannerMountSelected()
{
    update_2DScannerRange();
    update_ToolItemList();
}

void ToolOperationPanel::slot_2DScannerRangeSelected()
{
    update_ToolItemList();
}

void ToolOperationPanel::update_TorchTubeType()
{
    m_ui->comboBox_TorchTubeType->clear();
    auto tubeTypes = m_DatabasePtr->getAvailableTorchTubeType((Robot::Brand_WeldTorch)m_ui->comboBox_TorchBrand->currentIndex());
    for(auto t_type : tubeTypes){
        m_ui->comboBox_TorchTubeType->addItem(tr(Robot::StrList_TorchTubeType[(int)t_type]));
    }
}

void ToolOperationPanel::update_TorchTubeLength()
{
    m_ui->comboBox_TorchTubeLength->clear();
    auto tubeLength = m_DatabasePtr->getAvailableTorchTubeLength((Robot::Brand_WeldTorch)m_ui->comboBox_TorchBrand->currentIndex(),
                                                                 (Robot::Type_TorchTube)m_ui->comboBox_TorchTubeType->currentIndex());
    for(auto t_length : tubeLength){
        m_ui->comboBox_TorchTubeLength->addItem(QString::number(t_length));
    }
}

void ToolOperationPanel::update_2DScannerMount()
{
    m_ui->comboBox_2DScannerMount->clear();
    auto mountType = m_DatabasePtr->getAvailabeScannerMountPose((Robot::Brand_2DScanner)m_ui->comboBox_2DScannerBrand->currentIndex());
    for(auto t_type : mountType){
        m_ui->comboBox_2DScannerMount->addItem(tr(Robot::StrList_2DScannerMount[(int)t_type]));
    }
}

void ToolOperationPanel::update_2DScannerRange()
{
    m_ui->comboBox_2DScannerRange->clear();
    auto rangeList = m_DatabasePtr->getAvailableScannerRange((Robot::Brand_2DScanner)m_ui->comboBox_2DScannerBrand->currentIndex(),
                                                              Robot::ToolDatabase::findMountPose(m_ui->comboBox_2DScannerMount->currentText().toStdString()));
    for(auto t_range : rangeList){
        m_ui->comboBox_2DScannerRange->addItem(QString::number(t_range));
    }
}

void ToolOperationPanel::update_ToolItemList()
{
    m_stringList->clear();
    switch(m_ToolType){
    case Robot::ToolType::WeldTorch:{
        auto t_items = m_DatabasePtr->getParaMatchedTool_WeldTorch((Robot::Brand_WeldTorch)m_ui->comboBox_TorchBrand->currentIndex(),
                                                                   (Robot::Type_TorchTube)m_ui->comboBox_TorchTubeType->currentIndex(),
                                                                   0.0);

        for(auto& t_item : t_items){
            m_stringList->append(QString::fromStdString(t_item.torch_Name));
        }
    }
        break;
    case Robot::ToolType::_2DScanner:{
        auto t_items = m_DatabasePtr->getParaMatchedTool_2DScanner((Robot::Brand_2DScanner)m_ui->comboBox_2DScannerBrand->currentIndex(),
                                                                    Robot::ToolDatabase::findMountPose(m_ui->comboBox_2DScannerMount->currentText().toStdString()),
                                                                    0.0);

        for(auto& t_item : t_items){
            m_stringList->append(QString::fromStdString(t_item.scanner_Name));
        }
    }
        break;
    case Robot::ToolType::_3DCamera:
        break;
    case Robot::ToolType::Gripper:
        break;
    default:
        break;
    }
    m_stringListModel->setStringList(*m_stringList);
}

void ToolOperationPanel::slot_ImportSelectedTool() {

    Gui::Command::openCommand("Insert Tool");
    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "import Robot");
    auto str_selectedItemInfo = m_DatabasePtr->getResFilePath();
    switch(m_ToolType){
    case Robot::ToolType::WeldTorch:
        Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.insert_WeldTorch(\"%s\",\"%s\")",
                                (str_selectedItemInfo+"/Torch/"+m_selectedToolName+".tor").c_str(),
                                m_DocPtr->getName());
        break;
    case Robot::ToolType::_2DScanner:
        Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.insert_2DScanner(\"%s\",\"%s\")",
                                (str_selectedItemInfo+"/Sensor/Scanner"+m_selectedToolName+".lsr").c_str(),
                                m_DocPtr->getName());
        break;
    case Robot::ToolType::_3DCamera:
        Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.insert_3DCamera(\"%s\",\"%s\")",
                                str_selectedItemInfo.c_str(),
                                m_DocPtr->getName());
        break;
    case Robot::ToolType::Gripper:
        Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.insert_Gripper(\"%s\",\"%s\")",
                                str_selectedItemInfo.c_str(),
                                m_DocPtr->getName());
        break;
    default:
        break;
    }
    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.activeDocument().activeView().viewAxonometric()");
    Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.SendMsgToActiveView(\"ViewFit\")");
    Gui::Command::commitCommand();
    return accept();
}

void ToolOperationPanel::slot_CancelOperation()
{
    return reject();
}

void ToolOperationPanel::slot_SelectionChanged(QItemSelection selection)
{
    if (selection.indexes().isEmpty())
        return;
    auto itemName_QString = selection.indexes().first().data(Qt::DisplayRole).toString();
    m_selectedToolName = std::string((const char*)itemName_QString.toLocal8Bit().constData());
}

void ToolOperationPanel::slot_CreateNewTool()
{
      QStringList filter;
      filter << QString::fromLatin1("STEP (*.stp *.step)");
      filter << QString::fromLatin1("STEP with colors (*.stp *.step)");
      filter << QString::fromLatin1("IGES (*.igs *.iges)");
      filter << QString::fromLatin1("IGES with colors (*.igs *.iges)");
      QString select;
      QString fn = Gui::FileDialog::getOpenFileName(Gui::getMainWindow(),
                                                    tr("Tool Model Selection"),
                                                    QString(),
                                                    filter.join(QLatin1String(";;")),
                                                    &select);
      auto pDocPtr = App::GetApplication().newDocument("Tool Setup");
      if (!fn.isEmpty()) {
        Gui::Command::openCommand("Create Tool");
        Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "import Robot");
        switch(m_ToolType){
        case Robot::ToolType::WeldTorch:
            Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.create_WeldTorch(\"%s\",\"%s\")",
                                    (const char *)fn.toUtf8(),
                                    pDocPtr->getName());
            break;
        case Robot::ToolType::_2DScanner:
            Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.create_2DScanner(\"%s\",\"%s\")",
                                    (const char *)fn.toUtf8(),
                                    pDocPtr->getName());
            break;
        case Robot::ToolType::_3DCamera:
            Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.create_3DCamera(\"%s\",\"%s\")",
                                    (const char *)fn.toUtf8(),
                                    pDocPtr->getName());
            break;
        case Robot::ToolType::Gripper:
            Gui::Command::doCommand(Gui::Command::DoCmd_Type::Doc, "Robot.create_Gripper(\"%s\",\"%s\")",
                                    (const char *)fn.toUtf8(),
                                    pDocPtr->getName());
            break;
        default:
            break;
        }
        if (pDocPtr->getObjects().size() == 1) {
          Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.activeDocument().activeView().viewAxonometric()");
          Gui::Command::doCommand(Gui::Command::DoCmd_Type::Gui, "Gui.SendMsgToActiveView(\"ViewFit\")");
        }
        Gui::Command::commitCommand();
      }
    return accept();
}

void ToolOperationPanel::accept() {
    return this->done(QDialog::Accepted);
}

void ToolOperationPanel::reject() {
    return this->done(QDialog::Rejected);
}




#include "moc_ToolOperationPanel.cpp"

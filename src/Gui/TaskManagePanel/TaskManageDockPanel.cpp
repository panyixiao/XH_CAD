// Created by Yixiao 2022/05/12

#include "PreCompiled.h"
#ifndef _PreComp_
#define emit Q_EMIT
#endif
#include "DockWindow.h"
#include "TaskManageDockPanel.h"
#include "TaskView/TaskDialog.h"
#include <QTabWidget>

using namespace Gui;
using namespace Gui::DockWnd;
using namespace Gui::TaskManage;

TaskManageDockPanel::TaskManageDockPanel(Gui::Document *pcDoc, QWidget *parent)
    : DockWindow(pcDoc, parent) {
  setWindowTitle(tr("Task Panel"));
  QGridLayout *pLayout = new QGridLayout(this);
  pLayout->setSpacing(0);
  pLayout->setMargin(0);
  m_tab = new QTabWidget();
  m_tab->setObjectName(tr("ComboTab"));
  m_tab->setTabPosition(QTabWidget::North);
  m_tab->setTabShape(QTabWidget::Triangular);
  pLayout->addWidget(m_tab, 0, 0);
  QObject::connect(m_tab, SIGNAL(currentChanged(int)),
                   this, SLOT(tabChanged(int)));
}

TaskManageDockPanel::~TaskManageDockPanel() {}

bool TaskManageDockPanel::showDialog(TaskManageDlg_sptr dlg_Ptr) {
  if (dlg_Ptr == nullptr)
    return false;
  addNewDialog(dlg_Ptr);
  setActiveDialog(dlg_Ptr);
  return true;
}

void TaskManageDockPanel::closeCurrentDialog() {
  if (m_ActiveDlg != nullptr) {
    // Remove Tab
    m_tab->removeTab(m_tab->indexOf(m_ActiveDlg.get()));
    // Remove from registered dlg Container;
    auto dlg_ptr = std::find_if(
        m_DialogContainer.begin(), m_DialogContainer.end(),
        [this](const TaskManageDlg_sptr dlgPtr) {
          return this->m_ActiveDlg->getDialogType() == dlgPtr->getDialogType();
        });
    if (dlg_ptr != m_DialogContainer.end()) {
      m_DialogContainer.erase(dlg_ptr);
    }
  }
  if (!m_DialogContainer.empty())
    setActiveDialog(m_DialogContainer.front());
}

void TaskManageDockPanel::setActiveDialog(TaskManageDlg_sptr dlgPtr) {
  m_ActiveDlg = dlgPtr;
  connectSignals();
  for (int i = 0; i < m_tab->count(); i++) {
    m_tab->setTabEnabled(i, false);
  }
  m_tab->setTabEnabled(m_tab->indexOf(m_ActiveDlg.get()), true);
  m_tab->setCurrentWidget(m_ActiveDlg.get());

  m_tab->resize(dlgPtr->width(), m_tab->height());
}

void TaskManageDockPanel::addNewDialog(TaskManageDlg_sptr t_DlgPtr) {
  if (t_DlgPtr == nullptr)
    return;
  // Remove from registered dlg Container;
  auto dlg_Iter = std::find_if(m_DialogContainer.begin(), m_DialogContainer.end(),
                              [t_DlgPtr](const TaskManageDlg_sptr c_DlgPtr) {
                                    return t_DlgPtr->getDialogType() == c_DlgPtr->getDialogType();});
  if (dlg_Iter != m_DialogContainer.end()) {
    m_tab->removeTab(m_tab->indexOf(dlg_Iter->get()));
    m_DialogContainer.erase(dlg_Iter);
  }
  m_DialogContainer.push_back(t_DlgPtr);
  m_tab->addTab(t_DlgPtr.get(), t_DlgPtr->windowTitle());
}

bool TaskManageDockPanel::connectSignals() {
  if (m_ActiveDlg == nullptr)
    return false;
  // make connection of slots & signals
  connect(m_ActiveDlg.get(), SIGNAL(accepted()), this, SLOT(activeDlg_accepted()));
  connect(m_ActiveDlg.get(), SIGNAL(rejected()), this, SLOT(activeDlg_rejected()));
  return true;
}

bool TaskManageDockPanel::activeDlg_accepted() {
  if (m_ActiveDlg == nullptr)
    return true;
  emit dlg_Accepted(m_ActiveDlg->getEditingObject());
  closeCurrentDialog();
  return true;
}

bool TaskManageDockPanel::activeDlg_rejected() {
  if (m_ActiveDlg == nullptr)
    return true;
  emit dlg_Rejected(m_ActiveDlg->getEditingObject());
  closeCurrentDialog();
  return true;
}

void TaskManageDockPanel::helpRequest() {}

void TaskManageDockPanel::clicked(QAbstractButton *button) {}

void TaskManageDockPanel::tabChanged(int index) {
  //    auto rd_dlg = dynamic_cast<RD_Panel::TaskManageDialog*>(m_tab->widget(index));
  //    updateActiveDlg(rd_dlg);
  //    showActiveDialog();
}

#include "TaskManagePanel/moc_TaskManageDockPanel.cpp"

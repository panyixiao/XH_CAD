// Created by Yixiao 2022/05/10

#include "Mod/Robot/Gui/PreCompiled.h"
#ifndef _PreComp_
#endif

#include <Base/Console.h>
#include <Gui/Application.h>
#include <Gui/Document.h>
#include <Gui/SoFCUnifiedSelection.h>
#include <Gui/View3DInventor.h>
#include <Gui/View3DInventorViewer.h>
#include <Inventor/nodes/SoPickStyle.h>

#include "DocUtility.h"

using namespace RobotGui;
using namespace Gui;

DocUtility::DocUtility() {}

bool DocUtility::setSceneGraphSelectable(bool selectable) {
  Gui::Document *doc = Gui::Application::Instance->activeDocument();
  if (doc == nullptr)
    return false;
  auto view = dynamic_cast<View3DInventor *>(doc->getActiveView());
  if (view == nullptr) {
    Base::Console().Error("Failed to get View3DInventor from current document ActiveView!\n");
    return false;
  }
  auto m_3dInventorViewer = view->getViewer();
  if (m_3dInventorViewer == nullptr) {
    Base::Console().Error("Failed to get View3DInventorViewer from current "
                          "document View3DInventor!\n");
    return false;
  }

  if (selectable){
    SoNode *child = static_cast<SoFCUnifiedSelection *>(m_3dInventorViewer->getSceneGraph()) ->getChild(0);
    if (child && child->isOfType(SoPickStyle::getClassTypeId()))
        static_cast<SoFCUnifiedSelection *>(m_3dInventorViewer->getSceneGraph())->removeChild(child);
  }
  else {
    SoPickStyle *rootPickStyle = new SoPickStyle();
    rootPickStyle->style = SoPickStyle::UNPICKABLE;
    static_cast<SoFCUnifiedSelection *>(m_3dInventorViewer->getSceneGraph())->insertChild(rootPickStyle, 0);
  }
  return true;
}

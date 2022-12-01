// Created by Yixiao 2022/04/24
#include "Mod/Robot/Gui/PreCompiled.h"

#ifndef _PreComp_
# include <Inventor/SoDB.h>
# include <Inventor/SoInput.h>
# include <Inventor/SbVec3f.h>
# include <Inventor/nodes/SoSeparator.h>
# include <Inventor/nodes/SoTransform.h>
# include <Inventor/nodes/SoSphere.h>
# include <Inventor/nodes/SoRotation.h>
# include <Inventor/actions/SoSearchAction.h>
# include <Inventor/draggers/SoJackDragger.h>
# include <Inventor/VRMLnodes/SoVRMLTransform.h>
# include <Inventor/nodes/SoBaseColor.h>
# include <Inventor/nodes/SoCoordinate3.h>
# include <Inventor/nodes/SoDrawStyle.h>
# include <Inventor/nodes/SoFaceSet.h>
# include <Inventor/nodes/SoLineSet.h>
# include <Inventor/nodes/SoMarkerSet.h>
# include <Inventor/nodes/SoShapeHints.h>
# include <QFile>
#endif

#include "ViewProviderScannerObject.h"
#include "TaskDlgToolSetupPanel.h"
#include <App/Application.h>
#include <App/Document.h>
#include <Base/Console.h>
#include <Gui/Command.h>
#include <Gui/Control.h>
#include <functional>

using namespace std::placeholders;

using namespace Robot;
using namespace RobotGui;

PROPERTY_SOURCE(RobotGui::ViewProviderScannerObject, PartGui::ViewProviderPartExt)

ViewProviderScannerObject::ViewProviderScannerObject() {
    pcScanRoot = new Gui::SoFCSelection();
    pcScanRoot->highlightMode = Gui::SoFCSelection::OFF;
    pcScanRoot->selectionMode = Gui::SoFCSelection::SEL_OFF;
    pcScanRoot->ref();

    pcCoords = new SoCoordinate3();
    pcCoords->ref();
    pcDrawStyle = new SoDrawStyle();
    pcDrawStyle->ref();
    pcDrawStyle->style = SoDrawStyle::LINES;
    pcDrawStyle->lineWidth = 2;

    pcLines = new SoLineSet;
    pcLines->ref();


}

ViewProviderScannerObject::~ViewProviderScannerObject() {
    pcScanRoot->unref();
    pcCoords->unref();
    pcDrawStyle->unref();
    pcLines->unref();
}

void ViewProviderScannerObject::onChanged(const App::Property *prop)
{

    ViewProviderToolObject::onChanged(prop);
}

void ViewProviderScannerObject::attach(App::DocumentObject *obj) {

    ViewProviderToolObject::attach(obj);

    SoSeparator* linesep = new SoSeparator;
    SoBaseColor * basecol = new SoBaseColor;
    basecol->rgb.setValue( 1.0f, 0.0f, 0.0f );
    linesep->addChild(basecol);
    linesep->addChild(pcCoords);
    linesep->addChild(pcLines);

    // Draw markers
    SoBaseColor * markcol = new SoBaseColor;
    markcol->rgb.setValue( 1.0f, 0.0f, 0.0f );
    SoMarkerSet* marker = new SoMarkerSet;
    marker->markerIndex=SoMarkerSet::CROSS_5_5;
    linesep->addChild(markcol);
    linesep->addChild(marker);

    auto root = static_cast<SoGroup*>(getDisplayMaskMode("Flat Lines"));
    root->addChild(linesep);
}

void ViewProviderScannerObject::updateData(const App::Property *prop) {

    Robot::ScannerObject * scannerPtr = static_cast<Robot::ScannerObject *>(pcObject);
    if(prop == &scannerPtr->LaserOn){
        if(scannerPtr->LaserOn.getValue()){
            pcCoords->point.deleteValues(0);
            pcCoords->point.setNum(4);
            updateScanTrianglePose();
        }
        else{
            pcCoords->point.deleteValues(0);
            pcLines->numVertices.set1Value(0, 0);
        }
    }
    else if(prop == &scannerPtr->Placement){
        updateScanTrianglePose();
    }
    ViewProviderToolObject::updateData(prop);
}

bool ViewProviderScannerObject::setEdit(int ModNum) {
  if (ModNum == Gui::ViewProvider::EditMode::Default) {
    auto t_Scanner = static_cast<Robot::ScannerObject *>(pcObject);
    Gui::TaskView::TaskDialog *dlg = new TaskDlgToolObject(t_Scanner);
    if(dlg == nullptr)
        return false;
    Gui::Control().showDialog(dlg);
  }
  return true;
}

void ViewProviderScannerObject::setDisplayMode(const char *displayModeName) {
  ViewProviderToolObject::setDisplayMode(displayModeName);
}

std::vector<std::string> ViewProviderScannerObject::getDisplayModes() const {
    return ViewProviderToolObject::getDisplayModes();
}

bool ViewProviderScannerObject::doubleClicked() {
    return this->setEdit(Gui::ViewProvider::EditMode::Default);
}

void ViewProviderScannerObject::updateScanTrianglePose()
{
    Robot::ScannerObject * scannerPtr = static_cast<Robot::ScannerObject *>(pcObject);

    if(pcCoords->point.getNum() != 4)
        return;

    auto ref_Vertex = scannerPtr->Trans_O2M.getValue()*scannerPtr->Trans_M2T.getValue();
    pcCoords->point.set1Value(0,ref_Vertex.getPosition().x,
                                ref_Vertex.getPosition().y,
                                ref_Vertex.getPosition().z);
    auto ref_Mid = scannerPtr->getTransform_Origin2Front();

    auto diff = std::atan(scannerPtr->ScanAmplitute.getValue()*M_PI/180.0/2)*
                          scannerPtr->ScanDistance.getValue();
    Base::Placement p1,p2;

    p1 = ref_Mid * Base::Placement(Base::Vector3d(diff,0,0),Base::Rotation());
    p2 = ref_Mid * Base::Placement(Base::Vector3d(-diff,0,0),Base::Rotation());
    pcCoords->point.set1Value(1,p1.getPosition().x,
                                p1.getPosition().y,
                                p1.getPosition().z);
    pcCoords->point.set1Value(2,p2.getPosition().x,
                                p2.getPosition().y,
                                p2.getPosition().z);
    pcCoords->point.set1Value(3,ref_Vertex.getPosition().x,
                                ref_Vertex.getPosition().y,
                                ref_Vertex.getPosition().z);
    pcLines->numVertices.set1Value(0, 4);
}

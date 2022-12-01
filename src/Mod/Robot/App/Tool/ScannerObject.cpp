// Created by Yixiao 2022-06-15

#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "ScannerObject.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include "Mod/Robot/App/Utilites/PartUtility.h"

#include <App/Property.h>
#include <Base/Console.h>
#include <Base/FileInfo.h>
#include <Base/Reader.h>
#include <Base/Writer.h>


using namespace App;
using namespace Robot;
//using namespace RD_CAD_Utility;

PROPERTY_SOURCE(Robot::ScannerObject, Robot::ToolObject)

ScannerObject::ScannerObject() {

    m_Type = ToolType::Scanner;

    ADD_PROPERTY_TYPE(ScanDistance,(400.0),"Property",Prop_None, "A distance that Scanners can detect");
    ADD_PROPERTY_TYPE(ScanAmplitute,(15.0),"Property",Prop_None, "Angle Amplitute Scanners can detect");
    ADD_PROPERTY_TYPE(LaserOn, (false),"Property", Prop_None, "Switch to Turn on Laser");
    updateTip2FrontTrans();
}

ScannerObject::~ScannerObject() {}


bool ScannerObject::attachObject(DocumentObject *t_obj) {
    return ToolObject::attachObject(t_obj);
}

bool ScannerObject::detachObjcet(DocumentObject *t_obj) {
    return ToolObject::detachObjcet(t_obj);
}


void ScannerObject::Save(Base::Writer &writer) const {

  ToolObject::Save(writer);
}

void ScannerObject::Restore(Base::XMLReader &reader) {

  ToolObject::Restore(reader);
}

void ScannerObject::onChanged(const Property *prop)
{
    if(prop == &ScanDistance){
        updateTip2FrontTrans();
    }
    else if(prop == &Trans_M2T){
        updateTip2FrontTrans();
    }
    ToolObject::onChanged(prop);
}

void ScannerObject::onDocumentRestored() {
    ToolObject::onDocumentRestored();
}

void ScannerObject::updateTip2FrontTrans()
{
//    bool flip = false;
    double distance = ScanDistance.getValue();
    double y,p,r;
    Trans_M2T.getValue().getRotation().getYawPitchRoll(y,p,r);
    Base::Rotation t_Rot;
    if(std::abs(p) > 120 || std::abs(r) > 120){
        distance = -distance;
        t_Rot.setYawPitchRoll(0,0,180);
    }
    Trans_T2F.setValue(Base::Placement(Base::Vector3d(0,0,distance),
                                             t_Rot));
}

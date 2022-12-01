// Created by Yixiao 20220424
#include "Mod/Robot/App/PreCompiled.h"
#ifndef _PreComp_
#endif

#include "TorchObject.h"
#include "Mod/Robot/App/Utilites/DS_Utility.h"
#include "Mod/Robot/App/Utilites/PartUtility.h"

#include <App/Property.h>
#include <Base/Console.h>
#include <Base/FileInfo.h>
#include <Base/Reader.h>
#include <Base/Writer.h>


using namespace App;
using namespace Robot;

PROPERTY_SOURCE(Robot::TorchObject, Robot::ToolObject)

TorchObject::TorchObject() {
    m_Type = ToolType::WeldTorch;
    ADD_PROPERTY_TYPE(SparkOn,(false),"Property", Prop_None,"A Switch to Set Spark On/Off");
}

TorchObject::~TorchObject() {
}

void TorchObject::Save(Base::Writer &writer) const {

  ToolObject::Save(writer);
}

void TorchObject::Restore(Base::XMLReader &reader) {

  ToolObject::Restore(reader);
}

void TorchObject::onChanged(const Property *prop)
{
    ToolObject::onChanged(prop);
}

void TorchObject::onDocumentRestored() {
    ToolObject::onDocumentRestored();
}


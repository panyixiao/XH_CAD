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

bool TorchObject::saveTool()
{
    auto t_FileName = FilePath_Param.getStrValue();
    if(t_FileName.empty()){
        Base::Console().Error("Empty Param File Name");
        return false;
    }
    Base::FileInfo file(t_FileName.c_str());
    if (file.extension().empty()){
        t_FileName  += "/Torch/" + m_Info.torch_Name + ".tor";
        FilePath_Param.setValue(t_FileName);
    }
    m_FileOperator.openFile(t_FileName);
    m_FileOperator.insertItem(QObject::tr("TorchName"), QString::fromStdString(m_Info.torch_Name));
    m_FileOperator.insertItem(QObject::tr("TubeType"), QString::number((int)m_Info.tube_Type));
    m_FileOperator.insertItem(QObject::tr("TubeLength"), QString::number((uint)m_Info.tube_Length));
    return ToolObject::saveTool();
}

bool TorchObject::loadTool(const std::string &filePath)
{
    m_FileOperator.openFile(filePath, QIODevice::ReadOnly);
    m_Info.model_FilePath = m_FileOperator.readStringPropFromFile(QObject::tr("FilePath_Solid"));
    m_Info.torch_Name = m_FileOperator.readStringPropFromFile(QObject::tr("TorchName"));
    m_Info.tube_Type = (Type_TorchTube)m_FileOperator.readNumberPropFromFile(QObject::tr("TubeType"));
    m_Info.tube_Length = m_FileOperator.readNumberPropFromFile(QObject::tr("TubeLength"));
    return ToolObject::loadTool(filePath);
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


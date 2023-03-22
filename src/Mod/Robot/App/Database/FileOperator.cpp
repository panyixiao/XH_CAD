// Created By Yixiao 2023-03-17

//#include <Mod/Robot/App/Tool/ToolObject.h>
#include "FileOperator.h"

using namespace Robot;

QJsonObject FileOperator::poseToJsonObject(const Base::Placement &t_Pose)
{
    QJsonObject t_JsonObject;
    t_JsonObject.insert(QString::fromLocal8Bit("tX"), QString::number(t_Pose.getPosition().x));
    t_JsonObject.insert(QString::fromLocal8Bit("tY"), QString::number(t_Pose.getPosition().y));
    t_JsonObject.insert(QString::fromLocal8Bit("tZ"), QString::number(t_Pose.getPosition().z));
    double rx,ry,rz;
    t_Pose.getRotation().getYawPitchRoll(rz,ry,rx);
    t_JsonObject.insert(QString::fromLocal8Bit("rX"), QString::number(rx));
    t_JsonObject.insert(QString::fromLocal8Bit("rY"), QString::number(ry));
    t_JsonObject.insert(QString::fromLocal8Bit("rZ"), QString::number(rz));
    return t_JsonObject;
}

bool FileOperator::openFile(const std::string &t_FilePath, const QIODevice::OpenMode t_Mode)
{
    bool newFileOpened = false;
    if(m_FilePtr == nullptr){
        m_FilePtr = new QFile(QString::fromStdString(t_FilePath));
        newFileOpened = m_FilePtr->open(t_Mode);
    }
    else{
        if(m_FilePtr->isOpen() && m_FilePtr->fileName().toStdString() != t_FilePath){
            m_FilePtr->close();
            m_FilePtr->setFileName(QString::fromStdString(t_FilePath));
            newFileOpened = m_FilePtr->open(t_Mode);
        }
    }
    if(newFileOpened){
        auto byteArray = m_FilePtr->readAll();
        QJsonParseError err_msg;
        QJsonDocument t_Doc = QJsonDocument::fromJson(byteArray, &err_msg);
        if(!t_Doc.isNull() && err_msg.error == QJsonParseError::NoError){
            m_JsonDataBuffer = t_Doc.object();
        }
    }
    return true;
}

void FileOperator::closeFile()
{
    if(!m_FilePtr->isOpen() || m_FilePtr == nullptr)
        return;
    m_FilePtr->close();
}

const std::string FileOperator::readStringPropFromFile(const QString &t_KeyVal)
{
    std::string result;
    if(!m_FilePtr->isOpen())
        return result;
    if(m_JsonDataBuffer.contains(t_KeyVal)){
        result = m_JsonDataBuffer.value(t_KeyVal).toString().toStdString();
    }
    return result;
}

double FileOperator::readNumberPropFromFile(const QString &t_KeyVal)
{
    double result = 0;
    if(!m_FilePtr->isOpen())
        return -1;
    if(m_JsonDataBuffer.contains(t_KeyVal)){
        result = m_JsonDataBuffer.value(t_KeyVal).toString().toDouble();
    }
    return result;
}

const Base::Placement FileOperator::readPosePropFromFile(const QString &t_KeyVal)
{
    Base::Placement t_Pose;
    if(!m_FilePtr->isOpen())
        return t_Pose;
    if(m_JsonDataBuffer.contains(t_KeyVal)){
        auto poseData = m_JsonDataBuffer.value(t_KeyVal).toObject();
        auto tx = poseData.value(QObject::tr("tX")).toString().toDouble();
        auto ty = poseData.value(QObject::tr("tY")).toString().toDouble();
        auto tz = poseData.value(QObject::tr("tZ")).toString().toDouble();
        auto rx = poseData.value(QObject::tr("rX")).toString().toDouble();
        auto ry = poseData.value(QObject::tr("rY")).toString().toDouble();
        auto rz = poseData.value(QObject::tr("rZ")).toString().toDouble();
        t_Pose.setPosition(Base::Vector3(tx,ty,tz));
        Base::Rotation rot;
        rot.setYawPitchRoll(rz,ry,rx);
        t_Pose.setRotation(rot);
    }
    return t_Pose;
}

bool FileOperator::insertItem(const QString keyName, const QString value)
{
    if(!m_FilePtr->isOpen())
        return false;
    if(m_JsonDataBuffer.contains(keyName))
        return false;
//    m_JsonDataBuffer.insert(m_JsonDataBuffer.size(),value);
    m_JsonDataBuffer.insert(keyName, value);
    return true;
}

bool FileOperator::insertItem(const QString keyName, const QJsonObject &t_obj)
{
    if(!m_FilePtr->isOpen())
        return false;
    if(m_JsonDataBuffer.contains(keyName))
        return false;
//    m_JsonDataBuffer.insert(m_JsonDataBuffer.size(),t_obj);
    m_JsonDataBuffer.insert(keyName, t_obj);
    return true;
}

bool FileOperator::saveFile()
{
    if(!m_FilePtr->isOpen())
        return false;
    QJsonDocument t_Doc = QJsonDocument(m_JsonDataBuffer);
//    t_Doc.setObject(m_JsonArray);
    QByteArray t_byteArray = t_Doc.toJson(QJsonDocument::Compact);
    auto result = m_FilePtr->write(t_byteArray);
    m_FilePtr->close();
    m_FilePtr = nullptr;
    return result!=-1;
}

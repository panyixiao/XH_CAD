// Created By Yixiao 2023-3-15

#ifndef _JSONOPERATOR_
#define _JSONOPERATOR_

#include <Base/Placement.h>
#include <QJsonValue>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QFile>
#include <QDir>
#include <QDateTime>


namespace Robot
{
class ToolObject;
class FileOperator{
public:
    bool openFile(const std::string& t_FilePath, const QIODevice::OpenMode t_Mode = QIODevice::ReadWrite | QIODevice::Truncate);
    void closeFile();
    const std::string readStringPropFromFile(const QString& t_KeyVal);
    double readNumberPropFromFile(const QString& t_KeyVal);
    const Base::Placement readPosePropFromFile(const QString& t_KeyVal);
    bool insertItem(const QString keyName, const QString value);
    bool insertItem(const QString keyName, const QJsonObject& t_obj);
    bool saveFile();
    bool restoreToolObjectFromFile(const std::string& t_FilePath, Robot::ToolObject* t_ToolPtr);
    static QJsonObject poseToJsonObject(const Base::Placement& t_Pose);
protected:

private:
    QFile* m_FilePtr = nullptr;
//    QJsonArray m_JsonDataBuffer;
    QJsonObject m_JsonDataBuffer;
};
}



#endif

// Created By Yixiao 2022-04-05

#ifndef ROBOT_TASKOBJECT_H
#define ROBOT_TASKOBJECT_H

#include <App/Document.h>
#include <App/GeoFeature.h>
#include <App/PropertyFile.h>
#include <App/PropertyGeo.h>
#include "Mod/Robot/App/PreCompiled.h"
#include "Mod/Robot/App/Trac/RobotProgram.h"
#include "ActionObject.h"
#include <Mod/Robot/App/Utilites/Export_Utility.h>
#include <Mod/Robot/App/Utilites/FileIO_Utility.h>

namespace Robot
{

class RobotExport TaskObject : public App::GeoFeature
{
    PROPERTY_HEADER(Robot::TaskObject);

public:
    /// Constructor
    TaskObject(void);
    virtual ~TaskObject();
    virtual void onDocumentRestored();
    void insertAction(App::DocumentObject* n_Action);
    void removeAction(const size_t action_ID);
    const std::vector<DocumentObject*> getActionList();
    virtual App::DocumentObjectExecReturn * recompute();
    /// returns the type name of the ViewProvider
    virtual const char* getViewProviderName(void) const {
        return "RobotGui::ViewProviderTaskObject";
    }
    virtual App::DocumentObjectExecReturn *execute(void) {
        return App::DocumentObject::StdReturn;
    }
    virtual short mustExecute(void) const;
    virtual PyObject *getPyObject(void);
    void moveActionUp(const std::string& act_Name);
    void moveActionDown(const std::string& act_Name);

    const RobotProg_sptr getTaskProgramPtr() const{
        return m_ProgramPtr;
    }
    size_t getTaskTracPointNumber() const;

    bool exportTaskProgram(const std::string filePath, bool splitProgram = true);
    void onDelete();

public:
    App::PropertyStringList ActionList;
    App::PropertyPlacement TaskOrigin;
    App::PropertyBool refreshTracData;
protected:
    /// get called by the container when a property has changed
    virtual void onChanged (const App::Property* prop);
    bool updateTracData();

protected:
    RobotProg_sptr m_ProgramPtr;
    FileIO_Utility m_Export;
};

} //namespace Robot


#endif // ROBOT_ROBOTOBJECT_H

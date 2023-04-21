
#ifndef ROBOT_ACTIONOBJECT_H
#define ROBOT_ACTIONOBJECT_H

#include <Mod/Robot/App/PreCompiled.h>
#include <App/GeoFeature.h>
#include <App/PropertyFile.h>
#include <App/PropertyGeo.h>
#include "Action.h"

namespace Robot
{

class RobotExport ActionObject : public App::DocumentObject
{
    PROPERTY_HEADER(Robot::ActionObject);

public:
    /// Constructor
    ActionObject(void);
    virtual ~ActionObject();

    virtual App::DocumentObjectExecReturn *execute(void) {
        return App::DocumentObject::StdReturn;
    }

    virtual short mustExecute(void) const;
    virtual PyObject *getPyObject(void);

protected:
    /// get called by the container when a property has changed
    virtual void onChanged (const App::Property* prop);

private:
    Action     t_Action;
};

} //namespace Robot


#endif // ROBOT_ROBOTOBJECT_H

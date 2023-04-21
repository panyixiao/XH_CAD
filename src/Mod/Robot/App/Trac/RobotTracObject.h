//Created By Yixiao 2022-6-18

#ifndef ROBOT_TRAJECTORYOBJECT_H
#define ROBOT_TRAJECTORYOBJECT_H

#include <App/Document.h>
#include <App/GeoFeature.h>
#include <App/PropertyFile.h>
#include <App/PropertyGeo.h>

#include "RobotProgram.h"
#include "RobotWaypoint.h"
#include "MoveCommand.h"
#include "CoordCommand.h"


using RobotProg_sptr = std::shared_ptr<Robot::RobotProgram>;
using RobotWaypoint_sptr = std::shared_ptr<Robot::RobotWaypoint>;

namespace Robot
{

enum class TracType{
    FREETRAC = 0,
    SCANTRAC,
    SEAMTRAC
};

class RobotExport RobotTracObject : public App::GeoFeature
{
    PROPERTY_HEADER(Robot::RobotTracObject);

public:
    /// Constructor
    RobotTracObject(void);
    virtual ~RobotTracObject();

    /// returns the type name of the ViewProvider
    virtual const char* getViewProviderName(void) const {
        return "RobotGui::ViewProviderRobotTrajectory";
    }
    virtual App::DocumentObjectExecReturn *execute(void) {
        return App::DocumentObject::StdReturn;
    }

    virtual short mustExecute(void) const;
    virtual PyObject *getPyObject(void);
    // Operator
    bool setOperator(const std::string& t_Name);
    const std::string& getOperatorName() const{
        return ExecutorName.getStrValue();
    }

    bool isEmpty();

    //Trac
    void setTracManager(const std::string& t_Name);
    App::DocumentObject* getTracManager() const;
    // Type: Freerun / EdgeBased
    void setTracType(const TracType& t_Type);
    const TracType& getTracType() const;

    // Program
    void insertCMD_MOVE(const std::string executorName,
                        const Robot::RobotWaypoint& t_Pnt,
                        const MoveType t_Type = Robot::MoveType::MOVL,
                        const MovePrec t_Prec = Robot::MovePrec::FINE,
                        const float Vel = 100.0,
                        const float _BL = 5.0,
                        const float _VBL = 0.0);
    void insertCMD_SwitchTool(const std::string executorName,
                              const ToolType &t_Type);
    void insertCMD_SetToolActivate(const std::string executorName,
                                   const ToolType &t_Type);
    void insertCMD_SetToolDeActivate(const std::string executorName,
                                     const ToolType &t_Type);
    void insertCMD_ChangeCord(const std::string executorName,
                              const CordType& t_Type, const uint coordID = 0);

    void setWaypointPose(const int pntID,  const uint rbtID, const Base::Placement& t_pose);
    void setWaypointAdjust(const int pntID, const uint rbtID, const Base::Placement& t_adjust);
    void setAdjustPoseToRest(const int s_PntID,  const uint rbtID, const Base::Placement &t_adjust);
    bool removeTargetCommand(const int cmdID);

    virtual void Save (Base::Writer &writer) const;
    virtual void Restore(Base::XMLReader &reader);
    virtual void onDocumentRestored();

    const RobotProg_sptr getRobotProgramSptr(){
        return m_ProgramPtr;
    }

    void setRobotProgramPtr(const RobotProg_sptr t_ProgramPtr){
        m_ProgramPtr = t_ProgramPtr;
    }
    const std::vector<RobotWaypoint_sptr>& getWaypointData() const{
        return m_ProgramPtr->getWaypointData();
    }

    App::PropertyPlacement TracOrigin;
    App::PropertyInteger   WaypointNumber;
    App::PropertyString    ExecutorName;
    App::PropertyString    TracManagerName;
    App::PropertyInteger   TracTypeID;

protected:
    /// get called by the container when a property has changed
    virtual void onChanged (const App::Property* prop);
    void resetData();

protected:
    RobotProg_sptr   m_ProgramPtr = std::make_shared<Robot::RobotProgram>();
    TracType         m_TYPE = TracType::FREETRAC;
};

} //namespace Robot


#endif // ROBOT_ROBOTOBJECT_H

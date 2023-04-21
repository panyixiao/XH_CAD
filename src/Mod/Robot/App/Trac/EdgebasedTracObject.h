/***************************************************************************
 *   Copyright (c) 2010 Jürgen Riegel (juergen.riegel@web.de)              *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/


#ifndef ROBOT_EDGEBASEDTRACObject_H
#define ROBOT_EDGEBASEDTRACObject_H

#include <App/GeoFeature.h>
#include <App/PropertyFile.h>
#include <App/PropertyGeo.h>
#include "RobotTracObject.h"

namespace Robot
{

struct ScanTracProperty{
    float scanSpeed;

};

struct SeamTracProperty{
    float weldspeed;
};

class RobotExport EdgebasedTracObject : public RobotTracObject
{
    PROPERTY_HEADER(Robot::RobotTracObject);

public:
    /// Constructor
    EdgebasedTracObject(void);
    virtual ~EdgebasedTracObject();

    bool updateReferences();
    void setTracSafePoint(const GroupPose& t_Pose);
    bool generateConstraint();
    bool generateProgram();
    bool regenerateProgram();

    void setEdgeTracType(const TracType &t_Type);
    void cutEdgeAtPoint(const uint position, bool abandonRest);


    App::PropertyString          LinkedObject;
    App::PropertyFloat           InterpoDist;
    App::PropertyLinkSub         EdgeSource;
    App::PropertyLinkSub         FaceSource;    // TODO: This limits selected reference faces must belong to 1 object;
    App::PropertyBool            UseRotation;

    // Trac Commmon Property
    App::PropertyBool            Reverse_EdgeDir;
    App::PropertyBool            Reverse_RefFace_1;
    App::PropertyBool            Reverse_RefFace_2;
    App::PropertyFloatConstraint Accelerate;
    App::PropertyFloatConstraint Angle_Roll;
    App::PropertyFloatConstraint Angle_Pitch;
    App::PropertyFloatConstraint Angle_Yaw;
    App::PropertyFloat           Offset_Z;
    App::PropertyFloat           Offset_W;
    App::PropertyFloat           Offset_B;
    App::PropertyFloatList       CurrentExternalAxis;
    // Seam Trac Property
    App::PropertyInteger         TorchToolID;
    App::PropertyFloat           WeldSpeed;
    App::PropertyFloat           ApproachSpeed;
    // Scan Trac Property
    App::PropertyInteger         ScanToolID;
    App::PropertyFloat           ScanSpeed;

    /// set by execute with the number of clusters found
    int NbrOfCluster;
    /// set by execute with the number of all edges
    int NbrOfEdges;

    /// returns the type name of the ViewProvider
    virtual const char* getViewProviderName(void) const {
        return "RobotGui::ViewProviderRobotTrajectory";
    }
    virtual App::DocumentObjectExecReturn *execute(void);

protected:
    /// get called by the container when a property has changed
    virtual void onChanged (const App::Property* prop);
    std::vector<Base::Placement> generateEdgePoses(const std::vector<TopoDS_Face> faceRefs,
                                                   const std::vector<TopoDS_Edge> edgeRefs,
                                                   const float interpoLen);
private:
    TracType m_TracType;
    GroupPose m_TracSafePoint;
    std::vector<Base::Placement> edge_Poses;
    std::vector<TopoDS_Face> edge_FaceRef;
    std::vector<TopoDS_Edge> edge_EdgeRef;

    std::vector<Base::Placement> edge2_Poses;
    std::vector<TopoDS_Face> edge2_FaceRef;
    std::vector<TopoDS_Edge> edge2_EdgeRef;
    std::vector<double> c_ExtJointVal;

};

} //namespace Robot


#endif // ROBOT_ROBOTOBJECT_H

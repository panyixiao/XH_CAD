/***************************************************************************
 *   Copyright (c) 2008 Jürgen Riegel (juergen.riegel@web.de)              *
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

#ifndef ROBOT_VIEWPROVIDERROBOT6AXISOBJECT_H
#define ROBOT_VIEWPROVIDERROBOT6AXISOBJECT_H

#include <Inventor/VRMLnodes/SoVRMLTransform.h>
#include <Gui/ViewProviderGeometryObject.h>
#include <Gui/SoFCSelection.h>
#include <Gui/SoFCCSysDragger.h>
#include <Base/Placement.h>
#include <Mod/Robot/App/Utilites/MeshUtility.h>
#include "Mod/Robot/Gui/Utilites/DraggerUtility.h"

namespace RobotGui
{

typedef std::function<void()>       RobotVP_voidCallBack;
typedef std::function<void(bool)>   RobotVP_boolCallBack;

class RobotGuiExport ViewProviderRobot6AxisObject : public Gui::ViewProviderGeometryObject
{
    PROPERTY_HEADER(RobotGui::ViewProviderRobot6AxisObject);

public:
    /// constructor.
    ViewProviderRobot6AxisObject();

    /// destructor.
    ~ViewProviderRobot6AxisObject();

    App::PropertyInteger CurrentInteractiveCoord;

    void attach(App::DocumentObject *pcObject);
    void setDisplayMode(const char* ModeName);
    std::vector<std::string> getDisplayModes() const;

    void updateData(const App::Property*);
    virtual void onChanged(const App::Property* prop);

    /// for simulation without changing the document:
    bool setTipPosition(const Base::Placement& new_Pose);
    void setTipPoseByTeach(const Base::Placement& new_Pose);

    virtual bool doubleClicked();
    bool setEdit(int ModNum);
    void unsetEdit(int ModeNum);    
    virtual std::vector<App::DocumentObject *> claimChildren(void) const override;


protected:
    void DraggerMotionCallback(InteractiveDragger *t_dragger);
    void DraggerFinishCallback(InteractiveDragger *t_dragger);
    bool generateLinkMeshNodes(const char *FileName);
    bool generateLinkMeshNodes_fromURDF(const char *urdf_FilePath);
    bool updatelinkmeshPoses(const Base::Placement &basePose);
    bool callbackRegistered();

protected:
    // ROBOT OBJECT
    Gui::SoFCSelection    *  pcRobotRoot = nullptr;
    SoTransform           *  pcTcpTransform = nullptr;
    // ROBOT LINKS
    float                    mesh_sf = 1000;
    SoGroup               *  m_LinkMeshGroup = nullptr;
    std::vector<std::string> m_LinkNames;

    // ROBOT TIPS
    SoGroup               * m_TcpRoot = nullptr;
    InteractiveDragger    * pcDragger = nullptr;

    // view provider of the toolshape if set
    App::DocumentObject   * toolShapeObj = nullptr;
    Gui::ViewProvider     * toolShapeVP = nullptr;

    RobotVP_voidCallBack    callBack_UpdatePanelWidgets = nullptr;
};

} //namespace RobotGui


#endif // ROBOT_VIEWPROVIDERROBOTOBJECT_H

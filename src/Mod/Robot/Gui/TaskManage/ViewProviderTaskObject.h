// Created By Yixiao 2022-05-10


#ifndef ROBOT_VIEWPROVIDERTASKOBJECT_H
#define ROBOT_VIEWPROVIDERTASKOBJECT_H

#include <Mod/Robot/Gui/Trac/ViewProviderRobotTrajectory.h>
#include <Gui/ViewProviderGeometryObject.h>
#include <Gui/SoFCSelection.h>


namespace RobotGui
{

class ViewProviderTaskObject : public Gui::ViewProviderGeometryObject
{
    PROPERTY_HEADER(RobotGui::ViewProviderTaskObject);

public:
    /// constructor.
    ViewProviderTaskObject();

    /// destructor.
    ~ViewProviderTaskObject();

    void attach(App::DocumentObject *pcObject);
    void setDisplayMode(const char* ModeName);
    std::vector<std::string> getDisplayModes() const;
    void updateData(const App::Property*);
    virtual bool doubleClicked();
    virtual std::vector<App::DocumentObject *> claimChildren(void) const override;
    bool setEdit(int ModNum);
    void unsetEdit();
    bool onDelete(const std::vector<std::string> &subNames);

protected:

};

} //namespace RobotGui


#endif // ROBOT_VIEWPROVIDERROBOTOBJECT_H

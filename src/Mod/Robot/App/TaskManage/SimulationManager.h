// Created by Yixiao 2022/05/10
#ifndef RD_SIMULATIONMANAGER_H
#define RD_SIMULATIONMANAGER_H

#include "Action.h"
#include <App/DocumentObject.h>
#include <Base/Persistence.h>

// Robot Trac Simulation
namespace Robot {

static const float SimulationFrequence = 1000.0;

enum SimulationType { Action, Moment, WorkFlow, RoboTrac };

class SimulationManager : public App::DocumentObject {

public:
  SimulationManager();

  //  resetSimulationStatus();
  void changeSimStep(double new_simualtionStepLen);
  void changeSimObjectVisiblity(bool visible);
  bool calculateSimulationTime();

  double getSimulationTotalTime();
  double getSimulationStpLen() { return m_SimulationTimeStep; }

  void updateSimulationTargetStatus(const float timerPosition);
  virtual void Save(Base::Writer &writer) const;
  virtual void Restore(Base::XMLReader &reader);

protected:
  bool executeActionAtTime(const float t_Position);
  //  bool executeTrajectoryAtTime(const float t_Position);
  bool executeMomentAtTime(const float t_Position);
  bool executeWorkFlowAtTime(const float t_Position);

private:
  float m_SimulationTotalTime = 0;
  float m_SimulationTimeStep = 1 / SimulationFrequence;

  SimulationType m_simType;
};
}

#endif // RD_SIMULATIONMANAGER_H

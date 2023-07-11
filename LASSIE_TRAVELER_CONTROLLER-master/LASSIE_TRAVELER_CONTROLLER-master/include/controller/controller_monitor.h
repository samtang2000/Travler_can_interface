/**
 *   Copyright [2021] <Shipeng Liu>
 *    e-mail: 1196075299@qq.com
 */

#ifndef CONTROLLER_MONITOR_H_
#define CONTROLLER_MONITOR_H_

#include <iostream>
#include <stdint.h>
#include <vector>
#include <string>
#include "controller/basecontroller.h"
#include "controller/pid_speed_controller.h"
#include "proxy/control_data.h"

namespace traveler_namespace{
namespace control{

enum ControllerID {
    PID_CONTROLLER = 0
};

class ControllerMonitor{
  public:
    static ControllerMonitor& GetStateMonitor() {
      static ControllerMonitor singleton;
      return singleton;
    }
    void Init();
    void ControlCommand(Traveler &);
  private:
    BaseController* pid_controller_ptr_;

  
};


} //namespace control
} //namespace traveler_namespace


#endif
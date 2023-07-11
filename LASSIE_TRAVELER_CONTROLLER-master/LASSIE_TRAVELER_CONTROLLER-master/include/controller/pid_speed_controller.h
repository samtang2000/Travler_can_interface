/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-27 15:44:32 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 17:54:19
 */

#ifndef PID_SPEED_CONTROLLER_H_
#define PID_SPEED_CONTROLLER_H_

#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include "controller/basecontroller.h"
#include "proxy/control_data.h"
#include "rclcpp/rclcpp.hpp"

namespace traveler_namespace{
namespace control{
class PID_speed_controller : public BaseController
{

public:
    PID_speed_controller() = default;

    virtual ~PID_speed_controller() = default;

    void Init() override;

    void ComputeControlCommand(Traveler &) override;

    void Reset() override;

    void Stop() override;

private:


};
} // namespace control
} // namespace traveler_namespace

#endif
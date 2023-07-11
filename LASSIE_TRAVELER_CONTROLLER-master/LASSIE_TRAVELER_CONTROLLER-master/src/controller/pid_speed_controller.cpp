/*
 * @Author: Ryoma Liu -- ROBOLAND
 * @Date: 2021-11-27 15:47:05
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 19:15:40
 */

#include "controller/pid_speed_controller.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
const double PI = 3.141592653589793238463;

using namespace std;

namespace traveler_namespace
{
    namespace control
    {
        void PID_speed_controller::Init()
        {
        }

        void PID_speed_controller::ComputeControlCommand(Traveler &)
        {
            // generate_temp_traj(traveler_);
        }

        void PID_speed_controller::Reset() {}

        void PID_speed_controller::Stop()
        {
        }

    } // namespace control
} // namespace traveler_namespace

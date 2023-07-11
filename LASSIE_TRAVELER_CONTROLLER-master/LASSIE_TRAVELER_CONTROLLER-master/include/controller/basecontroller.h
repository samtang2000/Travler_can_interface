/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-27 15:42:07 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2021-11-27 15:43:09
 */

#ifndef BASE_CONTROLLER_H_
#define BASE_CONTROLLER_H_
#include "proxy/control_data.h"

namespace traveler_namespace {
namespace control {
class BaseController {
 public:
  BaseController() = default;

  virtual ~BaseController() = default;

  virtual void Init() = 0;

  virtual void ComputeControlCommand(Traveler &) = 0;

  virtual void Reset() = 0;

  virtual void Stop() = 0;
};
}  // namespace control
}  // namespace traveler_namespace
#endif

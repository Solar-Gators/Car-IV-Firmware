/*
 * PID.hpp
 *
 *  Created on: Jun 28, 2022
 *      Author: John Carr
 */

#ifndef SOLARGATORSBSP_STM_DRIVERS_INC_PID_HPP_
#define SOLARGATORSBSP_STM_DRIVERS_INC_PID_HPP_

namespace SolarGators {
namespace Drivers {

struct PID {
  PID(float Kp, float Ki, float Kd, float T);
  ~PID();
  void SetOutLimits(float limMin, float limMax);
  void SetIntegLimits(float limMin, float limMax);
  void SetFilter(float tau);
  float Update(float setpoint, float measurement);
  float GetPrevOutput();

private:
  /* Controller gains */
  const float Kp_;
  const float Ki_;
  const float Kd_;

  /* Derivative low-pass filter time constant */
  float tau_;

  /* Output limits */
  float limMin_;
  float limMax_;

  /* Integrator limits */
  float limMinInt_;
  float limMaxInt_;

  /* Sample time (in seconds) */
  const float T;

  /* Controller "memory" */
  float integrator_;
  float prevError_;          /* Required for integrator */
  float differentiator_;
  float prevMeasurement_;    /* Required for differentiator */

  /* Controller output */
  float out_;

};

} /* namespace Drivers */
} /* namespace SolarGators */

#endif /* SOLARGATORSBSP_STM_DRIVERS_INC_PID_HPP_ */

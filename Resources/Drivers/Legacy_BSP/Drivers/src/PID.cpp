/*
 * PID.cpp
 *
 *  Created on: Jun 28, 2022
 *      Author: John Carr
 */

#include <PID.hpp>

namespace SolarGators {
namespace Drivers {

PID::PID(float Kp, float Ki, float Kd, float T):
    Kp_(Kp),Ki_(Ki),Kd_(Kd),T(T / 1000.0)
{

  /* Clear controller variables */
  integrator_ = 0.0f;
  prevError_  = 0.0f;

  differentiator_  = 0.0f;
  prevMeasurement_ = 0.0f;

  out_ = 0.0f;
}

PID::~PID()
{ }

void PID::SetOutLimits(float limMin, float limMax)
{
  limMin_ = limMin;
  limMax_ = limMax;
}
void PID::SetIntegLimits(float limMin, float limMax)
{
  limMinInt_ = limMin;
  limMaxInt_ = limMax;
}
// tau is in milliseconds then updated to seconds
void PID::SetFilter(float tau)
{
  tau_ = tau;
}

float PID::Update(float setpoint, float measurement)
{
  /*
  * Error signal
  */
  float error = setpoint - measurement;

  /*
  * Proportional
  */
  float proportional = Kp_ * error;

  /*
  * Integral
  */
  integrator_ = integrator_ + 0.5f * Ki_ * T * (error + prevError_);

  /* Anti-wind-up via integrator clamping */
  if (integrator_ > limMaxInt_) {

      integrator_ = limMaxInt_;

  } else if (integrator_ < limMinInt_) {

      integrator_ = limMinInt_;

  }

  /*
  * Derivative (band-limited differentiator)
  */

  differentiator_ = -(2.0f * Kd_ * (measurement - prevMeasurement_) /* Note: derivative on measurement, therefore minus sign in front of equation! */
                      + (2.0f * tau_ - T) * differentiator_)
                      / (2.0f * tau_ + T);

  /*
  * Compute output and apply limits
  */
  out_ = proportional + integrator_ + differentiator_;

  if (out_ > limMax_) {

      out_ = limMax_;

  } else if (out_ < limMin_) {

      out_ = limMin_;

  }

  /* Store error and measurement for later use */
  prevError_       = error;
  prevMeasurement_ = measurement;

  /* Return controller output */
  return out_;
}

float PID::GetPrevOutput()
{
  return out_;
}

} /* namespace Drivers */
} /* namespace SolarGators */

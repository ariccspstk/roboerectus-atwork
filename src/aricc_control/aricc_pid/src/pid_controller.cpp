#include "aricc_pid/pid_controller.h"
#include <cstdio>
#include <boost/math/special_functions/fpclassify.hpp>

namespace aricc_control{
  PidController::PidController(double P, double I, double D, double I1, double I2):p_gain_(P), i_gain_(I), d_gain_(D), i_max_(I1), i_min_(I2){
    p_error_last_ = 0.0;
    p_error_ = 0.0;
    d_error_ = 0.0;
    i_error_ = 0.0;
    cmd_ = 0.0;
    last_i_error = 0.0;
  }

  PidController::~PidController(){
  }

  void PidController::initPid(double P, double I, double D, 
    double I1, double I2){
    p_gain_ = P;
    i_gain_ = I;
    d_gain_ = D;
    i_max_ = I1;
    i_min_ = I2;
    reset();
  }

  void PidController::reset(){
    p_error_last_ = 0.0;
    p_error_ = 0.0;
    d_error_ = 0.0;
    i_error_ = 0.0;
    cmd_ = 0.0;
  }

  void PidController::getGains(double &p, double &i, double &d, 
    double &i_max, double &i_min){
    p = p_gain_;
    i = i_gain_;
    d = d_gain_;
    i_max = i_max_;
    i_min = i_min_;
  }

  void PidController::setGains(double P, double I, double D, 
    double I1, double I2){
    p_gain_ = P;
    i_gain_ = I;
    d_gain_ = D;
    i_max_ = I1;
    i_min_ = I2;
  }

  double PidController::updatePid(double error, 
    boost::posix_time::time_duration dt){
    double p_term, d_term, i_term;
    p_error_ = error; //this is pError = pState-pTarget
    double deltatime = (double)dt.total_microseconds()/1000.0; //in ms
    if( deltatime == 0.0 || 
        boost::math::isnan(error) || 
        boost::math::isinf(error))
      return 0.0;

    // Calculate proportional contribution to command
    p_term = p_gain_ * p_error_;
    // Calculate the integral error
    i_error_ = last_i_error + deltatime * p_error_;
    last_i_error = deltatime * p_error_;
    //Calculate integral contribution to command
    i_term = i_gain_ * i_error_;
    // Limit i_term so that the limit is meaningful in the output
    if (i_term > i_max_){
      i_term = i_max_;
      i_error_=i_term/i_gain_;
    }
    else if (i_term < i_min_){
      i_term = i_min_;
      i_error_=i_term/i_gain_;
    }  
    // Calculate the derivative error
    if (deltatime != 0){
      d_error_ = (p_error_ - p_error_last_) / deltatime;
      p_error_last_ = p_error_;
    }
    // Calculate derivative contribution to command
    d_term = d_gain_ * d_error_;
    cmd_ = -p_term - i_term - d_term;
  
   // printf(" p_error_ %lf  i_error_ %lf  p_term %lf i_term %lf  dt %lf out %lf\n", p_error_, i_error_, p_term, i_term, deltatime, cmd_);

    return cmd_;
  }

  double PidController::updatePid(double error, double error_dot, 
    boost::posix_time::time_duration dt){
    double p_term, d_term, i_term;
    p_error_ = error; //this is pError = pState-pTarget
    d_error_ = error_dot;
    double deltatime = (double)dt.total_microseconds()/1000.0;  //in milli seconds

    if( deltatime == 0.0 || 
        boost::math::isnan(error) || boost::math::isinf(error) || 
        boost::math::isnan(error_dot) || boost::math::isinf(error_dot))
      return 0.0;
    // Calculate proportional contribution to command
    p_term = p_gain_ * p_error_;
    // Calculate the integral error
    i_error_ = last_i_error + deltatime * p_error_;
    last_i_error = deltatime * p_error_;
   // i_error_ = i_error_ + deltatime * p_error_;
   //   printf("i_error_ %lf dt.fractional_seconds() %lf\n", i_error_, deltatime);

    //Calculate integral contribution to command
    i_term = i_gain_ * i_error_;
    // Limit i_term so that the limit is meaningful in the output
    if (i_term > i_max_){
      i_term = i_max_;
      i_error_=i_term/i_gain_;
    }
    else if (i_term < i_min_){
      i_term = i_min_;
      i_error_=i_term/i_gain_;
    }
    // Calculate derivative contribution to command
    d_term = d_gain_ * d_error_;
    cmd_ = -p_term - i_term - d_term;
    return cmd_;
  }

  void PidController::setCurrentCmd(double cmd){
    cmd_ = cmd;
  }

  double PidController::getCurrentCmd(){
    return cmd_;
  }

  void PidController::getCurrentPIDErrors(double& pe, double& ie, double& de){
    pe = p_error_;
    ie = i_error_;
    de = d_error_;
  }
}

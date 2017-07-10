#ifndef ARICC_CONTROL_PID_CONTROLLER_H
#define ARICC_CONTROL_PID_CONTROLLER_H

#include <string>
#include <boost/thread.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#define SLEEP_MILLISEC(millisec) boost::this_thread::sleep(boost::posix_time::milliseconds((millisec)));
#define SLEEP_MICROSEC(microsec) boost::this_thread::sleep(boost::posix_time::microseconds((microsec)));
#define SLEEP_SEC(sec) boost::this_thread::sleep(boost::posix_time::seconds((sec)));

/***************************************************/
/*! \class PidController
    \brief A basic pid class.

    This class implements a generic structure that
    can be used to create a wide range of pid
    controllers. It can function independently or
    be subclassed to provide more specific controls
    based on a particular control loop.

    In particular, this class implements the standard
    pid equation:

    \f$command  = -p_{term} - i_{term} - d_{term} \f$

    where: <br>
    <UL TYPE="none">
    <LI>  \f$ p_{term}  = p_{gain} * p_{error} \f$
    <LI>  \f$ i_{term}  = i_{gain} * i_{error} \f$
    <LI>  \f$ d_{term}  = d_{gain} * d_{error} \f$
    <LI>  \f$ i_{error} = i_{error} + p_{error} * dt \f$
    <LI>  \f$ d_{error} = (p_{error} - p_{error last}) / dt \f$
    </UL>

    given:<br>
    <UL TYPE="none">
    <LI>  \f$ p_{error}  = p_{state} - p_{target} \f$.
    </UL>
    @section Usage

    To use the Pid class, you should first call some version of init()
    (in non-realtime) and then call updatePid() at every update step.
    For example:

\verbatim
control_toolbox::Pid pid;
pid.initPid(6.0, 1.0, 2.0, 0.3, -0.3);
double position_desi_ = 0.5;
...
ros::Time last_time = ros::Time::now();
while (true) {
  ros::Time time = ros::Time::now();
  double effort = pid.updatePid(currentPosition() - position_desi_, time - last_time);
  last_time = time;
}
\endverbatim

*/
/***************************************************/
namespace aricc_control{

  class PidController{
  public:

  /*!
   * \brief Constructor, zeros out Pid values when created and
   * initialize Pid-gains and integral term limits:[iMax:iMin]-[I1:I2].
   *
   * \param P  The proportional gain.
   * \param I  The integral gain.
   * \param D  The derivative gain.
   * \param I1 The integral upper limit.
   * \param I2 The integral lower limit.
   */
  PidController(double P = 0.0, double I = 0.0, double D = 0.0, double I1 = 0.0, double I2 = -0.0);

  /*!
   * \brief Destructor of Pid class.
   */
  ~PidController();

  /*!
   * \brief Update the Pid loop with nonuniform time step size.
   *
   * \param p_error  Error since last call (p_state-p_target)
   * \param dt Change in time since last call
   */
  double updatePid(double p_error, boost::posix_time::time_duration dt);

  /*!
   * \brief Initialize PID-gains and integral term limits:[iMax:iMin]-[I1:I2]
   *
   * \param P  The proportional gain.
   * \param I  The integral gain.
   * \param D  The derivative gain.
   * \param I1 The integral upper limit.
   * \param I2 The integral lower limit.
   */
  void initPid(double P, double I, double D, double I1, double I2);
  
  /*!
   * \brief Reset the state of this PID controller
   */
  void reset();

  /*!
   * \brief Set current command for this PID controller
   */
  void setCurrentCmd(double cmd);

  /*!
   * \brief Return current command for this PID controller
   */
  double getCurrentCmd();

  /*!
   * \brief Return PID error terms for the controller.
   * \param pe  The proportional error.
   * \param ie  The integral error.
   * \param de  The derivative error.
   */
  void getCurrentPIDErrors(double& pe, double& ie, double& de);

  /*!
   * \brief Set PID gains for the controller.
   * \param P  The proportional gain.
   * \param I  The integral gain.
   * \param D  The derivative gain.
   * \param i_max
   * \param i_min
   */
  void setGains(double P, double I, double D, double i_max, double i_min);

  /*!
   * \brief Get PID gains for the controller.
   * \param p  The proportional gain.
   * \param i  The integral gain.
   * \param d  The derivative gain.
   * \param i_max The max integral windup.
   * \param i_mim The min integral windup.
   */
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

  /*!
   * \brief Update the Pid loop with nonuniform time step size. This update call 
   * allows the user to pass in a precomputed derivative error. 
   *
   * \param error  Error since last call (p_state-p_target)
   * \param error_dot d(Error)/dt since last call
   * \param dt Change in time since last call
   */
  double updatePid(double error, double error_dot, boost::posix_time::time_duration dt);

  PidController &operator =(const PidController& p){
    if (this == &p)
      return *this;

    p_gain_ = p.p_gain_;
    i_gain_ = p.i_gain_;
    d_gain_ = p.d_gain_;
    i_max_ = p.i_max_;
    i_min_ = p.i_min_;

    p_error_last_ = p_error_ = i_error_ = d_error_ = cmd_ = 0.0;
    return *this;
  }

  private:
    double p_error_last_; /**< _Save position state for derivative state calculation. */
    double p_error_; /**< Position error. */
    double d_error_; /**< Derivative error. */
    double i_error_; /**< Integator error. */
    double p_gain_;  /**< Proportional gain. */
    double i_gain_;  /**< Integral gain. */
    double d_gain_;  /**< Derivative gain. */
    double i_max_;   /**< Maximum allowable integral term. */
    double i_min_;   /**< Minimum allowable integral term. */
    double cmd_;     /**< Command to send. */
    double last_i_error;
  };
}
#endif

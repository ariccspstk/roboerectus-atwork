#ifndef PIDCONTROLLER_h
#define PIDCONTROLLER_h

#define AUTOMATIC  1
#define MANUAL	   0
#define DIRECT     0
#define REVERSE    1

class PIDController {
  private:
    float kp_;// * (P)roportional Tuning Parameter
    float ki_;// * (I)ntegral Tuning Parameter
    float kd_;// * (D)erivative Tuning Parameter
    int direction_;

    float input_;   
    float setpoint_;
    float last_input_;
    float item_;
    float out_min_;
    float out_max_;

    unsigned long last_time_;
    unsigned long sample_time_;
    
    bool inAuto_;
    
    /* Initialize()****************************************************************
     *	does all the things that need to happen to ensure a bumpless transfer
     *  from manual to automatic mode.
     ******************************************************************************/
    void Initialize(){
      last_input_ = input_;
      item_ = input_;
      if (item_ > out_max_) item_ = out_max_;
      else if (item_ < out_min_) item_ = out_min_;
    }


  public:

    /*Constructor (...)*********************************************************
     *    The parameters specified here are those for for which we can't set up
     *    reliable defaults, so we need to have the user set them.
     ***************************************************************************/
    PIDController(float Input, float Setpoint,
                  float Kp, float Ki, float Kd, int Direction) {              
      input_ = Input;
      setpoint_ = Setpoint;
      inAuto_ = false;

      SetOutputLimits(0, 255);	//default output limit corresponds to the arduino pwm limits

      sample_time_ = 100; //default Controller Sample Time is 0.1 seconds

      SetControllerDirection(Direction);
      SetTunings(Kp, Ki, Kd);
      last_time_ = millis() - sample_time_;
    }

    ~PIDController() {}


    /* Compute() **********************************************************************
     *     This, as they say, is where the magic happens.  this function should be called
     *   every time "void loop()" executes.  the function will decide for itself whether a new
     *   pid Output needs to be computed.  returns true when the output is computed,
     *   false when nothing has been done.
     **********************************************************************************/
    bool Compute(float& output ){
      if (!inAuto_) return false;
      unsigned long now = millis();
      unsigned long timeChange = (now - last_time_);
      
      if (timeChange >= sample_time_){
        /*Compute all the working error variables*/ 
        float error = setpoint_ - input_;
        item_ += (ki_ * error);
        if (item_ > out_max_) item_ = out_max_;
        else if (item_ < out_min_)  item_ = out_min_;
        float diff_input = (input_ - last_input_);

        /*Compute PID Output*/
        output = kp_ * error + item_ - kd_ * diff_input;

        if (output > out_max_) output = out_max_;
        else if (output < out_min_) output = out_min_;

        /*Remember some variables for next time*/
        last_input_ = input_;
        last_time_ = now;
        return true;
      }
      else return false;
    }


    /* SetTunings(...)*************************************************************
     * This function allows the controller's dynamic performance to be adjusted.
     * it's called automatically from the constructor, but tunings can also
     * be adjusted on the fly during normal operation
     ******************************************************************************/
    void SetTunings(float Kp, float Ki, float Kd){
      if (Kp < 0 || Ki < 0 || Kd < 0) return;
      float sample_time_InSec = ((float)sample_time_) / 1000;
      kp_ = Kp;
      ki_ = Ki * sample_time_InSec;
      kd_ = Kd / sample_time_InSec;

      if (direction_ == REVERSE){
        kp_ = (0 - kp_);
        ki_ = (0 - ki_);
        kd_ = (0 - kd_);
      }
    }

    /* Setsample_time_(...) *********************************************************
     * sets the period, in Milliseconds, at which the calculation is performed
     ******************************************************************************/
    void SetSampleTime_(int time){
      if (time > 0){
        float ratio  = (float)time/(float)sample_time_;
        ki_ *= ratio;
        kd_ /= ratio;
        sample_time_ = (unsigned long)time;
      }
    }

    /* SetOutputLimits(...)****************************************************
     *     This function will be used far more often than SetInputLimits.  while
     *  the input to the controller will generally be in the 0-1023 range (which is
     *  the default already,)  the output will be a little different.  maybe they'll
     *  be doing a time window and will need 0-8000 or something.  or maybe they'll
     *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
     *  here.
     **************************************************************************/
    void SetOutputLimits(float Min, float Max){
      if (Min >= Max) return;
      out_min_ = Min;
      out_max_ = Max;
    }

    /* SetMode(...)****************************************************************
     * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
     * when the transition from manual to auto occurs, the controller is
     * automatically initialized
     ******************************************************************************/
    void SetMode(int Mode){
      bool newAuto = (Mode == AUTOMATIC);
      if (newAuto == !inAuto_){ /*we just went from manual to auto*/
        Initialize();
      }
      inAuto_ = newAuto;
    }

    /* SetControllerDirection(...)*************************************************
     * The PID will either be connected to a DIRECT acting process (+Output leads
     * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
     * know which one, because otherwise we may increase the output when we should
     * be decreasing.  This is called from the constructor.
     ******************************************************************************/
    void SetControllerDirection(int Direction){
      if (inAuto_ && Direction != direction_)
      {
        kp_ = (0 - kp_);
        ki_ = (0 - ki_);
        kd_ = (0 - kd_);
      }
      direction_ = Direction;
    }

    /* Status Funcions*************************************************************
     * Just because you set the Kp=-1 doesn't mean it actually happened.  these
     * functions query the internal state of the PID.  they're here for display
     * purposes.  this are the functions the PID Front-end uses for example
     ******************************************************************************/
    float GetKp() {
      return  kp_;
    }
    float GetKi() {
      return  ki_;
    }
    float GetKd() {
      return  kd_;
    }
    int GetMode() {
      return  inAuto_ ? AUTOMATIC : MANUAL;
    }
    int GetDirection() {
      return direction_;
    }
};

#endif

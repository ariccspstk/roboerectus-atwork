#ifndef SIMPLEMEDIANFILTER_h
#define SIMPLEMEDIANFILTER_h
#include <inttypes.h>

template <typename T, uint16_t size_>
class SimpleMedianFilter {
  public:
    SimpleMedianFilter() :
      cur_pos_(0) {}

    ~SimpleMedianFilter() {}

    void push(T value) {
      uint16_t temp_pos = cur_pos_++ % size_;
      buf_[temp_pos] = value;
      if (cur_pos_ == size_) cur_pos_ = 0;
    }

    void initalize(T value) {
      buf_[size_] = {value};
      for (uint16_t i = 0; i < size_; ++i) buf_[i] = value;
      cur_pos_ = 0;
    }

    T pop() {
      T swap = 0;
      if (size_ <= 1) return buf_[0];
      for (uint16_t i = 0; i < size_ - 1; i++) {
        for (uint16_t j = i + 1; j < size_; j++) {
          if (buf_[i] > buf_[j]) {
            swap = buf_[i];
            buf_[i] = buf_[j];
            buf_[j] = swap;
          }
        }
      }

      //find the middle value of the sample
      T middle = (size_ + 1) / 2;
      T median = buf_[middle];
      return median;
    }

  private:
    T buf_[size_];
    uint16_t cur_pos_;
};

// Taken from http://en.wikipedia.org/wiki/Low-pass_filter
template <typename T>
class LowPassFilter {
  private:
    //Low pass filter params
    T input_; 
    T last_input_;
    float dt_;
    const float rc_;
    const float eps_;
    
    unsigned long last_time_;
    unsigned long sample_time_;
    
  public:
    LowPassFilter(float rc, float eps){
      sample_time_ = 100; //default Controller Sample Time is 0.1 seconds
      SetTunings(rc,eps);
      last_time_ = millis() - sample_time_;
    };
    ~LowPassFilter();
  
    T Compute(T input){
      input_ = input;
      float a = dt_ / (dt_ + rc_); //smoothing factor
      float res = last_input_ + (input_ - last_input_) * a;
      if ((res * res) <= eps_) res = 0;
      //Remember some variables for next time
      last_input_ = input_;
      return (T)res;
    }
    
    void SetTunings(float rc,float eps){
      rc_ = rc;
      eps_ = eps;
    }
};

#endif

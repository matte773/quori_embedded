// Implements a discrete IIR first order butterworth low pass filter with corner
// frequency fc and sampling frequency fs.  The digital filter is of the form:
//
//         b0 + b1*z^-1
// G(z) = ---------------
//          1 + a1*z^-1
//
// b0 = b1 = (1-alpha)/2
// a1 = -alpha
//
// alpha = (1-wac)/(1+wac) // filter constant
// wac = tan(wc/2);        // pre-warped analog design frequency
// wc = 2*pi*fc/fs;        // normalized frequency
//
// This filter achieves -Inf db gain and -90 deg phase at the Nyquist
// frequency and accurately places the -3 db corner frequency at fc with
// phase -45 deg. (The discrete single pole "alpha filter" has none of these
// properties.)
//
// Setting corner_hz > sample_hz or NaN puts the filter into a pass-through
// mode. Corner frequencies above 1/4 the sampling frequency achieve a bounded
// corner frequency of 1/4 the sampling frequency.

#ifndef LPFBUTTER1_HPP
#define LPFBUTTER1_HPP

//#include "arm_math.h"
//#include "angle_math.h"

class LpfButter1 {
  public:
    LpfButter1(float corner_hz, uint32_t sample_hz):
      y1_(0.0f),
      u1_(0.0f),
      corner_hz_(corner_hz),
      sample_hz_(sample_hz)
    {
      update_alpha();
    };

    virtual ~LpfButter1(){};

    void set_corner_hz(float hz) {
      if(hz != corner_hz_) {
        corner_hz_ = hz;
        update_alpha();
      }
    };
    
    void set_sample_hz(uint32_t hz) {
      if(hz != sample_hz_) {
        sample_hz_ = hz;
        update_alpha();
      }
    };

    float get_corner_hz() const {return corner_hz_;};

    uint32_t get_sample_hz() const {return sample_hz_;};

    virtual float sample(float u) {
      y1_ = neg_a1_*y1_ + b0_*u + b1_*u1_;
      u1_ = u;
      return y1_;
    }

    float value() const {return y1_;};

    void set_value(float val) {y1_ = val; u1_ = val;};

    float get_prev_sample() const {return u1_;}

  protected:
    float neg_a1_;
    float b0_;
    float b1_;
    float y1_;
    float u1_;

  private:
    float corner_hz_;
    uint32_t sample_hz_;

    void update_alpha() {
      // enforce upper bound on realized corner frequency as 1/4 sample_hz
      if(corner_hz_ <= 0.25f*sample_hz_) {
        float arg = PI*static_cast<float>(corner_hz_)/sample_hz_;
        float wac = sin(arg)/cos(arg); //replaced arm_cos_f32 with cos and arm_sin_f32 with sin
        float alpha   = (1-wac)/(1+wac);
        neg_a1_ = alpha;
        b0_     = 0.5*(1-alpha);
        b1_     = b0_;
      }
      else {
        neg_a1_ = 0;
        b0_     = 1;
        b1_     = 0;
      }
    };
};

#endif // LPFBUTTER1_HPP


/*
  Implements y = alpha*a +(1-alpha)*y as a low-pass filter. The state
  is y. Typically alpha is small, like 0.01.

*/
#include "OnePole.h"


void OnePoleF_init(OnePoleFState_t *s, float alpha, float alpha_mav, float threshmult){
  s->alpha = alpha;
  s->beta = 1 - alpha;
  s->alpha_mav = alpha_mav;
  s->beta_mav = 1.0 - alpha_mav;
  s->threshmult = threshmult;
  
  // 5 time constants should allow the filter to settle
  s->settleSamples = 5 * 1/alpha;
  if (s->settleSamples < ((512*2*3)/2) * 6) {
	  s->settleSamples = ((512*2*3)/2) * 6;
  }
  s->settleSamplesMAV = 5*1/alpha_mav + s->settleSamples;
  
  s->mav = 0;
  s->first = true;
}

bool OnePoleF_update(OnePoleFState_t *s, float sample){
  if( s->first){
    s->state = sample;
    s->first = false;
  }
  
  // Update the filter state
  s->state = s->alpha*sample + s->beta * s->state;

  // If we're not done settling the filter, return
  if(s->settleSamples > 0) {
    s->settleSamples --;
    return false;
  }  
  
  // Compute the residual (high pass)
  float res = sample - s->state;
  
  // at this point we have settled the filter, and should update the
  // MAV filter state.

  // This conditional is important for numerical stability, otherwise
  // the large value of the residual corrupts (and overflows)
  // TODO: add overflow reset

  // Update the average variance
  if(res <0) { //res = -res;
	  s->mav = s->alpha_mav * -res + s->beta_mav * s->mav;
  } else {
	  s->mav = s->alpha_mav * res + s->beta_mav * s->mav;
  }

  if(s->settleSamplesMAV > 0) {
    s->settleSamplesMAV --;
    return false;
  }
  
  // To return true we need several conditions to be true:
  // 1. we must have settled the filter
  // 2. we must have res > 0 (rising signal detection)
  // 3. we must res^2 > (estimated variance)*threshmult

  if(res > 0 && res > s->mav * s->threshmult){
    return true;
  }

  return false;
}

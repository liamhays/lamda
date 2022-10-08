#ifndef _INCLUDE_ONEPOLE
#define _INCLUDE_ONEPOLE

#include <stdbool.h>

typedef struct OnePoleFState {
  float alpha;
  float beta;
  float state;
  unsigned int settleSamples;

  float alpha_mav;
  float beta_mav;
  float mav; // moving average high-pass of signal's square residual
  unsigned int settleSamplesMAV;

  float threshmult;
  bool first;
} OnePoleFState_t;

void OnePoleF_init(OnePoleFState_t *s, float alpha, float alpha_mav, float threshmult);
bool OnePoleF_update(OnePoleFState_t *s, float sample);


#endif // _INCLUDE_ONEPOLE

#ifndef __MY_RAMP_H_
#define __MY_RAMP_H_

#include "ramp.h"

typedef struct {
	float Save_ramp_time;
	Ramp_Typedef *Ramp;
	uint8_t Ramp_State;
}Ramp_Global_Typedef;


void Sleep_Slope(Ramp_Global_Typedef *Ramp_Global);
void Awake_Slope(Ramp_Global_Typedef *Ramp_Global);
void Reset_Global_Slope(Ramp_Global_Typedef *Ramp_Global);
#define Ramp_Wake  0
#define Ramp_Sleep 1
#endif

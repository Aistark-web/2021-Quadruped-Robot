#include "my_ramp.h"


/**
	*@biref б�¾������
	*/
void Sleep_Slope(Ramp_Global_Typedef *Ramp_Global)
{
	if(Ramp_Global->Ramp_State == Ramp_Wake)
	{
		Ramp_Global->Save_ramp_time = Get_TimerTick() - Ramp_Global->Ramp->StartTick;
		Ramp_Global->Ramp_State = Ramp_Sleep;
	}
}

/**
	*@biref б�¾������
	*/
void Awake_Slope(Ramp_Global_Typedef *Ramp_Global)
{
	if(Ramp_Global->Ramp_State == Ramp_Sleep)
	{
		Ramp_Global->Ramp->StartTick = Get_TimerTick() - Ramp_Global->Save_ramp_time;
		Ramp_Global->Ramp_State = Ramp_Wake;
	}
}

void Reset_Global_Slope(Ramp_Global_Typedef *Ramp_Global)
{
	Ramp_Global->Ramp_State = Ramp_Wake;
}
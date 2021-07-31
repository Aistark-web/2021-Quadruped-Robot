#include "my_ramp.h"


/**
	*@biref Ð±ÆÂ¾ä±úÐÝÃß
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
	*@biref Ð±ÆÂ¾ä±ú»½ÐÑ
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
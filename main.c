/*----------------------------------------------------------------\
@ DCSP

Title            : gimbal_6_StabilizationLoop
Author           : Yejun_Lee
Created          : 10-06-2026
Modified         : 10-06-2026

----------------------------------------------------------------*/

#define _CRT_SECURE_NO_WARNINGS

#include "mygimbal_runexp.h"

void main(void)
{
	Program_Initialization();

	do
	{
		Import_Time();
		Import_Data();

		Digital_Controller();

		Export_Data();

		Store_Data_To_Buffer();

		Time_Management();

	} while (Check_Stop_Condition());

	Data_Recording();

	Program_Termination();
}
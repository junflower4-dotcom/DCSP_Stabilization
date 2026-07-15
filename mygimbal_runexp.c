/*----------------------------------------------------------------\
@ DCSP

Title            : mygimbal_runexp.c
Author           : Yejun_Lee
Created          : 10-06-2026
Modified         : 10-06-2026

Description      : Common experiment runner
----------------------------------------------------------------*/

#define _CRT_SECURE_NO_WARNINGS

#include "mygimbal_runexp.h"

/* ===============================
		Global Variable Definition
================================= */

// Task
static TaskHandle taskAI = 0;		// AI0 = Disturbance AI2 = Vgyro, AI3 = Vpoten
static TaskHandle taskAO = 0;		// AO0 = Switch, AO1 = Vcmd

static int count = 0;
static int emergency_stop = 0;

static double time_init = 0.0;
static double time_curr = 0.0;
static double time_elapsed = 0.0;
static double time_exp = 0.0;

static double Vcmd = 0.0;
static double Vcontroller_command = 0.0;
static double Vc = 0.0;

static double Vgyro_offset = 0.0;

static float64 Vdisturbance = 0.0;		// AI0
static float64 Vgyro = 0.0;				// AI2
static float64 Vpoten = 0.0;			// AI3

static double Omega_h = 0.0;
static double Psi_h = 0.0;

static double Omega_h_filter = 0.0;		// LPF filterd omega

static double Psi_init = 0.0;		// initial angle
static double Psi_cmd = 0.0;		// target angle
static double Psi_error = 0.0;		// angle error

static double Omega_cmd = 0.0;						// target Omega
static double Omega_error = 0.0;					// Omega error
static double Omega_error_prev = 0.0;				// Omega error prev
static double Omega_error_integral = 0.0;			// integral of Omega error
static double Omega_error_integral_update = 0.0;	// integrel update 

static int designation_first_loop = 1;
static int stabilization_first_loop = 1;

static int keyborad = 0;

// N_STEP_EXP: Input.h¿¡ Á¤ÀÇµÊ.
static double OutTime[N_STEP_EXP] = { 0.0, };

static double OutVcmd[N_STEP_EXP] = { 0.0, };
static double OutVc[N_STEP_EXP] = { 0.0, };

static double OutVdisturbance[N_STEP_EXP] = { 0.0, };

static double OutVgyro[N_STEP_EXP] = { 0.0, };
static double OutOmega_h[N_STEP_EXP] = { 0.0, };
static double OutOmega_h_filter[N_STEP_EXP] = { 0.0, };

static double OutVpoten[N_STEP_EXP] = { 0.0, };
static double OutPsi_h[N_STEP_EXP] = { 0.0, };

// AI
static float64 AI_Data[3] = { 0.0, 0.0, 0.0};
static int32 samplesRead = 0;

// AO
static float64 AO_Data[2] = { 0.0, 0.0 };
static int32 samplesWritten = 0;

/* ===============================
		Main Function Define
================================= */

void Read_AI(void)
{
	DAQmxReadAnalogF64(taskAI, 1, 10.0, DAQmx_Val_GroupByScanNumber, AI_Data, 3, &samplesRead, NULL);

	Vgyro		 = AI_Data[0];
	Vpoten		 = AI_Data[1];
	Vdisturbance = AI_Data[2];
}

void Write_AO(double Vswitch, double Vmotor)
{
	AO_Data[0] = Vswitch;
	AO_Data[1] = Vmotor;

	DAQmxWriteAnalogF64(taskAO, 1, 1, 5.0, DAQmx_Val_GroupByScanNumber, AO_Data, &samplesWritten, NULL);
}

void Program_Initialization(void)
{
	count = 0;
	emergency_stop = 0;

	time_init = 0.0;
	time_curr = 0.0;
	time_elapsed = 0.0;
	time_exp = 0.0;

	Vcmd = 0.0;
	Vcontroller_command = 0.0;
	Vc = STANDARD_VOLTAGE;

	Vgyro_offset = 0.0;

	Vdisturbance = 0.0;
	Vgyro = 0.0;
	Vpoten = 0.0;

	Omega_h = 0.0;
	Psi_h = 0.0;

	Omega_h_filter = 0.0;

	Psi_cmd = 0.0;
	Psi_error = 0.0;

	Omega_cmd = 0.0;
	Omega_error = 0.0;
	Omega_error_prev = 0.0;
	Omega_error_integral = 0.0;
	Omega_error_integral_update = 0.0;

	designation_first_loop = 1;
	stabilization_first_loop = 1;

	DAQmxCreateTask("", &taskAI);
	DAQmxCreateTask("", &taskAO);

	DAQmxCreateAIVoltageChan(taskAI, "Dev4/ai2", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
	DAQmxCreateAIVoltageChan(taskAI, "Dev4/ai3", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
	DAQmxCreateAIVoltageChan(taskAI, "Dev4/ai0", "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, "");
	
	DAQmxCreateAOVoltageChan(taskAO, "Dev4/ao0", "", 0.0, 5.0, DAQmx_Val_Volts, "");
	DAQmxCreateAOVoltageChan(taskAO, "Dev4/ao1", "", 0.0, 5.0, DAQmx_Val_Volts, "");

	DAQmxStartTask(taskAI);
	DAQmxStartTask(taskAO);

	printf("Press Enter to start the program...\n");
	getchar();

	Write_AO(SWITCH_ON, STANDARD_VOLTAGE);

	printf("\nKeep the gimbal stationary.\n");
	printf("Calculating gyro offset. N = %d\n", N_OFFSET);

	// Vgyro_offset = Calculate_GyroBias(taskAI2, N_OFFSET);
	Vgyro_offset = Calculate_GyroBias(taskAI, N_OFFSET);

	printf("Dataset gyro offset    = %.10f V\n", GYRO_OFFSET_DATASET);
	printf("Calculated gyro offset = %.10f V\n", Vgyro_offset);
	printf("Offset difference      = %.10f V\n\n", Vgyro_offset - GYRO_OFFSET_DATASET);

	printf("AI channels : Dev4/ai0 = Vdisturbance, Dev4/ai2 = Vgyro, Dev4/ai3 = Vpoten\n");
	printf("AO channels : Dev4/ao0 = switch, Dev4/ao1 = motor command\n");

	printf("===== %s Experiment Start =====\n", GetExperimentName());
	printf("Output file : %s\n", GetOutputFileName());
	printf("Final time  : %.3f s\n", GetExperimentFinalTime());
	printf("Total step  : %d\n", N_STEP_EXP);
	printf("Sampling Hz : %.3f Hz\n", SAMPLING_FREQ);
	printf("Press 'f' to emergency stop.\n\n");

	time_init = GetWindowTime();
}

void Import_Time(void)
{
	time_curr = GetWindowTime();
	time_elapsed = time_curr - time_init;

	time_exp = (double)count * SAMPLING_TIME;
}

void Import_Data(void)
{
	Read_AI();	// AIO ¡æ Vdisturbance, AI2 ¡æ Vgyro, AI3 ¡æ Vpoten

	Omega_h = GyroOutput(Vgyro, Vgyro_offset);	// rad/s
	Psi_h = PotentioOutput(Vpoten);				// rad
}

void Digital_Controller(void)
{

	if (stabilization_first_loop == 1)
	{
		Omega_cmd = 0;	//step: 3.242657
		Omega_h_filter = Omega_h;

		Omega_error = 0.0;
		Omega_error_prev = 0.0;
		Omega_error_integral = 0.0;
		Omega_error_integral_update = 0.0;

		stabilization_first_loop = 0;
	}

	Omega_h_filter = LPF_1D(Omega_h, Omega_h_filter, GYRO_LPF_CUTOFF_FREQ, SAMPLING_TIME);
	
	Omega_error = Omega_cmd - Omega_h_filter;

	Omega_error_integral_update = Omega_error_integral + 0.5 * SAMPLING_TIME * (Omega_error + Omega_error_prev);

	Vcontroller_command = STABILIZATION_KP * Omega_error + STABILIZATION_KI * Omega_error_integral_update;

	// Anti-Windup (Clamping)
	if ((Vcontroller_command >= VGYRO_REL_SAT_NEG) && (Vcontroller_command <= VGYRO_REL_SAT_POS))
	{
		Omega_error_integral = Omega_error_integral_update;
	}

	Vcontroller_command = LimitVoltage(Vcontroller_command, VGYRO_REL_SAT_NEG, VGYRO_REL_SAT_POS);

	Omega_error_prev = Omega_error;

	Vcmd = Vcontroller_command;

	Vc = STANDARD_VOLTAGE + LinearmappingFunction(Vcmd);

	Vc = LimitVoltage(Vc, 0.0, 5.0);

	if ((count % 20) == 0)
	{
		printf(
			"\nOmega_cmd = %.4f rad/s | Omega_h = %.4f rad/s | Omega_h_filter = %.4f rad/s | Omega_error = %.4f rad/s | Omega_error_integral = %.4f | Vcmd = %.4f | Vc = %.4f",
			Omega_cmd,
			Omega_h,
			Omega_h_filter,
			Omega_error,
			Omega_error_integral,
			Vcmd,
			Vc
		);
		fflush(stdout);
	}
}

void Export_Data(void)
{
	Write_AO(SWITCH_ON, Vc);
}


void Time_Management(void)
{
	while (1)
	{
		time_curr = GetWindowTime();
		time_elapsed = time_curr - time_init;

		Emergency_Stop_Check();

		if (emergency_stop == 1)
		{
			return;
		}

		if (time_elapsed >= SAMPLING_TIME * 1000.0 * (1.0 + (double)count))
		{
			break;
		}
	}

	count++;
}

void Store_Data_To_Buffer(void)
{
	if ((count >= 0) && (count < N_STEP_EXP))
	{
		OutTime[count] = time_exp;

		OutVcmd[count] = Vcmd;
		OutVc[count] = Vc;

		OutVdisturbance[count] = Vdisturbance;

		OutVgyro[count] = Vgyro;
		OutOmega_h[count] = Omega_h;
		OutOmega_h_filter[count] = Omega_h_filter;

		OutVpoten[count] = Vpoten;
		OutPsi_h[count] = Psi_h;
	}
}

int Check_Stop_Condition(void)
{
	Emergency_Stop_Check();

	if (emergency_stop == 1)
	{
		return 0;
	}

	if (count >= N_STEP_EXP)
	{
		return 0;
	}

	return 1;
}

void Data_Recording(void)
{
	SaveFile(GetOutputFileName(), count, OutTime, OutVcmd, OutVc, OutVdisturbance, OutVgyro, OutOmega_h, OutOmega_h_filter, OutVpoten, OutPsi_h);

	printf("\nSaved: %s\n", GetOutputFileName());
	printf("Saved sample count: %d\n", count);
}

void Program_Termination(void)
{
	printf("\nStopping motor and closing DAQ...\n");

	Write_AO(SWITCH_OFF, STANDARD_VOLTAGE);

	if (taskAI != 0) 
	{ 
		DAQmxStopTask(taskAI); 
		DAQmxClearTask(taskAI); 
		taskAI = 0; 
	}

	if (taskAO != 0)
	{
		DAQmxStopTask(taskAO);
		DAQmxClearTask(taskAO);
		taskAO = 0;
	}

	printf("Program finished.\n");
}

/* ===============================
		Utility Function Define
================================= */

double GetWindowTime(void)
{
	LARGE_INTEGER liEndCounter;
	LARGE_INTEGER liFrequency;

	QueryPerformanceCounter(&liEndCounter);
	QueryPerformanceFrequency(&liFrequency);

	return (liEndCounter.QuadPart / (double)(liFrequency.QuadPart) * 1000.0);
}

double Calculate_GyroBias(TaskHandle taskAI_gyro, int N)
{
	float64 aiData[3] = { 0.0, 0.0, 0.0 };
	int32 samplesRead = 0;

	float64 Vin_gyro = 0.0;

	double y = 0.0;
	double y_bar = 0.0;

	int k = 0;
	int k_avg = 0;

	for (k = 1; k <= N; k++)
	{
		DAQmxReadAnalogF64(taskAI_gyro, 1, 10.0, DAQmx_Val_GroupByScanNumber, aiData, 3, &samplesRead, NULL);

		Vin_gyro = aiData[0];
		y = Vin_gyro;

		if (k > N / 2)
		{
			k_avg++;

			y_bar = (1.0 - 1.0 / (double)k_avg) * y_bar
				+ (1.0 / (double)k_avg) * y;
		}
	}

	return y_bar;
}

double GyroOutput(double Vgyro, double Vgyro_offset)
{
	double Omega_h = 0.0;

	Omega_h = GYRO_K * (Vgyro - Vgyro_offset);  // rad/s

	return Omega_h;
}

double PotentioOutput(double Vpoten)
{
	double Psi_h = 0.0;

	Psi_h = (Vpoten - POTEN_OFFSET) / POTEN_K;  // rad

	return Psi_h;
}

double LimitVoltage(double x, double xmin, double xmax)
{
	if (x < xmin)
	{
		return xmin;
	}

	if (x > xmax)
	{
		return xmax;
	}

	return x;
}

void Emergency_Stop_Check(void)
{
	if (_kbhit())
	{
		keyborad = _getch();

		if (keyborad == 'f')
		{
			emergency_stop = 1;
			Write_AO(SWITCH_OFF, STANDARD_VOLTAGE);

			printf("[EMERGENCY STOP]\n");
		}
#ifdef USE_POTEN_MANUAL_COMMAND
		else
		{
			Poten_Command_KeyInput(keyborad);
		}
#endif // USE_POTNE_MANUAL_COMMAND

	}
}

void SaveFile(const char* filename, int count, double* OutTime, double* OutVcmd, double* OutVc, double* OutVdisturbance, double* OutVgyro, double* OutOmega_h, double* OutOmega_h_filter, double* OutVpoten, double* OutPsi_h)
{
	FILE* pFile = NULL;

	int idx = 0;

	if (filename == NULL)
	{
		printf("File name is NULL.\n");
		return;
	}

	pFile = fopen(filename, "w+t");

	if (pFile == NULL)
	{
		printf("File open error: %s\n", filename);
		return;
	}

	fprintf(pFile, "time Vcmd Vc Vdisturbance Vgyro omega_h omega_h_filter Vpoten psi_h\n");

	for (idx = 0; idx < count; idx++)
	{
		fprintf(
			pFile, "%20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f %20.10f\n",
			OutTime[idx],
			OutVcmd[idx],
			OutVc[idx],
			OutVdisturbance[idx],
			OutVgyro[idx],
			OutOmega_h[idx],
			OutOmega_h_filter[idx],
			OutVpoten[idx],
			OutPsi_h[idx]
		);
	}

	fclose(pFile);
}

// Linearization Funtion Define
double Poly2(double a, double b, double c, double x)
{
	return a * x * x + b * x + c;
}

double SolveQuadraticInRange(double a, double b, double c, double Vout_motor, double xmin, double xmax, double reference)
{
	double A = a;
	double B = b;
	double C = c - Vout_motor;

	double D = 0.0;
	double sqrtD = 0.0;

	double r1 = 0.0;
	double r2 = 0.0;

	int valid1 = 0;
	int valid2 = 0;

	const double EPS = 1.0e-12;

	if (fabs(A) < EPS)
	{
		if (fabs(B) < EPS)
		{
			return LimitVoltage(reference, xmin, xmax);
		}

		r1 = -C / B;

		return LimitVoltage(r1, xmin, xmax);
	}

	D = B * B - 4.0 * A * C;

	if (D < 0.0)
	{
		double y_min = Poly2(a, b, c, xmin);
		double y_max = Poly2(a, b, c, xmax);

		if (fabs(y_min - Vout_motor) < fabs(y_max - Vout_motor))
		{
			return xmin;
		}
		else
		{
			return xmax;
		}
	}

	sqrtD = sqrt(D);

	r1 = (-B + sqrtD) / (2.0 * A);
	r2 = (-B - sqrtD) / (2.0 * A);

	valid1 = ((r1 >= xmin) && (r1 <= xmax));
	valid2 = ((r2 >= xmin) && (r2 <= xmax));

	if ((valid1 == 1) && (valid2 == 1))
	{
		if (fabs(r1 - reference) < fabs(r2 - reference))
		{
			return r1;
		}
		else
		{
			return r2;
		}
	}
	else if (valid1 == 1)
	{
		return r1;
	}
	else if (valid2 == 1)
	{
		return r2;
	}
	else
	{
		if (fabs(r1 - reference) < fabs(r2 - reference))
		{
			return LimitVoltage(r1, xmin, xmax);
		}
		else
		{
			return LimitVoltage(r2, xmin, xmax);
		}
	}
}

double LinearReferenceMap(double u, double u_start, double u_end, double y_start, double y_end)
{
	double alpha = 0.0;

	if (fabs(u_end - u_start) < 1.0e-12)
	{
		return y_start;
	}

	alpha = (u - u_start) / (u_end - u_start);

	if (alpha > 1.0)
	{
		alpha = 1.0;
	}
	else if (alpha < 0.0)
	{
		alpha = 0.0;
	}

	return y_start + alpha * (y_end - y_start);
}

double LinearmappingFunction(double Vcmd_output_controller)
{
	double Vcmd_target = 0.0;
	double Vgyro_rel_target = 0.0;
	
	double Vcmd_input_motor = 0.0;
	double reference = 0.0;

	Vcmd_target = LimitVoltage(Vcmd_output_controller, VGYRO_REL_SAT_NEG, VGYRO_REL_SAT_POS);

	// Deadzone
	if ((Vcmd_target > VGYRO_REL_CUT_NEG) && (Vcmd_target < VGYRO_REL_CUT_POS))
	{
		return 0.0;
	}

	Vgyro_rel_target = Vcmd_target;

	if (Vcmd_target >= VGYRO_REL_CUT_POS)
	{

		reference = LinearReferenceMap(Vcmd_target, VGYRO_REL_CUT_POS, VGYRO_REL_SAT_POS, VCMD_REAL_CUT_POS, VCMD_REAL_SAT_POS);

		Vcmd_input_motor = SolveQuadraticInRange(P_POS_A, P_POS_B, P_POS_C, Vgyro_rel_target, VCMD_REAL_CUT_POS, VCMD_REAL_SAT_POS, reference);

		return Vcmd_input_motor;
	}

	if (Vcmd_target <= VGYRO_REL_CUT_NEG)
	{
		reference = LinearReferenceMap(Vcmd_target, VGYRO_REL_CUT_NEG, VGYRO_REL_SAT_NEG, VCMD_REAL_CUT_NEG, VCMD_REAL_SAT_NEG);

		Vcmd_input_motor = SolveQuadraticInRange(P_NEG_A, P_NEG_B, P_NEG_C, Vgyro_rel_target, VCMD_REAL_SAT_NEG, VCMD_REAL_CUT_NEG, reference);

		return Vcmd_input_motor;
	}

	return 0.0;
}

// Controller Funtion Define
double LPF_1D(double x, double y_prev, double freq_cut, double Ts)
{
	// wc = cutoff freq
	double tau = 1.0 / (2.0 * UNIT_PI * freq_cut);
	double alpha = Ts / (tau + Ts);

	return y_prev + alpha * (x - y_prev);
}
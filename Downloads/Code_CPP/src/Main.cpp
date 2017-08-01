#define _USE_MATH_DEFINES

//#include <cstdlib>
//#include <time.h>
//#include <Windows.h>
//#include <random>
//#include <math.h>

#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <intrin.h>
#include "stdafx.h"
#include "KinovaTypes.h"
#include "omp.h"
#include "dislin.h"
#include "discpp.h"
#include "Defined_Macro.h"
#include "IMU_Sensor.h"
#include "EMG_Sensor.h"
#include "Control_JACO.h"
#include "Dislin_Scope.h"
#else
//#include <myserial.h>
#include <libkindrv/kindrv.h>
#include <sys/select.h>
#endif




using namespace Control_IMU_JACO;
using namespace System;
using namespace System::IO::Ports;
using namespace std;
	
//using namespace System::ComponentModel;
//using namespace System::Collections;
//using namespace System::Windows::Forms;
//using namespace System::Data;
//using namespace System::Drawing;
//using namespace System::Threading;

#ifdef _WIN32
HINSTANCE commandLayer_handle;
static int(*MyInitAPI)();
static int(*MyCloseAPI)();
static int(*MyMoveHome)();
static int(*MyInitFingers)();
static int(*MySendAdvanceTrajectory)(TrajectoryPoint);
static int(*MyGetQuickStatus)(QuickStatus &);
static int(*MySendJoystickCommand)(JoystickCommand);
static int(*MySetCartesianControl)();
static int(*MyStartControlAPI)();
static int(*MyStopControlAPI)();
static int(*MyGetControlMapping)(ControlMappingCharts &Response);
#else
// JACO
JacoArm *arm;
// USB
int result;
const char* usb_dev;
std::string usb_name;
// PRINTING
int flag = 0;
int c;
#endif

bool PORT_COM_OUVERT = 0;
const int Packet_Size = 32;
int PCKT [Packet_Size - 1];
static int S_Of_F = 51;
static int E_Of_F = 52;
int JACO_PREVIOUS_MODE = 0;
static bool ControlJACO = 0; //ON ou OFF
static bool Acq_Started = 0; //ON ou OFF
static bool Bouton_Hold = 0; //ON ou OFF
Byte Data_received = System::Convert::ToByte(0);
Byte Test;
char EVENT_SAFETY_KEY = NO_EVENT;
/////
Byte INIT_KY[256];
/////
//TIMING
double START_TIME = 0;
double END_TIME = 0;
double delay_TIME = 0;
double GLOBAL_TIME;
double INIT_TIME;
int Sec, Min, Hr;
int refresh = 0;
int elapsed = 20;

//KEYPAD
int in_kyp; char ch_kyp;
bool kyp_ready = 0;
int kyp_delay = 0;
	
const int nb_keys = 23;
float keypressed [nb_keys];

//PRINTING
bool PRINT = ON;
bool PRINT_CALIB = ON;

Control_JACO CTRL_IMU;

#ifdef _WIN32
//PLOT
bool PLOT_DATA = ON;

const int NB_SAMPLES_BY_PCKT = 9;
const int NB_SAMPLES_TO_PLOT = 11 * NB_SAMPLES_BY_PCKT;
const int NB_SCOPE_SAMPLES = 50 * NB_SAMPLES_TO_PLOT;

const int NB_TKE_TO_PLOT = 15;
const int NB_SCOPE_TKE = 50 * NB_TKE_TO_PLOT;

const int NB_IMU_TO_PLOT = 5;
const int NB_SCOPE_IMU = 50 * NB_IMU_TO_PLOT;

Dislin_Scope PLOT_EMG;
float new_emg[NB_SAMPLES_TO_PLOT]; int i_nemg = 0; 
float new2_emg[NB_SAMPLES_TO_PLOT]; int i_n2emg = 0;
Dislin_Scope PLOT_TKE;
float new_tke[NB_TKE_TO_PLOT]; int i_ntke = 0;
float th = 0;
Dislin_Scope PLOT_IMU;
float new_imu[4];
#endif

int mode_jaco_test;
int test_timer_jaco = 0;

void update_plot(void);
void update_console(void);
int send_cmd_JACO(void);
int check_keypad(void);
void change_mode_A(char value);
int ReadJACOMode(void);
int START_JACO_ARM(void);
int STOP_JACO_ARM(void);
int SET_JACO_ARM(void);
void refresh_jaco(void);

#ifndef _WIN32
/* keyboard */
struct termios orig_termios;

void reset_terminal_mode()
{
	tcsetattr(0, TCSANOW, &orig_termios);
}
void set_conio_terminal_mode()
{
	struct termios new_termios;

	/* take two copies - one for now, one for later */
	tcgetattr(0, &orig_termios);
	memcpy(&new_termios, &orig_termios, sizeof(new_termios));

	/* register cleanup handler, and set the new terminal mode */
	atexit(reset_terminal_mode);
	cfmakeraw(&new_termios);
	tcsetattr(0, TCSANOW, &new_termios);
}

int kbhit()
{
	struct timeval tv = { 0L, 0L };
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(0, &fds);
	return select(1, &fds, NULL, NULL, &tv);
}

int getch()
{
	int r;
	unsigned char c;
	if ((r = read(0, &c, sizeof(c))) < 0) {
		return r;
	}
	else {
		return c;
	}
}
#endif

[STAThread]
int main()
{
	//std::random_device                  rand_dev;
	//std::mt19937                        generator(rand_dev());
	printf(" ____     ___  ____  _____    _____  ______  _____  ____  ____  _____ _____ \n");
	printf("|_   |   /   ||  __||  _  |  |  _  ||  _   ||  _  ||_   ||  _ ||  __||_   _|\n");
	printf("  |  |  / _  || |   | | | |  | |_| || |_|  || | | |  |  || |_  | |     | | \n");
	printf("  |  | / |_| || |   | | | |  |  ___||  _  / | | | |  |  ||  _| | |     | | \n");
	printf(" _/  |/  / | || |__ | |_| |  | |    | | | \\ | |_| | _/  || |__ | |__   | |\n");
	printf("|___/|__/  |_||____||_____|  |_|    |_| |__||_____||___/ |____||____|  |_|\n");

	system("start /min /b XBOX\\JoyToKey.exe");
	printf("JoyToKey started\n");

	CTRL_IMU.initialize();
	
	int cpt = 0; int int_kyp = 0;
	for (int i_k = 0; i_k < nb_keys; i_k++)
		keypressed[i_k] = 0;

#ifdef _WIN32
	if (PLOT_DATA)
	{
		PLOT_EMG.initialize(EMG_SCOPE, 1, NB_SCOPE_SAMPLES, NB_SAMPLES_TO_PLOT, 2);
		PLOT_TKE.initialize(TKE_SCOPE, 2, NB_SCOPE_TKE, NB_TKE_TO_PLOT, 2);
		PLOT_IMU.initialize(IMU_SCOPE, 3, NB_SCOPE_IMU, NB_IMU_TO_PLOT, 2);
	}

	String^ Port_COM;
	SerialPort^  serialPort1;
	serialPort1 = gcnew SerialPort();
		
	printf("Available Ports:\n");
	for each (String^ s in SerialPort::GetPortNames())
	{
		Console::WriteLine("   {0}", s);
	}
	printf("Entrez le port COM: \n");
	Port_COM = Convert::ToString(Console::ReadLine());
		
	serialPort1->BaudRate = 1998848;//115200;//
	serialPort1->PortName = Port_COM;//"COM11";
	serialPort1->ReadTimeout = 100000;
	serialPort1->Open();
	
	if (serialPort1->IsOpen)
		PORT_COM_OUVERT = 1;
#else
	result=system("ls -l /dev/ttyUSB*");
	if (result != 0){
		std::cout << "\n no USB devices were found\n" << std::endl;
		return 0;
	}
	else {
		printf("\nEnter /dev/ttyUSB* wished \n");
		std::cin >> usb_name;
		usb_dev = usb_name.c_str();
	}
	mySerial serial1(usb_dev, 1998848);//115200;//);
#endif

	if (PORT_COM_OUVERT)
	{
		printf("COM Port opened... \nStart Acquisition?  YES[Y]  |  NO[N]\n");
		while ((ch_kyp != 'Y') && (ch_kyp != 'y') && (ch_kyp != 'N') && (ch_kyp != 'n'))
		{
			std::cin >> ch_kyp;
		}
		if (ch_kyp == 'Y' || ch_kyp == 'y')
		{
#ifdef _WIN32
			serialPort1->ReadExisting();
			system("cls");
#else
			//clear buffer
			system("clear");
#endif
			printf("Acquisition STARTED...\n");
		}
		else
		{
#ifdef _WIN32
			serialPort1->Close();
			printf("COM Port closed, program will stop...\n");
			Sleep(3000);
			return 0;
#else
			//close com port on RPi3
#endif
		}
	}	
		
	//timer_start(refresh_jaco, 1000);

	while (1)
	{

#ifndef _WIN32
		c = 0;
		set_conio_terminal_mode();
		if (kbhit()){
			c = getch();
		}
		reset_terminal_mode();
#endif

#ifdef _WIN32		
		while ((serialPort1->ReadByte() != S_Of_F));
		refresh++;

		for (cpt = 0; cpt < Packet_Size - 1; cpt++)
		{
			PCKT[cpt] = serialPort1->ReadByte();
		}
#else
		//read data and store in PCKT (RPi)
		while (PCKT[0] != S_Of_F)
		{
			serial1.Receive(PCKT, 1);
		}
		serial1.Receive(PCKT, Packet_Size - 1);
#endif

		if (PCKT[Packet_Size - 2] == E_Of_F)
		{	
			if (PCKT[0] == SAFETY_KEY_ID)
				EVENT_SAFETY_KEY = PCKT[13];

			CTRL_IMU.process_payload(PCKT);
			int res_update = CTRL_IMU.update_calibration();//renvoie 1 à la fin d'une calibration
			if (res_update)
				PRINT = 1;
#ifdef _WIN32
			update_plot();
#endif
			update_console();
		}
		change_mode_A(CTRL_IMU.EMG_1.EMG_DETECTED);
		//CTRL_IMU.EMG_1.EMG_DETECTED = 0;
		//serialPort1->ReadExisting();
		int etat = send_cmd_JACO ();
		int touch = check_keypad ();
	}
#pragma endregion
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
void change_mode_A(char value)
{
	if (value == 1)
	{
		if (ControlJACO == 1 && CTRL_IMU.EMG_CALIBRATED)
		{
			CTRL_IMU.MODE_JACO += 1;
			if (CTRL_IMU.MODE_JACO == NB_MODES_JACO)
				CTRL_IMU.MODE_JACO = 0;
				
			CTRL_IMU.B_INDEX1 = XYZ_BUTTON1;
			CTRL_IMU.B_INDEX2 = XYZ_BUTTON2;

			if (EMG_CONTRACTION_IS_SHORT)
			{
#ifdef _WIN32
				printf("short contraction detected...\n");
				(*MySendJoystickCommand)(CTRL_IMU.PushButton(CTRL_IMU.B_INDEX1));
				Sleep(50);
				(*MySendJoystickCommand)(CTRL_IMU.ReleaseButton(CTRL_IMU.B_INDEX1));
				//CTRL_IMU.MODE_JACO_RETURNED = ReadJACOMode();
#else
				//send joystick command for mode change
#endif
			}
			else if (EMG_CONTRACTION_IS_LONG)
			{
#ifdef _WIN32
				printf("long contraction detected...\n");
				(*MySendJoystickCommand)(CTRL_IMU.PushButton(CTRL_IMU.B_INDEX2));
				Sleep(50);
				(*MySendJoystickCommand)(CTRL_IMU.ReleaseButton(CTRL_IMU.B_INDEX2));
#else
				//send joystick command for mode change
#endif
			}
			else if (EMG_CONTRACTION_IS_VERY_LONG)
			{
				if (ControlJACO == OFF)
					SET_JACO_ARM();

				if (Bouton_Hold == 1 && ControlJACO == ON)
					Bouton_Hold = 0;
				else if (Bouton_Hold == 0 && ControlJACO == ON)
				{
					Bouton_Hold = 1;
					CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
					if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
					{
						PRINT = 0;
						printf("NEUTRAL IMU CALIBRATION...\n");
						CTRL_IMU.HEAD_IMU.CALIB_TYPE = NEUTRAL;
					}
					CTRL_IMU.InhibiteZctrl = 0;
				}
				//Console::WriteLine("BOUTON HOLD = " + Bouton_Hold + "\n");
				printf("BOUTON HOLD = %d\n", (int)Bouton_Hold);
			}
			RESET_CONTRACT_CHRONO;
		}
	}
}
#ifdef _WIN32
//////////////////////////////////////////////////////////////////////////////////////////
void update_plot(void)
{
	if (PLOT_DATA)
	{
		if (PCKT[0] == HEADSET)
		{
			new_imu[0] = CTRL_IMU.HEAD_IMU.pitchctrl;
			new_imu[1] = CTRL_IMU.HEAD_IMU.rollctrl;
			new_imu[2] = CTRL_IMU.HEAD_IMU.yawctrl;
			PLOT_IMU.update(new_imu, 3, 1);
		}
		else if (PCKT[0] == REFERENCE)
		{
		}
		else if (PCKT[0] == EMG1)
		{
			if (CTRL_IMU.EMG_1.NEW_CALIB)
			{
				CTRL_IMU.EMG_1.NEW_CALIB = 0;
				PLOT_TKE.update(CTRL_IMU.EMG_1.THRESHOLD, PLOT_TKE.nb_data_to_plot, 2);
				PLOT_TKE.update(CTRL_IMU.EMG_1.UPPER_THRESHOLD, CTRL_IMU.EMG_1.LOWER_THRESHOLD, PLOT_TKE.nb_data_to_plot, 3);
				CTRL_IMU.EMG_1.NEW_CALIB = 0;
			}
			for (int pemg = 0; pemg < NB_SAMPLES_BY_PCKT; pemg++)
			{
				new_emg[i_nemg + pemg] = CTRL_IMU.EMG_1.DATA_EMG_FILT[pemg] * 5;
				if (new_emg[i_nemg + pemg] >= 100)
					new_emg[i_nemg + pemg] = 100;
				else if (new_emg[i_nemg + pemg] <= -100)
					new_emg[i_nemg + pemg] = -100;
			}
			i_nemg += NB_SAMPLES_BY_PCKT;

			new_tke[i_ntke] = CTRL_IMU.EMG_1.MEAN_TKE_CURRENT;
			i_ntke += 1;

			if (i_nemg >= PLOT_EMG.nb_data_to_plot)
			{
				PLOT_EMG.update(new_emg, PLOT_EMG.nb_data_to_plot, 1);
				i_nemg = 0;
			}
			if (i_ntke >= PLOT_TKE.nb_data_to_plot)
			{
				PLOT_TKE.update(new_tke, PLOT_TKE.nb_data_to_plot, 1);
				PLOT_TKE.update(CTRL_IMU.EMG_1.THRESHOLD, PLOT_TKE.nb_data_to_plot, 2);
				PLOT_TKE.update(CTRL_IMU.EMG_1.UPPER_THRESHOLD, CTRL_IMU.EMG_1.LOWER_THRESHOLD, PLOT_TKE.nb_data_to_plot, 3);
				i_ntke = 0;
			}
		}
	}
}
//////////////////////////////////////////////////////////////////////////////////////////
#endif

void update_console(void)
{
#ifdef _WIN32
	if (START_TIME != 0)
		END_TIME = omp_get_wtime() - START_TIME;
	START_TIME = omp_get_wtime();
#endif
	if (refresh == elapsed)
	{
		if (PRINT)
		{
#ifdef _WIN32			
			system("cls");
#else
			system("clear");
#endif
			printf(" ____     ___  ____  _____    _____  ______  _____  ____  ____  _____ _____ \n");
			printf("|_   |   /   ||  __||  _  |  |  _  ||  _   ||  _  ||_   ||  _ ||  __||_   _|\n");
			printf("  |  |  / _  || |   | | | |  | |_| || |_|  || | | |  |  || |_  | |     | | \n");
			printf("  |  | / |_| || |   | | | |  |  ___||  _  / | | | |  |  ||  _| | |     | | \n");
			printf(" _/  |/  / | || |__ | |_| |  | |    | | | \\ | |_| | _/  || |__ | |__   | |\n");
			printf("|___/|__/  |_||____||_____|  |_|    |_| |__||_____||___/ |____||____|  |_|\n");

			printf("_______________________________________\n");

			if (ControlJACO == 1)
				printf("API SET\n");
			else
				printf("API CLOSED\n");

			if (Bouton_Hold == 1)
				printf("CONTROL SET\n");
			else
				printf("CONTROL STOPPED\n");

			if (EVENT_SAFETY_KEY == ENTER_STANDBY)
				printf("SYSTEM IN STANDBY MODE\n");
			else if (EVENT_SAFETY_KEY == ENTER_CALIBRATION)
				printf("SYSTEM IN CALIBRATION MODE\n");
			else if (EVENT_SAFETY_KEY == ENTER_CONTROL)
				printf("SYSTEM IN CONTROL MODE\n");

			printf("variable test routine : %d\n", test_timer_jaco);

			printf("***************************************\n");
			if (CTRL_IMU.HEAD_IMU_STATE < CTRL_IMU.limit_missing)
			{
				printf("FREQ HEADSET = %d Hz\n", CTRL_IMU.HEAD_IMU.FREQ);
				printf("RAW  : PITCH = %d __ ROLL  = %d __ YAW  = %d | [deg]\n", (int)CTRL_IMU.HEAD_IMU.pitchraw, (int)CTRL_IMU.HEAD_IMU.rollraw, (int)CTRL_IMU.HEAD_IMU.yawraw);
				printf("CTRL : PITCH = %d __ ROLL  = %d __ YAW  = %d | [deg]\n", (int)CTRL_IMU.HEAD_IMU.pitchctrl, (int)CTRL_IMU.HEAD_IMU.rollctrl, (int)CTRL_IMU.HEAD_IMU.yawctrl);
				printf("     : AMP   = %f __ THETA = %f | [deg]\n", (float)CTRL_IMU.Amp, (float)CTRL_IMU.Theta);
				if (CTRL_IMU.IMU_CALIBRATED)
					printf("HEADSET CALIBRATED\n");
				else
					printf("HEADSET NOT CALIBRATED\n");
			}
			else
				printf("HEADSET SENSOR NOT DETECTED\n");
			printf("***************************************\n");
			if (CTRL_IMU.CHAIR_IMU_STATE < CTRL_IMU.limit_missing)
			{
				printf("FREQ CHAIR  = %d Hz\n", CTRL_IMU.CHAIR_IMU.FREQ);
				printf("HEADING JACO  : %d [deg]\n", (int)CTRL_IMU.CHAIR_IMU.yawraw);
				printf("CHAIR VELOCITY = %d deg/s   |  THRESHOLD = %d deg/s\n", (int)CTRL_IMU.CHAIR_IMU.MeanVelYaw, (int)CTRL_IMU.CHAIR_IMU.ThRotReference);
			}
			else
				printf("REFERENCE SENSOR NOT DETECTED\n");
			printf("***************************************\n");
			if (CTRL_IMU.EMG_1_STATE < CTRL_IMU.limit_missing)
			{
				printf("FREQ EMG  = %d Hz\n", CTRL_IMU.EMG_1.FREQ * 9);
				printf("CURRENT TKE  = %f  |  THRESHOLD TKE = %f\n", CTRL_IMU.EMG_1.MEAN_TKE_CURRENT, CTRL_IMU.EMG_1.THRESHOLD);
				printf("UPPER THRES. = %f  |  LOWER THRES   = %f\n", CTRL_IMU.EMG_1.UPPER_THRESHOLD, CTRL_IMU.EMG_1.LOWER_THRESHOLD);
				printf("DETECTION EMG = %d |  CONTRACTION DURATION = %d\n ", CTRL_IMU.EMG_1.EMG_DETECTED, CTRL_IMU.EMG_1.time_contraction);
				if (CTRL_IMU.EMG_CALIBRATED)
					printf("EMG CALIBRATED\n");
				else
					printf("EMG NOT CALIBRATED\n");
			}
			else
				printf("EMG SENSOR NOT DETECTED\n");
			printf("***************************************\n");
			//printf("PARAMS : Ctrl Z = %d | Ref active = %d | TH ROT CHAIR = %d deg/s (window = %d)\n", CTRL_IMU.ZActive, CTRL_IMU.ReferenceActive, (int)CTRL_IMU.CHAIR_IMU.ThRotReference, CTRL_IMU.CHAIR_IMU.nb_val_gyr);
			if (CTRL_IMU.CHAIR_IMU_STATE < CTRL_IMU.limit_missing)
				printf("HEADING OFFSET = %d\n", (int)CTRL_IMU.Theta_offset);
			//Console::WriteLine("DIRECTION = " + Convert::ToString(CTRL_IMU.DIRECTION)); //+ " ____ CmdYout = " + Convert::ToString(CJACO.CmdXout));
			printf("CMDX  = %f | CMDY = %f | CMDZ = %f \n", CTRL_IMU.CmdX_new, CTRL_IMU.CmdY_new, CTRL_IMU.CmdZ_new);
			if (CTRL_IMU.ZActive)
				printf("Yaw Control Active\n");
			else
				printf("Yaw Control Not Active\n");
#ifdef _WIN32
			if (PRINT_CALIB)
			{
				printf("***************************************\n");
				Console::WriteLine("THETA     = " + Convert::ToString(CTRL_IMU.Theta) + "|  AMP = " + Convert::ToString(CTRL_IMU.Amp)); 
				Console::WriteLine("MODE ALGO = " + Convert::ToString(CTRL_IMU.MODE_JACO) + "|  MODE ROBOT = " + Convert::ToString(CTRL_IMU.MODE_JACO_RETURNED));
				printf("_______________________________________\n");
				Console::WriteLine("MIN FORW  = " + Convert::ToString(CTRL_IMU.AminForw) + "|  MIN BACK  = " + Convert::ToString(CTRL_IMU.AminBack));
				Console::WriteLine("MAX FORW  = " + Convert::ToString(CTRL_IMU.AmaxForw) + "|  MAX BACK  = " + Convert::ToString(CTRL_IMU.AmaxBack));
				Console::WriteLine("YAW FORW  = " + Convert::ToString(CTRL_IMU.YawCtrlForw) + "|  YAW BACK  = " + Convert::ToString(CTRL_IMU.YawCtrlBack));
				printf("_______________________________________\n");
				Console::WriteLine("MIN RIGHT = " + Convert::ToString(CTRL_IMU.AminRight) + "|  MIN LEFT  = " + Convert::ToString(CTRL_IMU.AminLeft));
				Console::WriteLine("MAX RIGHT = " + Convert::ToString(CTRL_IMU.AmaxRight) + "|  MAX LEFT  = " + Convert::ToString(CTRL_IMU.AmaxLeft));
				Console::WriteLine("YAW RIGHT  = " + Convert::ToString(CTRL_IMU.YawCtrlRight) + "|  YAW LEFT  = " + Convert::ToString(CTRL_IMU.YawCtrlLeft));
				printf("_______________________________________\n");
				Console::WriteLine("ROT MIN RIGHT = " + Convert::ToString(CTRL_IMU.RotminRight) + "|  ROT MIN LEFT  = " + Convert::ToString(CTRL_IMU.RotminLeft));
				Console::WriteLine("ROT MAX RIGHT = " + Convert::ToString(CTRL_IMU.RotmaxRight) + "|  ROT MAX LEFT  = " + Convert::ToString(CTRL_IMU.RotmaxLeft));
				Console::WriteLine("AMP ROT RIGHT = " + Convert::ToString(CTRL_IMU.AmpRotRight) + "|  AMP ROT LEFT   = " + Convert::ToString(CTRL_IMU.AmpRotLeft));
				printf("_______________________________________\n"); 
				Console::WriteLine("ZONE FORW  = " + Convert::ToString(CTRL_IMU.ZoneForw) + "|  ZONE BACK  = " + Convert::ToString(CTRL_IMU.ZoneBack));
				Console::WriteLine("ZONE RIGHT = " + Convert::ToString(CTRL_IMU.ZoneRight) + "|  ZONE LEFT  = " + Convert::ToString(CTRL_IMU.ZoneLeft));
			}
#endif
			/*printf("			                  __\n");
			printf("			                 / _|\n");
			printf("			            ____/ /\n");
			printf("			    _______/      |_____\n");
			printf("			   |  _____        _____|\n");
			printf("			   |  |    \\____  |\n");
			printf("			   |  |         \\ \\ \n");
			printf("			  /  /           \\_\\ \n");
			printf("			 /  /\n");
			printf("			|  |\n");
			printf("			|  |\n");
			printf("		  __|__|_\n");
			printf("		 / / / / /\n");*/
		}
		refresh = 0;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////
int send_cmd_JACO (void)
{
	int result;
	if (Bouton_Hold == 1 && ControlJACO == 1 && CTRL_IMU.IMU_CALIBRATED)
	{
#ifdef _WIN32
		result = (*MySendJoystickCommand)(CTRL_IMU.CommandJACO(1));
#else
		//send joystick commande for control (CTRL_IMU.CommandJACO(1))
#endif
		if (result != 1)
			printf("Command not transmitted to JACO, please retry...\n");
	}
	else if (Bouton_Hold == 0 && ControlJACO == 1)
	{
#ifdef _WIN32
		result = (*MySendJoystickCommand)(CTRL_IMU.CommandJACO(0));
#else
		//send joystick commande for control (CTRL_IMU.CommandJACO(1))
#endif
		if (result != 1)
			printf("Command not transmitted to JACO, please retry...\n");
	}
	return result;
}
//////////////////////////////////////////////////////////////////////////////////////////
int check_keypad(void)
{
	int num_kyp = 0;
	if (kyp_delay != 0)
	{
		kyp_delay++;
		if (kyp_delay == 5)
			kyp_delay = 0;
	}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if (GetAsyncKeyState(SPACEBAR_KY) && kyp_ready && (keypressed[0] == 0))//STOP/START API
#else
	if (c == SPACEBAR_KY && kyp_ready && (keypressed[0] == 0))
#endif
	{
		keypressed[0] = 1;
		num_kyp = 1;
		#pragma region STOP/START API
		kyp_delay = 1;
		SET_JACO_ARM();
	#pragma endregion
	}
	else {keypressed[0] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(U_KY)))//LOCK/UNLOCK KEYPAD
#else
	if (c == U_KY)
#endif
	{
		if (keypressed[1] == 0)
		{
			#pragma region LOCK/UNLOCK KEYPAD
			kyp_delay = 1;
			if (kyp_ready == 0)
			{
				kyp_ready = 1;
				printf("Keyboard Unlocked \n");
			}
			else
			{
				kyp_ready = 0;
				printf("Keyboard Locked \n");
			}
			#pragma endregion
		}
		keypressed[1] = 1;
		num_kyp = 2;
	}
	else {keypressed[1] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32	
	if (GetAsyncKeyState(D_KY) && kyp_ready)
#else
	if (c == D_KY && kyp_ready)
#endif
	{
		if (keypressed[2] == 0)
		{
			#pragma region EN/DISABLE - SET 0
			kyp_delay = 1;
			if (Bouton_Hold == 1)
				Bouton_Hold = 0;
			else
			{
				Bouton_Hold = 1;
				CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
				if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
				{
					PRINT = 0;
					printf("NEUTRAL IMU CALIBRATION...\n");
					CTRL_IMU.HEAD_IMU.CALIB_TYPE = NEUTRAL;
				}
				CTRL_IMU.InhibiteZctrl = 0;
			}
			Console::WriteLine("BOUTON HOLD = " + Bouton_Hold + "\n");
			#pragma endregion
		}
		keypressed[2] = 1;
		num_kyp = 3;
	}
	else {keypressed[2] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if (GetAsyncKeyState(O_KY) && kyp_ready)//CHANGE MODE B (BOUTON 2)
#else
	if (c == O_KY && kyp_ready)
#endif

	{
		if (keypressed[3] == 0)
		{
			#pragma region CHANGE MODE 2
			kyp_delay = 1;
			if (ControlJACO)
			{
				CTRL_IMU.MODE_JACO += 1;
				if (CTRL_IMU.MODE_JACO == NB_MODES_JACO)
					CTRL_IMU.MODE_JACO = 0;
					
				CTRL_IMU.B_INDEX1 = XYZ_BUTTON2;
				CTRL_IMU.B_INDEX2 = 0;
				(*MySendJoystickCommand)(CTRL_IMU.PushButton(CTRL_IMU.B_INDEX1));
				Sleep(50);
				(*MySendJoystickCommand)(CTRL_IMU.ReleaseButton(CTRL_IMU.B_INDEX1));
			}
			#pragma endregion
		}
		keypressed[3] = 1;
		num_kyp = 4;
	}
	else {keypressed[3] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if (GetAsyncKeyState(P_KY) && kyp_ready)//CHANGE MODE A (BOUTON 1)
#else
	if (c == P_KY && kyp_ready)
#endif
	{
		if (keypressed[4] == 0)
		{
			#pragma region CHANGE MODE 3
			kyp_delay = 1;
			if (ControlJACO)
			{
				CTRL_IMU.MODE_JACO += 1;
				if (CTRL_IMU.MODE_JACO == NB_MODES_JACO)
					CTRL_IMU.MODE_JACO = 0;

				CTRL_IMU.B_INDEX1 = XYZ_BUTTON1;
				CTRL_IMU.B_INDEX2 = 0;
				(*MySendJoystickCommand)(CTRL_IMU.PushButton(CTRL_IMU.B_INDEX1));
				Sleep(50);
				(*MySendJoystickCommand)(CTRL_IMU.ReleaseButton(CTRL_IMU.B_INDEX1));
			}
			#pragma endregion
		}
		keypressed[4] = 1;
		num_kyp = 5;
	}
	else {keypressed[4] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if (GetAsyncKeyState(Z_KY) && kyp_ready)//Calibrate NEUTRAL (BOUTON 5)
#else
	if (c == Z_KY && kyp_ready)
#endif
	{
		if (keypressed[5] == 0)
		{
			#pragma region CALIB NEUTRAL POSITION
			kyp_delay = 1;
			CTRL_IMU.EMG_1.CALIB_EMG_INDEX = 0;
			if (CTRL_IMU.EMG_1.CALIB_EMG_INDEX != -1)
			{
				PRINT = 0;
				printf("NEUTRAL EMG CALIBRATION...\n");
			}
			#pragma endregion
		}

		keypressed[5] = 1;
		num_kyp = 6;
	}
	else {keypressed[5] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if (GetAsyncKeyState(F_KY) && kyp_ready)//ENABLE/DISABLE PRINTING
#else
	if (c == F_KY && kyp_ready)
#endif
	{
		if (keypressed[6] == 0)
		{
			kyp_delay = 1;
			if (PRINT == 1)
			{
				PRINT = 0;
				printf("AFFICHAGE INACTIF \n");
			}
			else
			{
				PRINT = 1;
				printf("AFFICHAGE ACTIVF \n");
			}
		}

		keypressed[6] = 1;
		num_kyp = 7;
	}
	else {keypressed[6] = 0;}
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(add_KY) && kyp_ready))//UP ARROW --> INCREASE EMG THRESHOLD
#else
	if (c == add_KY && kyp_ready)
#endif
	{
		if (keypressed[7] == 0)
		{
			kyp_delay = 1;
			if (CTRL_IMU.EMG_1.SENSOR_ON)
			{
				if (CTRL_IMU.EMG_1.STD_TKE == 0)
					CTRL_IMU.EMG_1.STD_TKE = 1;
				else
					CTRL_IMU.EMG_1.K_TH++;
				printf("threshold increased...");
				CTRL_IMU.EMG_1.NEW_CALIB = 1;
			}
			keypressed[7] = 1;
			num_kyp = 8;
		}
	}
	else { keypressed[7] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(sub_KY) && kyp_ready))//DOWN ARROW -->	DECREASE EMG THRESHOLD
#else
	if (c == sub_KY && kyp_ready)
#endif
	{
		if (keypressed[8] == 0)
		{
			kyp_delay = 1;
			if (CTRL_IMU.EMG_1.SENSOR_ON)
			{
				if (CTRL_IMU.EMG_1.K_TH == 1 && CTRL_IMU.EMG_1.STD_TKE != 0)
				{
					CTRL_IMU.EMG_1.STD_TKE--;
					CTRL_IMU.EMG_1.NEW_CALIB = 1;
					printf("threshold decreased...");
				}
				else if (CTRL_IMU.EMG_1.K_TH != 1)
				{
					CTRL_IMU.EMG_1.K_TH--;
					CTRL_IMU.EMG_1.NEW_CALIB = 1;
					printf("threshold decreased...");
				}
				else
					printf("not valid...");
			}
			keypressed[8] = 1;
			num_kyp = 9;
		}
	}
	else { keypressed[8] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32	
	if ((GetAsyncKeyState(up_arrow) && kyp_ready))//MAX FORTH CALIBRATION
#else
	if (c == up_arrow && kyp_ready)
#endif
	{
		if (keypressed[9] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("MAX FORWARD CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_FORTH;
			}
			keypressed[9] = 1;
			num_kyp = 10;
		}
	}
	else { keypressed[9] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(down_arrow) && kyp_ready))//MAX BACK CALIBRATION
#else
	if (c == down_arrow && kyp_ready)
#endif
	{
		if (keypressed[10] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("MAX BACKWARD CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_BACK;
			}
			keypressed[10] = 1;
			num_kyp = 11;
		}
	}
	else { keypressed[10] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(right_arrow) && kyp_ready))//MAX RIGHT CALIBRATION
#else
	if (c == right_arrow && kyp_ready)
#endif
	{
		if (keypressed[11] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("MAX RIGHT CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_RIGHT;
			}
			keypressed[11] = 1;
			num_kyp = 12;
		}
	}
	else { keypressed[11] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(left_arrow) && kyp_ready))//MAX LEFT CALIBRATION
#else
	if (c == left_arrow && kyp_ready)
#endif
	{
		if (keypressed[12] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("MAX LEFT CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_LEFT;
			}
			keypressed[12] = 1;
			num_kyp = 13;
		}
	}
	else { keypressed[12] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32
	if ((GetAsyncKeyState(numpad0) && kyp_ready))//NEUTRAL IMU CALIBRATION
#else
	if (c == numpad0 && kyp_ready)
#endif
	{
		if (keypressed[13] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("NEUTRAL IMU CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = NEUTRAL;
			}
			keypressed[13] = 1;
			num_kyp = 12;
		}
	}
	else { keypressed[13] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32	
	if ((GetAsyncKeyState(numpad4) && kyp_ready))//NEUTRAL IMU CALIBRATION
#else
	if (c == numpad4 && kyp_ready)
#endif
	{
		if (keypressed[14] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("ROT MAX LEFT CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_ROT_LEFT;
			}
			keypressed[14] = 1;
			num_kyp = 15;
		}
	}
	else { keypressed[14] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32	
	if ((GetAsyncKeyState(numpad6) && kyp_ready))//NEUTRAL IMU CALIBRATION
#else
	if (c == numpad6 && kyp_ready)
#endif
	{
		if (keypressed[15] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("ROT MAX RIGHT CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = MAX_ROT_RIGHT;
			}
			keypressed[15] = 1;
			num_kyp = 16;
		}
	}
	else { keypressed[15] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32	
	if (GetAsyncKeyState(E_KY) && kyp_ready)//Z Control disabled/enabled
#else
	if (c == E_KY && kyp_ready)
#endif
	{
		if (keypressed[16] == 0)
		{
			kyp_delay = 1;
			if (CTRL_IMU.ZActive == 1)
			{
				CTRL_IMU.ZActive = 0;
				printf("Z Control disabled...\n");
			}
			else if (CTRL_IMU.ZActive == 0)
			{
				CTRL_IMU.ZActive = 1;
				printf("Z Control enabled...\n");
			}
		}

		keypressed[16] = 1;
		num_kyp = 17;
	}
	else { keypressed[16] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32	
	if (GetAsyncKeyState(V_KY) && kyp_ready)//Validate control heading offset
#else
	if (c == V_KY && kyp_ready)
#endif	
	{
		if (keypressed[17] == 0)
		{
			CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX = 0;
			if (CTRL_IMU.HEAD_IMU.CALIB_IMU_INDEX != -1)
			{
				PRINT = 0;
				printf("HEADING OFFSET CALIBRATION...\n");
				CTRL_IMU.HEAD_IMU.CALIB_TYPE = HEADING_OFFSET;
				SENSOR_HEADING_COMPUTATION;
				if (CTRL_IMU.CHAIR_IMU.SENSOR_ON)
					CTRL_IMU.ReferenceActive = 1;
			}
			keypressed[17] = 1;
			num_kyp = 18;
		}
	}
	else { keypressed[17] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32	
	if ((GetAsyncKeyState(numpad8) && kyp_ready))//LOWER_THRESHOLD INCREASE
#else
	if (c == numpad8 && kyp_ready)
#endif
	{
		if (keypressed[18] == 0)
		{
			if (CTRL_IMU.EMG_1.SENSOR_ON)
			{
				if (CTRL_IMU.EMG_1.LOWER_THRESHOLD + 1 < CTRL_IMU.EMG_1.UPPER_THRESHOLD)
				{
					CTRL_IMU.EMG_1.LOWER_THRESHOLD += 1;
					printf("threshold increased...");
					CTRL_IMU.EMG_1.NEW_CALIB = 1;
				}
				else
					printf("not valid...");
			}
			keypressed[18] = 1;
			num_kyp = 19;
		}
	}
	else { keypressed[18] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32	
	if ((GetAsyncKeyState(numpad2) && kyp_ready))//LOWER_THRESHOLD DECREASE
#else
	if (c == numpad2 && kyp_ready)
#endif	
	{
		if (keypressed[19] == 0)
		{
			if (CTRL_IMU.EMG_1.SENSOR_ON)
			{
				if (CTRL_IMU.EMG_1.LOWER_THRESHOLD - 1 >= 0)
				{
					CTRL_IMU.EMG_1.LOWER_THRESHOLD -= 1;
					printf("threshold decreased...");
					CTRL_IMU.EMG_1.NEW_CALIB = 1;
				}
				else
					printf("not valid...");
			}
			keypressed[19] = 1;
			num_kyp = 20;
		}
	}
	else { keypressed[19] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32	
	if ((GetAsyncKeyState(numpad9) && kyp_ready))//UPPER_THRESHOLD INCREASE
#else
	if (c == numpad9 && kyp_ready)
#endif	
	{
		if (keypressed[20] == 0)
		{
			if (CTRL_IMU.EMG_1.SENSOR_ON)
			{
				CTRL_IMU.EMG_1.UPPER_THRESHOLD += 1;
				printf("upper-threshold increased...");
				CTRL_IMU.EMG_1.NEW_CALIB = 1;
			}
			keypressed[20] = 1;
			num_kyp = 21;
		}
	}
	else { keypressed[20] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32	
	if ((GetAsyncKeyState(numpad3) && kyp_ready))//UPPER_THRESHOLD DECREASE
#else
	if (c == numpad3 && kyp_ready)
#endif	
	{
		if (keypressed[21] == 0)
		{
			if (CTRL_IMU.EMG_1.SENSOR_ON)
			{
				if (CTRL_IMU.EMG_1.UPPER_THRESHOLD - 1 > CTRL_IMU.EMG_1.LOWER_THRESHOLD)
				{
					CTRL_IMU.EMG_1.UPPER_THRESHOLD -= 1;
					printf("upper-threshold decreased...\n");
					CTRL_IMU.EMG_1.NEW_CALIB = 1;
				}
				else
					printf("not valid...");
			}
			keypressed[21] = 1;
			num_kyp = 20;
		}
	}
	else { keypressed[21] = 0; }
	/*__________________________________________________________________________________*/
#ifdef _WIN32	
	if ((GetAsyncKeyState(W_KY) && kyp_ready))//Toggle Z control
#else
	if (c == W_KY && kyp_ready)
#endif	
	{
		if (keypressed[22] == 0)
		{
			//PLOT_DATA = 0;
			if (CTRL_IMU.ZActive)
			{
				CTRL_IMU.ZActive = 0;
				printf("YAW CONTROL DISABLED...\n");
			}
			else
			{
				CTRL_IMU.ZActive = 1;
				printf("YAW CONTROL ENABLED...\n");
			}
			keypressed[22] = 1;
			num_kyp = 21;
		}
	}
	else { keypressed[22] = 0; }
	return num_kyp;
}

#ifdef _WIN32
int ReadJACOMode(void)
{
	int status_jaco, mode_jaco;
	mode_jaco = 0;
	ControlMappingCharts MappingJACO;
	(*MyGetControlMapping)(MappingJACO);
	mode_jaco = MappingJACO.Mapping[2].ActualModeA;
	return mode_jaco;
}
#endif

int START_JACO_ARM()
{
	int result;
	ControlJACO = 1;
#ifdef _WIN32	
	//1st step = creer, instancier, activer les instances de contrôle de JACO
	//commandLayer_handle = LoadLibrary(L"Kinova.API.UsbCommandLayer.dll");
	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");
	if (commandLayer_handle == NULL)
		printf("Error while loading library. Cannot perform test. Leaving... \n");
	else
		printf("API loaded Successfully... \n");

	//Initialise the function pointer from the API
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
	MySendJoystickCommand = (int(*)(JoystickCommand)) GetProcAddress(commandLayer_handle, "SendJoystickCommand");
	MySetCartesianControl = (int(*)()) GetProcAddress(commandLayer_handle, "SetCartesianControl");
	MyStartControlAPI = (int(*)()) GetProcAddress(commandLayer_handle, "StartControlAPI");
	MyStopControlAPI = (int(*)()) GetProcAddress(commandLayer_handle, "StopControlAPI");
	MyGetQuickStatus = (int(*)(QuickStatus &)) GetProcAddress(commandLayer_handle, "GetQuickStatus");
	MyGetControlMapping = (int(*)(ControlMappingCharts &)) GetProcAddress(commandLayer_handle, "GetControlMapping");
	//Verify that all functions has been loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MyMoveHome == NULL) || (MySendJoystickCommand == NULL) || (MyStartControlAPI == NULL) || (MyStopControlAPI == NULL) || (MySetCartesianControl == NULL) || (MyGetQuickStatus == NULL) || (MyGetControlMapping == NULL))
		printf("Cannot load all the functions from the API. Please reinit the system...\n");
	else
	{
		result = (*MyInitAPI)();
		if (result == 1)
		{
			printf("The result of InitAPI = 1 \n");
			(*MyMoveHome)();						// Puts JACO in init position
			(*MySetCartesianControl)();
			(*MyStartControlAPI)();
			printf("JACO is ready... have fun ;) \n");
		}
		else
			printf("The result of InitAPI != 1 \n");
	}
#else
	KinDrv::init_usb();
	printf("Create a JacoArm \n");
	try {
		arm = new JacoArm();
		printf("Successfully connected to JACO arm! \n");
	}
	catch (KinDrvException &e) {
		printf("error %i: %s \n", e.error(), e.what());
		return 0;
	}

	printf("Gaining API control over the arm \n");
	arm->start_api_ctrl();

	//check if we need to initialize arm
	jaco_retract_mode_t mode = arm->get_status();
	printf("Arm is currently in state: %i \n", mode);
	if (mode == MODE_NOINIT) {
		//push the "HOME/RETRACT" button until arm is initialized
		jaco_joystick_button_t buttons = { 0 };
		buttons[2] = 1;
		arm->push_joystick_button(buttons);

		while (mode == MODE_NOINIT) {
			usleep(1000 * 10); // 10 ms
			mode = arm->get_status();
		}

		arm->release_joystick();
	}
	printf("Arm is initialized now, state: %i \n", mode);

	// we want to start from home_position
	goto_home(arm);

	arm->set_control_cart();
	result = mode;
#endif
	return result;
}

int STOP_JACO_ARM()
{
	ControlJACO = 0;
#ifdef _WIN32
	(*MySendJoystickCommand)(CTRL_IMU.CommandJACO(0));
	(*MyMoveHome)();
	(*MyStopControlAPI)();
	int result = (*MyCloseAPI)();
	Console::WriteLine("JACO's control has been stopped...");
	//serialPort1->ReadExisting();
#else
	//STOP API
	printf("JACO's control has been stopped...\n");
	ControlJACO = 0;
#endif
	return result;
}

int SET_JACO_ARM()
{
	if (ControlJACO == 1)
	{
		STOP_JACO_ARM();
	}
	else
	{
		START_JACO_ARM();
	}
	return ControlJACO;
}

/*void timer_start(std::function<void(void)> func, unsigned int interval)
{
	std::thread([func, interval]() {
		while (true)
		{
			func();
			std::this_thread::sleep_for(std::chrono::milliseconds(interval));
		}
	}).detach();
}


void refresh_jaco ()
{
	test_timer_jaco++;
}*/




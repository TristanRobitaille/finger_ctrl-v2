#pragma once
#include <math.h>
#include "Defined_Macro.h"

namespace Control_IMU_JACO
{
	class EMG_Sensor
	{
	public:

		char SENSOR_ON;
		int FREQ;

		//last update : Fs_EMG = 1125 Hz
		static const double num_bs_55_65Hz[5]; static const double den_bs_55_65Hz[5];
		static const double num_bp_80_450Hz[5]; static const double den_bp_80_450Hz[5];
		static const double num_bp_20_450Hz[5]; static const double den_bp_20_450Hz[5];

		//DATA EMG
		char index_EMG;
		static const int NB_SAMPLES_IN_PCKT = 9;
		float DATA_EMG[NB_SAMPLES_IN_PCKT];
		float DATA_EMG_FILT[NB_SAMPLES_IN_PCKT];
		double x_emg[5]; double y_emg[5]; double z_emg[5]; //FILTER

		//DATA TKE + ENV
		float DATA_TKE[NB_SAMPLES_IN_PCKT];
		static const int NB_SAMPLES_IN_WINDOW = 81;
		static const int NB_SAMPLES_OVERLAP = 45;
		int INDEX_WINDOW;
		float DATA_ENV[NB_SAMPLES_IN_WINDOW];
		float MEAN_TKE_CURRENT;
		double emg[3];
		char NEW_TKE;

		//DATA CALIB + DETECT
		float MEAN_TKE; float STD_TKE; int K_TH; float THRESHOLD;
		float UPPER_THRESHOLD; float LOWER_THRESHOLD;
		static const int NB_SAMPLES_CALIB = 100 * NB_SAMPLES_IN_PCKT;
		float DATA_TKE_CALIB[NB_SAMPLES_CALIB];
		int CALIB_EMG_INDEX;
		char EMG_DETECTED;
		static const int nb_detect = 200;
		char EMG_BUTTON[nb_detect];
		char i_button;
		int delay_detect, time_contraction, long_contraction_threshold, very_long_contraction_threshold;
		char NEW_CALIB, EMG_CALIBRATED;


		EMG_Sensor();
		~EMG_Sensor();
		void initialize(void);
		void read_new_data(int data[]);
		void filter_emg(void);
		void process_tke(void);
		void detect_tke(int nb);
		void trigger_detect_tke(int nb);
		float absolute(float var);
		float mean_tabf(float * tab, int taille);
		float mean_tabi(int * tab, int taille);
		float C2_3(int var);
	};
}

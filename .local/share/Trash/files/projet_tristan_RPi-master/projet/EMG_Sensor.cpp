#include "EMG_Sensor.h"

namespace Control_IMU_JACO
{
	//Fs_EMG = 1125 Hz
	const double EMG_Sensor::den_bp_20_450Hz[5] = { 1.0, -0.706332727403145, -0.823332488588912, 0.205925223216761, 0.354276046972334 };
	const double EMG_Sensor::num_bp_20_450Hz[5] = { 0.588587106635191, 0.0, -1.17717421327038, 0.0, 0.588587106635191 };

	const double EMG_Sensor::den_bp_80_450Hz[5] = { 1.0, -0.250198547923143, -0.566207313245305, 0.0213077489872791, 0.233041463693116 };
	const double EMG_Sensor::num_bp_80_450Hz[5] = { 0.454876458653252, 0.0, -0.909752917306504, 0.0, 0.454876458653252 };

	const double EMG_Sensor::den_bs_55_65Hz[5] = { 1.0, -3.70439515632780, 5.35297699455272, -3.56089634373912, 0.924054185873028 };
	const double EMG_Sensor::num_bs_55_65Hz[5] = { 0.961277079385909, -3.63264575003346, 5.35447702165393, -3.63264575003346, 0.961277079385909 };

	EMG_Sensor::EMG_Sensor()
	{}

	EMG_Sensor::~EMG_Sensor()
	{}

	void EMG_Sensor::initialize(void)
	{
		index_EMG = 0;
		INDEX_WINDOW = 0;
		NEW_TKE = 0;
		MEAN_TKE_CURRENT = 0;
		MEAN_TKE = 0.0; STD_TKE = 0.0; K_TH = 3.0;
		THRESHOLD = 0; UPPER_THRESHOLD = 0; LOWER_THRESHOLD = 0;
		CALIB_EMG_INDEX = -1;
		EMG_DETECTED = 0;
		i_button = 0;
		NEW_CALIB = 0;
		EMG_CALIBRATED = 0;
		delay_detect = 0;
		time_contraction = 0;
		long_contraction_threshold = 80;
		very_long_contraction_threshold = 200;
	}

	void EMG_Sensor::read_new_data(int data[])
	{
		SENSOR_ON = 1;
		if (data[1] != 0)
		{
			FREQ = 1000 / data[1];											// MSP430 Sensor Acquisition Frequency
		}

		for (int i = 1; i <= NB_SAMPLES_IN_PCKT; i++)
		{
			int j = (3 * i);
			DATA_EMG[i - 1] = C2_3((data[j] << 16) + (data[j + 1] << 8) + (data[j + 2])) / 1000;
			index_EMG = index_EMG + 1;
		}
	}

	void EMG_Sensor::filter_emg()
	{
		for (int i = 0; i < NB_SAMPLES_IN_PCKT; i++)
		{
			x_emg[0] = DATA_EMG[i];
			//y_emg[0] = x_emg[0];
			// bs filter
			y_emg[0] = num_bs_55_65Hz[0] * x_emg[0] + num_bs_55_65Hz[1] * x_emg[1] + num_bs_55_65Hz[2] * x_emg[2] + num_bs_55_65Hz[3] * x_emg[3] + num_bs_55_65Hz[4] * x_emg[4] - (den_bs_55_65Hz[1] * y_emg[1] + den_bs_55_65Hz[2] * y_emg[2] + den_bs_55_65Hz[3] * y_emg[3] + den_bs_55_65Hz[4] * y_emg[4]);
			x_emg[4] = x_emg[3]; x_emg[3] = x_emg[2]; x_emg[2] = x_emg[1]; x_emg[1] = x_emg[0];
			//y_emg[4] = y_emg[3]; y_emg[3] = y_emg[2]; y_emg[2] = y_emg[1]; y_emg[1] = y_emg[0];
			// bp filter
			z_emg[0] = num_bp_20_450Hz[0] * y_emg[0] + num_bp_20_450Hz[1] * y_emg[1] + num_bp_20_450Hz[2] * y_emg[2] + num_bp_20_450Hz[3] * y_emg[3] + num_bp_20_450Hz[4] * y_emg[4] - (den_bp_20_450Hz[1] * z_emg[1] + den_bp_20_450Hz[2] * z_emg[2] + den_bp_20_450Hz[3] * z_emg[3] + den_bp_20_450Hz[4] * z_emg[4]);
			y_emg[4] = y_emg[3]; y_emg[3] = y_emg[2]; y_emg[2] = y_emg[1]; y_emg[1] = y_emg[0];
			z_emg[4] = z_emg[3]; z_emg[3] = z_emg[2]; z_emg[2] = z_emg[1]; z_emg[1] = z_emg[0];

			DATA_EMG_FILT[i] = z_emg[0];
		}
	}

	void EMG_Sensor::process_tke()
	{
		int var;
		for (int itke = 0; itke < NB_SAMPLES_IN_PCKT; itke++)
		{
			emg[2] = DATA_EMG_FILT[itke] * 5;
			DATA_TKE[itke] = absolute(emg[1] * emg[1] - emg[0] * emg[2]);
			emg[0] = emg[1];
			emg[1] = emg[2];

			DATA_ENV[INDEX_WINDOW] = DATA_TKE[itke];
			//var = DATA_TKE[itke];
			INDEX_WINDOW += 1;
			if (INDEX_WINDOW >= NB_SAMPLES_IN_WINDOW)
			{
				MEAN_TKE_CURRENT = mean_tabf(DATA_ENV, NB_SAMPLES_IN_WINDOW);
				int var_nb = NB_SAMPLES_IN_WINDOW - NB_SAMPLES_OVERLAP;
				for (int j = 0; j < NB_SAMPLES_OVERLAP; j++)
					DATA_ENV[j] = DATA_ENV[var_nb + j];
				INDEX_WINDOW = NB_SAMPLES_OVERLAP;
				NEW_TKE = 1;
			}

			if (CALIB_EMG_INDEX != -1 && CALIB_EMG_INDEX != -2)
			{
				DATA_TKE_CALIB[CALIB_EMG_INDEX] = DATA_TKE[itke];
				CALIB_EMG_INDEX++;
				if (CALIB_EMG_INDEX >= NB_SAMPLES_CALIB)
				{
					CALIB_EMG_INDEX = -2;
					MEAN_TKE = mean_tabf(DATA_TKE_CALIB, NB_SAMPLES_CALIB);
					double SUM_STD = 0;
					for (int h = 0; h < NB_SAMPLES_CALIB; h++)
						SUM_STD += (DATA_TKE_CALIB[h] - MEAN_TKE)*(DATA_TKE_CALIB[h] - MEAN_TKE);
					STD_TKE = sqrt(SUM_STD / NB_SAMPLES_CALIB);
					THRESHOLD = MEAN_TKE + K_TH*STD_TKE;
					UPPER_THRESHOLD = THRESHOLD*1.2;
					LOWER_THRESHOLD = THRESHOLD*0.8;
					NEW_CALIB = 1;
					EMG_CALIBRATED = 1;
				}
			}
		}
	}

	void EMG_Sensor::detect_tke(int nb)
	{
		if (delay_detect != 0)
			delay_detect++;
		if (delay_detect == nb)
			delay_detect = 0;

		if ((MEAN_TKE_CURRENT >= THRESHOLD) && delay_detect == 0)
		{
			EMG_DETECTED = -1;
		}
		if ((MEAN_TKE_CURRENT < THRESHOLD))
		{
			if (EMG_DETECTED == -1 && delay_detect == 0)
			{
				EMG_DETECTED = 1;
				delay_detect = 1;
			}
			else
				EMG_DETECTED = 0;
		}
	}

	void EMG_Sensor::trigger_detect_tke(int nb)
	{
		if (delay_detect != 0)
			delay_detect++;
		if (delay_detect == nb)
			delay_detect = 0;

		if ((MEAN_TKE_CURRENT >= UPPER_THRESHOLD) && delay_detect == 0)
		{
			EMG_DETECTED = -1;
			if (time_contraction >= 0)
				START_CONTRACTION_CHRONO;
		}
		else if ((MEAN_TKE_CURRENT < LOWER_THRESHOLD))
		{
			if (EMG_DETECTED == -1 && delay_detect == 0)
			{
				EMG_DETECTED = 1;
				delay_detect = 1;
				/*if (time_contraction >= long_contraction_threshold)
					LONG_CONTRACTION_DETECTED;
					else
					SHORT_CONTRACTION_DETECTED;*/
				if (time_contraction >= very_long_contraction_threshold)
					VERY_LONG_CONTRACTION_DETECTED;
				else if (time_contraction >= long_contraction_threshold)
					LONG_CONTRACTION_DETECTED;
				else if (time_contraction < long_contraction_threshold)
					SHORT_CONTRACTION_DETECTED;
			}
			else
			{
				EMG_DETECTED = 0;
				time_contraction = 0;
			}
		}
	}

	float EMG_Sensor::absolute(float var)
	{
		if (var <= 0)
			return -var;
		else
			return var;
	}

	float EMG_Sensor::mean_tabf(float * tab, int taille)
	{
		int g; float sum = 0.0;
		for (g = 0; g < taille; g++)
			sum = sum + tab[g];
		return (sum / taille);
	}

	float EMG_Sensor::mean_tabi(int * tab, int taille)
	{
		int g; float sum = 0.0;
		for (g = 0; g < taille; g++)
			sum = sum + tab[g];
		return (sum / taille);
	}

	float EMG_Sensor::C2_3(int var)
	{
		float var1, var2;
		int mask = 8388607; //mask 0x7FFF
		var1 = var & mask;
		var2 = (var >> 23)*(-8388608);
		return (var1 + var2);
	}
}

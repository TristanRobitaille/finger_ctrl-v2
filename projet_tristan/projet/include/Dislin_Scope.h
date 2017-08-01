#pragma once
#include <iostream> 
#include "dislin.h"
#include "discpp.h"
#include "Defined_Macro.h"

namespace Control_IMU_JACO
{
	class Dislin_Scope
	{
	public:

		Dislin diag;
		char diag_type;
		int nb_data_to_plot;
		int nb_data_scope;
		int nb_plotted;
		int var_plotted;
		int nb2_plotted;
		int var2_plotted;
		int lowy, highy, stepy;

		//imu scope
		float *pitch_window = NULL;
		float *roll_window = NULL;
		float *yaw_window = NULL;
		float *ximu_window = NULL;
		float *pitch_to_plot = NULL;
		float *roll_to_plot = NULL;
		float *yaw_to_plot = NULL;
		float *ximu_to_plot = NULL;

		float *xref_to_plot = NULL;
		float *yawref_to_plot = NULL;
		float *xref_window = NULL;
		float *yawref_window = NULL;

		//emg scope
		float *emg_window = NULL;
		float *xemg_window = NULL;
		float *emg_to_plot = NULL;
		float *xemg_to_plot = NULL;

		float *emg2_window = NULL;
		float *xemg2_window = NULL;
		float *emg2_to_plot = NULL;
		float *xemg2_to_plot = NULL;

		//tke scope
		float *tke_window = NULL;
		float *xtke_window = NULL;
		float *tke_to_plot = NULL;
		float *xtke_to_plot = NULL;
		float *THtke_window = NULL;
		float *THuptke_window = NULL;
		float *THlowtke_window = NULL;
		float *xTHtke = NULL;

		Dislin_Scope();
		~Dislin_Scope();
		void initialize(char scope_type, char scope_level, int scope_size, int size_plot);
		void initialize(char scope_type, char scope_level, int scope_size, int size_plot, int nb_curves);
		void update(float *new_plot, int size_new);
		void update(float *new_plot, int size_new, char signal_id);
		void update(float new_plot, int size_new, char signal_id);
		void update(float threshold_up, float threshold_low, int size_new, char signal_id);
	};
}

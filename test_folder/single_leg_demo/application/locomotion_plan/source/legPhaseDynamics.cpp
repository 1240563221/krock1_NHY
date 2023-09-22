#include "single_leg_controller.hpp"

void Locomotion_controller::legPhaseDynamics(double t)
{
  static MatrixXd legCPGtheta=MatrixXd::Ones(4,2)*0.5*2*my_pi; 

    //===================================== LEG CPG ===================================================
	for(int i=0; i<4; i++){
        legPhase[i] = GP1.freq_walk*t+GP1.phShifts(i);
        legPhase[i]=fmod(legPhase[i], 1.);
    }

    //============================== DETECT STANCE AND SWING TRANSITIONS =============================
    for(int i=0; i<4; i++){
        if(legPhase[i]<GP1.Duty)
        {
            legs_stance(i)=1;
        }
        else{
            legs_stance(i)=0;
        }
    }
    legs_stance_old=legs_stance;
}
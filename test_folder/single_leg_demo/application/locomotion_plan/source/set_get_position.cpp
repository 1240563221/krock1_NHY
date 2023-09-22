#include "single_leg_controller.hpp"

void Locomotion_controller::getposition(double *positon_input)
{
  // get motor position:
  for (int i = 0; i < P_MAX_ID; i++) {
      q_input(i) = positon_input[i];
      printf("position[%d]:%f\n",i,q_input(i));
  }
}

void Locomotion_controller::calcMotorPosition()
{
  for(int i=0; i<LEG_NUM; i++){
    q_output.block<4,1>(4*i,0) = calcLegAngles.block<4,1>(0,i);
  }
    q_output.block<3,1>(16, 0) = q0_trunk_from_spline;
}

void Locomotion_controller::setposition(double *position_output)
{
      // set MotorPosition:
    for(int i=0; i< P_MAX_ID; i++){
        if(i == 0 )
        {
          position_output[i] = -q_output(i);
          cout << "refLocation[" << i << "]: " << -q_output(i)<< endl;
        }
        else
        {
          position_output[i] = q_output(i);
          cout << "refLocation[" << i << "]: " << q_output(i)<< endl;
        }
    }
}

Matrix<double,10,1> Locomotion_controller::get_traj_log()
{
  static Matrix<double,10,1> traj;
  traj_log.block(0,0,3,1) = feetReference.block(0,0,3,1);
  traj_log.block(3,0,4,1) = q_output.block(0,0,4,1);
  traj_log.block(7,0,3,1) = p_cur;
  return traj;
}
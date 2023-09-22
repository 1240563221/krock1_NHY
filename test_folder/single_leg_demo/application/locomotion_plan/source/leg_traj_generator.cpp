#include "single_leg_controller.hpp"

void Locomotion_controller::leg_traj_generator()
{
  for(int i=0; i<LEG_NUM; i++){
		  if(legs_stance(i)==0){
            
        moveSwingLeg(i);
		  }
      else{
        moveStanceLeg(i);
      }
	  }

    // for(int i=0;i<LEG_NUM;i++)
    // {
    // feetReference_G.block(0,i,3,1) = AngleAxisd(-girdleCpgOutput(i/2), Vector3d::UnitZ())*feetReference.block(0,i,3,1);
    // }
    //need to recalculate
    calcLegAngles = leginversekinematics();
}

/* Takes care of a swing phase of the swinging leg */
bool Locomotion_controller::moveSwingLeg(int leg_index)
{
    static Vector3d trajPoint;
    static MatrixXd initPoint(3,4), middlePoint(3,4), finalPoint(3,4);
    //feet end point position in the girdle coordinate, so Fgird and Hgird are for, not be fixed to body

    //state transition from stance to swing
    initPoint.block<3,1>(0,leg_index) << GP1.midstance(0,leg_index)-GP1.ellipse_a, GP1.midstance(1,leg_index), GP1.midstance(2,leg_index);
    finalPoint.block<3,1>(0,leg_index) << GP1.midstance(0,leg_index)+GP1.ellipse_a, GP1.midstance(1,leg_index), GP1.midstance(2,leg_index);

    middlePoint.block<3,1>(0,leg_index)=GP1.midstance.block<3,1>(0,leg_index);
    middlePoint(2,leg_index) += GP1.swing_height(leg_index);

    //calc swing phase, see swing as a complete circloe
    swingPhase(leg_index)=(legPhase[leg_index]-GP1.Duty)/(1.-GP1.Duty);

    
    trajPoint=getSwingTrajectory(initPoint.block<3,1>(0,leg_index), middlePoint.block<3,1>(0,leg_index), finalPoint.block<3,1>(0,leg_index), swingPhase(leg_index));
    
    
    if(leg_index>1){
        trajPoint(0) += -IG-Hgird(0, 3);
    }    
    // get feetreference in girdle body frame
    feetReference.block<3,1>(0,leg_index) = trajPoint;

    if(swingPhase(leg_index)>=1){
        return true;
    }
    return false;
    
}

Vector3d Locomotion_controller::getSwingTrajectory(Vector3d initPoint, Vector3d middlePoint, Vector3d finalPoint, double phase)
{   
    Matrix<double, 3, 1> Scurve; 
    // cout << "index  "<< leg_index << endl;
    double s = phase*(1.-GP1.Duty);
    double T0 = 1.-GP1.Duty;

    if(phase < 0.5)
    {
        Scurve <<
        initPoint(0) + (finalPoint(0)-initPoint(0))
        * (s / T0 - 1. / 2. / my_pi * sin(2. * my_pi * s / T0)),
        middlePoint(1),
        middlePoint(2)+ GP1.swing_height(0)*
        (2 * s/T0 - 1./2./my_pi*sin(4 *my_pi * s /T0));
    }
    else
    {
        Scurve <<
        initPoint(0) + (finalPoint(0)-initPoint(0)) 
        * (s / T0 - 1. / 2. / my_pi * sin(2 * my_pi * s / T0)),
        middlePoint(1),
        middlePoint(2)+ GP1.swing_height(0)* 
        (2. - 2 * s / T0 + 1./2./my_pi * sin(4 * my_pi * s / T0));
    }

    // Scurve << initPoint(0,leg_index)+(finalPoint(0,leg_index)-initPoint(0,leg_index))*phase,
    //             initPoint(1,leg_index)+(finalPoint(1,leg_index)-initPoint(1,leg_index))*phase,
    //             GP1.midstance(2,leg_index)+abs(GP1.swing_height(leg_index)*sin(phase*my_pi));
    return Scurve;

} 

/* Takes care of a stance phase of the stancing leg */
bool Locomotion_controller::moveStanceLeg(int leg_index)
{
    static Vector3d trajPoint;
    static Matrix<double,3,4> initPoint, middlePoint, finalPoint;
    //midstance, stanceStart, stanceEstEnd;
    middlePoint = GP1.midstance;
    finalPoint.block(0,leg_index,3,1) << GP1.midstance(0,leg_index)-GP1.ellipse_a, GP1.midstance(1,leg_index), GP1.midstance(2,leg_index);
    initPoint.block(0,leg_index,3,1) <<  GP1.midstance(0,leg_index)+GP1.ellipse_a, GP1.midstance(1,leg_index), GP1.midstance(2,leg_index);

    stancePhase(leg_index)=legPhase[leg_index]/GP1.Duty;

    trajPoint <<
    initPoint(0,leg_index) + (finalPoint(0,leg_index)-initPoint(0,leg_index)) * 
    (stancePhase(leg_index) - 1./2./ my_pi *sin(2 * my_pi * stancePhase(leg_index))),
    middlePoint(1,leg_index),
    middlePoint(2,leg_index);

    // trajPoint=initPoint.block(0, leg_index, 3,1) + stancePhase(leg_index) * (finalPoint.block(0, leg_index, 3,1) - initPoint.block(0, leg_index, 3,1));
        
    if(leg_index>1){
        trajPoint(0) += -IG-Hgird(0, 3);
    }

    // get feetreference in girdle body frame
    feetReference.block<3,1>(0,leg_index) = trajPoint;  

    if(stancePhase(leg_index)>=1){
        return true;
    }
    return false;
    
}
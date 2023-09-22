#include "single_leg_controller.hpp"

void Locomotion_controller::forward_kinematics()
{
    static MatrixXd feetLocations_old(3,4);
    // Front girdle
    Fgird.setZero();
    Fgird.block<3,3>(0, 0)=MatrixXd::Identity(3,3);
    Fgird(3,3)=1;

    //Trunk,calculate hgirdle frame-from front to hind
    Rtr0<<  cos(q_input(16)), -sin(q_input(16)), 0, -l_trunk(0),
              sin(q_input(16)), cos(q_input(16)), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    Rtr_g = Rtr0;

    Rtr1<<cos(q_input(17)), -sin(q_input(17)), 0, -l_trunk(1),
            sin(q_input(17)), cos(q_input(17)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Rtr_g=Rtr_g*Rtr1;

    Rtr2<<cos(q_input(18)), -sin(q_input(18)), 0, -l_trunk(2),
            sin(q_input(18)), cos(q_input(18)), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    Rtr_g=Rtr_g*Rtr2;

    Rtr3<<1, 0, 0, -l_trunk(number_trunkmotor),
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;   
    Rtr_g=Rtr_g*Rtr3;
    //key calc, get hgirdle frame
    Hgird=Rtr_g;      
}


std::vector<Matrix<double, 4, 4>> Locomotion_controller::legKinematics(Vector4d q, int leg)
{

    static vector<Matrix4d> H(5);

    if(leg==0){     //FL
        H[0] << cos(q(0)), -sin(q(0)), 0, length_FL(0,0),
                sin(q(0)), cos(q(0)), 0, length_FL(0,1),
                0, 0, 1, length_FL(0,2),
                0, 0, 0, 1;
        H[1] << 1, 0, 0, length_FL(1,0),
                0, cos(q(1)), -sin(q(1)), length_FL(1,1),
                0, sin(q(1)), cos(q(1)), length_FL(1,2),
                0, 0, 0, 1;
        H[2] << cos(q(2)), 0, sin(q(2)), length_FL(2,0),
                0,  1, 0, length_FL(2,1),
                -sin(q(2)), 0, cos(q(2)), length_FL(2,2),
                0, 0, 0, 1;
        H[3] << 1, 0, 0, length_FL(3,0),
                0, cos(q(3)), -sin(q(3)), length_FL(3,1),
                0, sin(q(3)), cos(q(3)), length_FL(3,2),
                0, 0, 0, 1;
        H[4] << 1, 0, 0, length_FL(4,0),
                0, 1, 0, length_FL(4,1),
                0, 0, 1, length_FL(4,2),
                0, 0, 0, 1;
    }

    else if (leg==1){       //FR
        H[0] << cos(q(0)), -sin(q(0)), 0, length_FR(0,0),
                sin(q(0)), cos(q(0)), 0, length_FR(0,1),
                0, 0, 1, length_FR(0,2),
                0, 0, 0, 1;
        H[1] << 1, 0, 0, length_FR(1,0),
                0, cos(q(1)), -sin(q(1)), length_FR(1,1),
                0, sin(q(1)), cos(q(1)), length_FR(1,2),
                0, 0, 0, 1;
        H[2] << cos(q(2)), 0, sin(q(2)), length_FR(2,0),
                0,  1, 0, length_FR(2,1),
                -sin(q(2)), 0, cos(q(2)), length_FR(2,2),
                0, 0, 0, 1;
        H[3] << 1, 0, 0, length_FR(3,0),
                0, cos(q(3)), -sin(q(3)), length_FR(3,1),
                0, sin(q(3)), cos(q(3)), length_FR(3,2),
                0, 0, 0, 1;
        H[4] << 1, 0, 0, length_FR(4,0),
                0, 1, 0, length_FR(4,1),
                0, 0, 1, length_FR(4,2),
                0, 0, 0, 1;
    }
    else if (leg==2){       //HL
        H[0] << cos(q(0)), -sin(q(0)), 0, length_HL(0,0),
                sin(q(0)), cos(q(0)), 0, length_HL(0,1),
                0, 0, 1, length_HL(0,2),
                0, 0, 0, 1;
        H[1] << 1, 0, 0, length_HL(1,0),
                0, cos(q(1)), -sin(q(1)), length_HL(1,1),
                0, sin(q(1)), cos(q(1)), length_HL(1,2),
                0, 0, 0, 1;
        H[2] << cos(q(2)), 0, sin(q(2)), length_HL(2,0),
                0,  1, 0, length_HL(2,1),
                -sin(q(2)), 0, cos(q(2)), length_HL(2,2),
                0, 0, 0, 1;
        H[3] << 1, 0, 0, length_HL(3,0),
                0, cos(q(3)), -sin(q(3)), length_HL(3,1),
                0, sin(q(3)), cos(q(3)), length_HL(3,2),
                0, 0, 0, 1;
        H[4] << 1, 0, 0, length_HL(4,0),
                0, 1, 0, length_HL(4,1),
                0, 0, 1, length_HL(4,2),
                0, 0, 0, 1;
    } 
    else if (leg==3){       //HR
        H[0] << cos(q(0)), -sin(q(0)), 0, length_HR(0,0),
                sin(q(0)), cos(q(0)), 0, length_HR(0,1),
                0, 0, 1, length_HR(0,2),
                0, 0, 0, 1;
        H[1] << 1, 0, 0, length_HR(1,0),
                0, cos(q(1)), -sin(q(1)), length_HR(1,1),
                0, sin(q(1)), cos(q(1)), length_HR(1,2),
                0, 0, 0, 1;
        H[2] << cos(q(2)), 0, sin(q(2)), length_HR(2,0),
                0,  1, 0, length_HR(2,1),
                -sin(q(2)), 0, cos(q(2)), length_HR(2,2),
                0, 0, 0, 1;
        H[3] << 1, 0, 0, length_HR(3,0),
                0, cos(q(3)), -sin(q(3)), length_HR(3,1),
                0, sin(q(3)), cos(q(3)), length_HR(3,2),
                0, 0, 0, 1;
        H[4] << 1, 0, 0, length_HR(4,0),
                0, 1, 0, length_HR(4,1),
                0, 0, 1, length_HR(4,2),
                0, 0, 0, 1;
    }
    // feet location in girdle body frame
    // return H[0]*H[1]*H[2]*H[3];
    return H;
}

/* Returns jacobian matrix of one leg */
Matrix<double, 3, 4> Locomotion_controller::Jacob(Vector4d q, int leg)
{
    static MatrixXd J(3, 4);
    static double S0, S1, S2, S3, C0, C1, C2, C3;
    S0 = sin(q(0));
    S1 = sin(q(1));
    S2 = sin(q(2));
    S3 = sin(q(3));
    C0 = cos(q(0));
    C1 = cos(q(1));
    C2 = cos(q(2));
    C3 = cos(q(3));

    if(leg==0){     //FL
        J(0) = - l4*(C1*C0*S3 + C3*(-S0*S2 + C2*C0*S1))-C1*l2*C0;
        J(1) = - l4*(-S1*S0*S3 + C3*(C2*S0*C1))+S1*l2*S0;
        J(2) = - l4*(C3*(C0*C2 - S2*S0*S1));
        J(3) = - l4*(C1*S0*C3 - S3*(C0*S2 + C2*S0*S1));
        J(4) = - l4*(S0*C1*S3 + C3*(S0*C2*S1 + C0*S2)) - S0*C1*l2;
        J(5) = - l4*(C0*S1*S3 + C3*(-C0*C2*C1)) - C0*S1*l2;
        J(6) = - l4*(C3*(C0*S2*S1 + S0*C2));
        J(7) = - l4*(-C0*C1*C3 - S3*(-C0*C2*S1 + S0*S2));
        J(8) = 0;
        J(9) = -l4*(-C1*S3-S1*C2*C3) + l2*C1;
        J(10) = -l4*(-C1*S2*C3);
        J(11) = -l4*(-S1*C3-C1*C2*S3);
    }
    else if (leg==1){       //FR
        J(0) = -l4*(C0*C1*S3+(-S0*S2+C0*S1*S2)*C3) + l2*C0*C1;
        J(1) = -l4*(-S0*S1*S3+(S0*C1*S2)*C3) - l2*S0*S1;
        J(2) = -l4*(0 +(C0*C2+S0*S1*C2)*C3);
        J(3) = -l4*(S0*C1*C3 - (C0*S2+S0*S1*S2)*S3);
        J(4) = -l4*(S0*C1*S3+(C0*S2+S0*S1*S2)*C3) + l2*S0*C1 ;
        J(5) = -l4*(C0*S1*S3+(-C0*C1*S2)*C3) + l2*C0*S1 ;
        J(6) = -l4*(0+(S0*C2-C0*S1*C2)*C3) ;
        J(7) = -l4*(-C0*C1*C3-(S0*S2-C0*S1*S2)*S3);
        J(8) = 0;
        J(9) = -l4*(-C1*S3-S1*C2*C3) - l2*C1;
        J(10) = -l4*(-C1*S2*C3);
        J(11) = -l4*(-S1*C3-C1*C2*S3);
    }
    else if (leg==2){       //HL
        J(0) = -l4*(C0*C1*S3+(-S0*S2+C0*S1*S2)*C3) - l2*C0*C1;
        J(1) = -l4*(-S0*S1*S3+(S0*C1*S2)*C3) + l2*S0*S1;
        J(2) = -l4*(0 +(C0*C2+S0*S1*C2)*C3);
        J(3) = -l4*(S0*C1*C3 - (C0*S2+S0*S1*S2)*S3);
        J(4) = -l4*(S0*C1*S3+(C0*S2+S0*S1*S2)*C3) - l2*S0*C1 ;
        J(5) = -l4*(C0*S1*S3+(-C0*C1*S2)*C3) - l2*C0*S1 ;
        J(6) = -l4*(0+(S0*C2-C0*S1*C2)*C3) ;
        J(7) = -l4*(-C0*C1*C3-(S0*S2-C0*S1*S2)*S3);
        J(8) = 0;
        J(9) = -l4*(-C1*S3-S1*C2*C3) + l2*C1;
        J(10) = -l4*(-C1*S2*C3);
        J(11) = -l4*(-S1*C3-C1*C2*S3);
    }
    else if (leg==3){       //HR
        J(0) = -l4*(C0*C1*S3+(-S0*S2+C0*S1*S2)*C3) + l2*C0*C1;
        J(1) = -l4*(-S0*S1*S3+(S0*C1*S2)*C3) - l2*S0*S1;
        J(2) = -l4*(0 +(C0*C2+S0*S1*C2)*C3);
        J(3) = -l4*(S0*C1*C3 - (C0*S2+S0*S1*S2)*S3);
        J(4) = -l4*(S0*C1*S3+(C0*S2+S0*S1*S2)*C3) + l2*S0*C1 ;
        J(5) = -l4*(C0*S1*S3+(-C0*C1*S2)*C3) + l2*C0*S1 ;
        J(6) = -l4*(0+(S0*C2-C0*S1*C2)*C3) ;
        J(7) = -l4*(-C0*C1*C3-(S0*S2-C0*S1*S2)*S3);
        J(8) = 0;
        J(9) = -l4*(-C1*S3-S1*C2*C3) - l2*C1;
        J(10) = -l4*(-C1*S2*C3);
        J(11) = -l4*(-S1*C3-C1*C2*S3);
    }

    return J;
}



/* Solves spine inverse kinematics */
void Locomotion_controller::trunkInverseKinematics()
{
    // ------------------------------------------ RUNNING GIRDLE CPG AND SPLINING STUFF -------------------------------
    // spline parameter
    static MatrixXd t_spline(1, number_trunkmotor+2);
    static MatrixXd p_spline(2, t_spline.size());
    static Vector2d old_solution;

    // spline points and vectors
	double m0_spline, m1_spline;

    //girdleCpgOutput*=(1-abs(gamma));
    //fortraj: absolute frame to get absolute vector m0, m1;
    m0_spline = girdleCpgOutput(0);
    m1_spline = girdleCpgOutput(1);
    // get trunk angles
    // static double const scl=0.4;
    for(int i=0; i<number_trunkmotor; i++){
        q0_trunk_from_spline(i)=(m0_spline - m1_spline) / (double)number_trunkmotor;
    }
    for(int i=0; i<q0_trunk_from_spline.size(); i++){
        q0_trunk_from_spline(i) = q0_trunk_from_spline(i) > constrS/180.*my_pi ? constrS/180.*my_pi : q0_trunk_from_spline(i);
        q0_trunk_from_spline(i) = q0_trunk_from_spline(i) <-constrS/180.*my_pi ?-constrS/180.*my_pi : q0_trunk_from_spline(i);
    }

}

Matrix<double, 4,4> Locomotion_controller::leginversekinematics()
{
    static MatrixXd qSol = MatrixXd::Zero(4,4);
    qSol.block<4,1>(0,0)=iKinQpOases(0, feetReference.block<3,1>(0,0), GP1.qNULL.block<4,1>(0,0), qSol.block<4,1>(0,0), ikin_lam, ikin_M, ikin_max_dist, ikin_maxIter, ikin_tol, constrFL, ikin_constr_penalty);
    qSol.block<4,1>(0,1)=iKinQpOases(1, feetReference.block<3,1>(0,1), GP1.qNULL.block<4,1>(0,1), qSol.block<4,1>(0,1), ikin_lam, ikin_M, ikin_max_dist, ikin_maxIter, ikin_tol, constrFR, ikin_constr_penalty);
    qSol.block<4,1>(0,2)=iKinQpOases(2, feetReference.block<3,1>(0,2), GP1.qNULL.block<4,1>(0,2), qSol.block<4,1>(0,2), ikin_lam, ikin_M, ikin_max_dist, ikin_maxIter, ikin_tol, constrHL, ikin_constr_penalty);
    qSol.block<4,1>(0,3)=iKinQpOases(3, feetReference.block<3,1>(0,3), GP1.qNULL.block<4,1>(0,3), qSol.block<4,1>(0,3), ikin_lam, ikin_M, ikin_max_dist, ikin_maxIter, ikin_tol, constrHR, ikin_constr_penalty);
    return qSol;
}

/* Solves inverse kinematics for one leg by using Damped Inverse Jacobian method */
Vector4d Locomotion_controller::iKinQpOases(int leg, Vector3d pref, Vector4d qref, Vector4d q0, Vector2d lam, 
Matrix4d M, double max_dist, int maxIter, double tol, MatrixXd constr, Vector3d constr_penalty)
{
    static MatrixXd J(3,4), H(4,4), D(4,4), U(3,3), V(4,4), S(3,1), Jnull(4,4);
    static MatrixXd H_qp(4,4), h_qp(4,1), lb(4,1), ub(4,1), A_qp(2,4), lbA(2,1), ubA(2,1);
    static qpOASES::real_t H_qpoases[4*4], h_qpoases[4], lb_qpoases[4], ub_qpoases[4], A_qpoases[2*4], lbA_qpoases[2], ubA_qpoases[2];
    static Vector4d dqref, dq, q0_first;
    static Vector3d dpref, p0;
    static double norm_dp;
    static vector<bool> is_init={false, false, false, false};
    vector<Matrix4d> H_vec(5);  
    q0_first = q0;
    // for(int k=0; k<maxIter; k++){
        // get current position
        H_vec=legKinematics(q0, leg);
        H=H_vec[0]*H_vec[1]*H_vec[2]*H_vec[3]*H_vec[4];
        p0=H.block<3, 1>(0, 3);
        if(leg == 0)
        {
        p_cur = p0;
        }
        // get Jacobians
        J=Jacob(q0, leg);
        JacobiSVD<MatrixXd> svd(J, ComputeFullU | ComputeFullV);
        S=svd.singularValues();
        U=svd.matrixU();
        V=svd.matrixV();
        D=Matrix4d::Zero();
        D(3,3)=1;
        Jnull=V*D*V.transpose();
        // get reference velocities
        dqref=qref-q0;
        dpref=pref-p0;
        // limit maximum linear velocity
        norm_dp=sqrt(dpref(0)*dpref(0)+dpref(1)*dpref(1)+dpref(2)*dpref(2));
        if(norm_dp>max_dist){
            dpref=max_dist/norm_dp*dpref;
        }
        //=========================================== QP ================================================
        // construct qp matrices
        H_qp=J.transpose()*J + M + lam(0)*MatrixXd::Identity(4,4);
        h_qp=-J.transpose()*dpref - M*Jnull*dqref;
        lb=constr.block<1,4>(0,0).transpose() - q0;
        ub=constr.block<1,4>(1,0).transpose() - q0;
        Map<MatrixXd>( &H_qpoases[0], H_qp.cols(), H_qp.rows() ) = H_qp.transpose();
        Map<MatrixXd>( &h_qpoases[0], h_qp.rows(), h_qp.cols() ) = h_qp;
        Map<MatrixXd>( &lb_qpoases[0], lb.rows(), lb.cols() ) = lb;
        Map<MatrixXd>( &ub_qpoases[0], ub.rows(), ub.cols() ) = ub;

        A_qp << 1,1,0,0,
                1,-1,0,0;
        lbA <<  constr(0,0)*0 -13500*my_pi/180. -(q0(0)+q0(1)), 
                constr(0,1)*0 -13500*my_pi/180. -(q0(0)-q0(1));                  
        ubA <<  constr(1,0)*0 +13500*my_pi/180. -(q0(0)+q0(1)), 
                constr(1,1)*0 +13500*my_pi/180. -(q0(0)-q0(1));         
        Map<MatrixXd>( &A_qpoases[0], A_qp.cols(), A_qp.rows() ) = A_qp.transpose();
        Map<MatrixXd>( &lbA_qpoases[0], lbA.rows(), lbA.cols() ) = lbA;
        Map<MatrixXd>( &ubA_qpoases[0], ubA.rows(), ubA.cols() ) = ubA;
        // init qp solver
        qpOASES::int_t nWSR = 300;
        if(!is_init[leg]){
            qpOASES::QProblem tmpSolver(4, 2);
            ikinQpSolver.resize(LEG_NUM, qpOASES::QProblem(4, 2));
            ikinQpSolver[leg]=tmpSolver;
            qpOASES::Options myOptions;
            myOptions.printLevel = qpOASES::PL_NONE;
            ikinQpSolver[leg].setOptions(myOptions);
        }
        ikinQpSolver[leg].init( H_qpoases, h_qpoases, A_qpoases, lb_qpoases, ub_qpoases, lbA_qpoases, ubA_qpoases, nWSR, 0 );
        // get solution
        qpOASES::real_t sol_qpoases[4];
        ikinQpSolver[leg].getPrimalSolution(sol_qpoases);
        dq(0)=sol_qpoases[0];
        dq(1)=sol_qpoases[1];
        dq(2)=sol_qpoases[2];
        dq(3)=sol_qpoases[3];


        q0+=dq;
    return q0;
}
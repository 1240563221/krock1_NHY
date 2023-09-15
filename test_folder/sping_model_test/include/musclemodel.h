#ifndef __MUSCLE_MODEL_H__
#define __MUSCLE_MODEL_H__

#include "control_table_constants.hpp"

typedef struct 
{
    double m_x;
	double m_y;
	double m_z;

	double m_r;
	double m_K;
	double m_D;
	double m_ref;
	double m_I;
	double m_m;
	double m_L;
	double m_G;
	double m_torque;
    double m_Fext;
}muscleModelTPDF;


class Fourth_RK_CLASS
{
public:
	double m_h;
	int m_n;
	double m_x;
	double m_y;
	double m_z;

    muscleModelTPDF muscleParameters[P_MAX_ID];

	Fourth_RK_CLASS(int n, double h);


	//y' = z	y'' = 2*r**2*(Ky+Dy')  ---->  z' = 2*r**2*(Ky + Dz)
	double fun_z(double t, double y, double z)
	{
		return z;
	}
    
	double fun_y(double t, double y, double z)
	{
		double T_s1=0, T_s2=0, T_spring=0,D1=0,D2=0,D=0;
		if (m_ref < y)
            T_s1 = (m_ref-y)* m_K;
            
        if (m_ref > y)
            T_s2 = (m_ref-y)* m_K;
        
        T_spring = (T_s1+T_s2)/m_I;
        
        if (z > 0)
            D1 = -z*m_D;
        if (z < 0)
            D2 = -z*m_D;
            
        D = D1+D2/m_I;
        D = 0;

        // return   T_spring + D;
        // return  - m_Fext[0]*0.5   + T_spring + D;
		return -m_G * (1. / m_L) * sin(y-m_ref) - m_Fext[0]*0.03  + T_spring + D;
	}

	void RKfun(double *input, double *torque)
	{
        for (int i = 0; i < P_MAX_ID; i++)
        {
            m_Fext[i] = torque[i];
            cout << "m_Fext : " << m_Fext[i] << endl;
        }

		double K1y, K2y, K3y, K4y, K1z, K2z, K3z, K4z;

		K1y = K2y = K3y = K4y = K1z = K2z = K3z = K4z = 0.0;

		double yn_next = 0.0;
		double z_next = 0.0;
		double x,y,z;
		int n = m_n;
		while (n)
		{
			x = m_x;
			y=input[0];
			z=input[1];

			K1z = fun_x1(x, y,  z);
			K1y = fun_x2(x, y,  z);

			K2z = fun_x1(x + m_h/2., y + K1z * m_h/2., z + K1y * m_h/2.);
			K2y = fun_x2(x + m_h/2., y + K1z * m_h/2., z + K1y * m_h/2.);

			K3z = fun_x1(x + m_h/2., y + K2z * m_h/2., z + K2y * m_h/2.);
			K3y = fun_x2(x + m_h/2., y + K2z * m_h/2., z + K2y * m_h/2.);

			K4z = fun_x1(x + m_h, y + K3z * m_h, z + K3y * m_h);
			K4y = fun_x2(x + m_h, y + K3z * m_h, z + K3y * m_h);

			yn_next = y + m_h/6.0 * (K1z + 2*K2z + 2*K3z + K4z);
			z_next = z + m_h/6.0 * (K1y + 2*K2y + 2*K3y + K4y);

			// cout << "n = " << this->m_n - n + 1 << "时：  " << "yn = " << setprecision(5)<<yn_next<< 
			// 		"   yn' = "<< z_next << endl;
			m_x += m_h;
			m_y = yn_next;
			m_z = z_next;
			input[0] = m_y;
			input[1] = m_z;
			n--;
 
		}
	}

};



#endif
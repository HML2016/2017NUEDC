#include "Kalman.h"
#include "Control.h"

/*------------------------------------------
 ��������: �������˲�1
 ��������: ��
------------------------------------------*/
float Kalman_Filter_X(float Accel,float Gyro)		
{
	static float Gyro_y;   //Y�������������ݴ�
	static float angle = 0.0;
	static float Q_bias = 0.0;//������Ư��
	static float angle_err = 0.0;
	static float Q_angle = 0.31;  //�Ƕ��������Ŷ� �����ƹ����е����Э���QԽ����Խ���Ų���ֵ
	static float Q_gyro = 0.015;   //���ٶ��������Ŷȣ����ƹ����е����Э���
	static float R_angle =0.6;    //���ٶȼƲ�������Э���� RԽ����Խ����Ԥ��ֵ
	static float dt = 0.002;	   //dtΪ�˲�������ʱ��(��)
	static char  C_0 = 1;			//H����һ����
	static float PCt_0=0, PCt_1=0, E=0;//�м����
	static float K_0=0, K_1=0, t_0=0, t_1=0;//K ����������
	static float Pdot[4] ={0,0,0,0};//P�����м����
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };//P����X��Э����

	angle += (Gyro - Q_bias) * dt; //�������
	angle_err = Accel - angle;	//zk-�������

	Pdot[0] =Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��
	Pdot[1] = - PP[1][1];
	Pdot[2] = - PP[1][1];
	Pdot[3] =Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;	

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle += K_0 * angle_err;	 //���ŽǶ�
	Q_bias += K_1 * angle_err;	 
	Gyro_y = Gyro - Q_bias;	 //���Ž��ٶ�
	Angle.speed_x=Gyro_y;
	return (angle);				 //���սǶ�			 
}

/*------------------------------------------
 ��������: �������˲�2
 ��������: ��
------------------------------------------*/
float Kalman_Filter_Y(float Accel,float Gyro)		
{
	static float Gyro_x;   //X�������������ݴ�
	static float angle = 0.0;
	static float Q_bias = 0.0;
	static float angle_err = 0.0;
	static float Q_angle = 0.31;  //�Ƕ��������Ŷ�
	static float Q_gyro = 0.015;   //���ٶ��������Ŷ�
	static float R_angle = 0.6;
	static float dt = 0.002;	   //dtΪ�˲�������ʱ��(��)
	static char  C_0 = 1;
	static float PCt_0=0, PCt_1=0, E=0;
	static float K_0=0, K_1=0, t_0=0, t_1=0;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	angle += (Gyro - Q_bias) * dt; //�������
	angle_err = Accel - angle;	//zk-�������

	Pdot[0] =Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��
	Pdot[1] = - PP[1][1];
	Pdot[2] = - PP[1][1];
	Pdot[3] = Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;	

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle += K_0 * angle_err;	 //���ŽǶ�
	Q_bias += K_1 * angle_err;	 
	Gyro_x = Gyro - Q_bias;	 //���Ž��ٶ�
	Angle.speed_y=Gyro_x;
	return (angle);				 //���սǶ�			 
}

//һ�׻����˲�

float yijiehubu(float angle_m, float gyro_m)//�ɼ������ĽǶȺͽǼ��ٶ�

{
	static float K1 =0.01; // �Լ��ٶȼ�ȡֵ��Ȩ��
	static float dt=0.002;//ע�⣺dt��ȡֵΪ�˲�������ʱ��
	static float angle;
	
    angle = K1 * angle_m + (1-K1) * (angle + gyro_m * dt);

     return angle;

}

//���׻����˲�
float Erjielvbo(float angle_m,float gyro_m)//�ɼ������ĽǶȺͽǼ��ٶ�
{
		static float K =0.2; // �Լ��ٶȼ�ȡֵ��Ȩ��
		static float x1,x2,y1;
		static float dt=0.002;//ע�⣺dt��ȡֵΪ�˲�������ʱ��
		static float angle;

    x1=(angle_m-angle)*(1-K)*(1-K);
    y1=y1+x1*dt;
    x2=y1+2*(1-K)*(angle_m-angle)+gyro_m;
    angle=angle+ x2*dt;
	  return angle;
}


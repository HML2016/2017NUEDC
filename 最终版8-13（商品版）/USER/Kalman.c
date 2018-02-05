#include "Kalman.h"
#include "Control.h"

/*------------------------------------------
 函数功能: 卡尔曼滤波1
 函数参数: 无
------------------------------------------*/
float Kalman_Filter_X(float Accel,float Gyro)		
{
	static float Gyro_y;   //Y轴陀螺仪数据暂存
	static float angle = 0.0;
	static float Q_bias = 0.0;//陀螺仪漂移
	static float angle_err = 0.0;
	static float Q_angle = 0.31;  //角度数据置信度 （估计过程中的误差协方差）Q越大则越相信测量值
	static float Q_gyro = 0.015;   //角速度数据置信度（估计过程中的误差协方差）
	static float R_angle =0.6;    //加速度计测量过程协方差 R越大则越相信预测值
	static float dt = 0.002;	   //dt为滤波器采样时间(秒)
	static char  C_0 = 1;			//H矩阵一个数
	static float PCt_0=0, PCt_1=0, E=0;//中间变量
	static float K_0=0, K_1=0, t_0=0, t_1=0;//K 卡尔曼增益
	static float Pdot[4] ={0,0,0,0};//P矩阵中间变量
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };//P矩阵，X的协方差

	angle += (Gyro - Q_bias) * dt; //先验估计
	angle_err = Accel - angle;	//zk-先验估计

	Pdot[0] =Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
	Pdot[1] = - PP[1][1];
	Pdot[2] = - PP[1][1];
	Pdot[3] =Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;	

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle += K_0 * angle_err;	 //最优角度
	Q_bias += K_1 * angle_err;	 
	Gyro_y = Gyro - Q_bias;	 //最优角速度
	Angle.speed_x=Gyro_y;
	return (angle);				 //最终角度			 
}

/*------------------------------------------
 函数功能: 卡尔曼滤波2
 函数参数: 无
------------------------------------------*/
float Kalman_Filter_Y(float Accel,float Gyro)		
{
	static float Gyro_x;   //X轴陀螺仪数据暂存
	static float angle = 0.0;
	static float Q_bias = 0.0;
	static float angle_err = 0.0;
	static float Q_angle = 0.31;  //角度数据置信度
	static float Q_gyro = 0.015;   //角速度数据置信度
	static float R_angle = 0.6;
	static float dt = 0.002;	   //dt为滤波器采样时间(秒)
	static char  C_0 = 1;
	static float PCt_0=0, PCt_1=0, E=0;
	static float K_0=0, K_1=0, t_0=0, t_1=0;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

	angle += (Gyro - Q_bias) * dt; //先验估计
	angle_err = Accel - angle;	//zk-先验估计

	Pdot[0] =Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
	Pdot[1] = - PP[1][1];
	Pdot[2] = - PP[1][1];
	Pdot[3] = Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;	

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle += K_0 * angle_err;	 //最优角度
	Q_bias += K_1 * angle_err;	 
	Gyro_x = Gyro - Q_bias;	 //最优角速度
	Angle.speed_y=Gyro_x;
	return (angle);				 //最终角度			 
}

//一阶互补滤波

float yijiehubu(float angle_m, float gyro_m)//采集后计算的角度和角加速度

{
	static float K1 =0.01; // 对加速度计取值的权重
	static float dt=0.002;//注意：dt的取值为滤波器采样时间
	static float angle;
	
    angle = K1 * angle_m + (1-K1) * (angle + gyro_m * dt);

     return angle;

}

//二阶互补滤波
float Erjielvbo(float angle_m,float gyro_m)//采集后计算的角度和角加速度
{
		static float K =0.2; // 对加速度计取值的权重
		static float x1,x2,y1;
		static float dt=0.002;//注意：dt的取值为滤波器采样时间
		static float angle;

    x1=(angle_m-angle)*(1-K)*(1-K);
    y1=y1+x1*dt;
    x2=y1+2*(1-K)*(angle_m-angle)+gyro_m;
    angle=angle+ x2*dt;
	  return angle;
}


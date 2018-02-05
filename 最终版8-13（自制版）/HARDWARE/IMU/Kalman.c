//******Kalman滤波******//
//-------------------------------------------------------
static  float Q_angle=0.01, Q_gyro=0.0001, R_angle=10;
	//Q增大，动态响应增大
static float Pk[2][2] = { {1, 0}, {0, 1 }};//P矩阵，误差协方差
	
static float Pdot[4] ={0,0,0,0};//计算P矩阵中间变量

static float q_bias=0, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//-------------------------------------------------------
void Kalman_Filter(float angle_m,float gyro_m,float dt)			
{
        /******************第一个公式**********/
	Car_Angle+=(gyro_m-q_bias) * dt; ///预测值
        /*****************第二个公式***********/
	Pdot[0]=Q_angle - Pk[0][1] - Pk[1][0];
	Pdot[1]=- Pk[1][1];
	Pdot[2]=- Pk[1][1];
	Pdot[3]=Q_gyro;
	
	Pk[0][0] += Pdot[0] * dt;
	Pk[0][1] += Pdot[1] * dt;
	Pk[1][0] += Pdot[2] * dt;
	Pk[1][1] += Pdot[3] * dt;
	/*****************第三个公式***********/
	PCt_0 =  Pk[0][0];
	PCt_1 =  Pk[1][0];
	E = R_angle + PCt_0;
	K_0 = PCt_0 / E; ///卡尔曼增益
	K_1 = PCt_1 / E;
	
	/*****************第四个公式***********/
    angle_err = angle_m -Car_Angle;///测量值-预测值
	Car_Angle+= K_0 * angle_err; ///最优角度=预测值+卡尔曼增益*(测量值-预测值)
	q_bias	+= K_1 * angle_err;        
	Angle_Speed = gyro_m-q_bias;
        /*****************第五个公式***********/
	t_0 = PCt_0;
	t_1 = Pk[0][1];

	Pk[0][0] -= K_0 * t_0;
	Pk[0][1] -= K_0 * t_1;
	Pk[1][0] -= K_1 * t_0;
	Pk[1][1] -= K_1 * t_1;
}
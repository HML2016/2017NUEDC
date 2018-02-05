//******Kalman�˲�******//
//-------------------------------------------------------
static  float Q_angle=0.01, Q_gyro=0.0001, R_angle=10;
	//Q���󣬶�̬��Ӧ����
static float Pk[2][2] = { {1, 0}, {0, 1 }};//P�������Э����
	
static float Pdot[4] ={0,0,0,0};//����P�����м����

static float q_bias=0, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//-------------------------------------------------------
void Kalman_Filter(float angle_m,float gyro_m,float dt)			
{
        /******************��һ����ʽ**********/
	Car_Angle+=(gyro_m-q_bias) * dt; ///Ԥ��ֵ
        /*****************�ڶ�����ʽ***********/
	Pdot[0]=Q_angle - Pk[0][1] - Pk[1][0];
	Pdot[1]=- Pk[1][1];
	Pdot[2]=- Pk[1][1];
	Pdot[3]=Q_gyro;
	
	Pk[0][0] += Pdot[0] * dt;
	Pk[0][1] += Pdot[1] * dt;
	Pk[1][0] += Pdot[2] * dt;
	Pk[1][1] += Pdot[3] * dt;
	/*****************��������ʽ***********/
	PCt_0 =  Pk[0][0];
	PCt_1 =  Pk[1][0];
	E = R_angle + PCt_0;
	K_0 = PCt_0 / E; ///����������
	K_1 = PCt_1 / E;
	
	/*****************���ĸ���ʽ***********/
    angle_err = angle_m -Car_Angle;///����ֵ-Ԥ��ֵ
	Car_Angle+= K_0 * angle_err; ///���ŽǶ�=Ԥ��ֵ+����������*(����ֵ-Ԥ��ֵ)
	q_bias	+= K_1 * angle_err;        
	Angle_Speed = gyro_m-q_bias;
        /*****************�������ʽ***********/
	t_0 = PCt_0;
	t_1 = Pk[0][1];

	Pk[0][0] -= K_0 * t_0;
	Pk[0][1] -= K_0 * t_1;
	Pk[1][0] -= K_1 * t_0;
	Pk[1][1] -= K_1 * t_1;
}
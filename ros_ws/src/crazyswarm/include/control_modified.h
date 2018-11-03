#include <math.h>
#include <fstream>
#include "type_methode.h"
//#include <tf_conversions/tf_eigen.h>
#include "commons.h"
#include "pid.h"

//using namespace Eigen;
class Controller
{
public:
	Controller(int id)
	: loop_record(0)
	, m_group_index(0)
	,m_pidX(2.80, 0.08, 0.100, 3.0, 0.0, -5e4, 5e4, -3, 3) //kp, kd, ki, kpp, ff, minOutput, maxOutput, integratorMin, integratorMax;
	,m_pidY(2.80, 0.08, 0.10, 3.0, 0.0, -5e4, 5e4, -3, 3)//kp 22 kd 1.8 ki 2.0 kpp 7	//, m_resFnameRoot("/home/walt/catkin_ws/src/crazyflie_ros-first_trails/easyfly/resultat/vehicle0/")
	,m_pidZ(4.0, 0.01, 0.20, 2.0, 0.0, -5e4, 5e4, -3, 3)//kpp 3 170
//            ,m_pidX(10.0, 0.5, 1, 200, 0.0, -1e6, 1e6, -3, 3) //kp, kd, ki, kpp, ff, minOutput, maxOutput, integratorMin, integratorMax;
//            ,m_pidY(10.0, 0.5, 1, 200, 0.0, -1e6, 1e6, -3, 3)//kp 22 kd 1.8 ki 2.0 kpp 7
//            ,m_pidZ(5.0, 1.65, 5.0, 220.0, 0.0, -1e6, 1e6, -2, 2)//kpp 3 170
		, l_yawSp(0.0f)
		, yaw_deriv(0.0f)
		, initAcc_IIR(true)
		, initGyro_IIR(true)
	{

	 vel_estIMU.setZero();
	 vel_Sp.setZero();
	 vel_estVicon.setZero();
	 acc_IMU_wd.setZero();
	 _acc_Sp_W.setZero();
	 _acc_Sp_B.setZero();
	 //lacc_est_IMU.setZero();

	 _Zb_des.setZero();
	 _Xc_des.setZero();
	 _Xb_des.setZero();
	 _Yb_des.setZero();
	 _R_des.setZero();
	 e_R.setZero();

	 pos_Sp.setZero();
	 pos_estIMU.setZero();
	 R_est.setZero();
	 l_possp.setZero();
	 l_velsp.setZero();
	 acc_deriv.setZero();
	 l_acc_Sp.setZero();

	 acc_Sp_net.setZero();

	 RPY_sp.setZero();
	 RPY_des.setZero();
	 ml_control.setZero();
	 m_pidX.reset();
	 m_pidY.reset();
	 m_pidZ.reset();
	 time=0;
	 
	 printf("hello!! control.h\n");
	 char str_csv[50] ;
	 sprintf(str_csv,"/home/chengque/workspace/Cfs%d.csv",id);
	 Cf_csv.open(str_csv);printf("opened csv\n");
	 Cf_csv  <<"t,sx,sy,sz,x,y,z,vx,vy,vz,svx,svy,svz,ox,oy,oz,ex,ey,ez\n";
	};

	const float w_Vicon = 1.0f;
	const float w_IMU = 0.0f;
	const float wv_Vicon = 1;
	const float wv_IMU = 0;
	const float m_cutoff_freq = 2.0f; //cutoff f
    const float m_sample_freq = 300.0f; //sampling f
    const float m_fc_gyro = 0.5f;
    const int num_redording = 4096;
//    const float max_thrust = 0.5827*1.3;
    const float amplifier=13200;
	const float max_thrust = 6500;//amplifier*GRAVITY*MAX_THRUST/1000000.0f;

	const float KPxy = 0.06, KVxy = 0.02;//added by xs
	const float KPz = 0.18, KVz = 0.06;
	float err_x, err_y, err_z;
	float err_vx, err_vy, err_vz;

	float Pitch_Sp;
	float time;
	float Roll_Sp;
	char m_resFnameRoot[150];
	int loop_record;
	int m_group_index;
	std::ofstream Cf_csv;
protected:
	//att:
	Eigen::Vector3f e_R;
	Eigen::Vector4f ml_control;
	Eigen::Vector3f _Zb_des, _Xc_des, _Yb_des, _Xb_des, Thrust_des;//, F_des
	std::vector<Eigen::MatrixXf> PID_v;
	Eigen::Vector3f RPY_sp, RPY_des;
	Eigen::Matrix3f _R_des, R_est;

	Eigen::Vector3f vel_Sp, _acc_Sp_W, _acc_Sp_B, vel_estVicon, vel_estIMU, acc_IMU_wd, acc_deriv, l_acc_Sp, acc_Sp_net, h_omega;

	float l_yawSp, yaw_deriv;

	Eigen::Vector3f pos_Sp, l_possp, pos_estIMU, l_velsp;//, lacc_est_IMU
	PID m_pidX;
	PID m_pidY;
	PID m_pidZ;
	ros::Publisher m_pos_sppub, m_posEstPub, m_attSpPub;
	bool initAcc_IIR, initGyro_IIR;

	void resetposController(Eigen::Vector3f* pos_est_Vicon)
	{
		l_posVicon = *pos_est_Vicon;
		l_possp = *pos_est_Vicon;
		pos_estIMU = *pos_est_Vicon; //init of IMU position esti
	}

public:
    Eigen::Vector3f l_posVicon;
    void control_nonLineaire(const Eigen::Vector3f& pos_est_Vicon, Eigen::Vector4f& Sp, Eigen::Vector3f& Vel_ff,
                                  Eigen::Vector3f& acc_Sp, Eigen::Vector3f& Euler, float dt, Eigen::Vector4f* Output);
    void control_nonLineaire(const Eigen::Vector3f& pos_est_Vicon, double& Sp_z, double *vxy_sp,
                                         Eigen::Vector3f& Euler, float dt, Eigen::Vector4f* Output);
	// {
	// 	if(dt<0.1f)
	// 	{
	// 		_acc_Sp_W = *acc_Sp;
	// 		euler2rotation(Euler,&R_est);
	// 		body2earth(&R_est, acc_est_IMU, &acc_IMU_wd, 3);
	// 		for(int i=0; i<3; i++){
	// 			pos_Sp(i) = (*Sp)(i);
	// 		}
	// 		/**::position part::**/
	// 		acc_IMU_wd(2) += GRAVITY/1000.0f;
	// 		vel_estIMU += acc_IMU_wd*dt; 
	// 		pos_estIMU += vel_estIMU*dt;
	// 		pos_estIMU(2) = min(pos_estIMU(2),0.0f);

	// 		float x_temp_est = (*pos_est_Vicon)(0);//*w_Vicon + pos_estIMU(0)*w_IMU;
	// 		float y_temp_est = (*pos_est_Vicon)(1);//*w_Vicon + pos_estIMU(1)*w_IMU;
	// 		float z_temp_est = (*pos_est_Vicon)(2);//*w_Vicon + pos_estIMU(2)*w_IMU;
	
	// 		float x_sp = pos_Sp(0);
	// 		float y_sp = pos_Sp(1);
	// 		float z_sp = pos_Sp(2);
			
	// 		//pos_estUnited = (*pos_est_Vicon)*w_Vicon + pos_estIMU*w_IMU;
	// 		/*pos_Sp(0) += m_pidX.pp_update(x_temp_est , x_sp);
	// 		pos_Sp(1) += m_pidY.pp_update(y_temp_est , y_sp);
	// 		pos_Sp(2) += m_pidZ.pp_update(z_temp_est , z_sp);*/

	// 		//vec3f_derivative(&vel_Sp, &pos_Sp, &l_possp, dt);
	// 		vel_Sp = *Vel_ff;
	// 		vec3f_derivative(&vel_estVicon, pos_est_Vicon, &l_posVicon, dt);
			
	// 		//l_posVicon = *pos_est_Vicon;
	// 		//l_possp = pos_Sp;
	
	// 		float vx_temp_est = vel_estVicon(0);//*wv_Vicon + vel_estIMU(0)*wv_IMU;
	// 		float vy_temp_est = vel_estVicon(1);//*wv_Vicon + vel_estIMU(1)*wv_IMU;
	// 		float vz_temp_est = vel_estVicon(2);//*wv_Vicon + vel_estIMU(2)*wv_IMU;
	
	// 		float vx_sp = vel_Sp(0);
	// 		float vy_sp = vel_Sp(1);
	// 		float vz_sp = vel_Sp(2);
			
	// 		vel_Sp(0) =  m_pidX.pp_update(x_temp_est , x_sp); //+ff
	// 		vel_Sp(1) =  m_pidY.pp_update(y_temp_est , y_sp);
	// 		vel_Sp(2) =  m_pidZ.pp_update(z_temp_est , z_sp);

	// 		/*vel_Sp(0) += m_pidVx.pid_update(vx_temp_est,vx_sp,dt);
	// 		vel_Sp(1) += m_pidVx.pid_update(vx_temp_est,vy_sp,dt);
	// 		vel_Sp(2) += m_pidVx.pid_update(vx_temp_est,vz_sp,dt);*/

	// 		//vel_Sp = vel_estVicon;
	// 		l_velsp = vel_Sp;
	// 		//printf("%f\n",x_temp_est-x_sp );
	// 		_acc_Sp_W(0) =  m_pidX.pid_update(vx_temp_est,vel_Sp(0),dt);
	// 		_acc_Sp_W(1) =  m_pidY.pid_update(vy_temp_est,vel_Sp(1),dt);
	// 		_acc_Sp_W(2) =  m_pidZ.pid_update(vz_temp_est,vel_Sp(2),dt);

	// 		//_acc_Sp_W(2) =  m_pidVz.pid_update(z_temp_est,pos_Sp(2),dt);

	// 		//vec3f_derivative(&(*acc_Sp), &vel_Sp, &l_velsp, dt);
			
	// 		acc_Sp_net = _acc_Sp_W;

	// 		/*vec3f_derivative(&acc_deriv, &acc_Sp_net, &l_acc_Sp, dt);
	// 		//l_acc_Sp = acc_Sp_net;
	// 		acc_deriv(0) = acc_deriv(0) + m_pidVx.pid_update(vx_temp_est, vx_sp, dt);
	// 		acc_deriv(1) = acc_deriv(1) + m_pidVy.pid_update(vy_temp_est, vy_sp, dt);
	// 		acc_deriv(2) = acc_deriv(2) + m_pidVz.pid_update(vz_temp_est, vz_sp, dt);*/

	// 		_acc_Sp_W(0) = 0.0f;
	// 		_acc_Sp_W(1) = 0.0f;
	// 		_acc_Sp_W(2) = _acc_Sp_W(2) + GRAVITY/1000.0f * (float)VEHICLE_MASS;

	// 		/**::attitude part::**/

	// 		//acc2att(dt,Sp);
	// 		vec3f_passnorm(&_acc_Sp_W, &_Zb_des);

	// 		for (int i=0; i<3; i++)
	// 			_R_des(i,2) = _Zb_des(i);
	
	// 		_Xc_des(0) = cos((*Sp)(3));
	// 		_Xc_des(1) = sin((*Sp)(3));
	// 		_Xc_des(2) = 0;
			
	// 		/*float yaw_deriv = deriv_f((*Sp)(3),l_yawSp,dt);
	// 		l_yawSp = (*Sp)(3);*/
	// 		vec3f_cross(&_Zb_des, &_Xc_des, &_Yb_des);
	// 		vec3f_normalize(&_Yb_des);
	// 		vec3f_cross(&_Yb_des, &_Zb_des, &_Xb_des);
	
	// 		for (int i=0; i<3; i++)
	// 		{
	// 			_R_des(i,0) = _Xb_des(i);
	// 			_R_des(i,1) = _Yb_des(i);
	// 		}

	// 		rotation2euler(&_R_des,&RPY_des);

	// 		x_temp_est = (*Euler)(0);
	// 		y_temp_est = (*Euler)(1);
	// 		z_temp_est = (*Euler)(2);

	// 		x_sp = RPY_des(0);
	// 		y_sp = RPY_des(1);
	// 		z_sp = RPY_des(2);
	// 		/*
	// 		RPY_sp(0) += m_pidRoll.pp_update(x_temp_est,x_sp);
 //  			RPY_sp(1) += m_pidPitch.pp_update(y_temp_est,y_sp);
 //  			RPY_sp(2) += m_pidYaw.pp_update(z_temp_est,z_sp);*/

 //  			//printf("difference: %f  \n",RPY_des(1)- (*Euler)(1));
	// 		for(int i=0;i<3;i++){
	// 			//(*Output)(i) = RPY_sp(i);
	// 			(*Output)(i) = RPY_des(i);}
	// 		Eigen::Vector3f temp;
	// 		temp.setZero();
	// 		for(int i=0;i<3;i++){
	// 			temp(i) = _R_des(i,2);
	// 		}

	// 		float thrust_force = vec3f_dot(&_acc_Sp_W,&temp);
	// 		//earth2body(&_R_des, &_acc_Sp_W, &_acc_Sp_B, 3);
 //            //_acc_Sp_B = _acc_Sp_W * R_est.col(2);

	// 		thrust_force /= 460.0f;
	// 		thrust_force = std::min(thrust_force,max_thrust);
	// 		(*Output)(3) = thrust_force;
	// 		/**::recording datas::**/

	// 		/*if(loop_record<num_redording)
	// 		{
	// 		loop_record++;
	// 		char fname[250];
	// 		sprintf(fname,"%spos_est_Vicon_11.10(2).dat",m_resFnameRoot);
	// 		writeData_bin(fname,pos_est_Vicon);
	// 		sprintf(fname,"%sposSp_11.10(2).dat",m_resFnameRoot);
	// 		writeData_bin(fname,&pos_Sp);
	// 		sprintf(fname,"%svel_sp_11.10(2).dat",m_resFnameRoot);
	// 		writeData_bin(fname,&vel_Sp);
	// 		sprintf(fname,"%saccSp_11.10(2).dat",m_resFnameRoot);
	// 		writeData_bin(fname,&_acc_Sp_W);
	// 		sprintf(fname,"%sImuAcc_est_11.10(2).dat",m_resFnameRoot);
	// 		writeData_bin(fname,acc_est_IMU);
	// 		sprintf(fname,"%svel_estVicon_11.10(2).dat",m_resFnameRoot);
	// 		writeData_bin(fname,&vel_estVicon);
	// 		sprintf(fname,"%svel_estIMU_11.10(2).dat",m_resFnameRoot);
	// 		writeData_bin(fname,&vel_estIMU);
	// 		sprintf(fname,"%sImuAccWD_est_11.10(2).dat",m_resFnameRoot);
	// 		writeData_bin(fname,&acc_IMU_wd);
	// 		sprintf(fname,"%satt_Sp_11.10(2).dat",m_resFnameRoot);
	// 		writeData_bin(fname,&RPY_sp);
	// 		sprintf(fname,"%sRPY_des_11.10(2).dat",m_resFnameRoot);
	// 		writeData_bin(fname,&RPY_des);
	// 		sprintf(fname,"%spos_estIMU_11.10(2).dat",m_resFnameRoot);
	// 		writeData_bin(fname,&pos_estIMU);
	// 		sprintf(fname,"%satt_Est_11.10(2).dat",m_resFnameRoot);
	// 		writeData_bin(fname,Euler);
			
	// 		/*sprintf(fname,"%sImuAccWD_est_static_11.08(2).dat",m_resFnameRoot);
	// 		writeData_bin(fname,&acc_IMU_wd);
	// 		}*/
	// 	}
	// }	


	
	


	// map erro from so(3) to R3:
//	void evv_map(const Eigen::Matrix3f* _R_des, const Eigen::Matrix3f* R_B, Eigen::Vector3f* e_R)
//	{
//		//Eigen::Matrix3f _R_desT = _R_des->transpose().eval();
//		Eigen::Matrix3f e_Rtemp = (_R_des->transpose().eval() * (*R_B) - R_B->transpose().eval() * (*_R_des))/2.0f;
//		(*e_R)(0) = (e_Rtemp(1,2) - e_Rtemp(2,1));//roll err in rad<<14
//		(*e_R)(1) = (e_Rtemp(2,0) - e_Rtemp(0,2));//pitch err in rad<<14
//		(*e_R)(2) = (e_Rtemp(0,1) - e_Rtemp(1,0));//yaw err in rad<<14
//	}
//	void vec3fMinMaxInteg(Eigen::Vector3f* Intvec, Eigen::Vector3f* min_v, Eigen::Vector3f* max_v)
//	{
//		for (int i=0;i<3;i++){
//			(*Intvec)(i) = min(max((*Intvec)(i),(*min_v)(i)),(*max_v)(i));
//		}
//	}


//	void acc2att(const float dt, Eigen::Vector4f* Sp)
//	{
//		vec3f_passnorm(&_acc_Sp_B, &_Zb_des);
//
//		for (int i=0; i<3; i++)
//			_R_des(i,2) = _Zb_des(i);
//
//		_Xc_des(0) = cos((*Sp)(3));
//		_Xc_des(1) = sin((*Sp)(3));
//		_Xc_des(2) = 0;
//
//		float yaw_deriv = deriv_f((*Sp)(3),l_yawSp,dt);
//		l_yawSp = (*Sp)(3);
//		vec3f_cross(&_Zb_des, &_Xc_des, &_Yb_des);
//		vec3f_normalize(&_Yb_des);
//		vec3f_cross(&_Yb_des, &_Zb_des, &_Xb_des);
//
//		for (int i=0; i<3; i++)
//		{
//			_R_des(i,0) = _Xb_des(i);
//			_R_des(i,1) = _Yb_des(i);
//		}
//
//	}

	/*void reset_attController(Eigen::Vector3f* gyro);

	void resetposController(Eigen::Vector3f* pos_est_Vicon);

	void resetposSp(Eigen::Vector4f* posSp);

	void resetaccController(Eigen::Vector3f* acc_est_IMU);

	void position_control(Eigen::Vector3f* pos_est_Vicon, Eigen::Vector4f* Sp, Eigen::Vector4f* Output, Eigen::Vector3f* acc_est_IMU, Eigen::Vector3f* Euler, float dt);
	
	void attitude_control(Eigen::Vector4f* Sp, Eigen::Vector4f* Output, Eigen::Vector3f* Euler, Eigen::Vector3f* gyro, float dt);
	
	void evv_map(const Eigen::Matrix3f* _R_des, const Eigen::Matrix3f* R_B, Eigen::Vector3f* e_R); // map erro from so(3) to R3:*/
};
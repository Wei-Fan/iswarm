#include "control_modified.h"

void Controller::control_nonLineaire(const Eigen::Vector3f& pos_est_Vicon, Eigen::Vector4f& Sp, Eigen::Vector3f& Vel_ff,
                                          Eigen::Vector3f& acc_Sp, Eigen::Vector3f& Euler, float dt, Eigen::Vector4f* Output)
{
//    Eigen::Vector4f* Output;
    if(dt<0.1f)
    {
        _acc_Sp_W = acc_Sp;
        euler2rotation(&Euler,&R_est);
        for(int i=0; i<3; i++){
            pos_Sp(i) = Sp(i);
        }

        time=time+dt;

        float x_temp_est = pos_est_Vicon(0);
        float y_temp_est = pos_est_Vicon(1);
        float z_temp_est = pos_est_Vicon(2);

        float x_sp = pos_Sp(0);
        float y_sp = pos_Sp(1);
        float z_sp = pos_Sp(2);
        if(z_sp<-0.9f)
        {

            (*Output)(0)=0.0f;
            (*Output)(1)=0.0f;
            (*Output)(2)=0.0f;
            (*Output)(3)=500;
            Cf_csv <<time<<","<<pos_Sp(0)<<","<<pos_Sp(1)<<","<<pos_Sp(2)<<","<<x_temp_est <<","<< y_temp_est << "," << z_temp_est << "," << 0 << "," << 0
                   << "," << 0 << "," << 0 << "," << 0<<"," << 0
                   << "," << (*Output)(0)<<"," << (*Output)(1)<<"," << (*Output)(3)<< "," << 0 << "," << 0 << "," << 0<<"\n";
            return;
        }
        if(z_temp_est<-0.1f)
        {
            (*Output)(0)=0.0f;
            (*Output)(1)=0.0f;
            (*Output)(2)=0.0f;
            (*Output)(3)=ml_control(2)-100;
            ml_control(0)=(*Output)(0);
            ml_control(1)=(*Output)(1);
            ml_control(2)=(*Output)(3);

            Cf_csv <<time<<","<<pos_Sp(0)<<","<<pos_Sp(1)<<","<<pos_Sp(2)<<","<<x_temp_est <<","<< y_temp_est << "," << z_temp_est << "," << 0 << "," << 0
                   << "," << 0 << "," << 0 << "," << 0<<"," << 0
                   << "," << (*Output)(0)<<"," << (*Output)(1)<<"," << (*Output)(3)<< "," << 0 << "," << 0 << "," << 0 <<"\n";
            return;
        }

        /*cout<<"sp control (xyz)="<< pos_Sp(0) <<"   "<<
            pos_Sp(1)  <<"   "<<
            pos_Sp(2) <<endl;

        cout<<"est (xyz)="<<  x_temp_est <<"   "<<
             y_temp_est <<"   "<<
             z_temp_est <<endl;

        cout<<"err_r (xyz)="<< pos_Sp(0) - x_temp_est <<"   "<<
            pos_Sp(1) - y_temp_est <<"   "<<
            pos_Sp(2) - z_temp_est <<endl;*/

        vel_Sp = Vel_ff;
        vel_estVicon(0) = (pos_est_Vicon(0) - l_posVicon(0))/dt;
        vel_estVicon(1) = (pos_est_Vicon(1) - l_posVicon(1))/dt;
        vel_estVicon(2) = (pos_est_Vicon(2) - l_posVicon(2))/dt;

        vel_estVicon(0) = std::min(std::max(vel_estVicon(0),-3.0f),3.0f);
        vel_estVicon(1) = std::min(std::max(vel_estVicon(1),-3.0f),3.0f);
        vel_estVicon(2) = std::min(std::max(vel_estVicon(2),-3.0f),3.0f);

        /*cout<<"err_v (xyz)="<< vel_Sp(0) - vel_estVicon(0) <<"  "<<
            vel_Sp(1) - vel_estVicon(1) <<"  "<<
            vel_Sp(2) - vel_estVicon(2) <<endl;*/

        l_posVicon(0) = x_temp_est;
        l_posVicon(1) = y_temp_est;
        l_posVicon(2) = z_temp_est;

        float vx_temp_est = vel_estVicon(0);
        float vy_temp_est = vel_estVicon(1);
        float vz_temp_est = vel_estVicon(2);

//        vel_Sp(0) =  m_pidX.pp_update(x_temp_est , x_sp)*0.7f + Vel_ff(0)*0.3f; //+ff
//        vel_Sp(1) =  m_pidY.pp_update(y_temp_est , y_sp)*0.7f + Vel_ff(1)*0.3f;
//        vel_Sp(2) =  m_pidZ.pp_update(z_temp_est , z_sp)*0.7f + Vel_ff(2)*0.3f;

        vel_Sp(0) =  m_pidX.pp_update(x_temp_est , x_sp) + Vel_ff(0)*0.0f; //+ff
        vel_Sp(1) =  m_pidY.pp_update(y_temp_est , y_sp) + Vel_ff(1)*0.0f;
        vel_Sp(2) =  m_pidZ.pp_update(z_temp_est , z_sp) + Vel_ff(2)*0.0f;

        vel_Sp(0) = std::min(std::max(vel_Sp(0),-2.0f),2.0f);
        vel_Sp(1) = std::min(std::max(vel_Sp(1),-2.0f),2.0f);
        vel_Sp(2) = std::min(std::max(vel_Sp(2),-2.0f),2.0f);


        float vx_sp = vel_Sp(0);
        float vy_sp = vel_Sp(1);
        float vz_sp = vel_Sp(2);

        l_velsp = vel_Sp;

        _acc_Sp_W(0) =  m_pidX.pid_update(vx_temp_est,vel_Sp(0),dt);
        _acc_Sp_W(1) =  m_pidY.pid_update(vy_temp_est,vel_Sp(1),dt);
        _acc_Sp_W(2) =  m_pidZ.pid_update(vz_temp_est,vel_Sp(2),dt);




//        _acc_Sp_W(0) = 0.7f*_acc_Sp_W(0) + acc_Sp(0)*0.3f;
//        _acc_Sp_W(1) = 0.7f*_acc_Sp_W(1) + acc_Sp(1)*0.3f;
//        _acc_Sp_W(2) = 0.7f*_acc_Sp_W(2) + acc_Sp(2)*0.3f;

        _acc_Sp_W(0) = _acc_Sp_W(0) + acc_Sp(0)*0.0f;
        _acc_Sp_W(1) = _acc_Sp_W(1) + acc_Sp(1)*0.0f;
        _acc_Sp_W(2) = _acc_Sp_W(2) + acc_Sp(2)*0.0f;


        _acc_Sp_W(0) = std::max(std::min(_acc_Sp_W(0),2.0f),-2.0f);
        _acc_Sp_W(1) = std::max(std::min(_acc_Sp_W(1), 2.0f),-2.0f);
        _acc_Sp_W(2) = std::max(std::min(_acc_Sp_W(2), 3.0f),-4.0f);


        acc_Sp_net = _acc_Sp_W;

//        _acc_Sp_W(2) = _acc_Sp_W(2) + GRAVITY/100.0f * (float)VEHICLE_MASS;//9810 40
        //_acc_Sp_W(2) = _acc_Sp_W(2) + 5200;
        _acc_Sp_W(2) = _acc_Sp_W(2) + 9.8f;


        vec3f_passnorm(&_acc_Sp_W, &_Zb_des);

        for (int i=0; i<3; i++)
            _R_des(i,2) = _Zb_des(i);

        _Xc_des(0) = cos(Sp(3));
        _Xc_des(1) = sin(Sp(3));
        _Xc_des(2) = 0;

        vec3f_cross(&_Zb_des, &_Xc_des, &_Yb_des);
        vec3f_normalize(&_Yb_des);
        vec3f_cross(&_Yb_des, &_Zb_des, &_Xb_des);

        for (int i=0; i<3; i++)
        {
            _R_des(i,0) = _Xb_des(i);
            _R_des(i,1) = _Yb_des(i);
        }

        rotation2euler(&_R_des,&RPY_des);

        x_sp = RPY_des(0);
        y_sp = RPY_des(1);
        z_sp = RPY_des(2);

        //cout<<RPY_des(0)<<"\t"<<RPY_des(1)<<"\t"<<RPY_des(2)<<std::endl;
        for(int i=0;i<2;i++){
            (*Output)(i) = RPY_des(i);
        }





//        (*Output)(0) = (*Output)(0);//by xs

        Eigen::Vector3f temp;
        temp.setZero();
        for(int i=0;i<3;i++){
            temp(i) = _R_des(i,2);
        }

        float thrust_force = vec3f_dot(&_acc_Sp_W,&temp)*480;
        float att_lim=std::max(12.0f/std::max((thrust_force-5600)/100,1.0f),4.0f);

        (*Output)(2) = 0;  //yaw rate
        (*Output)(0) = std::max(std::min((*Output)(0)*57.3f,att_lim),-att_lim);
        (*Output)(1) = std::max(std::min(-(*Output)(1)*57.3f,att_lim),-att_lim);

//        thrust_force /= 470.0f;
        thrust_force = std::min(thrust_force,max_thrust);
//        thrust_force = 2500.0f;
        (*Output)(3) = thrust_force;
        if((*Output)(0)-ml_control(0)>0.5f)
        {
            (*Output)(0)=ml_control(0)+0.5f;
        }
        if((*Output)(0)-ml_control(0)<-0.5f)
        {
            (*Output)(0)=ml_control(0)-0.5f;
        }

        if((*Output)(1)-ml_control(1)>0.5f)
        {
            (*Output)(1)=ml_control(1)+0.5f;
        }
        if((*Output)(1)-ml_control(1)<-0.5f)
        {
            (*Output)(1)=ml_control(1)-0.5f;
        }




        if(ml_control(2)>2000.0f) {
            if ((*Output)(2) - ml_control(2) > 200.0f) {
                (*Output)(2) = ml_control(2) + 200.0f;
            }
            if ((*Output)(2) - ml_control(2) < -200.0f) {
                (*Output)(2) = ml_control(2) - 200.0f;
            }
        }



        if(pos_Sp(2)<0.1)
        {

            (*Output)(0)=0.0f;
            (*Output)(1)=0.0f;
            (*Output)(2)=0.0f;

        }


        ml_control(0)=(*Output)(0);
        ml_control(1)=(*Output)(1);
        ml_control(2)=(*Output)(3);

        Cf_csv <<time<<","<<pos_Sp(0)<<","<<pos_Sp(1)<<","<<pos_Sp(2)<<","<<x_temp_est <<","<< y_temp_est << "," << z_temp_est << "," <<vel_estVicon(0)<< "," << vel_estVicon(1)
                                  << "," << vel_estVicon(2) << "," << vel_Sp(0) << "," << vel_Sp(1)<<"," << vel_Sp(2) 
                                  << "," << (*Output)(0)<<"," << (*Output)(1)<<"," << (*Output)(3)<<","<<Euler(0)<<","<<Euler(1)<<","<<Euler(2)<<"\n";

    }
//    return *Output;
}

void Controller::control_nonLineaire(const Eigen::Vector3f& pos_est_Vicon, double& Sp_z, double *vxy_sp,
                                        Eigen::Vector3f& Euler, float dt, Eigen::Vector4f* Output)
{
//    Eigen::Vector4f* Output;
    if(dt<0.1f)
    {
//        _acc_Sp_W = acc_Sp;
        euler2rotation(&Euler,&R_est);
        pos_Sp(0) = 0.0f;
        pos_Sp(1) = 0.0f;
        pos_Sp(2) = Sp_z;

        time=time+dt;

        float x_temp_est = pos_est_Vicon(0);
        float y_temp_est = pos_est_Vicon(1);
        float z_temp_est = pos_est_Vicon(2);

//        float x_sp = pos_Sp(0);
//        float y_sp = pos_Sp(1);
        float z_sp = pos_Sp(2);
        if(z_sp<-0.9f)
        {

            (*Output)(0)=0.0f;
            (*Output)(1)=0.0f;
            (*Output)(2)=0.0f;
            (*Output)(3)=500;
            Cf_csv <<time<<","<<pos_Sp(0)<<","<<pos_Sp(1)<<","<<pos_Sp(2)<<","<<x_temp_est <<","<< y_temp_est << "," << z_temp_est << "," << 0 << "," << 0
                   << "," << 0 << "," << 0 << "," << 0<<"," << 0
                   << "," << (*Output)(0)<<"," << (*Output)(1)<<"," << (*Output)(3)<< "," << 0 << "," << 0 << "," << 0<<"\n";
            return;
        }
        if(z_temp_est<-0.1f)
        {
            (*Output)(0)=0.0f;
            (*Output)(1)=0.0f;
            (*Output)(2)=0.0f;
            (*Output)(3)=ml_control(2)-100;
            ml_control(0)=(*Output)(0);
            ml_control(1)=(*Output)(1);
            ml_control(2)=(*Output)(3);

            Cf_csv <<time<<","<<pos_Sp(0)<<","<<pos_Sp(1)<<","<<pos_Sp(2)<<","<<x_temp_est <<","<< y_temp_est << "," << z_temp_est << "," << 0 << "," << 0
                   << "," << 0 << "," << 0 << "," << 0<<"," << 0
                   << "," << (*Output)(0)<<"," << (*Output)(1)<<"," << (*Output)(3)<< "," << 0 << "," << 0 << "," << 0 <<"\n";
            return;
        }

        /*cout<<"sp control (xyz)="<< pos_Sp(0) <<"   "<<
            pos_Sp(1)  <<"   "<<
            pos_Sp(2) <<endl;

        cout<<"est (xyz)="<<  x_temp_est <<"   "<<
             y_temp_est <<"   "<<
             z_temp_est <<endl;

        cout<<"err_r (xyz)="<< pos_Sp(0) - x_temp_est <<"   "<<
            pos_Sp(1) - y_temp_est <<"   "<<
            pos_Sp(2) - z_temp_est <<endl;*/

        vel_estVicon(0) = (pos_est_Vicon(0) - l_posVicon(0))/dt;
        vel_estVicon(1) = (pos_est_Vicon(1) - l_posVicon(1))/dt;
        vel_estVicon(2) = (pos_est_Vicon(2) - l_posVicon(2))/dt;

        vel_estVicon(0) = std::min(std::max(vel_estVicon(0),-3.0f),3.0f);
        vel_estVicon(1) = std::min(std::max(vel_estVicon(1),-3.0f),3.0f);
        vel_estVicon(2) = std::min(std::max(vel_estVicon(2),-3.0f),3.0f);

        l_posVicon(0) = x_temp_est;
        l_posVicon(1) = y_temp_est;
        l_posVicon(2) = z_temp_est;

        float vx_temp_est = vel_estVicon(0);
        float vy_temp_est = vel_estVicon(1);
        float vz_temp_est = vel_estVicon(2);

//        vel_Sp(0) =  m_pidX.pp_update(x_te0)mp_est , x_sp)*0.7f + Vel_ff(0)*0.3f; //+ff
//        vel_Sp(1) =  m_pidY.pp_update(y_temp_est , y_sp)*0.7f + Vel_ff(1)*0.3f;
//        vel_Sp(2) =  m_pidZ.pp_update(z_temp_est , z_sp)*0.7f + Vel_ff(2)*0.3f;

        vel_Sp(0) =  vxy_sp[0]; //m_pidX.pp_update(x_temp_est , x_sp) + Vel_ff(0)*0.0f; //+ff
        vel_Sp(1) =  vxy_sp[1]; //m_pidY.pp_update(y_temp_est , y_sp) + Vel_ff(1)*0.0f;
        vel_Sp(2) =  m_pidZ.pp_update(z_temp_est , z_sp); // + Vel_ff(2)*0.0f;

        vel_Sp(0) = std::min(std::max(vel_Sp(0),-1.5f),1.5f);
        vel_Sp(1) = std::min(std::max(vel_Sp(1),-1.5f),1.5f);
        vel_Sp(2) = std::min(std::max(vel_Sp(2),-2.0f),2.0f);


//        float vx_sp = vel_Sp(0);
//        float vy_sp = vel_Sp(1);
//        float vz_sp = vel_Sp(2);

        l_velsp = vel_Sp;

        _acc_Sp_W(0) =  m_pidX.pid_update(vx_temp_est,vel_Sp(0),dt);
        _acc_Sp_W(1) =  m_pidY.pid_update(vy_temp_est,vel_Sp(1),dt);
        _acc_Sp_W(2) =  m_pidZ.pid_update(vz_temp_est,vel_Sp(2),dt);




//        _acc_Sp_W(0) = 0.7f*_acc_Sp_W(0) + acc_Sp(0)*0.3f;
//        _acc_Sp_W(1) = 0.7f*_acc_Sp_W(1) + acc_Sp(1)*0.3f;
//        _acc_Sp_W(2) = 0.7f*_acc_Sp_W(2) + acc_Sp(2)*0.3f;

//        _acc_Sp_W(0) = _acc_Sp_W(0) + acc_Sp(0)*0.0f;
//        _acc_Sp_W(1) = _acc_Sp_W(1) + acc_Sp(1)*0.0f;
//        _acc_Sp_W(2) = _acc_Sp_W(2) + acc_Sp(2)*0.0f;


        _acc_Sp_W(0) = std::max(std::min(_acc_Sp_W(0),2.0f),-2.0f);
        _acc_Sp_W(1) = std::max(std::min(_acc_Sp_W(1), 2.0f),-2.0f);
        _acc_Sp_W(2) = std::max(std::min(_acc_Sp_W(2), 3.0f),-4.0f);


        acc_Sp_net = _acc_Sp_W;

//        _acc_Sp_W(2) = _acc_Sp_W(2) + GRAVITY/100.0f * (float)VEHICLE_MASS;//9810 40
        //_acc_Sp_W(2) = _acc_Sp_W(2) + 5200;
        _acc_Sp_W(2) = _acc_Sp_W(2) + 9.8f;


        vec3f_passnorm(&_acc_Sp_W, &_Zb_des);

        for (int i=0; i<3; i++)
            _R_des(i,2) = _Zb_des(i);

        _Xc_des(0) = 1; //cos(Sp(3));
        _Xc_des(1) = 0; //sin(Sp(3));
        _Xc_des(2) = 0;

        vec3f_cross(&_Zb_des, &_Xc_des, &_Yb_des);
        vec3f_normalize(&_Yb_des);
        vec3f_cross(&_Yb_des, &_Zb_des, &_Xb_des);

        for (int i=0; i<3; i++)
        {
            _R_des(i,0) = _Xb_des(i);
            _R_des(i,1) = _Yb_des(i);
        }

        rotation2euler(&_R_des,&RPY_des);

//        x_sp = RPY_des(0);
//        y_sp = RPY_des(1);
//        z_sp = RPY_des(2);

        //cout<<RPY_des(0)<<"\t"<<RPY_des(1)<<"\t"<<RPY_des(2)<<std::endl;
        for(int i=0;i<2;i++){
            (*Output)(i) = RPY_des(i);
        }





//        (*Output)(0) = (*Output)(0);//by xs

        Eigen::Vector3f temp;
        temp.setZero();
        for(int i=0;i<3;i++){
            temp(i) = _R_des(i,2);
        }

        float thrust_force = vec3f_dot(&_acc_Sp_W,&temp)*480;
        float att_lim=std::max(12.0f/std::max((thrust_force-5600)/100,1.0f),4.0f);

        (*Output)(2) = 0;  //yaw rate
        (*Output)(0) = std::max(std::min((*Output)(0)*57.3f,att_lim),-att_lim);
        (*Output)(1) = std::max(std::min(-(*Output)(1)*57.3f,att_lim),-att_lim);

//        thrust_force /= 470.0f;
        thrust_force = std::min(thrust_force,max_thrust);
//        thrust_force = 2500.0f;
        (*Output)(3) = thrust_force;
        if((*Output)(0)-ml_control(0)>0.5f)
        {
            (*Output)(0)=ml_control(0)+0.5f;
        }
        if((*Output)(0)-ml_control(0)<-0.5f)
        {
            (*Output)(0)=ml_control(0)-0.5f;
        }

        if((*Output)(1)-ml_control(1)>0.5f)
        {
            (*Output)(1)=ml_control(1)+0.5f;
        }
        if((*Output)(1)-ml_control(1)<-0.5f)
        {
            (*Output)(1)=ml_control(1)-0.5f;
        }




        if(ml_control(2)>2000.0f) {
            if ((*Output)(2) - ml_control(2) > 200.0f) {
                (*Output)(2) = ml_control(2) + 200.0f;
            }
            if ((*Output)(2) - ml_control(2) < -200.0f) {
                (*Output)(2) = ml_control(2) - 200.0f;
            }
        }



        if(pos_Sp(2)<0.1)
        {

            (*Output)(0)=0.0f;
            (*Output)(1)=0.0f;
            (*Output)(2)=0.0f;

        }


        ml_control(0)=(*Output)(0);
        ml_control(1)=(*Output)(1);
        ml_control(2)=(*Output)(3);

        Cf_csv <<time<<","<<pos_Sp(0)<<","<<pos_Sp(1)<<","<<pos_Sp(2)<<","<<x_temp_est <<","<< y_temp_est << "," << z_temp_est << "," <<vel_estVicon(0)<< "," << vel_estVicon(1)
               << "," << vel_estVicon(2) << "," << vel_Sp(0) << "," << vel_Sp(1)<<"," << vel_Sp(2)
               << "," << (*Output)(0)<<"," << (*Output)(1)<<"," << (*Output)(3)<<","<<Euler(0)<<","<<Euler(1)<<","<<Euler(2)<<"\n";

    }
//    return *Output;
}
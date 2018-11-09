#include <RoleCommander.h>
#include <string>
// #include <stdio.h> //sprintf
#include <sstream>
#include <std_srvs/Trigger.h>
//#include <cmuswarm_msgs/BehaviourRequest.h>
//#include <cmuswarm_msgs/ObjectCoverageLocation.h>
//#include <cmuswarm_msgs/ObjectCoverageLocations.h>
#include <iostream>
#include <math.h>

#include <thread>
#include <chrono>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <Hungarian.h>
// #include <ros/ros.h>

using namespace std;
static int DEFAULT_RATE = 50;
static int MAX_QUEUE_SIZE = 10;
static int ASSIGNMENT_INTERVAL = 3;
static int M_INF = 100;
// static int CONNECT_RADIUS = 16;

RoleCommander::RoleCommander() {
    /*ros initlization*/
    // ros::NodeHandle local("cpp_role_commander");
    ros::NodeHandle global;
    this->global = global;

    ROS_INFO("~~~~~~~~~~Initializing Role Commander");

    /*establish the assignment communication channels*/
    this->assignment_command_pub = global.advertise<std_msgs::Int32MultiArray>("/role_assignment_command",
                                                                               MAX_QUEUE_SIZE);
    this->assignment_command_sub = global.subscribe("/role_assignment_request", MAX_QUEUE_SIZE,
                                                    &RoleCommander::ra_request_cb, this);

    this->active = true;
    this->enable_assign = false;
    // this->enable_assign = true;
    // all_position.resize(8);
    this->isFirst = true;
    this->execute_time = ros::Time::now().toSec();
    this->request_time = this->execute_time;
}

void RoleCommander::start() {
    ros::Rate r(DEFAULT_RATE);

    while(ros::ok()) {
    	
        if (this->enable_assign && this->active)
        {

            // ROS_INFO("*************1");/
            /* obtain informaiton for every crazyflie*/
//
//            pair<double, double> all_robot_position;

            /*Generate formation*/
            int ROBOT_MAX = all_position.size();
            Eigen::MatrixXd formation(ROBOT_MAX, 2);
            double target_position[2];
            target_position[0] = 0.0;
            target_position[1] = 0.0;
            /*Generate formation*/
            auto n1 = ROBOT_MAX - 1;
            double CIRCLE_RADIUS = m_formation_scale;

            // Circle case
            if (formation_type == "circle")
            {
                Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(ROBOT_MAX,0,n1).array()*(2*M_PI/ROBOT_MAX);
                formation << CIRCLE_RADIUS * t.array().cos(), CIRCLE_RADIUS * t.array().sin(); //formation
            }
            else if (formation_type == "square") // Square case
            {
                double total_length = m_formation_scale * 4.0;
                double x = -(m_formation_scale/2.0);
                double y = -(m_formation_scale/2.0);
                double dl = total_length / double(ROBOT_MAX);
                double prev_l = 0;
                double dx, dy;
                for(int i = 0;i < ROBOT_MAX;i++)
                {
                    double l = dl * (i+1);
                    if(l <= total_length/4.0)
                    {
                        // go right
                        // ROS_INFO("go right");
                        x = x + dl;
                        y = -(m_formation_scale/2.0);
                    }
                    else if(l > total_length/4.0 && l <= total_length/2.0)
                    {
                        // go up
                        // ROS_INFO("go up");
                        if(prev_l < total_length/4.0)
                        {
                            dx = m_formation_scale/2.0 - x;
                            dy = dl - dx;
                            x = m_formation_scale/2.0;
                            y = y + dy;
                        }
                        else{
                            y = y + dl;
                        }

                    }
                    else if(l > total_length/2.0 && l <= total_length * 3.0/4.0)
                    {
                        // go left
                        // ROS_INFO("go left");
                        if(prev_l < total_length/2.0){
                            dy = m_formation_scale/2.0 - y;
                            dx = dl - dy;
                            y = m_formation_scale/2.0;
                            x = x - dx;
                        }
                        else{
                            x = x - dl;
                        }
                    }
                    else{
                        // go down
                        // ROS_INFO("go down");
                        if(prev_l < total_length * 3.0/4.0){
                            dx = x + m_formation_scale/2.0;
                            dy = dl - dx;
                            x = -(m_formation_scale/2.0);
                            y = y - dy;
                        }
                        else{
                            y = y - dl;
                        }
                    }

                    formation(i,0) = x;
                    formation(i,1) = y;
                    prev_l = l;
                }
            }
            else{
                ROS_ERROR("generate formation fails!");
            }
            for (int i = 0; i < ROBOT_MAX; ++i)
            {
                cout<<formation(i,0)<<" "<<formation(i,1)<<endl;
            }
            /*
             *Solve the role assignment
             *Input: all_position, formation; Output: assignment
             */

            // ROS_INFO("*************2");

            std::vector<std::vector<double>> cost(ROBOT_MAX, std::vector<double>(ROBOT_MAX,1));
            for (int i = 0; i < ROBOT_MAX; ++i) // agents
            {
                for (int j = 0; j < ROBOT_MAX; ++j) // targets
                {
                    cost[i][j] = sqrt(pow(formation(j,0)+target_position[0]-all_position[i].first, 2)+pow(formation(j,1)+target_position[1]-all_position[i].second, 2));
                }
            }

            std::vector<int> assignment_t = assignment;
            HungarianAlgorithm HungAlgo;
            bool stop = false;
            while(!stop){
                HungAlgo.Solve(cost, assignment_t);

                /*find the maximum entity in the record*/
                double max_t = -1;
                for (int i = 0; i < ROBOT_MAX; ++i)
                {
                    double value_t = cost[i][assignment_t[i]];
                    if (max_t < value_t || max_t < 0)
                    {
                        max_t = value_t;
                    }
                }

                if (max_t < M_INF)
                {
                    for (int i = 0; i < ROBOT_MAX; ++i) // agents
                    {
                        for (int j = 0; j < ROBOT_MAX; ++j) // targets
                        {
                            double cost_ij = cost[i][j];
                            if (cost_ij > (max_t-0.01) && cost_ij < M_INF)
                            {
                                cost[i][j] = M_INF+0.01;
                            }
                        }
                    }
                    assignment = assignment_t;
                }else{
                    stop = true;
                }
            }


            // ROS_INFO("*************4");
            /*send out the result*/
            std_msgs::Int32MultiArray assign_msg;
            assign_msg.data.clear();
            for (int k = 0; k < assignment_id.size(); ++k) {
                assign_msg.data.push_back(assignment_id[k]);
            }
            for (int i = 0; i < assignment.size(); ++i)
            {
                assign_msg.data.push_back(assignment[i]);
            }


            assignment_command_pub.publish(assign_msg);

            // ROS_INFO("*************** send out assignment from the commander!");
            cout<<"commander return : ";
            for (int i = 0; i < all_position.size(); ++i)
            {
                cout<<unsigned(assignment_id[i])<<" : "<<formation(assignment[i],0)<<", "<<formation(assignment[i],1)<<" ";
            }
            cout<<endl;
            this->enable_assign = false;
        }

                        
        ros::spinOnce();
        r.sleep();
    }
}


void RoleCommander::ra_request_cb(const crazyflie_driver::IdPos &msg){
    
    this->request_time = ros::Time::now().toSec();
    if ((this->request_time - this->execute_time) > ASSIGNMENT_INTERVAL)
    {

//        ROS_INFO("!!!!!!!recieve");
        cout<<"recieve uav_info : ";
        int number = msg.id.size();
        assignment_id.clear();
        all_position.clear();
        for (int i = 0; i < number; ++i) {
            
            assignment_id.push_back(msg.id[i]);
            
            std::pair<double, double> tmp = make_pair(msg.x[i],msg.y[i]);
            all_position.push_back(tmp);
            cout<< unsigned(assignment_id[i]) << "~" << tmp.first << "," << tmp.second << " ";
        }
        
        cout<<endl;

        if (this->isFirst)
        {
            this->isFirst = false;
            this->assignment.resize(number);
        }

        this->enable_assign = true;
        this->execute_time = this->request_time;
    }
}


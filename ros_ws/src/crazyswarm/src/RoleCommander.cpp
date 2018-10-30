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
#include <std_msgs/Empty.h>

#include <Hungarian.h>
// #include <ros/ros.h>

using namespace std;
static int DEFAULT_RATE = 50;
static int MAX_QUEUE_SIZE = 50;
static int ASSIGNMENT_INTERVAL = 5;
static int M_INF = 100;
// static int CONNECT_RADIUS = 16;

RoleCommander::RoleCommander() {
    /*ros initlization*/
    // ros::NodeHandle local("cpp_role_commander");
    ros::NodeHandle global("");
    // this->local = local;
    this->global = global;

    ROS_INFO("Initializing Role Commander");

    this->swarm_prefix = global.getParam("/swarm_prefix",swarm_prefix);
    if(!global.getParam("/swarm_prefix",swarm_prefix)) {
        ROS_WARN("/swarm_prefix "
                 "parameter not set. Default of %s used", swarm_prefix.c_str());
    }

    /*obtain the number of robots*/
    XmlRpc::XmlRpcValue robots;
    if(!global.getParam("/robot_ids", robots)){
        ROS_FATAL("Specify robot ids");
        exit(1);
    }
    this->number_of_robots = robots.size();
    
    /*preparation for collecting data from gazebo and user*/
//    this->control_request_sub = global.subscribe("/swarmbot0/cpp_role_formation_controller/control_requests/",MAX_QUEUE_SIZE,&RoleCommander::control_request_cb, this);
//    this->model_states_sub = global.subscribe("/gazebo/model_states", MAX_QUEUE_SIZE, &RoleCommander::model_states_cb, this);

    /*establish the assignment communication channels*/
    this->assignment_command_pub_v.resize(number_of_robots);
    this->assignment_command_sub = global.subscribe("/role_assignment_request", MAX_QUEUE_SIZE, &RoleCommander::ra_request_cb, this);

    for (int i = 0; i < this->number_of_robots; ++i)
    {
    	// sprintf(msg_name,"%s%d/bid_to_%d",this->swarm_prefix,this->robot_id,i);
    	// this->neighbor_agents_pub_v[i] = local.advertise<cmuswarm_msgs::BidMessage>(msg_name,MAX_QUEUE_SIZE);
    	std::stringstream pub_topic;
    	pub_topic << "/swarmbot" << i << "/assignment" ;
    	this->assignment_command_pub_v[i] = global.advertise<std_msgs::Int32MultiArray>(pub_topic.str(),MAX_QUEUE_SIZE);
    }

    this->active = false;
    this->enable_assign = true;    
    this->execute_time = ros::Time::now().toSec();
    this->request_time = this->execute_time;

    // this->connect_assignment.resize(this->number_of_robots);
    // for (int i = 0; i < this->number_of_robots; ++i)
    // {
    //     connect_assignment[i] = i;
    // }
    ROS_INFO("*************1");
}

void RoleCommander::start() {
    ros::Rate r(DEFAULT_RATE);

    while(ros::ok()) {
    	
        if (this->enable_assign && this->active)
        {

            // ROS_INFO("*************2");
            /* obtain informaiton for every crazyflie*/
            vector<robot_info> all_state;
//            preprocess(all_state);
//
//            pair<double, double> all_robot_position;
            vector<Eigen::Vector2f> all_position;
            for(int i = 0; i < all_state.size(); i++){
//                all_robot_position = make_pair(all_state[i].pose.position.x,all_state[i].pose.position.y);
                Eigen::Vector2f tmp(all_state[i].pose.position.x,all_state[i].pose.position.y);
                all_position.push_back(tmp);
            }

            /*Generate formation*/
            int ROBOT_MAX = all_state.size();
            auto n1 = ROBOT_MAX - 1;
            double CIRCLE_RADIUS = this->circle_radius;
            Eigen::MatrixXd formation(ROBOT_MAX, 2);
            double target_position[2];

            // Circle case
            if (this->formation_type == "circle")
            {
                Eigen::VectorXd t = Eigen::VectorXd::LinSpaced(ROBOT_MAX,0,n1).array()*(2*M_PI/ROBOT_MAX);
                formation << CIRCLE_RADIUS * t.array().cos(), CIRCLE_RADIUS * t.array().sin(); //formation
                target_position[0] = this->formation_center[0];
                target_position[1] = this->formation_center[1];
            }
            else if (this->formation_type == "square") // Square case
            {
                double total_length = this->square_length * 4.0; 
                double x = -(this->square_length/2.0); 
                double y = -(this->square_length/2.0); 
                double dl = total_length / (double) ROBOT_MAX; 
                double prev_l = 0;
                double dx, dy;
                for(int i = 0;i < ROBOT_MAX;i++)
                {
                    double l = dl * (i+1);    
                    if(l < total_length/4.0)
                    {
                        // go right
                        x = x + dl;
                        y = -(this->square_length/2.0);
                    }
                    else if(l >= total_length/4.0 && l < total_length/2.0)
                    {
                        // go up
                        if(prev_l < total_length/4.0)
                        {
                            dx = this->square_length/2.0 - x;
                            dy = dl - dx;
                            x = this->square_length/2.0;
                            y = y + dy;
                        }
                        else{
                            y = y + dl;
                        }
                        
                    }
                    else if(l >= total_length/2.0 && l < total_length * 3.0/4.0)
                    {
                        // go left
                        if(prev_l < total_length/2.0){
                            dy = this->square_length/2.0 - y;
                            dx = dl - dy;
                            y = this->square_length/2.0;
                            x = x - dx;
                        }
                        else{
                            x = x - dl;
                        }
                    }
                    else{
                        // go down
                        if(prev_l < total_length * 3.0/4.0){
                            dx = x + this->square_length/2.0;
                            dy = dl - dx;
                            x = -(this->square_length/2.0);
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
                target_position[0] = this->formation_center[0];
                target_position[1] = this->formation_center[1];
            }
            else{
                continue;
            }

            /*
             *Solve the role assignment
             *Input: all_state, formation; Output: assignment
             */

            // ROS_INFO("*************3");

            std::vector<std::vector<double>> cost(ROBOT_MAX, std::vector<double>(ROBOT_MAX,1));
            for (int i = 0; i < ROBOT_MAX; ++i) // agents
            {
                for (int j = 0; j < ROBOT_MAX; ++j) // targets
                {
                    cost[i][j] = sqrt(pow(formation(j,0)+target_position[0]-all_position[i][0], 2)+pow(formation(j,1)+target_position[1]-all_position[i][1], 2));
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
            for (int i = 0; i < assignment.size(); ++i)
            {
                assign_msg.data.push_back(assignment[i]);
            }
            for (int i = 0; i < ROBOT_MAX; ++i)
            {
                assignment_command_pub_v[i].publish(assign_msg);
            }
            ROS_INFO("*************** send out assignment from the commander!");
            this->enable_assign = false;
        }

                        
        ros::spinOnce();
        r.sleep();
    }
}

//void RoleCommander::control_request_cb(const cmuswarm_msgs::BehaviourRequest &req) {
//    if (req.parameters.strs.size() == 0)
//    {
//        ROS_WARN("Specify Formation Type!");
//        return;
//    }
//    std::string type = req.parameters.strs[0].value;
//
//    if (type == "circle")
//    {
//        if (req.parameters.doubles.size() != 3)
//        {
//            ROS_WARN("must contain parameters x,y,r");
//            return;
//        }
//        for (int i = 0; i < 3; ++i)
//        {
//            if (req.parameters.doubles[i].name == "x")
//            {
//                this->formation_center[0] = req.parameters.doubles[i].value;
//            }
//            else if (req.parameters.doubles[i].name == "y")
//            {
//                this->formation_center[1] = req.parameters.doubles[i].value;
//            }
//            else if (req.parameters.doubles[i].name == "r")
//            {
//                this->circle_radius = req.parameters.doubles[i].value;
//            }
//            this->formation_type = type;
//        }
//    }
//    else if (type == "square")
//    {
//        if (req.parameters.doubles.size() != 3)
//        {
//            ROS_WARN("must contain parameters x,y,l");
//            return;
//        }
//        for (int i = 0; i < 3; ++i)
//        {
//            if (req.parameters.doubles[i].name == "x")
//            {
//                this->formation_center[0] = req.parameters.doubles[i].value;
//            }
//            else if (req.parameters.doubles[i].name == "y")
//            {
//                this->formation_center[1] = req.parameters.doubles[i].value;
//            }
//            else if (req.parameters.doubles[i].name == "l")
//            {
//                this->square_length = req.parameters.doubles[i].value;
//            }
//            this->formation_type = type;
//        }
//
//    }
//    else
//    {
//        ROS_WARN("currently only support square and circle");
//        return;
//    }
//    // this->start_time = ros::Time::now().toSec();
//    this->active = true;
//    ROS_INFO("CPP Role Assignment Formation Controller activated.");
//}


void RoleCommander::ra_request_cb(const std_msgs::Empty &msg){
    this->request_time = ros::Time::now().toSec();
    if ((this->request_time - this->execute_time) > ASSIGNMENT_INTERVAL)
    {
        this->enable_assign = true;
        this->execute_time = this->request_time;
    }
}

//void RoleCommander::model_states_cb(const gazebo_msgs::ModelStates &states) {
//    this->curr_model_states = states;
//}
//
//
//void RoleCommander::preprocess(vector<robot_info> &all_state) {
//    if(this->curr_model_states.pose.size()==0) {
//        ROS_INFO("Awaiting model information for role assignment");
//    } else {
//    	all_state.resize(this->number_of_robots);
//        cmuswarm_msgs::ObjectCoverageLocations neighbours; //keeping track of neighbours so they can be broadcasted
//        pair<double, double> curr_obby;
//        robot_info all_robot;
//
//        for(int index = 0; index < this->curr_model_states.pose.size(); index++) {
//            //record this robot as a coverage region to send to collision calculator
//            string curr_model_name = this->curr_model_states.name[index];
//            if(curr_model_name.find(this->swarm_prefix) != -1){
//                all_robot.name = curr_model_name;   //now other robot contains our current robot
//                all_robot.pose = this->curr_model_states.pose[index];
//                int id = this->get_index(all_robot.name);
//                all_state[id] = all_robot;
//
//                // all_state.push_back(all_robot);
//                cmuswarm_msgs::ObjectCoverageLocation neighbour;
//                neighbour.name = curr_model_name;
//                neighbour.location.x = this->curr_model_states.pose[index].position.x;
//                neighbour.location.y = this->curr_model_states.pose[index].position.y;
//                neighbour.radius = 0.16;  //TODO: Make the robots radius not static. should be read from param server
//                neighbours.locations.push_back(neighbour);
//            }
//            // else if(curr_model_name.find("obby") != -1){
//            //     cmuswarm_msgs::ObjectCoverageLocation neighbour;
//            //     neighbour.name = curr_model_name;
//            //     neighbour.location.x = this->curr_model_states.pose[index].position.x;
//            //     neighbour.location.y = this->curr_model_states.pose[index].position.y;
//            //     neighbour.radius = 0.5; //TODO: read this off param server... we know for now obstacles have radius of 0.5
//            //     neighbours.locations.push_back(neighbour);
//            //     curr_obby = make_pair(neighbour.location.x,neighbour.location.y);
//            //     this->obby.push_back(curr_obby);
//            // }
//        }
//        // this->neighbours_pub.publish(neighbours);
//    }
//}

//int RoleCommander::get_index(std::string name){
//	// assuming the naming structure is prefix + id
//	int swarm_prefix_size = this->swarm_prefix.size();
//	int robot_name_size = name.size();
//	std::string index = name.substr(swarm_prefix_size,  robot_name_size);
//	return std::atoi(index.c_str());
//}

/**
* CLASS: RoleCommander
* DATE: 10/30/18
* AUTHOR: Weifan Zhang
* DESCRIPTION: A centralized role assignment commander using C++.
*/

#ifndef RoleCommander_H
#define RoleCommander_H
#include <ros/ros.h>
//#include <cmuswarm_msgs/BehaviourRequest.h>
//#include <cmuswarm_msgs/ObjectCoverageLocations.h>
//#include <cmuswarm_msgs/ObjectCoverageLocation.h>
//#include <std_srvs/Trigger.h>
#include <vector>
#include <utility>
//#include <gazebo_msgs/ModelStates.h>
#include <Eigen/Dense>
#include <eiquadprog.h>

// #include <std_msgs/Int8MultiArray.h>
// #include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>

#include "crazyflie_driver/IdPos.h"
/**
 * The ROS wrapper around a controller which will take in message and adapt them to the controllers input.
 */
class RoleCommander {
private:
    ros::NodeHandle global;
    // ros::NodeHandle local;

    std::string swarm_prefix = "robot";

//    /**
//     * Subscribe to the model states in the environment.
//     */
//    ros::Subscriber model_states_sub;
//    /**
//     * This will receive callbacks saying that we wish to begin the controller
//     */
//    ros::Subscriber control_request_sub;
    bool active;

    /**
     * The most recent state information received.
     */
//    gazebo_msgs::ModelStates curr_model_states;

    int number_of_robots;

    /**
     * Parameters of role assignment
     */
    bool enable_assign;
    bool isFirst;
    // double start_time;
    double request_time;
    double execute_time;
    ros::Publisher assignment_command_pub;
    ros::Subscriber assignment_command_sub;

    std::vector<int> assignment;
    std::vector<uint8_t> assignment_id;
    std::vector<std::pair<double, double>> all_position;

    /*
     * Parameters of formation
     */
    std::string formation_type = "square";
    double m_formation_scale = 1.5;

    /**
     * obtain the swarm positon from other node
     */
//    void preprocess(std::vector<robot_info> &all_state);

//    int get_index(std::string name);
public:

    /**
     * Initialize the controller.
     * @param robot_name the name of the robot (excluding swarm prefix).
     */
    RoleCommander();

    // std::string robot_id_to_topic(int robot_id, std::string topic_prefix);
    /**
     * Default starting function. Just call this to begin running the nodes main loop.
     */
    void start();
    //void controlLoop(double &u_v, double &u_w, std::pair<double, double> self_position, double self_heading, std::vector<std::pair<double, double>> position, std::vector<double> heading, std::vector<std::pair<double, double>> obstacle_centroids, std::vector<double> obstacle_radii);

//    void control_request_cb(const cmuswarm_msgs::BehaviourRequest &req);
//
//    void model_states_cb(const gazebo_msgs::ModelStates &states);

    void ra_request_cb(const crazyflie_driver::IdPos &msg);
};

#endif //RoleCommander_H

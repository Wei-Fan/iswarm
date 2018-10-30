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
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>

/**
 * This may be the actual lower level controller which does heavy calculations.
 */
struct robot_info {
    std::string name;
    geometry_msgs::Pose pose;
};
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
    // std::vector<int> assignment;
    bool enable_assign;
    // double start_time;
    double request_time;
    double execute_time;
    std::vector<ros::Publisher> assignment_command_pub_v;
    ros::Subscriber assignment_command_sub;

    std::vector<int> assignment;

    /*
     * Parameters of formation
     */
    std::string formation_type = "circle";
    double formation_center[2] = {0,0}; 
    double circle_radius = 0;
    double square_length = 0; 

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

    void ra_request_cb(const std_msgs::Empty &msg);
};

#endif //RoleCommander_H

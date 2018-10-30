#include <RoleCommander.h>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "role_commander");

    RoleCommander node;
    node.start();

    ros::spin();
    return 0;
}

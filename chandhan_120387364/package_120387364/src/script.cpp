#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <thread>
#include <chrono>
#include <vector> 

class Script : public rclcpp::Node
{
public:
    Script()
    : Node("script"){}

    void init()
    {

        // Define initial and target positions for the robot arm.
        std::vector<float> pose_1 = {-0.36355, -0.68732, 0.135, 0.0, 1.0, 0.0, 0.0};
        std::vector<float> pose_2 = {-0.17398, -0.68732, 0.135, 0.0, 1.0, 0.0, 0.0};

        // Initialize MoveGroupInterface for arm and hand of Panda robot.
        Panda_robot_arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "panda_arm");
        Panda_robot_hand = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "panda_hand");

        // Sequence of robot arm and hand movements.
        Open_hand();
        rclcpp::sleep_for(std::chrono::seconds(1));
        Move_to_position(pose_1);
        rclcpp::sleep_for(std::chrono::seconds(1));
        Close_hand();
        rclcpp::sleep_for(std::chrono::seconds(1));
        Move_to_position(pose_2);
        rclcpp::sleep_for(std::chrono::seconds(1));
        Open_hand();
        rclcpp::sleep_for(std::chrono::seconds(1));
        Home_position();
        rclcpp::sleep_for(std::chrono::seconds(1));
        Close_hand();

    }

private:

    // Members to hold move group interfaces for the robot's arm and hand
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> Panda_robot_arm;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> Panda_robot_hand;

    void Move_to_position(const std::vector<float>& pose)
    {
        // Specify the target pose for the end effector of the Panda_robot_arm
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = pose[3];
        target_pose.orientation.x = pose[4] ;
        target_pose.orientation.y = pose[5] ;
        target_pose.orientation.z = pose[6] ;
        target_pose.position.x = pose[0];
        target_pose.position.y = pose[1];
        target_pose.position.z = pose[2];

        // Set the target pose and plan to it
        Panda_robot_arm->setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan Panda_robot_arm_plan;
        Panda_robot_arm->plan(Panda_robot_arm_plan);

        // Execute the planned trajectory
        Panda_robot_arm->execute(Panda_robot_arm_plan);

    }

    // Function to open the robot hand (gripper)
    void Open_hand()
    {

        // Named target given in rviz
        Panda_robot_hand->setNamedTarget("open_gripper");
        Panda_robot_hand->move();

    }

    // Function to close the robot hand (gripper)
    void Close_hand()
    {

        // Named target given in rviz
        Panda_robot_hand->setNamedTarget("closed_gripper");
        Panda_robot_hand->move();

    }

    // Function to return the robot arm to its home position
    void Home_position()
    {

        // Named target given in rviz
        Panda_robot_arm->setNamedTarget("home_arm");
        Panda_robot_arm->move();

    }

    
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Script>();
    
    std::thread([&node]() {
        // Allow the node to fully start before calling init()
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        node->init();
    }).detach();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

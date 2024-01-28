#include <common/yaml.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <ros/ros.h>
//#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <control_baseline.h>

namespace autoplan
{
    namespace control
    {
        class ControlNode
        {
            public:
            ControlNode(int argc, char **argv, const std::string &config_file);

            void Initconfig(const std::string &config_file);

            void callbackTrajectory(const nav_msgs::Path::ConstPtr &msg);

            void callbacklocation(const nav_msgs::Odometry::ConstPtr &msg);

            void callbackVelocity(const std_msgs::Float32::ConstPtr &msg);

            void ControlCmdHandle(const carla_msgs::CarlaEgoVehicleControl &msg);

            private:
                std::shared_ptr<ros::NodeHandle> nh;
                std::shared_ptr<controlbaseline> controlbaseline_;

                std::string role;
                std::string config_file_;
                std::string trajectory_topic_;
                std::string localization_topic_;
                std::string control_cmd_topic_;
                std::string velocity_topic_;
                
                ros::Subscriber sub_trajectory;
                ros::Subscriber sub_localization;
                ros::Subscriber sub_Velocity;
                ros::Publisher pub_control_cmd;
                
                Config config_;           

        };
    }
}
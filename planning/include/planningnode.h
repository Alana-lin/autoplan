#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <common/yaml.h>
#include <planning_baseline.h>

namespace autoplan
{
    namespace planning
    {
        class planingnode
        {
        public:
            planingnode(int argc, char **argv, const std::string &config_file);

            void Initconfig(const std::string &config_file);

            void callbacklocalizationEgo(const nav_msgs::Odometry::ConstPtr &msg);

            void callbacklocalizationAgent(const nav_msgs::Odometry::ConstPtr &msg);

            void callbackreferpathEgo(const nav_msgs::Path::ConstPtr &msg);

            void callbackreferpathAgent(const nav_msgs::Path::ConstPtr &msg);

            void callbackreferVelEgo(const std_msgs::Float32::ConstPtr &msg);

            void callbackreferVelAgent(const std_msgs::Float32::ConstPtr &msg);

            void predictonhandle(const nav_msgs::Path &msg);

            void trajectoryhandle(const nav_msgs::Path &msg);

        private:
            std::shared_ptr<ros::NodeHandle> nh;
            std::shared_ptr<PlanningBaseline> planningbaseline_;
            std::string config_file_;
            Config config_;

            // nav_msgs::Odometry locatlization_ego;
            // nav_msgs::Odometry locatlization_agent;
            //  nav_msgs::Path referpath_ego;
            //  nav_msgs::Path referpath_agent;
            //  nav_msgs::Path trajectory_;
            //  nav_msgs::Path prediction_;

            std::string locatlization_topic_ego;
            std::string locatlization_topic_agent;
            std::string referpath_topic_ego;
            std::string referpath_topic_agent;
            std::string refervel_topic_ego;
            std::string refervel_topic_agent;
            std::string prediction_topic_;
            std::string trajectory_topic_;

            ros::Subscriber sub_locatlization_ego;
            ros::Subscriber sub_locatlization_agent;
            ros::Subscriber sub_referpath_ego;
            ros::Subscriber sub_referpath_agent;
            ros::Subscriber sub_refervel_ego;
            ros::Subscriber sub_refervel_agent;

            ros::Publisher pub_prediction_agent;
            ros::Publisher pub_trajectory;
        };
    }
}
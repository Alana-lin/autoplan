#include <control_node.h>

namespace autoplan
{
    namespace control
    {
        static const char *NODE_NAME = "control_node";
        ControlNode::ControlNode(int argc, char **argv, const std::string &config_file)
        {
            ros::init(argc, argv, NODE_NAME);
            nh = std::make_shared<ros::NodeHandle>("~");
            Initconfig(config_file);

            sub_trajectory = nh->subscribe<nav_msgs::Path>(
                trajectory_topic_, 1, std::bind(&ControlNode::callbackTrajectory, this, std::placeholders::_1));

            sub_localization = nh->subscribe<nav_msgs::Odometry>(
                localization_topic_, 1, std::bind(&ControlNode::callbacklocation, this, std::placeholders::_1));

            sub_Velocity = nh->subscribe<std_msgs::Float32>(
                velocity_topic_, 1, std::bind(&ControlNode::callbackVelocity, this, std::placeholders::_1));

            pub_control_cmd = nh->advertise<carla_msgs::CarlaEgoVehicleControl>(control_cmd_topic_, 1);
            controlbaseline_->setChassisCallback(
                std::bind(&ControlNode::ControlCmdHandle, this, std::placeholders::_1));
            controlbaseline_->Process();
        }

        void ControlNode::callbackTrajectory(const nav_msgs::Path::ConstPtr &msg)
        {
            // nav_msgs::Path trajectory_;
            // trajectory_ = *msg;
            controlbaseline_->settrajectory(*msg);
        }

        void ControlNode::callbacklocation(const nav_msgs::Odometry::ConstPtr &msg)
        {
            // nav_msgs::Odometry localization_;
            // localization_ = *msg;
            controlbaseline_->setlocatlization(*msg);
        }

        void ControlNode::callbackVelocity(const std_msgs::Float32::ConstPtr &msg)
        {
            // std_msgs::Float32 velocity_;
            // velocity_ = *msg;
            std_msgs::Float32 vel;
            vel.data = 5;
            controlbaseline_->getControlbase()->setVelocity(vel);
        }

        void ControlNode::ControlCmdHandle(const carla_msgs::CarlaEgoVehicleControl &msg)
        {
            pub_control_cmd.publish(msg);
        }

        void ControlNode::Initconfig(const std::string &config_file)
        {
            nh->getParam("role", role);
            std::string carla = "/carla/";
            yaml::Node config_node = yaml::LoadFile(config_file);
            localization_topic_ = carla + role + config_node["localization_topic"].as<std::string>(localization_topic_);
            trajectory_topic_ = carla + role + config_node["trajectory_topic"].as<std::string>(trajectory_topic_);
            control_cmd_topic_ = carla + role + config_node["control_cmd_topic"].as<std::string>(control_cmd_topic_);
            velocity_topic_ = carla + role + config_node["velocity_topic"].as<std::string>(velocity_topic_);

            config_.Puresuit_K = config_node["puresuit_K"].as<double>(config_.Puresuit_K);
            config_.Puresuit_Lfc = config_node["puresuit_Lfc"].as<double>(config_.Puresuit_Lfc);
            config_.Kpb = config_node["PID_Kpb"].as<double>(config_.Kpb);
            config_.Kib = config_node["PID_Kib"].as<double>(config_.Kib);
            config_.Kdb = config_node["PID_Kdb"].as<double>(config_.Kdb);
            config_.Wheelbase_L = config_node["Wheelbase_L"].as<double>(config_.Wheelbase_L);

            controlbaseline_ = std::make_shared<controlbaseline>();
            controlbaseline_->Init(config_);
        }

    }
}

int main(int argc, char **argv)
{
    std::string config_file = "/home/alan/autoplan/src/control/config/control_node.yaml";
    autoplan::control::ControlNode control_node(argc, argv, config_file);
}
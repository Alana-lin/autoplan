#include <planningnode.h>

namespace autoplan
{
    namespace planning
    {
        static const char *NODE_NAME = "autoplan_node";
        planingnode::planingnode(int argc, char **argv, const std::string &config_file)
        {
            ros::init(argc, argv, NODE_NAME);
            nh = std::make_shared<ros::NodeHandle>();
            Initconfig(config_file);

            sub_locatlization_ego = nh->subscribe<nav_msgs::Odometry>(
                locatlization_topic_ego, 1, std::bind(&planingnode::callbacklocalizationEgo, this, std::placeholders::_1));

            sub_locatlization_agent = nh->subscribe<nav_msgs::Odometry>(
                locatlization_topic_agent, 1, std::bind(&planingnode::callbacklocalizationAgent, this, std::placeholders::_1));

            sub_referpath_ego = nh->subscribe<nav_msgs::Path>(
                referpath_topic_ego, 1, std::bind(&planingnode::callbackreferpathEgo, this, std::placeholders::_1));

            sub_referpath_agent = nh->subscribe<nav_msgs::Path>(
                referpath_topic_agent, 1, std::bind(&planingnode::callbackreferpathAgent, this, std::placeholders::_1));

            sub_refervel_ego = nh->subscribe<std_msgs::Float32>(
                refervel_topic_ego, 1, std::bind(&planingnode::callbackreferVelEgo, this, std::placeholders::_1));

            sub_refervel_agent = nh->subscribe<std_msgs::Float32>(
                refervel_topic_agent, 1, std::bind(&planingnode::callbackreferVelAgent, this, std::placeholders::_1));

            pub_trajectory = nh->advertise<nav_msgs::Path>(trajectory_topic_, 1);
            planningbaseline_->setPrePathEgocallback(
                std::bind(&planingnode::trajectoryhandle, this, std::placeholders::_1));

            pub_prediction_agent = nh->advertise<nav_msgs::Path>(prediction_topic_, 1);
            planningbaseline_->setPrePathAgentcallback(
                std::bind(&planingnode::predictonhandle, this, std::placeholders::_1));
            planningbaseline_->Process();
        }

        void planingnode::callbacklocalizationEgo(const nav_msgs::Odometry::ConstPtr &msg)
        {
            planningbaseline_->setodom_ego(*msg);
        }

        void planingnode::callbacklocalizationAgent(const nav_msgs::Odometry::ConstPtr &msg)
        {
            planningbaseline_->setodom_agent(*msg);
        }

        void planingnode::callbackreferpathEgo(const nav_msgs::Path::ConstPtr &msg)
        {
            // referpath_ego = *msg;
            planningbaseline_->setref_path_ego(*msg);
        }

        void planingnode::callbackreferpathAgent(const nav_msgs::Path::ConstPtr &msg)
        {
            planningbaseline_->setref_path_agent(*msg);
            // referpath_agent = *msg;
        }

        void planingnode::callbackreferVelEgo(const std_msgs::Float32::ConstPtr &msg)
        {
            planningbaseline_->setref_vel_ego(*msg);
        }

        void planingnode::callbackreferVelAgent(const std_msgs::Float32::ConstPtr &msg)
        {
            planningbaseline_->setref_vel_agent(*msg);
        }

        void planingnode::predictonhandle(const nav_msgs::Path &msg)
        {
            pub_prediction_agent.publish(msg);
            //planningbaseline_->setpre_path_agent(msg);
        }

        void planingnode::trajectoryhandle(const nav_msgs::Path &msg)
        {
            //planningbaseline_->setpre_path_ego(msg);
            pub_trajectory.publish(msg);
        }

        void planingnode::Initconfig(const std::string &config_file)
        {
            yaml::Node config_node = yaml::LoadFile(config_file);
            locatlization_topic_ego = config_node["locatlization_ego_topic"].as<std::string>(locatlization_topic_ego);
            locatlization_topic_agent = config_node["locatlization_agent_topic"].as<std::string>(locatlization_topic_agent);
            referpath_topic_ego = config_node["referpath_ego_topic"].as<std::string>(referpath_topic_ego);
            referpath_topic_agent = config_node["referpath_agent_topic"].as<std::string>(referpath_topic_agent);
            refervel_topic_ego = config_node["refervel_ego_topic"].as<std::string>(refervel_topic_ego);
            refervel_topic_agent = config_node["refervel_agent_topic"].as<std::string>(refervel_topic_agent);
            prediction_topic_ = config_node["prediction_topic"].as<std::string>(prediction_topic_);
            trajectory_topic_ = config_node["trajectory_topic"].as<std::string>(trajectory_topic_);

            config_.dt = config_node["dt"].as<double>(config_.dt);
            config_.K = config_node["K"].as<double>(config_.K);
            config_.predict_horizon = config_node["predict_horizon"].as<int>(config_.predict_horizon);
            config_.r = config_node["r"].as<double>(config_.r);

            planningbaseline_ = std::make_shared<PlanningBaseline>();
            planningbaseline_->Init(config_);
        }
    }
}

int main(int argc, char **argv)
{
    std::string config_file = "/home/alan/autoplan/src/planning/config/planning_node.yaml";
    autoplan::planning::planingnode node(argc, argv, config_file);
}
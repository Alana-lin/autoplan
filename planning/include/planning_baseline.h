#include <morca.h>
#include <config.h>

namespace autoplan
{
    namespace planning
    {
        class PlanningBaseline
        {
        public:
            void Init(Config config);
            void Update(Vehicle &ego, Vehicle &agent, double dt);
            void Process();
            typedef std::function<void(nav_msgs::Path &)> PrePathEgocallback;
            typedef std::function<void(nav_msgs::Path &)> PrePathAgentcallback;
            // typedef std::function<void(autoplan::planning::Vector2D &)>PreVelEgocallback;
            // typedef std::function<void(autoplan::planning::Vector2D &)>PreVelAgentcallback;

            void setodom_ego(nav_msgs::Odometry odom_ego_)
            {
                odom_ego = odom_ego_;
            };

            void setodom_agent(nav_msgs::Odometry odom_agent_)
            {
                odom_agent = odom_agent_;
            };

            void setref_path_ego(nav_msgs::Path ref_path_ego_)
            {
                ref_path_ego = ref_path_ego_;
            };

            void setref_path_agent(nav_msgs::Path ref_path_agent_)
            {
                ref_path_agent = ref_path_agent_;
            };

            void setpre_path_ego(nav_msgs::Path pre_path_ego_)
            {
                pre_path_ego = pre_path_ego_;
            };

            void setpre_path_agent(nav_msgs::Path pree_path_agent_)
            {
                pre_path_agent = pree_path_agent_;
            };

            void setref_vel_ego(std_msgs::Float32 ref_vel_ego_)
            {
                ref_vel_ego = ref_vel_ego_;
            };

            void setref_vel_agent(std_msgs::Float32 ref_vel_agent_)
            {
                ref_vel_agent = ref_vel_agent_;
            };

            void setPrePathEgocallback(PrePathEgocallback handle)
            {
                prePathEgocallback_=handle;
            };

            void setPrePathAgentcallback(PrePathAgentcallback handle)
            {
                prePathAgentcallback_=handle;
            };

            // void setPreVelEgocallback(PreVelEgocallback handle)
            // {
            //     preVelEgocallback_=handle;
            // };

            // void setPreVelAgentcallback(PreVelAgentcallback handle)
            // {
            //     preVelAgentcallback_=handle;
            // };

        private:
            nav_msgs::Path pre_path_ego;   // prediction path of ego
            nav_msgs::Path pre_path_agent; // prediction path of agent

            nav_msgs::Path ref_path_ego;
            nav_msgs::Path ref_path_agent;

            std_msgs::Float32 ref_vel_ego;   // velicity of ego
            std_msgs::Float32 ref_vel_agent; // velicity of agent

            nav_msgs::Odometry odom_ego;
            nav_msgs::Odometry odom_agent;

            PrePathEgocallback prePathEgocallback_;
            PrePathAgentcallback prePathAgentcallback_;
            // PreVelEgocallback preVelEgocallback_;
            // PreVelAgentcallback preVelAgentcallback_;
            double K;
            int predict_horizon;
            double dt;
            double r;
            double ref_vel0, ref_vel1, ref_vel2;
        };
    }
}
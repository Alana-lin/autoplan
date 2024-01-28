#include <planning_baseline.h>

namespace autoplan
{
    namespace planning
    {
        void PlanningBaseline::Init(Config config)
        {
            K = config.K;
            predict_horizon = config.predict_horizon;
            dt = config.dt;
            r = config.r;
        }

        void PlanningBaseline::Update(Vehicle &ego, Vehicle &agent, double dt)
        {
            auto out_ego = ego, out_agent = agent;
            auto ego_v = out_ego.getv();
            auto agent_v = out_agent.getv();
            ego_v.seth(utility::GetHeading(ego, 2));
            agent_v.seth(utility::GetHeading(agent, 2));
            out_ego.setv(ego_v);
            out_agent.setv(agent_v);
            //out_ego.getv().seth(utility::GetHeading(ego, 5));
            //out_agent.getv().seth(utility::GetHeading(agent, 5));
            int co_flag1, co_flag2;
            auto tmp_agent = out_agent, tmp_ego = out_ego;
            out_agent.ORCA(tmp_ego, 2, co_flag2); // std::cout<<"3:"<<out_ego.p.x<<std::endl;
            out_ego.ORCA(tmp_agent, 2, co_flag1); // std::cout<<"2:"<<out_ego.p.x<<std::endl;
            std::cout << "out_ego.u.value: "<<out_ego.getu().getvalue() << "," <<"out_agent.u.value: "<< out_agent.getu().getvalue() << std::endl;
//std::cout<<"out_ego.u.value: "<<out_ego.u.value<<","<<"out_agent.u.value: "<<out_agent.u.value<<std::endl;
            if (co_flag1 || co_flag2)
            {
                const auto &v_a = out_agent.getv().getvalue();
                const auto &v_e = out_ego.getv().getvalue();
                auto beta_e = pow(v_a / v_e, K) / (pow(v_e / v_a, K) + pow(v_a / v_e, K));
                auto beta_a = pow(v_e / v_a, K) / (pow(v_e / v_a, K) + pow(v_a / v_e, K));
                out_ego.setv(out_ego.getv() - beta_e * out_ego.getu());
                out_agent.setv(out_agent.getv() - beta_a * out_agent.getu());
            }
            //utility::RestrictVH(out_ego, ego);
            //utility::RestrictVH(out_agent, agent);

            out_ego.setp(out_ego.getp() + dt * out_ego.getv()); // std::cout<<out_ego.p.x<<std::endl;
            out_agent.setp(out_agent.getp() + dt * out_agent.getv());
            ego = out_ego;
            agent = out_agent;
        }

        void PlanningBaseline::Process()
        {
            pre_path_ego.header.frame_id = "/map";
            pre_path_agent.header.frame_id = "/map";
            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                // ego vehicle
                // odom_ego.pose.pose.orientation;//ori
                geometry_msgs::PoseStamped local_pose_ego;
                local_pose_ego.pose = odom_ego.pose.pose;
                pre_path_ego.poses.push_back(local_pose_ego);
                Vector2D p0;
                p0.setxy(odom_ego.pose.pose.position.x, odom_ego.pose.pose.position.y);
                double v0_value_ = 2;//ref_vel0;
                double v0_heading_ = common::math::GetHeading(odom_ego.pose.pose.orientation);
                Vector2D v0(v0_value_, v0_heading_);
                Vehicle ego_vehicle(r, p0, v0, ref_path_ego);

                // agent vehicle
                geometry_msgs::PoseStamped local_pose_agent;
                local_pose_agent.pose = odom_agent.pose.pose;
                pre_path_agent.poses.push_back(local_pose_agent);
                Vector2D p1;
                p1.setxy(odom_agent.pose.pose.position.x, odom_agent.pose.pose.position.y);
                double v1_value_ = 2;//ref_vel1;
                double v1_heading_ = common::math::GetHeading(odom_agent.pose.pose.orientation);
                Vector2D v1(v1_value_, v1_heading_);
                Vehicle agent_vehicle1(r, p1, v1, ref_path_agent);

                for (int i = 0; i < predict_horizon; i++)
                {
                    Update(ego_vehicle, agent_vehicle1, dt);
                    utility::setpose(local_pose_ego, ego_vehicle);
                    utility::setpose(local_pose_agent, agent_vehicle1);
                    pre_path_ego.poses.push_back(local_pose_ego);
                    pre_path_agent.poses.push_back(local_pose_agent);
                }
                prePathEgocallback_(pre_path_ego);
                prePathAgentcallback_(pre_path_agent);
                // preVelEgocallback_(v0);
                // preVelAgentcallback_(v1);
                pre_path_ego.poses.clear();
                pre_path_agent.poses.clear();
                // v0.clear();
                // v1.clear();
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

    }
}
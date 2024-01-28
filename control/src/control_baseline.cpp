#include <control_baseline.h>

namespace autoplan
{
    namespace control
    {
        void controlbaseline::Init(Config config)
        {
            controlbase_ = std::make_shared<controlbase>();
            controlbase_->Init(config);
        }

        void controlbaseline::Process()
        {
            int lp_rate = 500;
            ros::Rate loop_rate(lp_rate);
            double dt = 1.0 / lp_rate;
            while (ros::ok())
            {
                int index;
                ros::spinOnce();
                heading = common::math::RestrictHeading(common::math::GetHeading(odom.pose.pose.orientation));
                if (odom.pose.pose.position.x != 0 && path.poses.size() > 0)
                {
                    controlbase_->TargetIndex(odom.pose.pose.position.x, odom.pose.pose.position.y,
                                                               odom.twist.twist.linear.x, path);
                    controlbase_->pursuit_controller(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.twist.twist.linear.x,
                                                     heading, path, steer);

                    auto err = controlbase_->getvel().data - odom.twist.twist.linear.x;
                    controlbase_->err_buffer.push_front(err);
                    if (controlbase_->err_buffer.size() > 5)
                    {
                        controlbase_->err_buffer.pop_back();
                    }

                    double long_control = controlbase_->LongPIDcontroller(controlbase_->err_buffer, dt);
                    //std::cout<<"long_control: "<<long_control<<std::endl;
                    control_.throttle = 0;
                    control_.brake = 0;
                    if (long_control >= 0)
                    {
                        control_.throttle = long_control;
                        control_.brake = 0;
                    }
                    else
                    {
                        control_.throttle = 0;
                        control_.brake = 0;
                        if (long_control < -5)
                        {
                            control_.brake = std::max(-long_control, 0.0);
                        }
                    }
                    control_.steer = -steer;
                    control_.gear = 1;
                    // double s, l;
                    // common::math::C2F(odom, path, odom.pose.pose.position.x, odom.pose.pose.position.y, s, l);
                    // if (s < 8 && fabs(l) < 2.5)
                    // {
                    //     if (1)
                    //     {
                    //         control_.throttle = 0;
                    //         control_.brake = 100;
                    //         // std::cout<<role_name<<": Emergency Stop"<<std::endl;
                    //     }
                    // }
                    chassiscallback_(control_);
                }
                loop_rate.sleep();
                ros::spinOnce();
            }//ros::ok();
        }

    } // namespace control
} // naamespace autoplan
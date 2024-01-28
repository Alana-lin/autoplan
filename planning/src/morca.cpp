#include <morca.h>

namespace autoplan
{
    namespace planning
    {
        // class Vector2D
        Vector2D::Vector2D(double value_, double heading_)
        {
            value = value_;
            heading = heading_;
            x = value * cos(heading);
            y = value * sin(heading);
        }

        void Vector2D::setxy(double x_, double y_)
        {
            x = x_;
            y = y_;
            heading = atan2(y, x);
            value = sqrt(x * x + y * y);
        }

        void Vector2D::seth(double heading_)
        {
            heading = heading_;
            x = value * cos(heading);
            y = value * sin(heading);
        }

        void Vector2D::setv(double value_)
        {
            value = value_;
            x = value * cos(heading);
            y = value * sin(heading);
        }

        Vector2D operator+(const Vector2D &a, const Vector2D &b)
        {
            Vector2D res;
            res.x = b.x + a.x;
            res.y = b.y + a.y;
            res.heading = atan2(res.y, res.x);
            res.value = sqrt(res.x * res.x + res.y * res.y);
            return res;
        }

        Vector2D operator-(const Vector2D &a, const Vector2D &b)
        {
            Vector2D res;
            res.x = b.x - a.x;
            res.y = b.y - a.y;
            res.heading = atan2(res.y, res.x);
            res.value = sqrt(res.x * res.x + res.y * res.y);
            return res;
        }

        Vector2D operator-(Vector2D &a)
        {
            a.x = -a.x;
            a.y = -a.y;
            a.heading = atan2(a.y, a.x);
            a.value = sqrt(a.x * a.x + a.y * a.y);
            return a;
        }

        Vector2D operator*(const double &a, const Vector2D &b)
        {
            Vector2D res;
            res.x = b.x * a;
            res.y = b.y * a;
            res.heading = atan2(res.y, res.x);
            res.value = sqrt(res.x * res.x + res.y * res.y);
            return res;
        }

        // class Vehicle
        void Vehicle::ORCA(Vehicle agent, const double &tao, int &co_flag)
        {
            co_flag = 0;
            Vector2D ab = agent.p - p;
            Vector2D vab = v - agent.v;
            // Step 1. Judge the location of vab compared to the VO obstacle
            auto theta = abs(asin((r + agent.r) / ab.getvalue())); // the angle of ab and vab
            if (abs(vab.getheading() - ab.getheading()) < theta)
            {
                double dis0 = common::math::GetDistance((tao * vab).getx(), (tao * vab).gety(), ab.getx(), ab.gety());
                double dis1 = common::math::GetDistance((tao * vab).getx(), (tao * vab).gety(), 0, 0);
                double dis2 = common::math::GetDistance(0, 0, ab.getx(), ab.gety());
                if (dis1 > dis2)
                    co_flag = 2; // vab locates in the sector
                else if (dis0 < (r + agent.r))
                    co_flag = 1; // vab locates in the circle
            }

            if (co_flag)
            {
                double u2, u3;
                // set the value and heading of the control variable
                if (co_flag == 1)
                {
                    u.setv((r + agent.r) / tao - common::math::GetDistance(vab.getx(), vab.gety(), ab.getx() / tao, ab.gety() / tao));
                    u.seth(atan2(vab.gety() - ab.gety() / tao, (vab.getx() - ab.getx() / tao)));
                }
                else
                {
                    double k2 = tan(ab.getheading() - theta);
                    double k3 = tan(ab.getheading() + theta); // two edge
                    u2 = common::math::GetVerticalDis(k2, 0, vab.getx(), vab.gety());
                    u3 = common::math::GetVerticalDis(k3, 0, vab.getx(), vab.gety());
                    if (u2 > u3)
                    {
                        u.setv(u2);
                        u.seth(common::math::RestrictHeading(ab.getheading() - theta - common::math::PI / 2));
                    }
                    else
                    {
                        u.setv(u3);
                        u.seth(common::math::RestrictHeading(ab.getheading() + theta + common::math::PI / 2));
                    }
                }
            }
            else
            {
                u.clear();
            }
        }

        // class utility
        void utility::RestrictVH(Vehicle &out_ego, Vehicle ego)
        {
            double dv = out_ego.getv().getvalue() - ego.getv().getvalue();
            double dh = common::math::RestrictHeading(out_ego.getv().getheading() - ego.getv().getheading());
            dv = std::min(std::max(dv, -4.4495), 0.0027);
            dh = std::min(std::max(dh, -0.4982), 0.4982);
            Vector2D v_ego = ego.getv();
            v_ego.seth(ego.getv().getheading() + dh);
            out_ego.setv(v_ego);
            v_ego.setv(out_ego.getv().getvalue() + dv);
            out_ego.setv(v_ego);
            // out_ego.getv().seth(ego.getv().getheading() + dh);
            // out_ego.getv().setv(ego.getv().getvalue() + dv);
        }

        double utility::GetHeading(const Vehicle vehicle, double pre_num)
        {
            auto p = vehicle.getref();
            auto index = common::math::ClosestPoint(vehicle.getp().getx(), vehicle.getp().gety(), p);
            auto pre_index = std::min(int(p.poses.size() - 1), int(index + pre_num));
            double h;
            if (p.poses.size() > 0)
            {
                h = atan2(p.poses.at(pre_index).pose.position.y - vehicle.getp().gety(),
                          p.poses.at(pre_index).pose.position.x - vehicle.getp().getx());
            }
            else
            {
                h = 0;
                std::cout << "No reference line!!!!" << std::endl;
            }
            return h;
        }

        void utility::setpose(geometry_msgs::PoseStamped &pose, Vehicle vehicle)
        {
            pose.pose.position.x = vehicle.getp().getx();
            pose.pose.position.y = vehicle.getp().gety();
            pose.pose.position.z = vehicle.getv().getvalue(); // use z to represent velocity value
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(vehicle.getv().getheading());
        }

    }
}
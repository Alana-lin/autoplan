#include <iostream>
#include <common/Math.h>
#include <common/yaml.h>
namespace autoplan
{
    namespace common
    {
        namespace math
        {
            double GetHeading(geometry_msgs::Quaternion ori)
            {
                tf::Quaternion quat;
                tf::quaternionMsgToTF(ori, quat);
                double roll, pitch, yaw;
                tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
                return yaw;
            }

            double GetDistance(double x1, double y1, double x2, double y2)
            {
                return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
            }

            double RestrictHeading(double heading)
            {
                while (heading > PI)
                {
                    heading = heading - 2 * PI;
                }
                while (heading < -PI)
                {
                    heading = heading + 2 * PI;
                }
                return heading;
            }

            int ClosestPoint(double x0, double y0, nav_msgs::Path path)
            {
                int index = 0;
                double min = INFINITY;
                for (int j = 0; j < path.poses.size(); j++)
                {
                    double d = GetDistance(x0, y0, path.poses[j].pose.position.x, path.poses[j].pose.position.y);
                    if (d < min)
                    {
                        min = d;
                        index = j;
                    }
                }
                return index;
            }

            void C2F(nav_msgs::Odometry odom,nav_msgs::Path path, double x, double y, double &s, double &l )
            {
                int cur_i = ClosestPoint(odom.pose.pose.position.x, odom.pose.pose.position.y, path);
                int tar_i = ClosestPoint(x, y, path);
                l = GetDistance(path.poses[tar_i].pose.position.x, path.poses[tar_i].pose.position.y,
                                x, y);
                double theta1 = atan2(odom.pose.pose.position.y - path.poses[cur_i].pose.position.y,
                                      odom.pose.pose.position.x - path.poses[cur_i].pose.position.x);
                double theta2 = -GetHeading(path.poses[tar_i].pose.orientation);
                double delta_theta = RestrictHeading(theta2 - theta1);
                if (delta_theta < 0)
                {
                    l = -l;
                }
                s = 0;
                for (int i = cur_i; i < tar_i; i++)
                {
                    if (i < path.poses.size())
                    {
                        s = s + GetDistance(path.poses[i].pose.position.x, path.poses[i].pose.position.y,
                                            path.poses[i + 1].pose.position.x, path.poses[i + 1].pose.position.y);
                    }
                }
            }

            void F2C(nav_msgs::Odometry odom,nav_msgs::Path path,double s, double l, double &x, double &y )
            {
                int cur_i = ClosestPoint(odom.pose.pose.position.x, odom.pose.pose.position.y, path);
                int tar_i = 0;
                for (int i = cur_i; i < path.poses.size(); i++)
                {
                    s = s - GetDistance(path.poses[i].pose.position.x, path.poses[i].pose.position.y,
                                        path.poses[i + 1].pose.position.x, path.poses[i + 1].pose.position.y);
                    if (s < 0)
                    {
                        tar_i = i;
                        break;
                    }
                }
                double heading = -GetHeading(path.poses[tar_i].pose.orientation);
                x = path.poses[tar_i].pose.position.x + l * sin(heading);
                y = path.poses[tar_i].pose.position.y + l * cos(heading);
            }

            double GetVerticalDis(double k, double b, double x0, double y0)
            {
                return abs(k * x0 - y0 + b) / (sqrt(k * k + 1));
            }

        } // namespace math;
    }     // namespace common;
} // namespace autoplan;
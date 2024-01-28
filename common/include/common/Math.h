#include <string>
// #include <ros/ros.h>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

namespace autoplan
{
    namespace common
    {
        namespace math
        {

            double GetHeading(geometry_msgs::Quaternion ori);
            double GetDistance(double x1, double y1, double x2, double y2);
            double RestrictHeading(double heading);
            int ClosestPoint(double x0, double y0,nav_msgs::Path path);
            void F2C(nav_msgs::Odometry odom,nav_msgs::Path path,double s, double l, double &x, double &y);
            void C2F(nav_msgs::Odometry odom,nav_msgs::Path path,double x, double y, double &s, double &l);
            void GraphConstruct();
            void GraphVisable();
            double GetVerticalDis(double k, double b, double x0, double y0); // for morca
            const double PI = 3.1415926;
            //nav_msgs::Path path;

        }
    }
}
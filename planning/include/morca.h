#include <common/Math.h>

namespace autoplan
{
    namespace planning
    {
        class Vector2D
        {
        public:
            Vector2D(){};
            Vector2D(double value_, double heading_);
            void setxy(double x_, double y_);
            void seth(double heading_);
            void setv(double value_);
            void clear()
            {
                value = 0;
                heading = 0;
                x = 0;
                y = 0;
            };
            double getvalue() const
            {
                return value;
            };
            double getx() const
            {
                return x;
            };
            double gety() const
            {
                return y;
            };
            double getheading() const
            {
                return heading;
            };
            friend Vector2D operator+(const Vector2D &a, const Vector2D &b);
            friend Vector2D operator-(const Vector2D &a, const Vector2D &b);
            friend Vector2D operator-(Vector2D &a);
            friend Vector2D operator*(const double &a, const Vector2D &b);

        private:
            double value;
            double x;
            double y;
            double heading;
        };

        class Vehicle
        {
        public:
            Vehicle(double r_, Vector2D p_, Vector2D v_)
            {
                r = r_;
                p = p_;
                v = v_;
            };
            Vehicle(double r_, Vector2D p_, Vector2D v_, nav_msgs::Path ref_)
            {
                r = r_;
                p = p_;
                v = v_;
                ref = ref_;
            };
            void ORCA(Vehicle agent, const double &tao, int &co_flag);
            Vector2D getp()const
            {
                return p;
            };
            void setp(Vector2D p_){
                p = p_;
            };
            Vector2D getv()const
            {
                return v;
            };
            void setv(Vector2D v_){
                v = v_;
            };
            Vector2D getu()const
            {
                return u;
            };
            void setu(Vector2D u_){
                u = u_;
            };
            nav_msgs::Path getref() const//it's just read only, but cannot change any members of this class in this function
            {
                return ref;
            };

        private:
            nav_msgs::Path ref;
            double r;   // radius
            Vector2D p; // position
            Vector2D v; // velocity
            Vector2D u; // control variable
        };

        class utility
        {
            public:
            static void RestrictVH(Vehicle &out_ego,Vehicle ego);
            static double GetHeading(const Vehicle vehicle,double pre_num);
            static void setpose(geometry_msgs::PoseStamped &pose,Vehicle vehicle);

        };

    }
}
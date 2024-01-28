#include <control_base.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>

namespace autoplan
{
    namespace control
    {
        class controlbaseline
        {
        public:
            typedef std::function<void(carla_msgs::CarlaEgoVehicleControl &)> ChassisCallback;

            void Init(Config config);

            void Process();

            void setChassisCallback(ChassisCallback handle)
            {
                chassiscallback_ = handle;
            };

            void setlocatlization(nav_msgs::Odometry odom_)
            {
                odom = odom_;
            };

            void settrajectory(nav_msgs::Path trajectory_)
            {
                path = trajectory_;
            };

            std::shared_ptr<controlbase> getControlbase()
            {
                return controlbase_;
            };

        private:
            std::shared_ptr<controlbase> controlbase_;
            ChassisCallback chassiscallback_;
            nav_msgs::Path path;
            nav_msgs::Odometry odom;
            carla_msgs::CarlaEgoVehicleControl control_;
            double heading;
            double steer;
        };
    }
}
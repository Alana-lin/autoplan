#include <common/Math.h>
#include <config.h>

namespace autoplan
{
    namespace control
    {
        class controlbase
        {
        public:
            void Init(Config config);

            void TargetIndex(double x0, double y0, double v0, nav_msgs::Path p);

            void pursuit_controller(double x0, double y0, double v0, double heading,
                                                                                nav_msgs::Path p, double &delta);

            double LongPIDcontroller(std::deque<double> buffer, double dt);

            std_msgs::Float32 getvel()
            {
                return vel;
            };
            std::deque<double> err_buffer;

            void setVelocity(std_msgs::Float32 velocity)
            {
                vel=velocity;
            };

        private:
            double Puresuit_K;
            double Puresuit_Lfc;
            double Wheelbase_L;
            // double Lf;
            int pre_index = 0;
            double Kpb;
            double Kib;
            double Kdb;
            std_msgs::Float32 vel;
            const double PI = 3.1415926;
            
        };
    }
}
#include <control_base.h>

namespace autoplan
{
    namespace control
    {
        void controlbase::Init(Config config)
        {
            Puresuit_K = config.Puresuit_K;
            Puresuit_Lfc = config.Puresuit_Lfc;
            Kpb = config.Kpb;
            Kib = config.Kib;
            Kdb = config.Kdb;
            Wheelbase_L = config.Wheelbase_L;
        }

        void controlbase::TargetIndex(double x0, double y0, double v0, nav_msgs::Path p)
        {
            double min = abs(sqrt(pow(x0 - p.poses[0].pose.position.x, 2) + pow(y0 - p.poses[0].pose.position.y, 2)));

            // std::cout<<path.poses.size()<<std::endl;
            for (int j = 0; j < p.poses.size(); j++)
            {
                double d = abs(sqrt(pow(x0 - p.poses[j].pose.position.x, 2) + pow(y0 - p.poses[j].pose.position.y, 2)));
                if (d < min)
                {
                    min = d;
                    pre_index = j;
                }
            }
            double L = 0.0;
            double Lf = Puresuit_K * v0 + Puresuit_Lfc;
            while (Lf > L && (pre_index + 1) < p.poses.size())
            {
                double dx = p.poses[pre_index + 1].pose.position.x - p.poses[pre_index].pose.position.x;
                double dy = p.poses[pre_index + 1].pose.position.y - p.poses[pre_index].pose.position.y;
                L += sqrt(pow(dx, 2) + pow(dy, 2));
                pre_index += 1;
            }
        }

        void controlbase::pursuit_controller(double x0, double y0, double v0, double heading,
                                             nav_msgs::Path p, double &delta)
        {
            double tx, ty, alpha;
            std::cout << "pre_index: " << pre_index << std::endl;
            if (pre_index < p.poses.size())
            {
                tx = p.poses[pre_index].pose.position.x;
                ty = p.poses[pre_index].pose.position.y;
                vel.data = p.poses[pre_index].pose.position.z;
            }
            else
            {
                tx = p.poses[p.poses.size() - 1].pose.position.x;
                ty = p.poses[p.poses.size() - 1].pose.position.y;
                vel.data = p.poses[p.poses.size() - 1].pose.position.z;
                pre_index = p.poses.size() - 1;
            }
            std::cout << "ty,y0,tx,x0: " << ty << "," << y0 << "," << tx << "," << x0 << std::endl;
            std::cout << "heading: " << heading << std::endl;
            alpha = atan2(ty - y0, tx - x0) - heading;
            std::cout << "alpha" << alpha << std::endl;
            if (v0 < 0)
            {
                alpha = PI - alpha;
            }
            double Lf = Puresuit_K * v0 + Puresuit_Lfc;
            std::cout << "Lf:" << Lf << std::endl;
            delta = atan2(2.0 * Wheelbase_L * sin(alpha) / Lf, 1.0);
            std::cout << "delta:" << delta << std::endl;
        }

        double controlbase::LongPIDcontroller(std::deque<double> buffer, double dt)
        {
            double ep, ei, ed = 0;
            if (err_buffer.size() > 0)
            {
                ep = err_buffer.front();
            }
            if (err_buffer.size() > 1)
            {
                ed = (err_buffer.front() - err_buffer.at(1)) / dt;
                for (auto i = err_buffer.begin(); i != err_buffer.end(); i++)
                {
                    ei += *i * dt;
                }
            }
            return Kpb * ep + Kib * ei + Kdb * ed;
        }

    }
}
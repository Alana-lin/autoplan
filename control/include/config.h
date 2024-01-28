#include <std_msgs/Float32.h>

struct Config
{
    // BaseLine:
    double Hard_Brake_P;
    int Control_mode_flag;
    std::string pursuit_log_root; 
    // Base:
    double Puresuit_K;
    double Puresuit_Lfc;
    double Kpb;
    double Kib;
    double Kdb;
    double RatioSteer_m_n;
    double Wheelbase_L;
};

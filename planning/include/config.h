#include <std_msgs/Float32.h>

struct Config
{
    double K;
    int predict_horizon;
    double dt;
    double r;
};

#include <ros/ros.h>
#include <rosplane_msgs/Waypoint.h>
#include <vector>

class RosplaneDubins {
  public:
    void init(ros::NodeHandle& nh);
    std::vector<float> wps;

  private:
    void computePoints();

    double left_y;
    double right_y;
    double r;
    double height;
    double hunter_killer_x;
    double hunter_killer_y;
    float Va;
    int num_waypoints;
    int num_loops;
    int speed_red_factor;

    ros::Publisher waypointPublisher;
};
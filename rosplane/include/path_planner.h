#include <ros/ros.h>
#include <rosplane_msgs/Waypoint.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

class RosplaneDubins {
  public:
    void init(ros::NodeHandle& nh);
    void run();

  private:
    void computeTangencyPoints();
    void computePoints();

    double start_x;
    double left_y;
    double right_y;
    double r;
    double height;
    double hunter_killer_x;
    double hunter_killer_y;
    double tangency_angle_;
    double loop_entry_point_x;
    double loop_entry_point_y;
    double loop_exit_point_x;
    double loop_exit_point_y;
    float Va;
    int num_waypoints;
    int num_loops;

    std::vector<float> wps;

    visualization_msgs::MarkerArray marker_array;
    ros::Publisher waypointPublisher;
    ros::Publisher visualize_points_pub;
};
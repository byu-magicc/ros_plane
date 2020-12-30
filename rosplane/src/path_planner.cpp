#include <ros/ros.h>
#include <rosplane_msgs/Waypoint.h>

#define num_waypoints 8  // kept 8 initially

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosplane_simple_path_planner");

    ros::NodeHandle nh_;
    ros::Publisher waypointPublisher = nh_.advertise<rosplane_msgs::Waypoint>("waypoint_path", 10);

    float Va = 12;
    float wps[5 * num_waypoints] = {
        // 17.3143, 40.6263, -20, -0.765049, Va,
        // these were the points that try to generate a figure like the actual problem statement..launch pos was -40.0, r_min was 25.0, left_y was 48.66
        11.4256,
        26.4236,
        -20,
        0.426903,
        Va,
        -25,
        108.66,
        -20,
        -M_PI / 2,
        Va,
        11.4256,
        26.4236,
        -20,
        0,
        Va,
        -25,
        108.66,
        -20,
        -M_PI / 2,
        Va,
        11.4256,
        26.4236,
        -20,
        0,
        Va,
        -25,
        108.66,
        -20,
        -M_PI / 2,
        Va,
        11.4256,
        26.4236,
        -20,
        0,
        Va,
        -25,
        108.66,
        -20,
        -M_PI / 2,
        Va,

        // 0, 200, -20, 45 * M_PI / 180, Va, 200, 200, -20, 225 * M_PI / 180, Va,
        // 60,
        // 0,
        // -20,
        // 0 * M_PI / 180,
        // Va,
        // 150,
        // 50,
        // -20,
        // 180 * M_PI / 180,
        // Va,
        // 50,
        // 50,
        // -20,
        // 0 * M_PI / 180,
        // Va,
        // 150,
        // 50,
        // -20,
        // 180 * M_PI / 180,
        // Va,
    };

    for (int i(0); i < num_waypoints; i++) {
        ros::Duration(0.5).sleep();

        rosplane_msgs::Waypoint new_waypoint;

        new_waypoint.w[0] = wps[i * 5 + 0];
        new_waypoint.w[1] = wps[i * 5 + 1];
        new_waypoint.w[2] = wps[i * 5 + 2];
        new_waypoint.chi_d = wps[i * 5 + 3];

        new_waypoint.chi_valid = true;
        new_waypoint.Va_d = wps[i * 5 + 4];
        if (i == 0)
            new_waypoint.set_current = true;
        else
            new_waypoint.set_current = false;
        new_waypoint.clear_wp_list = false;

        waypointPublisher.publish(new_waypoint);
    }
    ros::Duration(1.5).sleep();

    return 0;
}

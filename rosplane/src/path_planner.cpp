#include <path_planner.h>

void RosplaneDubins::init(ros::NodeHandle& nh) {
    waypointPublisher = nh.advertise<rosplane_msgs::Waypoint>("waypoint_path", 40);

    nh.param<double>("R_min", r, 25.0);
    nh.param<double>("left_pylon_y", left_y, -8.66);
    nh.param<double>("right_pylon_y", right_y, -408.66);
    nh.param<double>("hunter_killer_x", hunter_killer_x, 5.0);
    nh.param<double>("hunter_killer_y", hunter_killer_y, 0.0);
    nh.param<double>("height", height, -20);
    nh.param<float>("Va", Va, 2.0);
    nh.param<int>("num_waypoints", num_waypoints, 80);
    nh.param<int>("num_loops", num_loops, 8);
    nh.param<int>("speed_red_factor", speed_red_factor, 1);

    wps.resize(5 * num_waypoints);
    RosplaneDubins::computePoints();
}

void RosplaneDubins::computePoints() {
    for (int i = 0; i < num_loops; i++) {
        if (i == 0) {
            wps[50 * i] = r;
            wps[50 * i + 1] = -80.66;
            wps[50 * i + 2] = -20;
            wps[50 * i + 3] = -M_PI / 2;
            wps[50 * i + 4] = Va / speed_red_factor;

            wps[50 * i + 5] = r;
            wps[50 * i + 6] = -200.66;
            wps[50 * i + 7] = -20;
            wps[50 * i + 8] = -M_PI / 2;
            wps[50 * i + 9] = Va;
        } else {
            wps[50 * i] = r;
            wps[50 * i + 1] = left_y + r;
            wps[50 * i + 2] = -20;
            wps[50 * i + 3] = -M_PI / 2;
            wps[50 * i + 4] = Va / speed_red_factor;

            wps[50 * i + 5] = r;
            wps[50 * i + 6] = left_y;
            wps[50 * i + 7] = -20;
            wps[50 * i + 8] = -M_PI / 2;
            wps[50 * i + 9] = Va;
        }

        wps[50 * i + 10] = r;
        wps[50 * i + 11] = right_y;
        wps[50 * i + 12] = -20;
        wps[50 * i + 13] = -M_PI / 2;
        wps[50 * i + 14] = Va / speed_red_factor;

        wps[50 * i + 15] = r;
        wps[50 * i + 16] = right_y - r;
        wps[50 * i + 17] = -20;
        wps[50 * i + 18] = -M_PI / 2;
        wps[50 * i + 19] = Va / speed_red_factor;

        wps[50 * i + 20] = 0;
        wps[50 * i + 21] = right_y - r;
        wps[50 * i + 22] = -20;
        wps[50 * i + 23] = M_PI;
        wps[50 * i + 24] = Va;

        // Lower half points start from here:
        wps[50 * i + 25] = -r;
        wps[50 * i + 26] = right_y - r;
        wps[50 * i + 27] = -20;
        wps[50 * i + 28] = M_PI / 2;
        wps[50 * i + 29] = Va / speed_red_factor;

        wps[50 * i + 30] = -r;
        wps[50 * i + 31] = right_y;
        wps[50 * i + 32] = -20;
        wps[50 * i + 33] = M_PI / 2;
        wps[50 * i + 34] = Va;

        if (i != num_loops - 1) {
            wps[50 * i + 35] = -r;
            wps[50 * i + 36] = left_y;
            wps[50 * i + 37] = -20;
            wps[50 * i + 38] = M_PI / 2;
            wps[50 * i + 39] = Va / speed_red_factor;

            wps[50 * i + 40] = -r;
            wps[50 * i + 41] = left_y + r;
            wps[50 * i + 42] = -20;
            wps[50 * i + 43] = M_PI / 2;
            wps[50 * i + 44] = Va;

            wps[50 * i + 45] = 0;
            wps[50 * i + 46] = left_y + r;
            wps[50 * i + 47] = -20;
            wps[50 * i + 48] = 0;
            wps[50 * i + 49] = Va / speed_red_factor;
        } else {
            wps[50 * i + 35] = -r;
            wps[50 * i + 36] = -200.66;
            wps[50 * i + 37] = -20;
            wps[50 * i + 38] = M_PI / 2;
            wps[50 * i + 39] = Va / speed_red_factor;

            wps[50 * i + 40] = -r;
            wps[50 * i + 41] = -80.66 + r;
            wps[50 * i + 42] = -20;
            wps[50 * i + 43] = M_PI / 2;
            wps[50 * i + 44] = Va;

            wps[50 * i + 45] = hunter_killer_x;
            wps[50 * i + 46] = hunter_killer_y;
            wps[50 * i + 47] = -20;
            wps[50 * i + 48] = 0.785;
            wps[50 * i + 49] = Va / speed_red_factor;
        }
    }
}

#include <path_planner.h>

void RosplaneDubins::init(ros::NodeHandle& nh) {
    waypointPublisher = nh.advertise<rosplane_msgs::Waypoint>("waypoint_path", 40);
    visualize_points_pub = nh.advertise<visualization_msgs::Marker>("visualize_points", 10);

    nh.param<double>("R_min", r, 25.0);
    nh.param<double>("launch_position_x", start_x, -5.0);
    nh.param<double>("left_pylon_y", left_y, 8.66);
    nh.param<double>("right_pylon_y", right_y, 408.66);
    nh.param<double>("height", height, -20);
    nh.param<float>("Va", Va, 2.0);
    nh.param<int>("num_waypoints", num_waypoints, 48);
    nh.param<int>("num_loops", num_loops, 8);
    nh.param<int>("speed_red_factor", speed_red_factor, 6);

    wps.resize(5 * num_waypoints);
    RosplaneDubins::computeTangencyPoints();
    RosplaneDubins::computePoints();
}

void RosplaneDubins::computeTangencyPoints() {
    //     double l = sqrt(start_x * start_x + left_y * left_y);  // launch position to left pylon distance
    //     double zen_angle = acos(left_y / l);                   // angle between y-axis and line joining launch position and pylon
    //     tangency_angle_ = acos(r / l) - zen_angle;
    //     loop_entry_point_x = r * sin(tangency_angle_);
    //     loop_entry_point_y = left_y - r * cos(tangency_angle_);
    //     loop_exit_point_x = -r * sin(tangency_angle_);
    //     loop_exit_point_y = left_y - r * cos(tangency_angle_);
}

void RosplaneDubins::computePoints() {
    // hunter_killer_x = -start_x;
    // hunter_killer_y = 0.0;
    // int i = 0;
    // visualization_msgs::MarkerArray marker_array;
    // visualization_msgs::Marker marker;
    // marker.type = visualization_msgs::Marker::ARROW;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.header.frame_id = "base_link";
    // marker.header.stamp = ros::Time();
    // marker.pose.position.x = 0;
    // marker.pose.position.y = 0;
    // marker.pose.position.z = 0;
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;
    // marker.scale.x = 5;
    // marker.scale.y = 5;
    // marker.scale.z = 2;
    // marker.color.a = 1.0;  // Don't forget to set the alpha!
    // marker.color.r = 0.0;
    // marker.color.g = 0.0;
    // marker.color.b = 1.0;
    // marker.lifetime = ros::Duration(0);
    // for (int j = 0; j < (num_waypoints - 1) / 4; j++) {
    //     if (!j) {
    //         wps[20 * i] = loop_entry_point_x;
    //         wps[20 * i + 1] = -loop_entry_point_y;
    //         wps[20 * i + 2] = height;
    //         wps[20 * i + 3] = -tangency_angle_;
    //         wps[20 * i + 4] = Va;
    //         marker.pose.position.x = wps[20 * i];
    //         marker.pose.position.y = -wps[20 * i + 1];
    //         marker.pose.position.z = -wps[20 * i + 2];
    //         marker.pose.orientation.x = cos(-tangency_angle_ / 2);
    //         marker.pose.orientation.y = -sin(-tangency_angle_ / 2);
    //         marker.pose.orientation.z = -sin(-tangency_angle_ / 2);
    //         marker.pose.orientation.w = cos(-tangency_angle_ / 2);
    //         marker_array.markers.push_back(marker);
    //     } else {
    //         wps[20 * i] = r;
    //         wps[20 * i + 1] = -left_y - left_y / 100.0;
    //         wps[20 * i + 2] = height;
    //         wps[20 * i + 3] = -M_PI / 2;
    //         wps[20 * i + 4] = Va;
    //         geometry_msgs::Point p;
    //         marker.pose.position.x = wps[20 * i];
    //         marker.pose.position.y = -wps[20 * i + 1];
    //         marker.pose.position.z = -wps[20 * i + 2];
    //         marker.pose.orientation.x = cos(-M_PI / 2);
    //         marker.pose.orientation.y = -sin(-M_PI / 2);
    //         marker.pose.orientation.z = -sin(-M_PI / 2);
    //         marker.pose.orientation.w = cos(-M_PI / 2);
    //         marker_array.markers.push_back(marker);
    //     }
    //     wps[20 * i + 5] = r;
    //     wps[20 * i + 6] = -right_y;
    //     wps[20 * i + 7] = height;
    //     wps[20 * i + 8] = -M_PI / 2;
    //     wps[20 * i + 9] = Va;
    //     marker.pose.position.x = wps[20 * i + 5];
    //     marker.pose.position.y = -wps[20 * i + 6];
    //     marker.pose.position.z = -wps[20 * i + 7];
    //     marker.pose.orientation.x = cos(-M_PI / 2);
    //     marker.pose.orientation.y = -sin(-M_PI / 2);
    //     marker.pose.orientation.z = -sin(-M_PI / 2);
    //     marker.pose.orientation.w = cos(-M_PI / 2);
    //     marker_array.markers.push_back(marker);

    //     wps[20 * i + 10] = -r;
    //     wps[20 * i + 11] = -right_y + right_y / 100.0;
    //     wps[20 * i + 12] = height;
    //     wps[20 * i + 13] = M_PI / 2;
    //     wps[20 * i + 14] = Va;
    //     marker.pose.position.x = wps[20 * i + 10];
    //     marker.pose.position.y = -wps[20 * i + 11];
    //     marker.pose.position.z = -wps[20 * i + 12];
    //     marker.pose.orientation.x = cos(M_PI / 2);
    //     marker.pose.orientation.y = -sin(M_PI / 2);
    //     marker.pose.orientation.z = -sin(M_PI / 2);
    //     marker.pose.orientation.w = cos(M_PI / 2);
    //     marker_array.markers.push_back(marker);

    //     if (j == (num_waypoints - 5) / 4) {
    //         wps[20 * i + 15] = loop_exit_point_x;
    //         wps[20 * i + 16] = -loop_exit_point_y;
    //         wps[20 * i + 17] = height;
    //         wps[20 * i + 18] = tangency_angle_;
    //         wps[20 * i + 19] = Va;
    //         marker.pose.position.x = wps[20 * i + 15];
    //         marker.pose.position.y = -wps[20 * i + 16];
    //         marker.pose.position.z = -wps[20 * i + 17];
    //         marker.pose.orientation.x = cos(tangency_angle_ / 2);
    //         marker.pose.orientation.y = -sin(tangency_angle_ / 2);
    //         marker.pose.orientation.z = -sin(tangency_angle_ / 2);
    //         marker.pose.orientation.w = cos(tangency_angle_ / 2);
    //         marker_array.markers.push_back(marker);
    //     } else {
    //         wps[20 * i + 15] = -r;
    //         wps[20 * i + 16] = -left_y;
    //         wps[20 * i + 17] = height;
    //         wps[20 * i + 18] = M_PI / 2;
    //         wps[20 * i + 19] = Va;
    //         marker.pose.position.x = wps[20 * i + 15];
    //         marker.pose.position.y = -wps[20 * i + 16];
    //         marker.pose.position.z = -wps[20 * i + 17];
    //         marker.pose.orientation.z = -sin(M_PI / 2);
    //         marker.pose.orientation.w = cos(M_PI / 2);
    //         marker_array.markers.push_back(marker);
    //     }
    //     i++;
    // }
    // wps[20 * i] = hunter_killer_x;
    // wps[20 * i + 1] = -hunter_killer_y;
    // wps[20 * i + 2] = height;
    // wps[20 * i + 3] = tangency_angle_;
    // wps[20 * i + 4] = Va;
    // marker.pose.position.x = wps[20 * i];
    // marker.pose.position.y = -wps[20 * i + 1];
    // marker.pose.position.z = -wps[20 * i + 2];
    // marker.pose.orientation.x = cos(tangency_angle_ / 2);
    // marker.pose.orientation.y = -sin(tangency_angle_ / 2);
    // marker.pose.orientation.z = -sin(tangency_angle_ / 2);
    // marker.pose.orientation.w = cos(tangency_angle_ / 2);
    // visualize_points_pub.publish(marker);
    // ros::Duration(0.5).sleep();
    // marker_array.markers.push_back(marker);
    // wps[0] = r;
    // wps[1] = -208.66;
    // wps[2] = -20;
    // wps[3] = -M_PI / 2;
    // wps[4] = Va;
    // wps[5] = -r;
    // wps[6] = -408.66;
    // wps[7] = -20;
    // wps[8] = M_PI / 2;
    // wps[9] = Va;
    for (int i = 0; i < num_loops; i++) {
        if (i == 0) {
            wps[30 * i] = r;
            wps[30 * i + 1] = -80.66;
            wps[30 * i + 2] = -20;
            wps[30 * i + 3] = -M_PI / 2;
            wps[30 * i + 4] = Va / speed_red_factor;
        } else {
            wps[30 * i] = r;
            wps[30 * i + 1] = -8.66;
            wps[30 * i + 2] = -20;
            wps[30 * i + 3] = -M_PI / 2;
            wps[30 * i + 4] = Va / speed_red_factor;
        }

        wps[30 * i + 5] = r;
        wps[30 * i + 6] = -200.66;
        wps[30 * i + 7] = -20;
        wps[30 * i + 8] = -M_PI / 2;
        wps[30 * i + 9] = Va;

        wps[30 * i + 10] = r;
        wps[30 * i + 11] = -320.66;
        wps[30 * i + 12] = -20;
        wps[30 * i + 13] = -M_PI / 2;
        wps[30 * i + 14] = Va / speed_red_factor;

        // Lower half points start from here:
        wps[30 * i + 15] = -r;
        wps[30 * i + 16] = -408.66;
        wps[30 * i + 17] = -20;
        wps[30 * i + 18] = M_PI / 2;
        wps[30 * i + 19] = Va / speed_red_factor;

        wps[30 * i + 20] = -r;
        wps[30 * i + 21] = -200.66;
        wps[30 * i + 22] = -20;
        wps[30 * i + 23] = M_PI / 2;
        wps[30 * i + 24] = Va;

        if (i != num_loops - 1) {
            wps[30 * i + 25] = -r;
            wps[30 * i + 26] = -80.66;
            wps[30 * i + 27] = -20;
            wps[30 * i + 28] = M_PI / 2;
            wps[30 * i + 29] = Va / speed_red_factor;
        } else {
            wps[30 * i + 25] = -r;
            wps[30 * i + 26] = -8.66;
            wps[30 * i + 27] = -20;
            wps[30 * i + 28] = M_PI / 2;
            wps[30 * i + 29] = Va / speed_red_factor;
        }
    }
    // wps[70] = -40.0;//position of hunter killer.
    // wps[71] = 0.0;
    // wps[72] = -20;
    // wps[73] = 0.426903;//This yaw value needs to be checked and after adding it num_waypoints will be 17.
    // wps[74] = Va;
}

// void RosplaneDubins::run(std::vector<waypoint_s>& waypoints_) {
//     for (int i(0); i < num_waypoints; i++) {
//         // ros::Duration(1.0).sleep();

//         // rosplane_msgs::Waypoint new_waypoint;

//         // new_waypoint.w[0] = wps[i * 5 + 0];
//         // new_waypoint.w[1] = wps[i * 5 + 1];
//         // new_waypoint.w[2] = wps[i * 5 + 2];
//         // new_waypoint.chi_d = wps[i * 5 + 3];

//         // new_waypoint.chi_valid = true;
//         // new_waypoint.Va_d = wps[i * 5 + 4];
//         // if (i == 0)
//         //     new_waypoint.set_current = true;
//         // else
//         //     new_waypoint.set_current = false;
//         // new_waypoint.clear_wp_list = false;

//         // waypointPublisher.publish(new_waypoint);
//         waypoint_s nextwp;
//         nextwp.w[0] = wps[i * 5 + 0];
//         nextwp.w[1] = wps[i * 5 + 1];
//         nextwp.w[2] = wps[i * 5 + 2];
//         nextwp.chi_d = wps[i * 5 + 3];
//         nextwp.chi_valid = true;
//         nextwp.Va_d = wps[i * 5 + 4];
//         waypoints_.push_back(nextwp);
//         num_waypoints_++;
//     }
//     ros::Duration(1.5).sleep();

//     while (ros::ok()) {
//         visualize_points_pub.publish(marker_array);
//         ros::Duration(0.5).sleep();
//     }
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "rosplane_simple_path_planner");
//     ros::NodeHandle nh;
//     RosplaneDubins trajectory;

//     // trajectory.init(nh);
//     // trajectory.run();
// }

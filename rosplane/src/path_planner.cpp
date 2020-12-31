#include <ros/ros.h>
#include <rosplane_msgs/Waypoint.h>
#include <visualization_msgs/MarkerArray.h>

#define num_waypoints 33  // kept 8 initially

int main(int argc, char** argv) {
    ros::init(argc, argv, "rosplane_simple_path_planner");

    ros::NodeHandle nh_;
    ros::Publisher waypointPublisher = nh_.advertise<rosplane_msgs::Waypoint>("waypoint_path", 10);
    ros::Publisher visualize_points_pub = nh_.advertise<visualization_msgs::Marker>("visualize_points", 10);

    double start_x, left_y, right_y, r, height;
    double hunter_killer_x, hunter_killer_y;
    nh_.param<double>("R_min", r, 25.0);
    nh_.param<double>("launch_position_x", start_x, -5.0);
    nh_.param<double>("left_pylon_y", left_y, 8.66);
    nh_.param<double>("right_pylon_y", right_y, 408.66);
    nh_.param<double>("height", height, -20);

    hunter_killer_x = -start_x;
    hunter_killer_y = 0.0;

    double l = sqrt(start_x * start_x + left_y * left_y);  // launch position to left pylon distance
    double zen_angle = acos(left_y / l);                   // angle between y-axis and line joining launch and pylon
    double tangency_angle_ = acos(r / l) - zen_angle;
    double loop_entry_point_x = r * sin(tangency_angle_);
    double loop_entry_point_y = left_y - r * cos(tangency_angle_);
    double loop_exit_point_x = -r * sin(tangency_angle_);
    double loop_exit_point_y = left_y - r * cos(tangency_angle_);

    float Va;
    nh_.param<float>("Va", Va, 2.0);
    float wps[5 * num_waypoints];
    // = {
    // 17.3143, 40.6263, -20, -0.765049, Va,
    // these were the points that try to generate a figure like the actual problem statement..launch pos was -40.0, r_min was 25.0, left_y was 48.66,
    // right_y was 108.66
    //     loop_entry_point_x,
    //     loop_entry_point_y,
    //     height,
    //     -tangency_angle_,
    //     Va,
    //     -r,
    //     right_y,
    //     height,
    //     M_PI / 2,
    //     Va,
    //     r,
    //     left_y,
    //     -20,
    //     -M_PI / 2,
    //     Va,
    //     -25,
    //     108.66,
    //     -20,
    //     -M_PI / 2,
    //     Va,
    //     11.4256,
    //     26.4236,
    //     -20,
    //     0,
    //     Va,
    //     -25,
    //     108.66,
    //     -20,
    //     -M_PI / 2,
    //     Va,
    //     11.4256,
    //     26.4236,
    //     -20,
    //     0,
    //     Va,
    //     -25,
    //     108.66,
    //     -20,
    //     -M_PI / 2,
    //     Va,

    //     // 0, 200, -20, 45 * M_PI / 180, Va, 200, 200, -20, 225 * M_PI / 180, Va,
    //     // 60,
    //     // 0,
    //     // -20,
    //     // 0 * M_PI / 180,
    //     // Va,
    //     // 150,
    //     // 50,
    //     // -20,
    //     // 180 * M_PI / 180,
    //     // Va,
    //     // 50,
    //     // 50,
    //     // -20,
    //     // 0 * M_PI / 180,
    //     // Va,
    //     // 150,
    //     // 50,
    //     // -20,
    //     // 180 * M_PI / 180,
    //     // Va,
    // };
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.frame_id = "base_link";
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration(0);

    int i = 0;
    for (int j = 0; j < (num_waypoints - 1) / 4; j++) {
        if (!j) {
            wps[20 * i] = loop_entry_point_x;
            wps[20 * i + 1] = loop_entry_point_y;
            wps[20 * i + 2] = height;
            wps[20 * i + 3] = -tangency_angle_;
            wps[20 * i + 4] = Va;
            geometry_msgs::Point p;
            p.x = wps[20 * i];
            p.y = wps[20 * i + 1];
            p.z = -wps[20 * i + 2];
            marker.points.push_back(p);
        } else {
            wps[20 * i] = r;
            wps[20 * i + 1] = left_y;
            wps[20 * i + 2] = height;
            wps[20 * i + 3] = -M_PI / 2;
            wps[20 * i + 4] = Va;
            geometry_msgs::Point p;
            p.x = wps[20 * i];
            p.y = wps[20 * i + 1];
            p.z = -wps[20 * i + 2];
            marker.points.push_back(p);
        }
        wps[20 * i + 5] = r;
        wps[20 * i + 6] = right_y;
        wps[20 * i + 7] = height;
        wps[20 * i + 8] = -M_PI / 2;
        wps[20 * i + 9] = Va;
        geometry_msgs::Point p;
        p.x = wps[20 * i + 5];
        p.y = wps[20 * i + 6];
        p.z = -wps[20 * i + 7];
        marker.points.push_back(p);

        wps[20 * i + 10] = -r;
        wps[20 * i + 11] = right_y;
        wps[20 * i + 12] = height;
        wps[20 * i + 13] = M_PI / 2;
        wps[20 * i + 14] = Va;
        // geometry_msgs::Point p;
        p.x = wps[20 * i + 10];
        p.y = wps[20 * i + 11];
        p.z = -wps[20 * i + 12];
        marker.points.push_back(p);

        if (j == (num_waypoints - 5) / 4) {
            wps[20 * i + 15] = loop_exit_point_x;
            wps[20 * i + 16] = loop_exit_point_x;
            wps[20 * i + 17] = height;
            wps[20 * i + 18] = tangency_angle_;
            wps[20 * i + 19] = Va;
            geometry_msgs::Point p;
            p.x = wps[20 * i + 15];
            p.y = wps[20 * i + 16];
            p.z = -wps[20 * i + 17];
            marker.points.push_back(p);
        } else {
            wps[20 * i + 15] = -r;
            wps[20 * i + 16] = left_y;
            wps[20 * i + 17] = height;
            wps[20 * i + 18] = M_PI / 2;
            wps[20 * i + 19] = Va;
            geometry_msgs::Point p;
            p.x = wps[20 * i + 15];
            p.y = wps[20 * i + 16];
            p.z = -wps[20 * i + 17];
            marker.points.push_back(p);
        }

        i++;
    }
    wps[20 * i] = hunter_killer_x;
    wps[20 * i + 1] = hunter_killer_y;
    wps[20 * i + 2] = height;
    wps[20 * i + 3] = tangency_angle_;
    wps[20 * i + 4] = Va;
    geometry_msgs::Point p;
    p.x = wps[20 * i];
    p.y = wps[20 * i + 1];
    p.z = -wps[20 * i + 2];
    marker.points.push_back(p);

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

    while(ros::ok()) {
        visualize_points_pub.publish(marker);
        ros::Duration(0.5).sleep();
    }

    return 0;
}

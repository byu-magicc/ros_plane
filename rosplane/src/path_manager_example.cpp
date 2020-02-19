#include "path_manager_example.h"
#include "ros/ros.h"
#include <cmath>

namespace rosplane
{

  path_manager_example::path_manager_example() : path_manager_base()
  {
    fil_state_ = fillet_state::STRAIGHT;
    dub_state_ = dubin_state::FIRST;
    current_path.end_plane.normal.setZero();
  }

  void path_manager_example::manage(const params_s &params, const input_s &input, output_s &output)
  {

    if (num_waypoints_ < 2)
    {
      ROS_WARN_THROTTLE(4, "No waypoints received! Loitering about origin at 50m");
      output.flag = false;
      output.Va_d = 12;
      output.c[0] = 0.0f;
      output.c[1] = 0.0f;
      output.c[2] = -50.0f;
      output.rho = params.R_min;
      output.lambda = 1;
      output.orbit_start = 0;
      output.orbit_end = 1.9999*M_PI;
    } else
    {
      int idx_z = (idx_a_ - 1 + num_waypoints_) % num_waypoints_;
      int idx_b = (idx_a_ + 1) % num_waypoints_;
      int idx_c = (idx_b + 1) % num_waypoints_;
      current_path = generate_path(params, 0, idx_a_, idx_b, idx_c, subpath_index_);
      Eigen::Vector3f p{input.pn, input.pe, -input.h};

      while (has_passed_plane(p))
      {
        if (++subpath_index_ >= current_path.num_paths)
        {
          subpath_index_ = 0;
          idx_a_ = (++idx_a_) % num_waypoints_;
        }
        idx_b = (idx_a_ + 1) % num_waypoints_;
        idx_c = (idx_b + 1) % num_waypoints_;
        current_path = generate_path(params, idx_z, idx_a_, idx_b, idx_c, subpath_index_);
      }
      output = current_path.output;
    }
  }

  bool path_manager_example::has_passed_plane(Eigen::Vector3f &p, SemiPlane &semiplane)
  {
    return ((p - semiplane.point).dot(semiplane.normal) > 0);
  }

  bool path_manager_example::has_passed_plane(Eigen::Vector3f &p)
  {
    return has_passed_plane(p, current_path.end_plane);
  }

  path_manager_example::Path
  path_manager_example::generate_path(const struct params_s &params, int idx_z, int idx_a, int idx_b, int idx_c,
                                      int subpath_index)
  {
    if (waypoints_[idx_a_].chi_valid)
    {
      return generate_dubins_path(params, idx_a, idx_b, idx_c, subpath_index);
    } else
    {
      if (params.do_fillets)
        return generate_fillet_path(params, idx_z, idx_a, idx_b, idx_c, subpath_index);
      else
        return generate_line_path(params, idx_a, idx_b, idx_c);
    }
  }

  rosplane_msgs::Extended_Path path_manager_example::generate_path_message(const Path &path)
  {
    rosplane_msgs::Extended_Path path_msg;
    rosplane_msgs::Current_Path &current_path_msg = path_msg.path;

    current_path_msg.path_type = path.output.flag;
    current_path_msg.Va_d = path.output.Va_d;
    current_path_msg.r[0] = path.output.r[0];
    current_path_msg.r[1] = path.output.r[1];
    current_path_msg.r[2] = path.output.r[2];
    current_path_msg.q[0] = path.output.q[0];
    current_path_msg.q[1] = path.output.q[1];
    current_path_msg.q[2] = path.output.q[2];
    current_path_msg.c[0] = path.output.c[0];
    current_path_msg.c[1] = path.output.c[1];
    current_path_msg.c[2] = path.output.c[2];
    current_path_msg.rho = path.output.rho;
    current_path_msg.lambda = path.output.lambda;

    path_msg.line_end[0] = path.output.line_end[0];
    path_msg.line_end[1] = path.output.line_end[1];
    path_msg.line_end[2] = path.output.line_end[2];

    path_msg.orbit_start = path.output.orbit_start;
    path_msg.orbit_end = path.output.orbit_end;

    return path_msg;
  }

  rosplane_msgs::Full_Path path_manager_example::generate_full_path(const path_manager_base::params_s &params)
  {
    rosplane_msgs::Full_Path full_path;
    full_path.label = "ROSplane Path Manager";
    for (size_t idx_a = 0; idx_a < num_waypoints_; idx_a++)
    {
      size_t idx_b = (idx_a + 1) % num_waypoints_;
      size_t idx_c = (idx_b + 1) % num_waypoints_;
      Path path = generate_path(params, 0, idx_a, idx_b, idx_c, 0);
      full_path.paths.push_back(generate_path_message(path));
      for (size_t subpath_index{1}; subpath_index < path.num_paths; subpath_index++)
        full_path.paths.push_back(generate_path_message(generate_path(params, 0, idx_a, idx_b, idx_c, subpath_index)));
    }
    return full_path;
  }

  path_manager_example::Path
  path_manager_example::generate_line_path(const struct params_s &params, int idx_a, int idx_b, int idx_c)
  {
    Path path;

    Eigen::Vector3f w_im1(waypoints_[idx_a].w);
    Eigen::Vector3f w_i(waypoints_[idx_b].w);
    Eigen::Vector3f w_ip1(waypoints_[idx_c].w);

    output_s &output = path.output;

    output.flag = true;
    output.Va_d = waypoints_[idx_a].Va_d;
    output.r[0] = w_im1(0);
    output.r[1] = w_im1(1);
    output.r[2] = w_im1(2);
    Eigen::Vector3f q_im1 = (w_i - w_im1).normalized();
    Eigen::Vector3f q_i = (w_ip1 - w_i).normalized();
    output.q[0] = q_im1(0);
    output.q[1] = q_im1(1);
    output.q[2] = q_im1(2);

    output.line_end[0] = w_i(0);
    output.line_end[1] = w_i(1);
    output.line_end[2] = w_i(2);

    Eigen::Vector3f n_i = (q_im1 + q_i).normalized();
    path.end_plane.point = w_i;
    path.end_plane.normal = n_i;
    path.num_paths = 0;

    return path;
  }

  path_manager_example::Path
  path_manager_example::generate_fillet_path(const struct params_s &params, int idx_z, int idx_a, int idx_b, int idx_c,
                                             int path_index)
  {
    if (num_waypoints_ < 3) //since it fillets don't make sense between just two points
    {
      return generate_line_path(params, idx_a, idx_b, idx_c);
    }

    Path path;
    output_s &output = path.output;
    path.num_paths = 2;

    Eigen::Vector3f waypoint_before_previous(waypoints_[idx_z].w);
    Eigen::Vector3f w_im1(waypoints_[idx_a].w);
    Eigen::Vector3f w_i(waypoints_[idx_b].w);
    Eigen::Vector3f w_ip1(waypoints_[idx_c].w);

    float R_min = params.R_min;

    output.Va_d = waypoints_[idx_a].Va_d;
    output.r[0] = w_im1(0);
    output.r[1] = w_im1(1);
    output.r[2] = w_im1(2);
    Eigen::Vector3f q_prev = (w_im1 - waypoint_before_previous).normalized();
    Eigen::Vector3f q_im1 = (w_i - w_im1).normalized();
    Eigen::Vector3f q_i = (w_ip1 - w_i).normalized();
    float beta = acosf(-q_im1.dot(q_i));
    float beta_prev = acosf(-q_prev.dot(q_im1));

    Eigen::Vector3f z;
    switch (path_index)
    {
      case 0:
        output.flag = true;
        output.q[0] = q_im1(0);
        output.q[1] = q_im1(1);
        output.q[2] = q_im1(2);
        output.c[0] = 1;
        output.c[1] = 1;
        output.c[2] = 1;
        output.rho = 1;
        output.lambda = 1;
        z = w_i - q_im1 * R_min / tanf(beta / 2.);
        output.line_end[0] = z(0);
        output.line_end[1] = z(1);
        output.line_end[2] = z(2);
        path.end_plane.point = z;
        path.end_plane.normal = q_im1;
        path.num_paths = 2;
        break;
      case 1:
        output.flag = false;
        output.q[0] = q_i(0);
        output.q[1] = q_i(1);
        output.q[2] = q_i(2);
        Eigen::Vector3f c = w_i - (q_im1 - q_i).normalized() * (R_min / sinf(beta / 2.0));
        output.c[0] = c(0);
        output.c[1] = c(1);
        output.c[2] = c(2);
        output.rho = R_min;
        output.lambda = ((q_im1(0) * q_i(1) - q_im1(1) * q_i(0)) > 0 ? 1 : -1);
        z = w_i + q_i * (R_min / tanf(beta / 2.0));
        output.orbit_start = atan2f(q_im1(1), q_im1(0));
        output.orbit_end = atan2f(q_i(1), q_i(0));

        path.end_plane.point = z;
        path.end_plane.normal = q_i;
        path.num_paths = 2;
        break;
    }
    return path;
  }

  path_manager_example::Path
  path_manager_example::generate_dubins_path(const struct params_s &params, int idx_a, int idx_b, int idx_c,
                                             int path_index)
  {
    Path path;
    output_s &output = path.output;
    path.num_paths = static_cast<int>(dubin_state::BEFORE_H3) + 1;
    output.Va_d = waypoints_[idx_a].Va_d;
    output.r[0] = 0;
    output.r[1] = 0;
    output.r[2] = 0;
    output.q[0] = 0;
    output.q[1] = 0;
    output.q[2] = 0;
    output.c[0] = 0;
    output.c[1] = 0;
    output.c[2] = 0;

    SemiPlane H1{
        dubinspath_.w1,
        dubinspath_.q1
    };
    SemiPlane H2{
        dubinspath_.w2,
        dubinspath_.q1
    };
    SemiPlane H3{
        dubinspath_.w3,
        dubinspath_.q3
    };

    switch (dub_state_)
    {
      case dubin_state::FIRST:
        dubinsParameters(waypoints_[0], waypoints_[1], params.R_min);
        // break intentionally omitted
      case dubin_state::BEFORE_H1_WRONG_SIDE:
        output.flag = false;
        output.c[0] = dubinspath_.cs(0);
        output.c[1] = dubinspath_.cs(1);
        output.c[2] = dubinspath_.cs(2);
        output.rho = dubinspath_.R;
        output.lambda = dubinspath_.lams;
        path.end_plane = -H1;
        break;
      case dubin_state::BEFORE_H1:
        output.flag = false;
        output.c[0] = dubinspath_.cs(0);
        output.c[1] = dubinspath_.cs(1);
        output.c[2] = dubinspath_.cs(2);
        output.rho = dubinspath_.R;
        output.lambda = dubinspath_.lams;
        path.end_plane = H1;
        break;
      case dubin_state::STRAIGHT:
        output.flag = true;
        output.r[0] = dubinspath_.w1(0);
        output.r[1] = dubinspath_.w1(1);
        output.r[2] = dubinspath_.w1(2);
        output.q[0] = dubinspath_.q1(0);
        output.q[1] = dubinspath_.q1(1);
        output.q[2] = dubinspath_.q1(2);
        output.rho = 1;
        output.lambda = 1;
        path.end_plane = H2;
        break;
      case dubin_state::BEFORE_H3_WRONG_SIDE:
        output.flag = false;
        output.c[0] = dubinspath_.ce(0);
        output.c[1] = dubinspath_.ce(1);
        output.c[2] = dubinspath_.ce(2);
        output.rho = dubinspath_.R;
        output.lambda = dubinspath_.lame;
        path.end_plane = -H3;
        break;
      case dubin_state::BEFORE_H3:
        output.flag = false;
        output.c[0] = dubinspath_.ce(0);
        output.c[1] = dubinspath_.ce(1);
        output.c[2] = dubinspath_.ce(2);
        output.rho = dubinspath_.R;
        output.lambda = dubinspath_.lame;
        path.end_plane = H3;
        break;
    }
  }

  Eigen::Matrix3f path_manager_example::rotz(float theta)
  {
    Eigen::Matrix3f R;
    R << cosf(theta), -sinf(theta), 0,
        sinf(theta), cosf(theta), 0,
        0, 0, 1;

    return R;
  }

  float path_manager_example::mo(float in)
  {
    float val;
    if (in > 0)
      val = fmod(in, 2.0 * M_PI_F);
    else
    {
      float n = floorf(in / 2.0 / M_PI_F);
      val = in - n * 2.0 * M_PI_F;
    }
    return val;
  }

  void path_manager_example::dubinsParameters(const waypoint_s start_node, const waypoint_s end_node, float R)
  {
    float ell = sqrtf((start_node.w[0] - end_node.w[0]) * (start_node.w[0] - end_node.w[0]) +
                      (start_node.w[1] - end_node.w[1]) * (start_node.w[1] - end_node.w[1]));
    if (ell < 2.0 * R)
    {
      ROS_ERROR("The distance between nodes must be larger than 2R.");
    } else
    {
      dubinspath_.ps(0) = start_node.w[0];
      dubinspath_.ps(1) = start_node.w[1];
      dubinspath_.ps(2) = start_node.w[2];
      dubinspath_.chis = start_node.chi_d;
      dubinspath_.pe(0) = end_node.w[0];
      dubinspath_.pe(1) = end_node.w[1];
      dubinspath_.pe(2) = end_node.w[2];
      dubinspath_.chie = end_node.chi_d;

      Eigen::Vector3f crs = dubinspath_.ps;
      crs(0) += R * (cosf(M_PI_2_F) * cosf(dubinspath_.chis) - sinf(M_PI_2_F) * sinf(dubinspath_.chis));
      crs(1) += R * (sinf(M_PI_2_F) * cosf(dubinspath_.chis) + cosf(M_PI_2_F) * sinf(dubinspath_.chis));
      Eigen::Vector3f cls = dubinspath_.ps;
      cls(0) += R * (cosf(-M_PI_2_F) * cosf(dubinspath_.chis) - sinf(-M_PI_2_F) * sinf(dubinspath_.chis));
      cls(1) += R * (sinf(-M_PI_2_F) * cosf(dubinspath_.chis) + cosf(-M_PI_2_F) * sinf(dubinspath_.chis));
      Eigen::Vector3f cre = dubinspath_.pe;
      cre(0) += R * (cosf(M_PI_2_F) * cosf(dubinspath_.chie) - sinf(M_PI_2_F) * sinf(dubinspath_.chie));
      cre(1) += R * (sinf(M_PI_2_F) * cosf(dubinspath_.chie) + cosf(M_PI_2_F) * sinf(dubinspath_.chie));
      Eigen::Vector3f cle = dubinspath_.pe;
      cle(0) += R * (cosf(-M_PI_2_F) * cosf(dubinspath_.chie) - sinf(-M_PI_2_F) * sinf(dubinspath_.chie));
      cle(1) += R * (sinf(-M_PI_2_F) * cosf(dubinspath_.chie) + cosf(-M_PI_2_F) * sinf(dubinspath_.chie));

      float theta, theta2;
      // compute L1
      theta = atan2f(cre(1) - crs(1), cre(0) - crs(0));
      float L1 =
          (crs - cre).norm() + R * mo(2.0 * M_PI_F + mo(theta - M_PI_2_F) - mo(dubinspath_.chis - M_PI_2_F))
          + R * mo(2.0 * M_PI_F + mo(dubinspath_.chie - M_PI_2_F) - mo(theta - M_PI_2_F));

      // compute L2
      ell = (cle - crs).norm();
      theta = atan2f(cle(1) - crs(1), cle(0) - crs(0));
      float L2;
      if (2.0 * R > ell)
        L2 = 9999.0f;
      else
      {
        theta2 = theta - M_PI_2_F + asinf(2.0 * R / ell);
        L2 = sqrtf(ell * ell - 4.0 * R * R) +
             R * mo(2.0 * M_PI_F + mo(theta2) - mo(dubinspath_.chis - M_PI_2_F))
             + R * mo(2.0 * M_PI_F + mo(theta2 + M_PI_F) - mo(dubinspath_.chie + M_PI_2_F));
      }

      // compute L3
      ell = (cre - cls).norm();
      theta = atan2f(cre(1) - cls(1), cre(0) - cls(0));
      float L3;
      if (2.0 * R > ell)
        L3 = 9999.0f;
      else
      {
        theta2 = acosf(2.0 * R / ell);
        L3 = sqrtf(ell * ell - 4 * R * R) +
             R * mo(2.0 * M_PI_F + mo(dubinspath_.chis + M_PI_2_F) - mo(theta + theta2))
             + R * mo(2.0 * M_PI_F + mo(dubinspath_.chie - M_PI_2_F) - mo(theta + theta2 - M_PI_F));
      }

      // compute L4
      theta = atan2f(cle(1) - cls(1), cle(0) - cls(0));
      float L4 =
          (cls - cle).norm() + R * mo(2.0 * M_PI_F + mo(dubinspath_.chis + M_PI_2_F) - mo(theta + M_PI_2_F))
          + R * mo(2.0 * M_PI_F + mo(theta + M_PI_2_F) - mo(dubinspath_.chie + M_PI_2_F));

      // L is the minimum distance
      int idx = 1;
      dubinspath_.L = L1;
      if (L2 < dubinspath_.L)
      {
        dubinspath_.L = L2;
        idx = 2;
      }
      if (L3 < dubinspath_.L)
      {
        dubinspath_.L = L3;
        idx = 3;
      }
      if (L4 < dubinspath_.L)
      {
        dubinspath_.L = L4;
        idx = 4;
      }

      Eigen::Vector3f e1;
      //        e1.zero();
      e1(0) = 1;
      e1(1) = 0;
      e1(2) = 0;
      switch (idx)
      {
        case 1:
          dubinspath_.cs = crs;
          dubinspath_.lams = 1;
          dubinspath_.ce = cre;
          dubinspath_.lame = 1;
          dubinspath_.q1 = (cre - crs).normalized();
          dubinspath_.w1 = dubinspath_.cs + (rotz(-M_PI_2_F) * dubinspath_.q1) * R;
          dubinspath_.w2 = dubinspath_.ce + (rotz(-M_PI_2_F) * dubinspath_.q1) * R;
          break;
        case 2:
          dubinspath_.cs = crs;
          dubinspath_.lams = 1;
          dubinspath_.ce = cle;
          dubinspath_.lame = -1;
          ell = (cle - crs).norm();
          theta = atan2f(cle(1) - crs(1), cle(0) - crs(0));
          theta2 = theta - M_PI_2_F + asinf(2.0 * R / ell);
          dubinspath_.q1 = rotz(theta2 + M_PI_2_F) * e1;
          dubinspath_.w1 = dubinspath_.cs + (rotz(theta2) * e1) * R;
          dubinspath_.w2 = dubinspath_.ce + (rotz(theta2 + M_PI_F) * e1) * R;
          break;
        case 3:
          dubinspath_.cs = cls;
          dubinspath_.lams = -1;
          dubinspath_.ce = cre;
          dubinspath_.lame = 1;
          ell = (cre - cls).norm();
          theta = atan2f(cre(1) - cls(1), cre(0) - cls(0));
          theta2 = acosf(2.0 * R / ell);
          dubinspath_.q1 = rotz(theta + theta2 - M_PI_2_F) * e1;
          dubinspath_.w1 = dubinspath_.cs + (rotz(theta + theta2) * e1) * R;
          dubinspath_.w2 = dubinspath_.ce + (rotz(theta + theta2 - M_PI_F) * e1) * R;
          break;
        case 4:
          dubinspath_.cs = cls;
          dubinspath_.lams = -1;
          dubinspath_.ce = cle;
          dubinspath_.lame = -1;
          dubinspath_.q1 = (cle - cls).normalized();
          dubinspath_.w1 = dubinspath_.cs + (rotz(M_PI_2_F) * dubinspath_.q1) * R;
          dubinspath_.w2 = dubinspath_.ce + (rotz(M_PI_2_F) * dubinspath_.q1) * R;
          break;
      }
      dubinspath_.w3 = dubinspath_.pe;
      dubinspath_.q3 = rotz(dubinspath_.chie) * e1;
      dubinspath_.R = R;
    }
  }

}//end namespace

#pragma once

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

struct ElastoPlasticSimParams {
  std::string ground_model;
  std::string robot_model;
  double robot_publish_rate;
  double object_publish_rate;
  double actuator_delay;
  double dt;
  double mpm_substep;
  double realtime_rate;
  bool visualize_drake_sim;
  bool publish_efforts;
  Eigen::VectorXd fixed_base_robot;
  std::string fixed_base_robot_frame;
  Eigen::VectorXd q_init_robot;
  Eigen::VectorXd q_init_object;
  Eigen::Vector3d object_half_widths;
  Eigen::Vector3d camera_pose;
  Eigen::Vector3d camera_target;
  bool use_franka;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(ground_model));
    a->Visit(DRAKE_NVP(robot_model));
    a->Visit(DRAKE_NVP(robot_publish_rate));
    a->Visit(DRAKE_NVP(object_publish_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(mpm_substep));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(visualize_drake_sim));
    a->Visit(DRAKE_NVP(publish_efforts));
    a->Visit(DRAKE_NVP(fixed_base_robot));
    a->Visit(DRAKE_NVP(fixed_base_robot_frame));
    a->Visit(DRAKE_NVP(q_init_robot));
    a->Visit(DRAKE_NVP(q_init_object));
    a->Visit(DRAKE_NVP(object_half_widths));
    a->Visit(DRAKE_NVP(camera_pose));
    a->Visit(DRAKE_NVP(camera_target));
    a->Visit(DRAKE_NVP(use_franka));
  }
};

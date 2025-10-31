#pragma once

#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

struct ElastoPlasticSimParams {
  std::string object_model;
  double dt;
  double realtime_rate;
  double actuator_delay;
  double robot_publish_rate;
  double object_publish_rate;
  bool visualize_drake_sim;
  bool publish_efforts;
  Eigen::VectorXd fixed_base_robot;
  std::string fixed_base_robot_frame;
  Eigen::VectorXd q_init_robot;
  Eigen::VectorXd q_init_object;
  double damping;
  double stiffness;
  double friction;
  double substep;
  int ppc;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(object_model));
    a->Visit(DRAKE_NVP(dt));
    a->Visit(DRAKE_NVP(realtime_rate));
    a->Visit(DRAKE_NVP(actuator_delay));
    a->Visit(DRAKE_NVP(robot_publish_rate));
    a->Visit(DRAKE_NVP(object_publish_rate));
    a->Visit(DRAKE_NVP(visualize_drake_sim));
    a->Visit(DRAKE_NVP(publish_efforts));
    a->Visit(DRAKE_NVP(fixed_base_robot));
    a->Visit(DRAKE_NVP(fixed_base_robot_frame));
    a->Visit(DRAKE_NVP(q_init_robot));
    a->Visit(DRAKE_NVP(q_init_object));
    a->Visit(DRAKE_NVP(damping));
    a->Visit(DRAKE_NVP(stiffness));
    a->Visit(DRAKE_NVP(friction));
    a->Visit(DRAKE_NVP(substep));
    a->Visit(DRAKE_NVP(ppc));
  }
};

#include <cmath>
#include <iostream>
#include <stdexcept>
#include "nav2_amcl/motion_model/differential_motion_model.hpp"
#include "/home/rb3302-7/rb3302_fresh/student_ws/src/rb3302_amcl_motion/differential_motion_model.hpp"

double step(nav2_amcl::MotionModel &model, double heading, double distance, double turn=0.) {
  pf_t pf{}; pf_sample_t sample{};
  sample.pose.v[2]=heading;
  pf.sets[0].sample_count=1; pf.sets[0].samples=&sample;
  pf_vector_t delta{{distance*cos(heading),distance*sin(heading),turn}};
  pf_vector_t pose{{delta.v[0],delta.v[1],heading+turn}};
  model.initialize(0.,0.,0.,0.,0.);
  model.odometryUpdate(&pf,pose,delta);
  if (fabs(remainder(sample.pose.v[2]-heading-turn,2*M_PI))>1e-8) throw std::runtime_error("yaw changed incorrectly");
  return sample.pose.v[0]*cos(heading)+sample.pose.v[1]*sin(heading);
}
int main() {
  nav2_amcl::DifferentialMotionModel upstream;
  nav2_amcl::RB3302DifferentialMotionModel fixed;
  std::cout << "Installed upstream: -0.00375 m odom -> " << step(upstream,0.,-.00375) << " m particle displacement\n";
  for(double a : {0.,M_PI/2,M_PI,-M_PI/2})
    for(double d : {-.00375,.00375,-.02,.02,0.})
      if(fabs(step(fixed,a,d)-d)>1e-8) throw std::runtime_error("wrong translation");
  if(fabs(step(fixed,0.,0.,.2))>1e-8) throw std::runtime_error("rotation translated");
  std::cout << "PASS: corrected forward/reverse steps for all four headings, large steps and pure rotation\n";
}

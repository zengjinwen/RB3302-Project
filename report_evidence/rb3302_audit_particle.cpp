#include <cmath>
#include <iostream>
#include <vector>
#include "nav2_amcl/motion_model/differential_motion_model.hpp"
#include "/home/rb3302-7/rb3302_fresh/student_ws/src/rb3302_amcl_motion/differential_motion_model.hpp"
double sample(nav2_amcl::MotionModel &m,double dx,double turn){
 std::vector<pf_sample_t> particles(100000);pf_t pf{};pf.sets[0].sample_count=particles.size();pf.sets[0].samples=particles.data();
 pf_vector_t delta{{dx,0.,turn}},pose=delta;m.initialize(.2,.2,.2,.2,.2);srand48(123);m.odometryUpdate(&pf,pose,delta);
 double sum=0.;for(auto &p:particles)sum+=p.pose.v[0];return sum/particles.size();
}
int main(){nav2_amcl::DifferentialMotionModel old;nav2_amcl::RB3302DifferentialMotionModel now;
 for(double dx:{0.,.000001,-.000001})std::cout<<"dx="<<dx<<" turn=.05 original_mean_dx="<<sample(old,dx,.05)<<" custom_mean_dx="<<sample(now,dx,.05)<<"\n";
}

#include <cassert>
#include <cmath>
#include <iostream>
#include "/home/rb3302-7/rb3302_fresh/student_ws/src/rb3302_amcl_motion/differential_motion_model.hpp"
int main(){
 nav2_amcl::RB3302DifferentialMotionModel m; m.initialize(.2,.2,.2,.2,.2);
 for(double heading:{0.,1.5707963267948966,3.141592653589793}){
  for(double d:{-.00375,0.,.00375}) for(int i=0;i<1000;i++){
   pf_t pf{};pf_sample_t s{};s.pose.v[2]=heading;pf.sets[0].samples=&s;pf.sets[0].sample_count=1;
   pf_vector_t delta{{d*cos(heading),d*sin(heading),0.}},pose{{delta.v[0],delta.v[1],heading}};
   m.odometryUpdate(&pf,pose,delta);
   double projected=s.pose.v[0]*cos(heading)+s.pose.v[1]*sin(heading);
   assert(projected*d>=0.);
   if(d==0.)assert(s.pose.v[0]==0.&&s.pose.v[1]==0.&&s.pose.v[2]==heading);
  }
 }
 m.initialize(0.,0.,0.,0.,0.);
 pf_t pf{};pf_sample_t s{};pf.sets[0].samples=&s;pf.sets[0].sample_count=1;
 pf_vector_t delta{{-.003,.002,.1}},pose=delta;
 m.odometryUpdate(&pf,pose,delta);
 assert(fabs(s.pose.v[0]+.003)<1e-12&&fabs(s.pose.v[1]-.002)<1e-12);
 std::cout<<"PASS: signed reverse/forward with configured noise, stationary unchanged, lateral signed displacement preserved\n";
}

#include <cmath>
#include <iostream>
#include <vector>
#include <random>
#include <iomanip>
#include "nav2_amcl/motion_model/differential_motion_model.hpp"
#include "/home/rb3302-7/rb3302_fresh/student_ws/src/rb3302_amcl_motion/differential_motion_model.hpp"
struct Result {double rmse=0,maxerr=0,finish=0,reverse=0;};
Result run(nav2_amcl::MotionModel &model,bool observe,int seed){
 const int n=1000;std::vector<pf_sample_t> samples(n);pf_t pf{};
 pf.sets[0].sample_count=n;pf.sets[0].samples=samples.data();
 model.initialize(.2,.2,.2,.2,.2);srand48(seed);
 std::mt19937 rng(seed);std::normal_distribution<double> initial(0.,.01);
 for(auto &s:samples){s.pose.v[0]=initial(rng);s.pose.v[1]=initial(rng);s.pose.v[2]=initial(rng);s.weight=1./n;}
 pf_vector_t truth{};Result result;double previous_x=0.;int count=0;
 auto step=[&](double distance,double turn,bool reversing){
  pf_vector_t delta{{distance*cos(truth.v[2]),distance*sin(truth.v[2]),turn}};
  for(int j=0;j<3;j++)truth.v[j]+=delta.v[j];
  model.odometryUpdate(&pf,truth,delta);
  if(observe){
   // Simplified map observation: 3 cm position likelihood, 3 degree heading.
   // This is NOT a simulation of the real lidar or the actual course map.
   std::vector<double>w(n);for(int i=0;i<n;i++){
    double dx=samples[i].pose.v[0]-truth.v[0],dy=samples[i].pose.v[1]-truth.v[1];
    double da=remainder(samples[i].pose.v[2]-truth.v[2],2*M_PI);
    w[i]=exp(-.5*((dx*dx+dy*dy)/(.03*.03)+da*da/(.05236*.05236)))+1e-300;
   }
   std::discrete_distribution<int>pick(w.begin(),w.end());auto old=samples;
   for(auto &s:samples)s=old[pick(rng)];
  }
  double x=0.,y=0.;for(auto &s:samples){x+=s.pose.v[0]/n;y+=s.pose.v[1]/n;}
  double err=hypot(x-truth.v[0],y-truth.v[1]);
  result.rmse+=err*err;result.maxerr=std::max(result.maxerr,err);result.finish=err;
  if(reversing)result.reverse+=x-previous_x;
  previous_x=x;count++;
 };
 // 8 Hz: forward 0.1 m/s for 6 s; reverse 0.03 m/s for 20 s.
 for(int i=0;i<48;i++)step(.0125,0.,false);
 for(int i=0;i<160;i++)step(-.00375,0.,true);
 // Stationary, turn 90 degrees, then forward/reverse in the new direction.
 for(int i=0;i<24;i++)step(0.,0.,false);
 for(int i=0;i<40;i++)step(0.,M_PI/80,false);
 for(int i=0;i<48;i++)step(.0125,0.,false);
 for(int i=0;i<160;i++)step(-.00375,0.,false);
 result.rmse=sqrt(result.rmse/count);return result;
}
int main(){std::cout<<std::fixed<<std::setprecision(4);
 for(bool observe:{false,true})for(bool fixed:{false,true}){
  double rmse=0.,end=0.,reverse=0.,maxerr=0.;
  for(int seed=1;seed<=20;seed++){
   nav2_amcl::DifferentialMotionModel orig;nav2_amcl::RB3302DifferentialMotionModel model;
   auto r=run(fixed?static_cast<nav2_amcl::MotionModel&>(model):static_cast<nav2_amcl::MotionModel&>(orig),observe,seed);
   rmse+=r.rmse/20;end+=r.finish/20;reverse+=r.reverse/20;maxerr=std::max(maxerr,r.maxerr);
  }
  std::cout<<(fixed?"signed":"original")<<" observation="<<observe<<" mean_RMSE_m="<<rmse<<" mean_final_error_m="<<end<<" reverse_dx_m="<<reverse<<" worst_error_m="<<maxerr<<"\n";
 }
}

#include <cstdio>
#include <vector>
#include "yesense_decoder.h"
#include "yesense_decoder_comm.h"
std::vector<unsigned char> packet(){std::vector<unsigned char> p={0x59,0x53,1,0,4,1,2,0,0}; unsigned char a=0,b=0;for(size_t i=2;i<p.size();i++){a+=p[i];b+=a;}p.push_back(a);p.push_back(b);return p;}
int main(){yesense::yesense_decoder d; yesense::yis_out_data_t out{};auto good=packet();auto bad=good;bad.back()^=1;bad.insert(bad.begin(),16,0xAA);printf("bad frame result=%d\n",d.data_proc(bad.data(),bad.size(),&out));int ok=0,full=0;for(int i=0;i<200;i++){int ret=d.data_proc(good.data(),good.size(),&out);ok+=ret==analysis_ok;full+=ret==buf_full;}printf("Following 200 valid frames: decoded=%d buffer_full=%d\n",ok,full);}

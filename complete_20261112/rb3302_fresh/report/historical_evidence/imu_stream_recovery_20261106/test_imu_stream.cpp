#include <cassert>
#include <cstdio>
#include <vector>
#include "yesense_decoder.h"
#include "yesense_decoder_comm.h"
using namespace yesense;
std::vector<unsigned char> frame(std::vector<unsigned char> payload={1,2,0,0}){std::vector<unsigned char> p={0x59,0x53,1,0,(unsigned char)payload.size()};p.insert(p.end(),payload.begin(),payload.end());unsigned char a=0,b=0;for(size_t i=2;i<p.size();++i){a+=p[i];b+=a;}p.push_back(a);p.push_back(b);return p;}
int feed(yesense_decoder& d,std::vector<unsigned char> p){yis_out_data_t o{};int ret=d.data_proc(p.data(),p.size(),&o),n=0;while(ret==analysis_ok){n++;ret=d.data_proc(nullptr,0,&o);}return n;}
int main(){auto good=frame();for(size_t split=1;split<good.size();split++){yesense_decoder d;assert(feed(d,{good.begin(),good.begin()+split})==0);assert(feed(d,{good.begin()+split,good.end()})==1);}
{yesense_decoder d;auto bad=good;bad.back()^=1;bad.insert(bad.begin(),16,0xaa);bad.insert(bad.end(),good.begin(),good.end());assert(feed(d,bad)==1);}
{yesense_decoder d;auto bad=frame({0x20,12,0});bad.insert(bad.end(),good.begin(),good.end());assert(feed(d,bad)==1);}
{yesense_decoder d;std::vector<unsigned char> burst;for(int i=0;i<50;i++)burst.insert(burst.end(),good.begin(),good.end());assert(feed(d,burst)==50);}
{yesense_decoder d;std::vector<unsigned char> overflow(2000,0xaa);overflow.insert(overflow.end(),good.begin(),good.end());assert(feed(d,overflow)==1);assert(feed(d,good)==1);}
{yesense_decoder d;std::vector<unsigned char> truncated={0x59,0x53,1,0,255};assert(feed(d,truncated)==0);int recovered=0;for(int i=0;i<100;i++)recovered+=feed(d,good);assert(recovered>0);assert(feed(d,good)==1);}
puts("PASS: all packet split positions, bad CRC then valid packet, malformed TLV, 50-frame burst, overflow recovery, corrupted length recovery.");}

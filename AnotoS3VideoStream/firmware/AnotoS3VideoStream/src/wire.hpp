#pragma once
// Shared, byte-order explicit protocol. No image array is stored in firmware flash.
#include <array>
#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <cstring>
namespace wire {
constexpr std::size_t HEADER=24, MAX_W=240, MAX_H=180, MAX_PAYLOAD=MAX_W*MAX_H+12;
constexpr std::uint8_t VERSION=2;
enum Type : std::uint8_t { HELLO=1, START=2, FRAME=3, END=4, HELLO_OK=129, START_OK=130, RESULT=131, END_OK=132, ERROR=255 };
constexpr std::uint8_t MAGIC[4]={'A','N','5','!'};
inline std::uint16_t u16(const std::uint8_t* p){ return std::uint16_t(p[0]) | (std::uint16_t(p[1])<<8); }
inline std::uint32_t u32(const std::uint8_t* p){return std::uint32_t(p[0])|(std::uint32_t(p[1])<<8)|(std::uint32_t(p[2])<<16)|(std::uint32_t(p[3])<<24);}
inline std::uint64_t u64(const std::uint8_t* p){return std::uint64_t(u32(p))|(std::uint64_t(u32(p+4))<<32);}
inline float f32(const std::uint8_t* p){static_assert(sizeof(float)==4,"32-bit float needed");auto v=u32(p);float f;std::memcpy(&f,&v,4);return f;}
inline void p16(std::uint8_t* p,std::uint16_t v){p[0]=v&255;p[1]=(v>>8)&255;}
inline void p32(std::uint8_t* p,std::uint32_t v){for(int i=0;i<4;++i)p[i]=std::uint8_t(v>>(8*i));}
inline void p64(std::uint8_t* p,std::uint64_t v){p32(p,std::uint32_t(v));p32(p+4,std::uint32_t(v>>32));}
inline void pf32(std::uint8_t* p,float f){std::uint32_t v;std::memcpy(&v,&f,4);p32(p,v);}
constexpr std::array<std::uint32_t,256> crc_table(){std::array<std::uint32_t,256> a{};for(std::uint32_t i=0;i<256;++i){auto c=i;for(int j=0;j<8;++j)c=(c>>1)^((c&1)?0xedb88320u:0u);a[i]=c;}return a;}
inline constexpr auto CRC_TABLE=crc_table();
inline std::uint32_t crc32(const std::uint8_t* p,std::size_t n){std::uint32_t c=0xffffffffu;for(std::size_t i=0;i<n;++i)c=(c>>8)^CRC_TABLE[(c^p[i])&255];return c^0xffffffffu;}
inline void header(std::uint8_t* out,std::uint8_t type,std::uint32_t seq,const std::uint8_t* data,std::size_t n){std::memcpy(out,MAGIC,4);out[4]=VERSION;out[5]=type;p16(out+6,0);p32(out+8,seq);p32(out+12,std::uint32_t(n));p32(out+16,crc32(out,16));p32(out+20,crc32(data,n));}
// Exactly one inbound image buffer. Completed payload remains valid until next consume().
// Header and body checksums are distinct; a corrupt length cannot overrun the buffer.
class Parser {
    std::array<std::uint8_t,HEADER> head_{};
    alignas(4) std::array<std::uint8_t,MAX_PAYLOAD> payload_{};
    std::size_t nh_=0,np_=0,expected_=0;
    bool body_=false,complete_=false;
public:
    std::uint64_t payload_started_us=0,last_progress_us=0;
    unsigned bad_headers=0;
    void reset(){nh_=np_=expected_=0;body_=complete_=false;payload_started_us=last_progress_us=0;}
    bool active()const{return !complete_ && (nh_!=0||body_);}
    const std::uint8_t* payload()const{return payload_.data();}
    std::size_t size()const{return expected_;}
    std::uint8_t type()const{return head_[5];}
    std::uint32_t seq()const{return u32(head_.data()+8);}
    bool crc_ok()const{return crc32(payload_.data(),expected_)==u32(head_.data()+20);}
    bool consume(const std::uint8_t* data,std::size_t n,std::size_t& used,std::uint64_t now){
        if(complete_)reset();
        used=0;
        while(used<n){
            last_progress_us=now;
            if(!body_){
                auto b=data[used++];
                if(nh_<4 && b!=MAGIC[nh_]){nh_=(b==MAGIC[0]?1:0);if(nh_)head_[0]=b;continue;}
                head_[nh_++]=b;
                if(nh_==HEADER){
                    if(head_[4]!=VERSION||u16(head_.data()+6)!=0||u32(head_.data()+12)>MAX_PAYLOAD||crc32(head_.data(),16)!=u32(head_.data()+16)){
                        ++bad_headers;reset();continue;
                    }
                    expected_=u32(head_.data()+12);np_=0;body_=true;payload_started_us=now;
                    if(expected_==0){complete_=true;return true;}
                }
            }else{
                auto k=std::min(n-used,expected_-np_);std::memcpy(payload_.data()+np_,data+used,k);np_+=k;used+=k;
                if(np_==expected_){complete_=true;return true;}
            }
        }
        return false;
    }
};
}

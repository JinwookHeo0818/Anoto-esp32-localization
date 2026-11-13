#pragma once
// Bounded LZ4 block decoder, written for this project. No heap allocation and
// no dependency on a codec library. This is lossless BYTE transport, not image
// compression/thresholding. The reconstructed image also passes a raw CRC32.
#include <cstdint>
#include <cstddef>
#include <cstring>
namespace wire {
inline bool unpack_lz4(const std::uint8_t* in,std::size_t ni,std::uint8_t* out,std::size_t no){
    if(!in||!out||!ni||!no)return false;
    std::size_t ip=0,op=0;
    auto length=[&](std::size_t& n)->bool{
        if(n!=15)return true;
        for(;;){
            if(ip>=ni)return false;
            const auto b=in[ip++];
            if(n>no||std::size_t(b)>no-n)return false;
            n+=b;if(b!=255)return true;
        }
    };
    while(ip<ni){
        const auto token=in[ip++];
        std::size_t literal=token>>4;
        if(!length(literal)||literal>ni-ip||literal>no-op)return false;
        if(literal)std::memcpy(out+op,in+ip,literal);
        ip+=literal;op+=literal;
        if(ip==ni)return op==no;
        if(ni-ip<2)return false;
        const std::size_t distance=std::size_t(in[ip])|(std::size_t(in[ip+1])<<8);ip+=2;
        if(!distance||distance>op)return false;
        std::size_t match=token&15;
        if(!length(match)||match>no-op||no-op-match<4)return false;
        match+=4;
        // Forward copy handles overlapping matches (distance may be one).
        // For disjoint matches the library bulk copy is considerably cheaper.
        if(distance>=match)std::memcpy(out+op,out+op-distance,match);
        else for(std::size_t k=0;k<match;++k)out[op+k]=out[op+k-distance];
        op+=match;
    }
    return false; // A valid block ends with a literal sequence.
}
}

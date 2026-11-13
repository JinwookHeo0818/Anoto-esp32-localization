#include "anoto_codec.hpp"
#include "codec_tables.hpp"
#include <cstring>
namespace anoto {
    namespace {
        int mod63(std::int64_t a) {
            int r=int(a%63);
            return r<0?r+63:r;
        }
        std::uint32_t prefix(std::uint32_t n) {
            // Only modulo 63 is needed. Bounded O(1) periodic sums, not O(coordinate).
            const std::uint8_t* p[4]= {
                tables::prefix1,tables::prefix2,tables::prefix3,tables::prefix4
            };
            const std::uint32_t len[4]= {
                236,233,31,241
            },w[4]= {
                1,3,9,18
            };
            std::uint64_t s=5ULL*n;
            for(int k=0;k<4;++k) s+=std::uint64_t(w[k])*((n/len[k])*std::uint64_t(p[k][len[k]])+p[k][n%len[k]]);
            return std::uint32_t(s%63);
        }
        bool axis(const std::uint8_t b[64],bool xaxis,std::uint32_t& value,int& first) {
            int loc[6];
            for(int i=0;i<6;++i) {
                unsigned key=0;
                for(int j=0;j<6;++j) key=(key<<1)|(xaxis?((b[j*8+i]>>1)&1):(b[i*8+j]&1));
                loc[i]=tables::inv0[key];
                if(loc[i]==255)return false;
            }
            first=loc[0];
            unsigned keys[4]= {
                0,0,0,0
            };
            for(int i=0;i<5;++i) {
                int d=(loc[i+1]-loc[i]+63)%63;
                if(d<5||d>58)return false;
                d-=5;
                keys[0]=keys[0]*3+unsigned(d%3);
                keys[1]=keys[1]*3+unsigned((d/3)%3);
                keys[2]=keys[2]*2+unsigned((d/9)%2);
                keys[3]=keys[3]*3+unsigned(d/18);
            }
            const std::uint8_t* inverse[4]= {
                tables::inv1,tables::inv2,tables::inv3,tables::inv4
            };
            std::uint64_t sum=0;
            for(int k=0;k<4;++k) {
                const unsigned r=inverse[k][keys[k]];
                if(r==255)return false;
                sum+=(std::uint64_t(r)*tables::crt[k])%tables::period;
            }
            value=std::uint32_t(sum%tables::period);
            return true;
        }
    }
    std::uint8_t Codec::symbol(std::uint32_t x,std::uint32_t y,int sx,int sy) {
        const int ix=mod63(std::int64_t(y)+sx+prefix(x));
        const int iy=mod63(std::int64_t(x)+sy+prefix(y));
        return std::uint8_t((tables::seq0[ix]<<1)|tables::seq0[iy]);
    }
    void Codec::encode8(std::uint32_t x,std::uint32_t y,std::uint8_t out[64],int sx,int sy) {
        unsigned px[8],py[8];
        for(unsigned i=0;i<8;++i) {
            px[i]=prefix(x+i);
            py[i]=prefix(y+i);
        }
        for(unsigned r=0;r<8;++r)for(unsigned c=0;c<8;++c)
        out[r*8+c]=std::uint8_t((tables::seq0[mod63(std::int64_t(y)+r+sx+px[c])]<<1)
        |tables::seq0[mod63(std::int64_t(x)+c+sy+py[r])]);
    }
    void Codec::rotate_ccw(const std::uint8_t input[64],std::uint8_t out[64],int turns) {
        turns=(turns%4+4)%4;
        std::uint8_t a[64],b[64];
        std::memcpy(a,input,64);
        constexpr std::uint8_t ccw[4]= {
            2,0,3,1
        };
        for(int t=0;t<turns;++t) {
            for(int r=0;r<8;++r)for(int c=0;c<8;++c)b[r*8+c]=ccw[a[c*8+(7-r)]];
            std::memcpy(a,b,64);
        }
        std::memcpy(out,a,64);
    }
    bool Codec::decode8(const std::uint8_t observed[64],Address& result,int sx,int sy,std::uint32_t maxx,std::uint32_t maxy) {
        for(int i=0;i<64;++i)if(observed[i]>3)return false;
        if(maxx<8||maxy<8||maxx>tables::period||maxy>tables::period)return false;
        bool found=false;
        Address answer;
        for(int k=0;k<4;++k) {
            std::uint8_t b[64],expected[64];
            rotate_ccw(observed,b,-k);
            std::uint32_t x,y;
            int lx,ly;
            if(!axis(b,true,x,lx)||!axis(b,false,y,ly))continue;
            if(x>maxx-8||y>maxy-8)continue;
            int u=mod63(std::int64_t(lx)-y-prefix(x)),v=mod63(std::int64_t(ly)-x-prefix(y));
            if((sx>=0&&u!=sx)||(sy>=0&&v!=sy))continue;
            encode8(x,y,expected,u,v);
            if(std::memcmp(expected,b,64)!=0)continue;
            if(found)return false;
            // Reject any multi-orientation ambiguity.
            answer.x=x;
            answer.y=y;
            answer.section_x=std::uint8_t(u);
            answer.section_y=std::uint8_t(v);
            answer.quarter_turn_ccw=std::uint8_t(k);
            found=true;
        }
        if(found)result=answer;
        return found;
    }
    std::size_t Codec::constant_table_bytes() {
        return tables::table_bytes;
    }
    std::uint32_t Codec::coordinate_period() {
        return tables::period;
    }
}

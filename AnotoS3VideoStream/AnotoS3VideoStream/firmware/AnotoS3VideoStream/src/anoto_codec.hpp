#pragma once
#include <cstddef>
#include <cstdint>
namespace anoto {
    // Symbols follow the supplied Python code: 0=U,1=R,2=L,3=D.
    // x-bit is bit 1; y-bit is bit 0. Never interchange this with little-endian packbits.
    struct Address {
        std::uint32_t x=0, y=0;
        std::uint8_t section_x=0, section_y=0, quarter_turn_ccw=0;
    };
    class Codec {
        public:
        static std::uint8_t symbol(std::uint32_t x, std::uint32_t y, int sx=10,int sy=2);
        static void encode8(std::uint32_t x,std::uint32_t y,std::uint8_t out[64],int sx=10,int sy=2);
        // Exactly 128 bits are verified. No nearest-code matching or full-floor lookup.
        // section=-1 permits any section; max_x/max_y are exclusive grid-address bounds.
        static bool decode8(const std::uint8_t observed[64],Address& result,
        int section_x=10,int section_y=2,
        std::uint32_t max_x=6667,std::uint32_t max_y=6667);
        static void rotate_ccw(const std::uint8_t input[64],std::uint8_t output[64],int turns);
        static std::size_t constant_table_bytes();
        static std::uint32_t coordinate_period();
    };
}
// namespace anoto

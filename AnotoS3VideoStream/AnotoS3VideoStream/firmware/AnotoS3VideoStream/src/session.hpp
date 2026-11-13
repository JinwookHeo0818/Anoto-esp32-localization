#pragma once
#include "anoto_engine.hpp"
#include "wire.hpp"
#include <memory>
#include <string>
namespace replay {
struct Runtime {
    std::uint64_t (*now_us)();
    void (*memory)(std::uint32_t&,std::uint32_t&,std::uint32_t&);
    void (*send)(std::uint8_t,std::uint32_t,const std::uint8_t*,std::size_t);
    const char* hardware;
};
class Session {
    Runtime rt_;
    std::unique_ptr<anoto::Engine> engine_;
    int width_=0,height_=0;
    std::uint32_t next_=0,last_frame_=0,skipped_=0;
    std::uint64_t last_ts_=0,last_dt_=0;
    bool active_=false,extended_result_=false;
    std::vector<std::uint8_t> unpacked_;
    void text(std::uint8_t type,std::uint32_t seq,const std::string& s);
public:
    explicit Session(Runtime rt):rt_(rt){}
    void handle(const wire::Parser& packet,std::uint64_t received_us);
    void abort(){engine_.reset();active_=false;}
};
}

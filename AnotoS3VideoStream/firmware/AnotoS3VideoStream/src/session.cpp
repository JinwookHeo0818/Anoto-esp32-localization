#include "session.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <cstdio>
#include <exception>
#include <stdexcept>
namespace replay {
namespace {
// Single source of truth for the engine AND the HELLO diagnostic metadata.
constexpr unsigned kFrameBudgetMs=400;
constexpr int kMaxRecoveryFits=6;
}
void Session::text(std::uint8_t type,std::uint32_t seq,const std::string& s){rt_.send(type,seq,reinterpret_cast<const std::uint8_t*>(s.data()),s.size());}
void Session::handle(const wire::Parser& packet,std::uint64_t received_us){
    auto seq=packet.seq();const auto* p=packet.payload();auto n=packet.size();auto tc=rt_.now_us();
    if(!packet.crc_ok()){text(wire::ERROR,seq,"PAYLOAD_CRC_MISMATCH; stop and start a new run");return;}
    auto crc_us=rt_.now_us()-tc;
    try{
        if(packet.type()==wire::HELLO){
            if(n)throw std::runtime_error("HELLO payload must be empty");
            char msg[512];std::snprintf(msg,sizeof(msg),"{\"firmware\":\"anoto-mcu-recovery-v7.2\",\"protocol\":2,\"core\":\"v7.2 bounded deduplicated recovery\",\"supports_skipped_frames\":true,\"supports_adaptive_grid\":true,\"hardware\":\"%s\",\"max_width\":240,\"max_height\":180,\"pixel_format\":\"GRAY8\",\"input_buffer_bytes\":%u,\"frame_budget_ms\":%u,\"recovery_policy\":\"v7_tracking_exact_code\"}",rt_.hardware,unsigned(wire::MAX_PAYLOAD),kFrameBudgetMs);
            text(wire::HELLO_OK,seq,msg);return;
        }
        if(packet.type()==wire::START){
            if(n!=32)throw std::runtime_error("START must contain 32 bytes");
            int w=wire::u16(p),h=wire::u16(p+2);float pitch=wire::f32(p+4),offset=wire::f32(p+8);
            int sx=wire::u16(p+12),sy=wire::u16(p+14),blocks=p[16],pred=p[17];
            float fw=wire::f32(p+20),fh=wire::f32(p+24),forward=wire::f32(p+28);
            if(w<32||w>240||h<32||h>180)throw std::runtime_error("SIZE: width 32..240, height 32..180");
            if(!std::isfinite(pitch)||!std::isfinite(offset)||pitch<=0||offset/pitch<=.05||offset/pitch>=.25)throw std::runtime_error("Invalid print pitch/offset");
            if(sx>62||sy>62||blocks<1||blocks>3||pred>2||p[18]>1||p[19]>1)throw std::runtime_error("Invalid section/block/prediction option");
            if(!std::isfinite(fw)||!std::isfinite(fh)||!std::isfinite(forward)||fw<8*pitch||fh<8*pitch||fw>1e6||fh>1e6||std::abs(forward)>3600)throw std::runtime_error("Invalid floor dimensions/forward angle");
            // Destroy old engine before allocating the next one to bound peak RAM.
            abort();anoto::Options o;
            o.pitch_mm=pitch;o.offset_mm=offset;o.section_x=sx;o.section_y=sy;o.floor_width_mm=fw;o.floor_height_mm=fh;o.forward_image_deg=forward;
            o.max_dots=256;o.max_fit_points=144;o.patch_side=12;o.max_blocks=80;
            o.min_confirm=blocks==1?1:2;o.target_confirm=blocks;
            o.integer_centroids=true;o.fast_rigid=true;o.cached_reacquire=true;
            o.adaptive_reuse=p[18]==0;
            o.fixed_camera=true;o.patch_tracking=true;o.use_previous=true;
            o.use_motion_prediction=pred!=0;o.nonholonomic_prediction=pred==2;
            // One 400 ms decoder budget for BOTH tracking and recovery. Not the v7.1
            // 180 ms cutoff. Checks are inside engine loops; USB reception is separate.
            // This is a cooperative limit, not a hard real-time deadline.
            o.max_recovery_fits=kMaxRecoveryFits;o.recovery_budget_ms=double(kFrameBudgetMs);
            engine_.reset(new anoto::Engine(o));width_=w;height_=h;next_=0;last_frame_=0;skipped_=0;last_ts_=last_dt_=0;active_=true;
            text(wire::START_OK,seq,"{\"started\":true,\"frames_skipped\":0}");return;
        }
        if(packet.type()==wire::END){
            if(n)throw std::runtime_error("END payload must be empty");
            char msg[128];std::snprintf(msg,sizeof(msg),"{\"frames_processed\":%lu,\"source_frames_skipped_before_last\":%lu}",(unsigned long)next_,(unsigned long)skipped_);text(wire::END_OK,seq,msg);abort();return;
        }
        if(packet.type()!=wire::FRAME)throw std::runtime_error("Unknown packet type");
        if(!active_||!engine_)throw std::runtime_error("NO_SESSION: send START first");
        if(n!=12+std::size_t(width_)*height_)throw std::runtime_error("FRAME_LENGTH_MISMATCH");
        auto frame=wire::u32(p);auto ts=wire::u64(p+4);
        // Source indices may jump because the host sends only the latest due frame.
        // Never confuse source index with the count of processed frames.
        if(next_ && frame<=last_frame_)throw std::runtime_error("FRAME_ORDER_MISMATCH: require increasing source indices");
        if(next_ && ts<=last_ts_)throw std::runtime_error("NON_MONOTONIC_SOURCE_TIMESTAMP");
        auto dt=next_?ts-last_ts_:0;bool reset_timing=false,velocity_guard=false;
        if(next_){
            const double ratio=last_dt_?double(dt)/last_dt_:1.;
            // A slow computation is not evidence that the verified grid vanished.
            // Keep it; only disable velocity extrapolation across a large/unreliable
            // interval. Current-frame code validation is still mandatory.
            if(dt>1000000||ratio<.25||ratio>3.){
                engine_->clear_velocity_history();engine_->set_prediction_scale(1.);velocity_guard=true;
            }else engine_->set_prediction_scale(ratio);
        }
        auto t0=rt_.now_us();auto o=engine_->process({p+12,width_,height_,width_});auto compute_us=rt_.now_us()-t0;
        skipped_+=next_?(frame-last_frame_-1):frame;last_frame_=frame;
        last_ts_=ts;if(dt)last_dt_=dt;++next_;
        std::uint32_t mem=0,largest=0,low=0;rt_.memory(mem,largest,low);
        std::array<std::uint8_t,232> out{};auto* q=out.data();
        auto put16=[&](std::uint16_t v){wire::p16(q,v);q+=2;};
        auto put32=[&](std::uint32_t v){wire::p32(q,v);q+=4;};
        auto put64=[&](std::uint64_t v){wire::p64(q,v);q+=8;};
        auto putf=[&](float v){wire::pf32(q,v);q+=4;};
        put32(frame);put64(ts);*q++=o.valid?1:0;*q++=o.reference_supported?1:0;
        std::uint16_t flags=(o.used_previous?1:0)|(o.used_rigid_grid?2:0)|(o.used_patch_sampling?4:0)|(o.used_motion_prediction?8:0)|(reset_timing?16:0)|(velocity_guard?32:0)|(o.used_adaptive_grid?64:0)|(o.profile_rebased?128:0)|(o.used_basis_reacquire?256:0)|(o.used_phase_alignment?512:0);
        put16(flags);put16(std::uint16_t(o.verified_blocks));put16(std::uint16_t(o.dots));put16(std::uint16_t(o.sampled_windows));put16(std::uint16_t(o.fit_attempts));
        auto us32=[](std::uint64_t v){return std::uint32_t(std::min<std::uint64_t>(v,0xffffffffu));};
        put32(us32(compute_us));put32(us32(received_us));put32(us32(crc_us));put32(mem);put32(largest);put32(low);
        const float nan=std::numeric_limits<float>::quiet_NaN();
        putf(o.valid?float(o.x_mm):nan);putf(o.valid?float(o.y_mm):nan);putf(o.valid?float(o.heading_deg):nan);
        putf(float(o.time.dots_ms));putf(float(o.time.geometry_ms));putf(float(o.time.address_ms));
        for(auto b:o.box){putf(o.valid?float(b.x):nan);putf(o.valid?float(b.y):nan);}
        std::strncpy(reinterpret_cast<char*>(q),o.status.c_str(),31);q+=32;
        std::strncpy(reinterpret_cast<char*>(q),o.reason.c_str(),95);q+=96;
        rt_.send(wire::RESULT,seq,out.data(),out.size());
    }catch(const std::exception& e){abort();text(wire::ERROR,seq,e.what());}
}
}

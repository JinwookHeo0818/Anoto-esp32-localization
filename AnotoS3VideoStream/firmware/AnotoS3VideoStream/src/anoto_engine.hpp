#pragma once
#include "anoto_codec.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
namespace anoto {
    struct GrayView {
        const std::uint8_t* data=nullptr;
        int width=0,height=0,stride=0;
    };
    struct Point {
        double x=0,y=0;
    };
    // Calibration contains NO absolute floor pose or full-floor lookup.  It is
    // specific to one resolution, crop, lens, focus, mounting height and tilt.
    struct CameraProfile {
        bool valid=false;
        int width=0,height=0,threshold=127;
        double pitch_mm=1.5,offset_mm=.25;
        Point reference;
        std::array<double,9> image_from_metric{{1,0,0,0,1,0,0,0,1}};
        double diameter_px=0,area_px=0,pitch_px=0;
    };
    // Increment between the PREVIOUS and CURRENT input frame, at the robot's
    // nonholonomic reference point (typically rear-axle midpoint).  Not a pose.
    struct MotionHint {
        bool valid=false;
        double forward_mm=0,turn_deg=0; // signed forward arc length; clockwise yaw
    };
    struct Options {
        double pitch_mm=1.5, offset_mm=.25;
        int section_x=10,section_y=2;
        double floor_width_mm=10000,floor_height_mm=10000;
        int max_dots=512,max_fit_points=196,max_blocks=80,min_confirm=2;
        bool robust=false,use_previous=true;
        double forward_image_deg=0;
        bool fixed_camera=true,patch_tracking=true;
        // This is a PREDICTOR only; measured poses are never projected onto a path.
        bool nonholonomic_prediction=true,use_motion_prediction=true;
        double camera_forward_mm=0,camera_right_mm=0;
        double max_predict_mm=15,max_predict_deg=20;
        int patch_side=14,max_recovery_fits=12;
        // v3: area-independent code verification remains exact. Target and minimum
        // are separate so 1-block comparisons actually stop at one (opt-in only).
        int target_confirm=3;
        bool integer_centroids=true,fast_rigid=true,cached_reacquire=true;
        // Reuse verified grids through small hand-held scale/tilt changes.
        // False preserves the strict fixed-optics policy for controlled comparisons.
        bool adaptive_reuse=true;
        // 0: no wall-time budget. A positive value is a SOFT recovery budget,
        // checked between fit attempts, not a hard real-time deadline.
        double recovery_budget_ms=0;

    };
    struct Timing {
        double dots_ms=0,geometry_ms=0,address_ms=0,total_ms=0;
    };
    struct Observation {
        bool valid=false,reference_supported=false,used_previous=false;
        bool used_cached_appearance=false,used_rigid_grid=false,used_patch_sampling=false;
        bool used_motion_hint=false,used_motion_prediction=false,nonholonomic_relaxed=false;
        bool calibration_ready=false,optics_consistent=true;
        int sampled_windows=0;
        int rigid_iterations=0,rigid_solves=0;
        bool used_fixed_reacquire=false;
        bool used_adaptive_grid=false,used_basis_reacquire=false,profile_rebased=false,used_phase_alignment=false;
        double rigid_residual_cells=0;

        double x_mm=0,y_mm=0,heading_deg=0,pitch_px=0,dot_diameter_px=0;
        int dots=0,valid_dots=0,verified_blocks=0,fit_attempts=0,dropped_dots=0;
        std::size_t workspace_capacity_bytes=0;
        Timing time;
        Address address;
        std::string status="LOST",reason,threshold;
        Point box[4];
        Point selected_dots[64];
        std::uint8_t selected_symbols[64] {
        };
    };
    class Engine {
        public:
        explicit Engine(const Options& options=Options());
        ~Engine();
        Engine(const Engine&)=delete;
        Engine& operator=(const Engine&)=delete;
        // No image ownership, file access, networking, sleeps, or entire-floor map.
        // A persistent instance may process sequential frames; each accepted frame
        // must still pass CURRENT-frame exact 8x8 regeneration and agreement checks.
        Observation process(GrayView image,Point reference= {
            -1,-1
        });
        // A hard reset removes calibration; reset_tracking keeps the fixed optics.
        void reset();
        void reset_tracking();
        // Timing-only guard: keep calibration AND last grid, remove constant-step velocity history.
        void clear_velocity_history();
        // Current source-frame interval / previous interval, set by the input layer.
        void set_prediction_scale(double ratio);
        CameraProfile camera_profile() const;
        bool set_camera_profile(const CameraProfile& profile);
        // Optional encoder/IMU increment is used ONLY as an initial guess.
        Observation process_with_motion(GrayView image,const MotionHint& hint,
                                        Point reference={-1,-1});
        private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
// namespace anoto

#ifndef RADAR_UTILS_H
#define RADAR_UTILS_H
#include <iostream>
#include <string>
#include "vector"
#include <array>
#include <opencv2/opencv.hpp>
#include <vision_interface/robot_slots.h>

namespace tdt_radar
{
    // detect 输出的 cars[i].center 是 bbox 底边中点 (rect.y + height*1.0),
    // 对应机器人轮子接触地面的点 (非装甲板). 因此 raycast 时 armor_h 应为 0,
    // 让射线直接打到地表 (HeightGrid surface_z), 之后 apply_chassis_offset
    // 沿远离相机方向 +HALF_CHASSIS 把触地点平移到底面正方形中心.
    constexpr float ARMOR_HEIGHT_HERO     = 0.0f;
    constexpr float ARMOR_HEIGHT_ENGINEER = 0.0f;
    constexpr float ARMOR_HEIGHT_INFANTRY = 0.0f;
    constexpr float ARMOR_HEIGHT_SENTRY   = 0.0f;
    constexpr float ARMOR_HEIGHT_DEFAULT  = 0.0f;
    inline float armor_height_for_number(int /*robot_no*/) {
        return 0.0f;  // bbox bottom = wheel ground contact, ray hits surface directly
    }
    inline float armor_height_for_slot(int slot) {
        return armor_height_for_number(slot_to_robot_number(slot));
    }

    class Parser_Points
    {
    public:
        Parser_Points(const std::string &points_name,
                      const std::string &points_yaml_path,
                      const std::string &camera_params_path,
                      const std::string &out_matrix_path,
                      float map_height = 15.0F,
                      bool points_in_referee_frame = true);
        float return_height(cv::Point2f &input_point);
        void Update();
        void World2Camera();
        std::vector<cv::Point3f> ReadPoints(const std::string &points_name);
        std::vector<cv::Point> Float2Int(std::vector<cv::Point2f> &FloatPoint);
        std::vector<cv::Point3f> Points_3D;
        std::vector<cv::Point> Points_2D;
        float Height=0;
    private:
        std::string points_yaml_path_;
        std::string camera_params_path_ = "./config/camera_params.yaml";
        std::string out_matrix_path_ = "./config/out_matrix.yaml";
        float map_height_ = 15.0F;
        bool points_in_referee_frame_ = true;
        cv::Mat world_rvec;
        cv::Mat world_tvec;
        cv::Mat camera_matrix;
        cv::Mat dist_coeffs;
    };
    // HeightGrid: O(1) ray-intersection against a regular XY heightmap.
    // For a regular-grid heightmap the camera ray hits the surface at a known XY
    // (computed by the flat-plane fallback).  We then refine with 2 iterations of
    // "look up actual Z → recompute XY intersection" which converges in ≤3 steps
    // even at sharp platform edges.  This is ~1000× faster than brute-force
    // Möller-Trumbore and equally accurate for RM field geometry.
    class HeightGrid
    {
    public:
        // Load the .bin file written by tools/gen_field_mesh.py.
        // Header: 6×double (x_min, y_min, nx, ny, res, reserved)
        // Body:   ny×nx float32 Z values, row-major (NaN = no surface)
        bool load(const std::string& bin_path);
        bool loaded() const { return !z_.empty(); }

        // Look up Z at world (wx, wy).  Returns NaN if outside grid or no surface.
        float z_at(float wx, float wy) const;

        // Refine a camera ray-intersection point.
        // Given camera centre (cx,cy,cz) and unit direction (dx,dy,dz),
        // returns the XY world position where the ray hits the field surface
        // (accounting for real terrain Z instead of a flat plane).
        // Returns (NaN,NaN) if the ray misses the grid entirely.
        // armor_h: height of armour plate above the surface (metres, e.g. 0.15)
        cv::Point2f intersect_ray(float cx, float cy, float cz,
                                  float dx, float dy, float dz,
                                  float armor_h = 0.15F,
                                  int   max_iter = 15) const;

    private:
        std::vector<float> z_;
        double x_min_ = 0, y_min_ = 0, res_ = 0.1;
        int    nx_ = 0, ny_ = 0;
    };

    // Lightweight 3-D mesh ray caster — no external dependencies.
    // Load an ASCII PLY (vertices + triangular faces) then call intersect() to
    // find the first surface hit from a camera ray.  Falls back gracefully when
    // no mesh is loaded.
    class MeshRaycaster
    {
    public:
        // Returns true if the PLY was loaded successfully.
        bool load(const std::string& ply_path);
        bool loaded() const { return !triangles_.empty(); }
        size_t face_count() const { return triangles_.size(); }

        // Cast a ray from `origin` in direction `dir` (need not be unit).
        // Returns true on hit and fills `hit` (world XYZ).  hit.z gives the
        // actual surface height — useful for elevated platforms / fortress.
        bool intersect(const std::array<float,3>& origin,
                       const std::array<float,3>& dir,
                       std::array<float,3>&       hit) const;

    private:
        struct Tri { std::array<float,3> v0, v1, v2; };
        std::vector<Tri> triangles_;
    };

    class parser
    {
    public:
        explicit parser(const std::string &points_yaml_path = "",
                        const std::string &camera_params_path = "",
                        const std::string &out_matrix_path = "",
                        float map_height = 15.0F,
                        bool points_in_referee_frame = true);
        void Change_Matrix();
        cv::Point2f parse(cv::Point2f &input_point);
        cv::Point2f parse(cv::Point2f &input_point, int robot_number);  // per-type armor height
        void draw_ui(cv::Mat &img);
        float get_height(cv::Point2f &input_point);
        cv::Point2f get_2d(cv::Point2f &input_point,float height);
        cv::Point2f get_2d_raycast(const cv::Point2f& pixel, float height,
                                   float armor_h = ARMOR_HEIGHT_DEFAULT);
        cv::Point2f world_to_pixel(float wx, float wy, float wz) const;
        std::vector<cv::Point2f> world_to_pixels(
            const std::vector<cv::Point2f>& world_xy, float wz) const;
        bool        extrinsics_valid() const { return extrinsics_valid_; }
        // Camera optical centre in calibration world frame (same as parse() output)
        cv::Point3f camera_world_pos() const {
            if (!extrinsics_valid_) return {0.f, 0.f, 0.f};
            return { static_cast<float>(C_cached_.at<double>(0)),
                     static_cast<float>(C_cached_.at<double>(1)),
                     static_cast<float>(C_cached_.at<double>(2)) };
        }
        // Load a low-poly ASCII PLY for mesh ray casting (replaces flat-plane
        // assumption in get_2d_raycast when a mesh is available).
        bool        load_mesh(const std::string& ply_path);
        bool        mesh_loaded() const { return raycaster_.loaded(); }
        bool        load_grid(const std::string& bin_path);
        bool        grid_loaded() const { return hgrid_.loaded(); }
        cv::Mat world_rvec;
        cv::Mat world_tvec;
        cv::Mat camera_matrix;
        cv::Mat dist_coeffs;
        std::string points_yaml_path_ = "./config/map/map_points.yaml";
        std::string camera_params_path_ = "./config/camera_params.yaml";
        std::string out_matrix_path_ = "./config/out_matrix.yaml";
        float map_height_ = 15.0F;
        bool points_in_referee_frame_ = true;
        std::map<std::string, std::unique_ptr<Parser_Points>> points_map;
    private:
        void precompute_extrinsics();
        cv::Mat R_cached_;
        cv::Mat Rt_cached_;
        cv::Mat C_cached_;
        bool    extrinsics_valid_ = false;
        HeightGrid    hgrid_;       // O(1) fast path
        MeshRaycaster raycaster_;   // fallback for arbitrary PLY
    };
}
#endif //RADAR_UTILS_H

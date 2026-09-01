#include "radar_utils.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>

// 兵种高度常量已在 radar_utils.h 中定义；此别名供文件内部 fallback 路径使用
constexpr float ARMOR_HEIGHT = tdt_radar::ARMOR_HEIGHT_DEFAULT;  // 0.20m
namespace tdt_radar {

// ── HeightGrid ───────────────────────────────────────────────────────────────
// O(1) iterative ray-heightmap intersection.
// The RM field is essentially a height map; for a camera ray coming from above,
// the XY intersection point can be found by 2–3 Newton-style iterations:
//   1. Guess XY assuming flat plane at Z0
//   2. Look up actual surface Z at (XY) → new Z0
//   3. Recompute XY with new Z0  (converges in ≤3 steps for ≤3.5m elevation change)

bool HeightGrid::load(const std::string& bin_path)
{
    z_.clear();
    std::ifstream f(bin_path, std::ios::binary);
    if (!f.is_open()) {
        std::cerr << "[HeightGrid] Cannot open " << bin_path << "\n";
        return false;
    }
    // Header: 6 × double  (x_min, y_min, nx, ny, res, reserved)
    double hdr[6] = {};
    f.read(reinterpret_cast<char*>(hdr), 6 * sizeof(double));
    if (!f) { std::cerr << "[HeightGrid] Header read failed\n"; return false; }
    x_min_ = hdr[0];  y_min_ = hdr[1];
    nx_    = static_cast<int>(hdr[2]);
    ny_    = static_cast<int>(hdr[3]);
    res_   = hdr[4];
    if (nx_ <= 0 || ny_ <= 0 || res_ <= 0) {
        std::cerr << "[HeightGrid] Bad header\n"; return false;
    }
    z_.resize(static_cast<size_t>(nx_) * ny_);
    f.read(reinterpret_cast<char*>(z_.data()), z_.size() * sizeof(float));
    if (!f) { std::cerr << "[HeightGrid] Data read failed\n"; return false; }
    std::cout << "[HeightGrid] Loaded " << bin_path
              << "  grid=" << nx_ << "×" << ny_
              << "  res=" << res_ << "m\n";
    return true;
}

float HeightGrid::z_at(float wx, float wy) const
{
    if (z_.empty()) return std::numeric_limits<float>::quiet_NaN();
    // Bilinear interpolation over the regular grid
    const float fx = static_cast<float>((wx - x_min_) / res_);
    const float fy = static_cast<float>((wy - y_min_) / res_);
    const int ix = static_cast<int>(fx);
    const int iy = static_cast<int>(fy);
    if (ix < 0 || ix >= nx_ - 1 || iy < 0 || iy >= ny_ - 1)
        return std::numeric_limits<float>::quiet_NaN();
    const float tx = fx - ix,  ty = fy - iy;
    const float z00 = z_[iy       * nx_ + ix    ];
    const float z10 = z_[iy       * nx_ + ix + 1];
    const float z01 = z_[(iy + 1) * nx_ + ix    ];
    const float z11 = z_[(iy + 1) * nx_ + ix + 1];
    // If any corner is NaN, use nearest valid corner
    if (std::isnan(z00) || std::isnan(z10) || std::isnan(z01) || std::isnan(z11)) {
        const float vals[4] = {z00, z10, z01, z11};
        for (float v : vals) if (!std::isnan(v)) return v;
        return std::numeric_limits<float>::quiet_NaN();
    }
    return (1 - tx) * (1 - ty) * z00 + tx * (1 - ty) * z10 +
           (1 - tx) *      ty  * z01 + tx *      ty  * z11;
}

cv::Point2f HeightGrid::intersect_ray(float cx, float cy, float cz,
                                      float dx, float dy, float dz,
                                      float armor_h, int max_iter) const
{
    const cv::Point2f nan_pt(std::numeric_limits<float>::quiet_NaN(),
                             std::numeric_limits<float>::quiet_NaN());
    if (z_.empty() || std::abs(dz) < 1e-6F) return nan_pt;

    // RM 场地中机器人能站立的最高水平面 (飞坡顶 / 中央高低台 / 梯形地) < 1m。
    // 高度网格按 max-Z 生成, 墙/塔/基地顶部会出现 1-3.5m 的 Z 值; 直接迭代
    // 会让 target_z 跳到结构物顶, 射线交点偏离实际地面 2-40 m。
    // 解决方案: 把 Z>1m 的格子视为非地面 (机器人不可能站在那里), 按 0 处理。
    constexpr float kMaxWalkableZ = 1.0f;
    // 欠松弛 + 收敛检测: 近水平视角下 (dz 很小) 迭代极不稳定, 容易在多个 Z
    // 之间振荡。用 α=0.5 衰减步长可让迭代收敛而不影响最终精度。
    constexpr float kRelax        = 0.5f;
    constexpr float kConvergeEps  = 0.01f;  // 1 cm convergence threshold

    float target_z   = armor_h;
    float wx = 0, wy = 0;
    float prev_target_z = target_z;
    for (int i = 0; i < max_iter; ++i) {
        const float t = (target_z - cz) / dz;
        if (t < 0.0F) return nan_pt;          // ray going away from surface
        wx = cx + t * dx;
        wy = cy + t * dy;
        float surf_z = z_at(wx, wy);
        if (std::isnan(surf_z)) return nan_pt; // outside field mesh
        if (surf_z > kMaxWalkableZ) surf_z = 0.0f;  // wall/tower → use ground
        const float desired = surf_z + armor_h;
        const float new_target = kRelax * desired + (1.0f - kRelax) * target_z;
        if (std::fabs(new_target - prev_target_z) < kConvergeEps) {
            target_z = new_target;
            // Final hit at converged Z
            const float tf = (target_z - cz) / dz;
            return cv::Point2f(cx + tf * dx, cy + tf * dy);
        }
        prev_target_z = target_z;
        target_z = new_target;
    }
    return cv::Point2f(wx, wy);
}

bool parser::load_grid(const std::string& bin_path)
{
    return hgrid_.load(bin_path);
}
// ── End HeightGrid ────────────────────────────────────────────────────────────

// ── MeshRaycaster ────────────────────────────────────────────────────────────
// Loads an ASCII PLY with "element vertex" and "element face" sections.
// Performs Möller–Trumbore ray-triangle intersection (front and back faces).

bool MeshRaycaster::load(const std::string& ply_path)
{
    triangles_.clear();
    std::ifstream f(ply_path);
    if (!f.is_open()) {
        std::cerr << "[MeshRaycaster] Cannot open " << ply_path << "\n";
        return false;
    }

    std::string line;
    int n_verts = 0, n_faces = 0;
    bool header_done = false;

    // Parse PLY header
    while (std::getline(f, line)) {
        if (line.rfind("element vertex", 0) == 0) {
            std::istringstream ss(line); std::string a, b;
            ss >> a >> b >> n_verts;
        } else if (line.rfind("element face", 0) == 0) {
            std::istringstream ss(line); std::string a, b;
            ss >> a >> b >> n_faces;
        } else if (line == "end_header") {
            header_done = true;
            break;
        }
    }
    if (!header_done || n_verts <= 0 || n_faces <= 0) {
        std::cerr << "[MeshRaycaster] PLY header parse failed (verts="
                  << n_verts << " faces=" << n_faces << ")\n";
        return false;
    }

    // Read vertices
    std::vector<std::array<float,3>> verts(n_verts);
    for (int i = 0; i < n_verts; ++i) {
        if (!std::getline(f, line)) return false;
        std::istringstream ss(line);
        ss >> verts[i][0] >> verts[i][1] >> verts[i][2];
    }

    // Read faces (expect "3 i0 i1 i2")
    triangles_.reserve(n_faces);
    for (int i = 0; i < n_faces; ++i) {
        if (!std::getline(f, line)) break;
        std::istringstream ss(line);
        int cnt, i0, i1, i2;
        ss >> cnt >> i0 >> i1 >> i2;
        if (cnt != 3) continue;
        if (i0 < 0 || i0 >= n_verts || i1 < 0 || i1 >= n_verts ||
            i2 < 0 || i2 >= n_verts) continue;
        triangles_.push_back({verts[i0], verts[i1], verts[i2]});
    }
    std::cout << "[MeshRaycaster] Loaded " << ply_path
              << "  verts=" << n_verts << "  faces=" << triangles_.size() << "\n";
    return !triangles_.empty();
}

bool MeshRaycaster::intersect(const std::array<float,3>& orig,
                              const std::array<float,3>& dir,
                              std::array<float,3>&       hit) const
{
    // Möller–Trumbore ray-triangle intersection.
    // Returns the closest hit with t > 0.
    constexpr float EPS = 1e-7F;
    float best_t = std::numeric_limits<float>::infinity();

    for (const auto& tri : triangles_) {
        const float* v0 = tri.v0.data();
        const float* v1 = tri.v1.data();
        const float* v2 = tri.v2.data();

        float e1[3] = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
        float e2[3] = {v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]};

        // h = dir × e2
        float h[3] = {
            dir[1]*e2[2] - dir[2]*e2[1],
            dir[2]*e2[0] - dir[0]*e2[2],
            dir[0]*e2[1] - dir[1]*e2[0]
        };
        const float a = e1[0]*h[0] + e1[1]*h[1] + e1[2]*h[2];
        if (std::abs(a) < EPS) continue;  // parallel

        const float f = 1.0F / a;
        float s[3] = {orig[0]-v0[0], orig[1]-v0[1], orig[2]-v0[2]};
        const float u = f * (s[0]*h[0] + s[1]*h[1] + s[2]*h[2]);
        if (u < 0.0F || u > 1.0F) continue;

        float q[3] = {
            s[1]*e1[2] - s[2]*e1[1],
            s[2]*e1[0] - s[0]*e1[2],
            s[0]*e1[1] - s[1]*e1[0]
        };
        const float v = f * (dir[0]*q[0] + dir[1]*q[1] + dir[2]*q[2]);
        if (v < 0.0F || u + v > 1.0F) continue;

        const float t = f * (e2[0]*q[0] + e2[1]*q[1] + e2[2]*q[2]);
        if (t > EPS && t < best_t) best_t = t;
    }

    if (std::isinf(best_t)) return false;
    hit[0] = orig[0] + best_t * dir[0];
    hit[1] = orig[1] + best_t * dir[1];
    hit[2] = orig[2] + best_t * dir[2];
    return true;
}

bool parser::load_mesh(const std::string& ply_path)
{
    return raycaster_.load(ply_path);
}
// ── End MeshRaycaster ─────────────────────────────────────────────────────────

namespace {
std::map<std::string, float> default_region_heights()
{
    return {
        {"Red_Trapezoid_Highland", 0.6F},
        {"Red_Fortress", 0.15F},
        {"Red_Road", 0.3F},
        {"Red_FlySlope", 1.0F},
        {"Center_HighLow", 0.8F},
        {"Blue_Road", 0.3F},
        {"Blue_Fortress", 0.15F},
        {"Blue_Base", 0.15F},
        {"Blue_Trapezoid_Highland", 0.6F},
    };
}

std::map<std::string, float> load_region_heights(const std::string& points_yaml_path)
{
    std::map<std::string, float> heights = default_region_heights();
    cv::FileStorage fs(points_yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return heights;
    }

    cv::FileNode heights_node = fs["Region_Heights"];
    if (heights_node.type() == cv::FileNode::MAP) {
        for (auto it = heights_node.begin(); it != heights_node.end(); ++it) {
            float value = 0.3F;
            *it >> value;
            heights[(*it).name()] = value;
        }
    }
    return heights;
}

std::vector<std::string> load_region_names(const std::string& points_yaml_path)
{
    std::vector<std::string> names;
    cv::FileStorage fs(points_yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        for (const auto& item : default_region_heights()) {
            names.push_back(item.first);
        }
        return names;
    }

    cv::FileNode root = fs.root();
    for (auto it = root.begin(); it != root.end(); ++it) {
        const std::string node_name = (*it).name();
        if (node_name == "Region_Heights") {
            continue;
        }

        if ((*it).type() != cv::FileNode::SEQ) {
            continue;
        }

        bool valid_seq = true;
        for (auto&& point_node : *it) {
            if (point_node.type() != cv::FileNode::MAP || point_node["x"].empty() ||
                point_node["y"].empty() || point_node["z"].empty()) {
                valid_seq = false;
                break;
            }
        }

        if (valid_seq) {
            names.push_back(node_name);
        }
    }

    if (names.empty()) {
        for (const auto& item : default_region_heights()) {
            names.push_back(item.first);
        }
    }
    return names;
}
}  // namespace

bool isPointInsideScreen(cv::Point2f point, int screenWidth,
                         int screenHeight)
{
    return point.x >= 0 && point.x <= screenWidth && point.y >= 0 &&
           point.y <= screenHeight;
}
parser::parser(const std::string& points_yaml_path,
               const std::string& camera_params_path,
               const std::string& out_matrix_path,
               float map_height,
               bool points_in_referee_frame)
{
    if (!points_yaml_path.empty()) {
        points_yaml_path_ = points_yaml_path;
    }
    if (!camera_params_path.empty()) {
        camera_params_path_ = camera_params_path;
    }
    if (!out_matrix_path.empty()) {
        out_matrix_path_ = out_matrix_path;
    }
    map_height_ = map_height;
    points_in_referee_frame_ = points_in_referee_frame;

    cv::FileStorage fs;
    fs.open(out_matrix_path_, cv::FileStorage::READ);
    fs["world_tvec"] >> this->world_tvec;
    fs["world_rvec"] >> this->world_rvec;
    fs.release();

    cv::FileStorage fs1;
    fs1.open(camera_params_path_, cv::FileStorage::READ);
    fs1["camera_matrix"] >> this->camera_matrix;
    fs1["dist_coeffs"] >> this->dist_coeffs;
    fs1.release();

    const auto region_names = load_region_names(points_yaml_path_);
    const auto region_heights = load_region_heights(points_yaml_path_);

    for (const auto& region_name : region_names) {
        points_map[region_name] =
            std::make_unique<Parser_Points>(region_name, points_yaml_path_,
                                            camera_params_path_, out_matrix_path_,
                                            map_height_, points_in_referee_frame_);
        auto h_it = region_heights.find(region_name);
        points_map[region_name]->Height =
            (h_it != region_heights.end()) ? h_it->second : 0.3F;
    }
    precompute_extrinsics();
}
void parser::Change_Matrix()
{
    cv::FileStorage fs;
    fs.open(out_matrix_path_, cv::FileStorage::READ);
    fs["world_tvec"] >> this->world_tvec;
    fs["world_rvec"] >> this->world_rvec;
    fs.release();
    precompute_extrinsics();
    for (auto& points : points_map) {
        points.second->Update();
    }
}
void parser::precompute_extrinsics()
{
    extrinsics_valid_ = false;
    if (world_rvec.empty() || world_tvec.empty() || camera_matrix.empty())
        return;
    cv::Rodrigues(world_rvec, R_cached_);
    Rt_cached_ = R_cached_.t();
    C_cached_  = -Rt_cached_ * world_tvec;
    extrinsics_valid_ = true;
}
cv::Point2f parser::world_to_pixel(float wx, float wy, float wz) const
{
    if (!extrinsics_valid_) return {-1.f, -1.f};
    const float y_off = points_in_referee_frame_ ? -map_height_ : 0.0F;
    std::vector<cv::Point3f> wpts = {{wx, wy + y_off, wz}};
    std::vector<cv::Point2f> ipts;
    cv::projectPoints(wpts, world_rvec, world_tvec, camera_matrix, dist_coeffs, ipts);
    return ipts[0];
}
std::vector<cv::Point2f> parser::world_to_pixels(
    const std::vector<cv::Point2f>& world_xy, float wz) const
{
    std::vector<cv::Point2f> pixels;
    pixels.reserve(world_xy.size());
    if (world_xy.empty()) return pixels;
    if (!extrinsics_valid_) {
        pixels.assign(world_xy.size(), cv::Point2f{-1.f, -1.f});
        return pixels;
    }

    // Callers pass referee-frame XY; projectPoints needs legacy-frame
    // (Y shifted by -map_height when calibrated with referee points).
    const float y_off = points_in_referee_frame_ ? -map_height_ : 0.0F;
    std::vector<cv::Point3f> wpts;
    wpts.reserve(world_xy.size());
    for (const auto& p : world_xy) {
        wpts.emplace_back(p.x, p.y + y_off, wz);
    }
    cv::projectPoints(wpts, world_rvec, world_tvec, camera_matrix, dist_coeffs, pixels);
    return pixels;
}
cv::Point2f parser::get_2d_raycast(const cv::Point2f& pixel, float height,
                                   float armor_h)
{
    const cv::Point2f nan_pt(std::numeric_limits<float>::quiet_NaN(),
                             std::numeric_limits<float>::quiet_NaN());
    if (!extrinsics_valid_) return nan_pt;

    std::vector<cv::Point2f> undist;
    cv::undistortPoints(
        std::vector<cv::Point2f>{pixel}, undist,
        camera_matrix, dist_coeffs, cv::noArray(), camera_matrix);
    const cv::Point2f up = undist[0];

    const double fx = camera_matrix.at<double>(0, 0);
    const double fy = camera_matrix.at<double>(1, 1);
    const double cx = camera_matrix.at<double>(0, 2);
    const double cy = camera_matrix.at<double>(1, 2);
    cv::Mat d_cam = (cv::Mat_<double>(3, 1)
        << (up.x - cx) / fx, (up.y - cy) / fy, 1.0);

    cv::Mat d_world = Rt_cached_ * d_cam;

    // Camera centre in world frame
    const std::array<float,3> orig = {
        static_cast<float>(C_cached_.at<double>(0)),
        static_cast<float>(C_cached_.at<double>(1)),
        static_cast<float>(C_cached_.at<double>(2))
    };
    const std::array<float,3> dir = {
        static_cast<float>(d_world.at<double>(0)),
        static_cast<float>(d_world.at<double>(1)),
        static_cast<float>(d_world.at<double>(2))
    };

    // ── Priority 1: HeightGrid O(1) iterative lookup ─────────────────────────
    // Regular-grid heightmap → bilinear Z lookup + 3-iteration ray refinement.
    // ~1000× faster than brute-force Möller-Trumbore, same accuracy for RM field.
    //
    // NOTE: HeightGrid / MeshRaycaster data is in referee frame (Y ∈ [0, 15]),
    // but solvePnP extrinsics use a legacy frame (Y shifted by -map_height).
    // Convert origin Y to referee before lookup; direction is unaffected (pure
    // translation).  Convert the result back to legacy so convert_parsed_y()
    // still works correctly downstream.
    const float y_shift = points_in_referee_frame_ ? map_height_ : 0.0F;

    if (hgrid_.loaded()) {
        cv::Point2f p = hgrid_.intersect_ray(
            orig[0], orig[1] + y_shift, orig[2],
            dir[0],  dir[1],            dir[2],
            armor_h);
        if (std::isfinite(p.x) && std::isfinite(p.y)) {
            p.y -= y_shift;   // back to legacy frame
            return p;
        }
        // Ray missed grid (sky / outside field) → fall through
    }

    // ── Priority 2: MeshRaycaster (arbitrary PLY, Möller-Trumbore) ───────────
    if (raycaster_.loaded()) {
        const std::array<float,3> ref_orig = {orig[0], orig[1] + y_shift, orig[2]};
        std::array<float,3> hit;
        if (raycaster_.intersect(ref_orig, dir, hit)) {
            hit[1] -= y_shift;  // back to legacy frame
            return cv::Point2f(hit[0], hit[1]);
        }
    }

    // ── Fallback: flat-plane intersection at Z = ARMOR_HEIGHT + zone_height ───
    const double dz = d_world.at<double>(2);
    if (std::abs(dz) < 1e-6) return nan_pt;

    const double target_z = static_cast<double>(armor_h + height);
    const double t = (target_z - C_cached_.at<double>(2)) / dz;
    if (t < 0.0) return nan_pt;

    return cv::Point2f(
        static_cast<float>(C_cached_.at<double>(0) + t * d_world.at<double>(0)),
        static_cast<float>(C_cached_.at<double>(1) + t * d_world.at<double>(1)));
}
void parser::draw_ui(cv::Mat& img)
{
    for (auto& points : points_map) {
        if (points.second->Points_2D.size() >= 2) {
            cv::polylines(img, points.second->Points_2D, true,
                          cv::Scalar(255, 255, 255));
        }
    }
}
cv::Point2f parser::parse(cv::Point2f& input_point)
{
    float temp_height = get_height(input_point);
    if (extrinsics_valid_) {
        return get_2d_raycast(input_point, temp_height);
    }
    return get_2d(input_point, temp_height);
}
cv::Point2f parser::parse(cv::Point2f& input_point, int robot_number)
{
    float temp_height = get_height(input_point);
    float armor_h     = tdt_radar::armor_height_for_number(robot_number);
    if (extrinsics_valid_) {
        return get_2d_raycast(input_point, temp_height, armor_h);
    }
    return get_2d(input_point, temp_height);  // fallback: no armor_h correction without extrinsics
}
float parser::get_height(cv::Point2f& input_point)
{
    for (auto& points : points_map) {
        if (points.second->return_height(input_point)) {
            return points.second->Height;
        }
    }
    return 0;
}
cv::Point2f parser::get_2d(cv::Point2f& input_point, float height)
{
    std::vector<cv::Point3f> world_points;
    // 备注: 此方法仅用于calibrate模块的draw_ui可视化调试，不在主运行链路中调用
    world_points.push_back(cv::Point3f(12.0f, 6.0f, ARMOR_HEIGHT + height));
    world_points.push_back(cv::Point3f(16.0f, 6.0f, ARMOR_HEIGHT + height));
    world_points.push_back(cv::Point3f(16.0f, 9.0f, ARMOR_HEIGHT + height));
    world_points.push_back(cv::Point3f(12.0f, 9.0f, ARMOR_HEIGHT + height));
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(world_points, world_rvec, world_tvec, camera_matrix,
                      dist_coeffs, image_points);
    std::vector<cv::Point2f> world_points2D;
    world_points2D.push_back(cv::Point2f(12, -6));
    world_points2D.push_back(cv::Point2f(16, -6));
    world_points2D.push_back(cv::Point2f(16, -8));
    world_points2D.push_back(cv::Point2f(12, -8));
    cv::Mat Perspective_matrix =
        cv::getPerspectiveTransform(image_points, world_points2D);
    cv::Mat srcPointMat(1, 1, CV_32FC2);
    srcPointMat.at<cv::Point2f>(0, 0) = input_point;
    cv::perspectiveTransform(srcPointMat, srcPointMat, Perspective_matrix);
    return srcPointMat.at<cv::Point2f>(0, 0);
}
std::vector<cv::Point3f>
Parser_Points::ReadPoints(const std::string& points_name)
{
    cv::FileStorage fs(points_yaml_path_,
                       cv::FileStorage::READ);  // 打开YAML文件

    if (!fs.isOpened()) {
        std::cout << "无法打开文件" << std::endl;
        exit(-1);
    }

    std::vector<cv::Point3f> points;

    cv::FileNode pointsNode = fs[points_name];
    if (pointsNode.type() != cv::FileNode::SEQ) {
        std::cout << "警告: 区域 " << points_name << " 不是点序列，已忽略"
                  << std::endl;
        return points;
    }

    for (auto&& it : pointsNode) {
        cv::Point3f point;
        it["x"] >> point.x;
        it["y"] >> point.y;
        it["z"] >> point.z;

        points.push_back(point);
    }

    // Convert once at load-time for compatibility with legacy internal frame.
    if (points_in_referee_frame_) {
        for (auto& p : points) {
            p.y -= map_height_;
        }
    }

    return points;
}
std::vector<cv::Point>
Parser_Points::Float2Int(std::vector<cv::Point2f>& FloatPoint)
{
    std::vector<cv::Point> dstPoint;
    for (auto& i : FloatPoint) {
        dstPoint.emplace_back(int(i.x), int(i.y));
    }
    return dstPoint;
}
void Parser_Points::World2Camera()
{
    if (Points_3D.empty()) {
        Points_2D.clear();
        return;
    }
    std::vector<cv::Point2f> temp_2D;
    cv::projectPoints(Points_3D, world_rvec, world_tvec, camera_matrix,
                      dist_coeffs, temp_2D);
    Points_2D = Float2Int(temp_2D);
}
Parser_Points::Parser_Points(const std::string& points_name,
                             const std::string& points_yaml_path,
                             const std::string& camera_params_path,
                             const std::string& out_matrix_path,
                             float map_height,
                             bool points_in_referee_frame)
{
    points_yaml_path_ = points_yaml_path;
    map_height_ = map_height;
    points_in_referee_frame_ = points_in_referee_frame;
    if (!camera_params_path.empty()) {
        camera_params_path_ = camera_params_path;
    }
    if (!out_matrix_path.empty()) {
        out_matrix_path_ = out_matrix_path;
    }

    cv::FileStorage fs;
    fs.open(camera_params_path_, cv::FileStorage::READ);
    fs["camera_matrix"] >> this->camera_matrix;
    fs["dist_coeffs"] >> this->dist_coeffs;
    fs.release();

    fs.open(out_matrix_path_, cv::FileStorage::READ);
    fs["world_tvec"] >> this->world_tvec;
    fs["world_rvec"] >> this->world_rvec;
    fs.release();
    std::vector<cv::Point3f> temp_3d = ReadPoints(points_name);
    this->Points_3D = temp_3d;
    World2Camera();
}

float Parser_Points::return_height(cv::Point2f& input_point)
{
    if (Points_2D.empty()) {
        return 0;
    }
    if (cv::pointPolygonTest(
            Points_2D, cv::Point((int)input_point.x, (int)input_point.y),
            false) >= 0) {
        return this->Height;
    } else {
        return 0;
    }
}
void Parser_Points::Update()
{
    cv::FileStorage fs;
    fs.open(out_matrix_path_, cv::FileStorage::READ);
    fs["world_tvec"] >> this->world_tvec;
    fs["world_rvec"] >> this->world_rvec;
    fs.release();
    cv::FileStorage fs2;
    fs2.open(camera_params_path_, cv::FileStorage::READ);
    if (fs2.isOpened()) {
        fs2["camera_matrix"] >> this->camera_matrix;
        fs2["dist_coeffs"]   >> this->dist_coeffs;
    }
    World2Camera();
}
}  // namespace tdt_radar

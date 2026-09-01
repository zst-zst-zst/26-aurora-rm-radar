#include "calibrate.h"
#include <algorithm>
#include <array>
#include <iomanip>
#include <string>

namespace tdt_radar {
namespace {

std::vector<cv::Point3f> default_calibration_points()
{
    return {
        cv::Point3f(5.471F, -7.5F, 0.0F),
        cv::Point3f(10.936F, -11.161F, 0.868F),
        cv::Point3f(25.49F, -7.5F, 1.24524F),
        cv::Point3f(16.925F, -3.625F, 1.745F),
        cv::Point3f(17.178F, -3.6643F, 1.8684F),  // cross_tower: self_tower 场地中心对称点
    };
}

const std::array<std::string, 5>& calibration_point_keys()
{
    static const std::array<std::string, 5> keys = {
        "self_fortress", "self_tower", "enemy_base", "enemy_tower", "cross_tower"};
    return keys;
}

bool read_point3f(cv::FileStorage& fs, const std::string& key, cv::Point3f& out)
{
    auto node = fs[key];
    if (node.type() != cv::FileNode::SEQ || node.size() < 3) {
        return false;
    }

    node[0] >> out.x;
    node[1] >> out.y;
    node[2] >> out.z;
    return true;
}

std::vector<cv::Point3f> load_calibration_points(const std::string& yaml_path,
                                                 const rclcpp::Logger& logger,
                                                 float map_height,
                                                 bool points_in_referee_frame)
{
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        RCLCPP_WARN(logger,
                    "Failed to open calibrate points file: %s, fallback to defaults",
                    yaml_path.c_str());
        return default_calibration_points();
    }

    // 5 个点：三个低点 (fortress / base / tower) + 两个高点 (self_tower / cross_tower)。
    // 原本的 enemy_high z=0.30 跟 cross_tower XY 重叠且贡献低，已換掉。
    const auto& keys = calibration_point_keys();

    std::vector<cv::Point3f> points;
    points.reserve(keys.size());
    for (const auto& key : keys) {
        cv::Point3f p;
        if (!read_point3f(fs, key, p)) {
            RCLCPP_WARN(logger,
                        "Invalid calibrate point key '%s' in %s, fallback to defaults",
                        key.c_str(), yaml_path.c_str());
            return default_calibration_points();
        }
        points.push_back(p);
    }

    // Convert once at load-time for compatibility with legacy internal frame.
    if (points_in_referee_frame) {
        for (auto& p : points) {
            p.y -= map_height;
        }
    }

    return points;
}

}  // namespace

Calibrate::Calibrate(const rclcpp::NodeOptions& options)
    : Node("radar_calibrate_node", options)
{
    std::cout << "Calibrate start" << std::endl;
    // 三窗口布局（1920×1080 屏，启动即可用，无需拖拽）：
    //   ROI       (  0,   0) 400×400   ←  右上方放大点
    //   cali_ref  (  0, 430) 400×400   ←  地标参考图（cali.png）
    //   calibrate (480, 180) 1440×900  ←  主标定画面
    cv::namedWindow("calibrate", cv::WINDOW_AUTOSIZE);
    cv::resizeWindow("calibrate", 1440, 900);
    cv::moveWindow("calibrate", 1920 - 1440, 1080 - 900);
    cv::namedWindow("ROI", cv::WINDOW_AUTOSIZE);
    cv::resizeWindow("ROI", 400, 400);
    cv::moveWindow("ROI", 0, 0);
    cv::setMouseCallback("calibrate", mousecallback, 0);

    // 第三个窗口：地标参考图。从 outputs/cali.png 或 cali.png 任一存在的位置加载。
    {
        const std::vector<std::string> candidates = {
            "./config/map/cali.png",
            "config/map/cali.png",
        };
        cv::Mat cali_ref;
        std::string used;
        for (const auto& p : candidates) {
            cali_ref = cv::imread(p, cv::IMREAD_COLOR);
            if (!cali_ref.empty()) { used = p; break; }
        }
        if (!cali_ref.empty()) {
            cv::Mat shown;
            cv::resize(cali_ref, shown, cv::Size(400, 400));
            cv::namedWindow("cali_ref", cv::WINDOW_AUTOSIZE);
            cv::moveWindow("cali_ref", 0, 430);
            cv::imshow("cali_ref", shown);
            std::cout << "[calibrate] cali reference loaded from " << used << std::endl;
        } else {
            std::cout << "[calibrate] cali.png not found in config/ or config/outputs/, "
                         "skip reference window" << std::endl;
        }
    }

    const auto camera_params_path =
        this->declare_parameter<std::string>("camera_params",
                                             "./config/camera_params.yaml");
    out_matrix_path_ = this->declare_parameter<std::string>(
        "out_matrix", "./config/out_matrix.yaml");

    cv::FileStorage fs;
    fs.open(camera_params_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to open camera params: %s", camera_params_path.c_str());
    } else {
        fs["camera_matrix"] >> camera_matrix;
        fs["dist_coeffs"] >> dist_coeffs;
        fs.release();
    }

    const auto calibrate_points_path =
        this->declare_parameter<std::string>("calibrate_points",
                                             "./config/calibrate_points_red.yaml");
    const auto map_height = this->declare_parameter<double>("map_height", 15.0);
    const auto calibrate_points_referee_frame =
        this->declare_parameter<bool>("calibrate_points_referee_frame", true);
    const auto map_points_referee_frame =
        this->declare_parameter<bool>("map_points_referee_frame", true);
    real_points =
        load_calibration_points(calibrate_points_path, this->get_logger(),
                                static_cast<float>(map_height),
                                calibrate_points_referee_frame);
    map_height_ = static_cast<float>(map_height);
    points_in_referee_frame_ = calibrate_points_referee_frame;
    // 优先用 launch 显式传入的 team_label，没传再从输出路径推断
    team_label_ = this->declare_parameter<std::string>("team_label", "");
    if (team_label_.empty()) {
        if (out_matrix_path_.find("red") != std::string::npos)       team_label_ = "RED";
        else if (out_matrix_path_.find("blue") != std::string::npos) team_label_ = "BLUE";
    }

    const auto map_points_path =
        this->declare_parameter<std::string>("map_points",
                                             "./config/map/map_points.yaml");
    parser_ = new parser(map_points_path, camera_params_path,
                         out_matrix_path_, static_cast<float>(map_height),
                         map_points_referee_frame);

    // 仅订压缩流。原始流 camera_image 留给 YOLO detect，避免双订阅抢按键。
    compressed_image_sub =
        this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "compressed_image", rclcpp::SensorDataQoS(),
            std::bind(&Calibrate::compressed_callback, this,
                      std::placeholders::_1));
    std::cout << "Calibrate end" << std::endl;
}

void Calibrate::compressed_callback(
    const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    auto    img = cv::imdecode(msg->data, cv::IMREAD_COLOR);
    orig_width_  = img.cols;
    orig_height_ = img.rows;
    orig_image_  = img.clone();

    // ROI 半边长与 wasd 步长按图像宽度自适应，保证 6MP / 低分辨率体验一致。
    roi_half_  = std::max(50, orig_width_ / 30);
    pick_step_ = std::max(1, orig_width_ / 1500);

    constexpr int max_display_width  = 1536;
    constexpr int max_display_height = 1024;
    const double display_scale = std::min(
        static_cast<double>(max_display_width) / std::max(1, orig_width_),
        static_cast<double>(max_display_height) / std::max(1, orig_height_));
    display_width_ = std::max(1, static_cast<int>(std::round(orig_width_ * display_scale)));
    display_height_ = std::max(1, static_cast<int>(std::round(orig_height_ * display_scale)));

    static int logged_width = 0;
    static int logged_height = 0;
    if (logged_width != orig_width_ || logged_height != orig_height_) {
        logged_width = orig_width_;
        logged_height = orig_height_;
        RCLCPP_INFO(this->get_logger(),
                    "Calibration image size: original=%dx%d display=%dx%d",
                    orig_width_, orig_height_, display_width_, display_height_);
    }

    cv::Mat calib_img;
    cv::resize(img, calib_img, cv::Size(display_width_, display_height_));
    cvimage_ = calib_img;
    if (is_calibrating) {
        const auto& keys = calibration_point_keys();
        const size_t idx = std::min(pick_points.size(), keys.size() - 1);
        const std::string current_label = (pick_points.size() < keys.size())
            ? keys[idx]
            : std::string("done");
        cv::putText(img, std::to_string(pick_points.size()),
                    cv::Point(50, 200), cv::FONT_HERSHEY_SIMPLEX, 3,
                    cv::Scalar(0, 0, 255), 2);
        cv::putText(img, "Point: " + current_label, cv::Point(50, 320),
                    cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 255), 2);
        cv::putText(img, "Press 'n' to add good point", cv::Point(50, 480),
                    cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0, 0, 255), 2);
        if (pick_points.size() == real_points.size()) {
            solve();
            parser_->Change_Matrix();
        }
    } else {
        parser_->draw_ui(img);
        cv::putText(img, "Press Enter to Calibrate !!!", cv::Point(50, 200),
                    cv::FONT_HERSHEY_SIMPLEX, 3, cv::Scalar(0, 0, 255), 2);
    }
    auto temp = img.clone();
    cv::resize(img, temp, cv::Size(display_width_, display_height_));
    cv::imshow("calibrate", temp);
    auto key = cv::waitKey(10);
    switch (key) {
    case 13:
        is_calibrating = true;
        {
            const auto& keys = calibration_point_keys();
            std::cout << "[calibrate] click order:" << std::endl;
            for (size_t i = 0; i < keys.size(); ++i) {
                std::cout << "  " << i << ": " << keys[i] << std::endl;
            }
        }
        break;
    default:
        break;
    }
}

// 把 (cx, cy) 处自适应大小区域抠出来，CLAHE 增强暗部细节后放大到 400×400 画十字。
// 暗的高地标点（self_tower 等）在原图发黑看不清，CLAHE 后能清楚看到尖角。
static cv::Mat make_roi_view(const cv::Mat& full, int cx, int cy)
{
    const int half = std::max(1, roi_half_);
    int x0 = std::clamp(cx - half, 0, std::max(1, full.cols - 2 * half));
    int y0 = std::clamp(cy - half, 0, std::max(1, full.rows - 2 * half));
    cv::Mat roi = full(cv::Rect(x0, y0, 2 * half, 2 * half)).clone();
    // CLAHE 只作用在亮度通道，保留颜色
    cv::Mat ycrcb;
    cv::cvtColor(roi, ycrcb, cv::COLOR_BGR2YCrCb);
    std::vector<cv::Mat> ch(3);
    cv::split(ycrcb, ch);
    static auto clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(ch[0], ch[0]);
    cv::merge(ch, ycrcb);
    cv::cvtColor(ycrcb, roi, cv::COLOR_YCrCb2BGR);

    cv::Mat dst;
    cv::resize(roi, dst, cv::Size(400, 400));
    cv::line(dst, cv::Point(200, 100), cv::Point(200, 300),
             cv::Scalar(0, 0, 255), 1);
    cv::line(dst, cv::Point(100, 200), cv::Point(300, 200),
             cv::Scalar(0, 0, 255), 1);
    return dst;
}

static cv::Point display_to_original(int x, int y)
{
    const double sx = static_cast<double>(orig_width_) /
                      std::max(1, display_width_);
    const double sy = static_cast<double>(orig_height_) /
                      std::max(1, display_height_);
    int ox = static_cast<int>(std::lround(x * sx));
    int oy = static_cast<int>(std::lround(y * sy));
    const int hx = std::max(1, roi_half_);
    ox = std::max(hx, std::min(ox, orig_width_  - hx));
    oy = std::max(hx, std::min(oy, orig_height_ - hx));
    return {ox, oy};
}

void mousecallback(int event, int x, int y, int flags, void* userdata)
{
    int temp_key = 0;

    switch (event) {
    case cv::EVENT_LBUTTONDOWN:
        if (is_calibrating && !orig_image_.empty()) {
            cv::Point orig_pt = display_to_original(x, y);
            do {
                temp_key = cv::waitKey(10);
                switch (temp_key) {
                case 'w': orig_pt.y -= pick_step_; break;
                case 'a': orig_pt.x -= pick_step_; break;
                case 's': orig_pt.y += pick_step_; break;
                case 'd': orig_pt.x += pick_step_; break;
                }
                {
                    const int hx = std::max(1, roi_half_);
                    orig_pt.x = std::max(hx, std::min(orig_pt.x, orig_width_  - hx));
                    orig_pt.y = std::max(hx, std::min(orig_pt.y, orig_height_ - hx));
                }
                cv::imshow("ROI", make_roi_view(orig_image_, orig_pt.x, orig_pt.y));
            } while (temp_key != 'n');

            {
                const auto& keys = calibration_point_keys();
                const size_t idx = pick_points.size();
                const std::string label = (idx < keys.size())
                    ? keys[idx]
                    : std::to_string(idx);
                std::cout << label << " x:" << orig_pt.x
                          << " y:" << orig_pt.y << std::endl;
            }
            pick_points.push_back(cv::Point2f(orig_pt.x, orig_pt.y));
        }
        break;

    case cv::EVENT_MOUSEMOVE:
        if (orig_image_.empty() || x > cvimage_.cols - 1 ||
            y > cvimage_.rows - 1 || x < 0 || y < 0)
            break;
        {
            cv::Point orig_pt = display_to_original(x, y);
            cv::imshow("ROI", make_roi_view(orig_image_, orig_pt.x, orig_pt.y));
        }
        break;
    }
}

void Calibrate::solve()
{
    if (camera_matrix.empty() || dist_coeffs.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Cannot solve PnP: camera_matrix/dist_coeffs is empty");
        pick_points.clear();
        is_calibrating = false;
        return;
    }
    if (real_points.size() != pick_points.size() || real_points.size() < 4) {
        RCLCPP_ERROR(this->get_logger(),
                     "Cannot solve PnP: real_points=%zu pick_points=%zu",
                     real_points.size(), pick_points.size());
        pick_points.clear();
        is_calibrating = false;
        return;
    }

    const bool epnp_ok = cv::solvePnP(real_points, pick_points, camera_matrix,
                                      dist_coeffs, rvec, tvec, false,
                                      cv::SOLVEPNP_EPNP);
    if (!epnp_ok || rvec.empty() || tvec.empty()) {
        RCLCPP_ERROR(this->get_logger(), "solvePnP(EPNP) failed; out_matrix not written");
        pick_points.clear();
        is_calibrating = false;
        return;
    }

    const bool iterative_ok = cv::solvePnP(real_points, pick_points, camera_matrix,
                                           dist_coeffs, rvec, tvec, true,
                                           cv::SOLVEPNP_ITERATIVE);
    if (!iterative_ok || rvec.empty() || tvec.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "solvePnP(ITERATIVE refine) failed; out_matrix not written");
        pick_points.clear();
        is_calibrating = false;
        return;
    }
    std::cout << "rvec:" << rvec << std::endl;
    std::cout << "tvec:" << tvec << std::endl;

    std::vector<cv::Point2f> projected;
    cv::projectPoints(real_points, rvec, tvec, camera_matrix, dist_coeffs,
                      projected);
    double total_err = 0.0;
    for (size_t i = 0; i < projected.size() && i < pick_points.size(); ++i) {
        const double err = cv::norm(projected[i] - pick_points[i]);
        total_err += err;
        const auto& keys = calibration_point_keys();
        const std::string label = (i < keys.size()) ? keys[i] : std::to_string(i);
        std::cout << "reproj[" << i << ":" << label << "] clicked=" << pick_points[i]
                  << " projected=" << projected[i]
                  << " err=" << err << " px" << std::endl;
    }
    if (!projected.empty()) {
        std::cout << "mean reprojection error: "
                  << total_err / static_cast<double>(projected.size())
                  << " px" << std::endl;
    }

    cv::FileStorage fs;
    fs.open(out_matrix_path_, cv::FileStorage::WRITE);
    fs << "world_rvec" << rvec;
    fs << "world_tvec" << tvec;
    fs.release();

    // ── 把 (rvec, tvec) 反算成"相机在裁判系下的位姿"打印出来 ─────────────────
    // PnP 给出的是 (世界点 → 相机系) 的变换：P_cam = R * P_world + t
    // 所以相机原点在世界(内部)系下的位置 = -R^T * t
    cv::Mat R;
    cv::Rodrigues(rvec, R);                   // 3x3
    cv::Mat cam_pos = -R.t() * tvec;          // 3x1 in internal frame
    double cam_x = cam_pos.at<double>(0);
    double cam_y = cam_pos.at<double>(1);
    double cam_z = cam_pos.at<double>(2);
    if (points_in_referee_frame_) {
        // load_calibration_points 里把 referee 点的 y 减了 map_height，
        // 所以现在的内部系 y 比裁判系 y 小 map_height，要加回去
        cam_y += map_height_;
    }

    // 相机姿态（相机系 → 世界系）的旋转 = R^T
    cv::Mat Rcw = R.t();
    // ZYX 欧拉角分解（yaw 绕世界 z，pitch 绕新 y，roll 绕新 x）
    constexpr double R2D = 180.0 / M_PI;
    double pitch_deg = -std::asin(Rcw.at<double>(2, 0)) * R2D;
    double yaw_deg   =  std::atan2(Rcw.at<double>(1, 0), Rcw.at<double>(0, 0)) * R2D;
    double roll_deg  =  std::atan2(Rcw.at<double>(2, 1), Rcw.at<double>(2, 2)) * R2D;

    std::cout << std::fixed << std::setprecision(2)
              << "\n"
              << "================ Calibration Result ================\n"
              << "Team     : " << (team_label_.empty() ? "(unknown)" : team_label_) << "\n"
              << "Camera pose in REFEREE frame (单位: m / deg)\n"
              << "  position : x = " << cam_x << "  y = " << cam_y
              << "  z = " << cam_z << "\n"
              << "  rotation : yaw = " << yaw_deg
              << "   pitch = " << pitch_deg
              << "   roll = " << roll_deg << "\n"
              << "  saved to : " << out_matrix_path_ << "\n"
              << "====================================================\n"
              << std::defaultfloat << std::endl;

    pick_points.clear();
    is_calibrating = false;
}
}  // namespace tdt_radar

RCLCPP_COMPONENTS_REGISTER_NODE(tdt_radar::Calibrate);

#pragma once
#include <QWidget>
#include <QPixmap>
#include <QMutex>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <vision_interface/msg/detect_result.hpp>

// Real-time field map widget.
// Loads a static field topview as background and overlays robot positions
// received from /kalman_detect (vision_interface::msg::DetectResult).
//
// Coordinate convention (matches debug_map.cpp w2p):
//   world (x, y) in metres, origin at red corner
//   widget_x = x / FIELD_W * width()
//   widget_y = (FIELD_H - y) / FIELD_H * height()
class FieldMapWidget : public QWidget {
    Q_OBJECT
public:
    explicit FieldMapWidget(QWidget* parent = nullptr);

    // Call once the rclcpp node is ready (after rosNode_ is created).
    void initSubscription(rclcpp::Node::SharedPtr node);

    // Override the background image path (call before or after init).
    void loadBackground(const QString& path);

protected:
    void paintEvent(QPaintEvent*) override;
    void resizeEvent(QResizeEvent*) override;

private:
    static constexpr float FIELD_W = 28.0f;
    static constexpr float FIELD_H = 15.0f;

    QPointF toWidget(float x, float y) const;
    QRect   fieldRect() const;  // letterboxed draw area inside the widget

    QPixmap bg_;
    QPixmap bgScaled_;
    bool    bgDirty_ = true;

    struct Pos { float x = 0.0f, y = 0.0f; };
    std::array<Pos, 5> redPos_{};
    std::array<Pos, 5> bluePos_{};
    QMutex posMutex_;

    rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr sub_;

    static constexpr const char* kLabels[5] = {"H", "E", "3", "4", "S"};
};

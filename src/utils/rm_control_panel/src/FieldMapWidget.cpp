#include "FieldMapWidget.h"
#include <QPainter>
#include <QResizeEvent>
#include <QFontMetrics>

constexpr const char* FieldMapWidget::kLabels[5];

FieldMapWidget::FieldMapWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(280, 150);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setAttribute(Qt::WA_OpaquePaintEvent);
    loadBackground(QString("/home/zst/T/config/outputs/RMUC2026_topview_dense.png"));
}

void FieldMapWidget::initSubscription(rclcpp::Node::SharedPtr node)
{
    if (!node) return;
    sub_ = node->create_subscription<vision_interface::msg::DetectResult>(
        "/kalman_detect", rclcpp::QoS(10),
        [this](const vision_interface::msg::DetectResult::SharedPtr msg) {
            {
                QMutexLocker lk(&posMutex_);
                for (int i = 0; i < 5; ++i) {
                    redPos_[i]  = { msg->red_x[i],  msg->red_y[i]  };
                    bluePos_[i] = { msg->blue_x[i], msg->blue_y[i] };
                }
            }
            // Thread-safe repaint request to Qt main thread
            QMetaObject::invokeMethod(this, [this]{ update(); }, Qt::QueuedConnection);
        });
}

void FieldMapWidget::loadBackground(const QString& path)
{
    QPixmap px(path);
    if (!px.isNull()) {
        bg_      = px;
        bgDirty_ = true;
        update();
    }
}

void FieldMapWidget::resizeEvent(QResizeEvent* e)
{
    QWidget::resizeEvent(e);
    bgDirty_ = true;
}

QRect FieldMapWidget::fieldRect() const
{
    // Letterbox: keep field aspect (FIELD_W:FIELD_H = 28:15) inside the widget.
    const qreal aspect = static_cast<qreal>(FIELD_W) / FIELD_H;
    const qreal wW = width();
    const qreal wH = height();
    if (wW <= 0 || wH <= 0) return QRect();
    qreal fw = wW, fh = wW / aspect;
    if (fh > wH) { fh = wH; fw = wH * aspect; }
    const int x = static_cast<int>((wW - fw) * 0.5);
    const int y = static_cast<int>((wH - fh) * 0.5);
    return QRect(x, y, static_cast<int>(fw), static_cast<int>(fh));
}

QPointF FieldMapWidget::toWidget(float x, float y) const
{
    const QRect r = fieldRect();
    return {
        r.x() + static_cast<qreal>(x) / FIELD_W * r.width(),
        r.y() + (1.0 - static_cast<qreal>(y) / FIELD_H) * r.height()
    };
}

void FieldMapWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);

    // ── Background ────────────────────────────────────────────────────────────
    // Fill the widget margins (letterbox bars) with the panel bg color so the
    // map stays centered and keeps its 28:15 aspect ratio.
    p.fillRect(rect(), QColor(0x0d, 0x14, 0x1d));
    const QRect fr = fieldRect();
    if (!bg_.isNull()) {
        if (bgDirty_ || bgScaled_.size() != fr.size()) {
            bgScaled_ = bg_.scaled(fr.size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
            bgDirty_  = false;
        }
        p.drawPixmap(fr.topLeft(), bgScaled_);
    } else {
        p.setPen(QPen(QColor(40, 80, 100), 1));
        p.drawRect(fr.adjusted(0, 0, -1, -1));
    }

    // ── Semi-transparent darkening so dots stand out (only over the field) ───
    p.fillRect(fr, QColor(0, 0, 0, 55));

    // ── Field boundary lines ─────────────────────────────────────────────────
    p.setPen(QPen(QColor(180, 200, 220, 120), 1));
    QPointF tl = toWidget(0, FIELD_H), tr = toWidget(FIELD_W, FIELD_H);
    QPointF bl = toWidget(0, 0),       br = toWidget(FIELD_W, 0);
    p.drawLine(tl, tr); p.drawLine(tr, br);
    p.drawLine(br, bl); p.drawLine(bl, tl);
    // Midline
    p.drawLine(toWidget(FIELD_W * 0.5f, 0), toWidget(FIELD_W * 0.5f, FIELD_H));

    // ── Robot radius (scales with widget) ────────────────────────────────────
    const qreal r = std::max(5.0, std::min(13.0, fr.width() * 0.020));
    QFont font;
    font.setPixelSize(std::max(8, static_cast<int>(r) - 1));
    font.setBold(true);
    p.setFont(font);

    // ── Draw robots ──────────────────────────────────────────────────────────
    struct Team { const std::array<Pos, 5>* arr; QColor fill; QColor ring; };
    const Team teams[2] = {
        { &redPos_,  QColor(220, 50,  50),  QColor(255, 170, 170) },
        { &bluePos_, QColor(40,  110, 220), QColor(160, 200, 255) },
    };

    QMutexLocker lk(&posMutex_);
    for (const auto& t : teams) {
        for (int i = 0; i < 5; ++i) {
            const float x = (*t.arr)[i].x;
            const float y = (*t.arr)[i].y;
            // Validity: skip untracked (0,0) and out-of-bounds
            if (x < 0.1f && y < 0.1f) continue;
            if (x < -2.0f || x > FIELD_W + 2.0f) continue;
            if (y < -2.0f || y > FIELD_H + 2.0f) continue;

            const QPointF pt = toWidget(x, y);

            // Drop shadow
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(0, 0, 0, 110));
            p.drawEllipse(pt, r + 2.0, r + 2.0);

            // Filled circle
            p.setBrush(t.fill);
            p.setPen(QPen(t.ring, 1.5));
            p.drawEllipse(pt, r, r);

            // Label
            p.setPen(Qt::white);
            p.drawText(QRectF(pt.x() - r, pt.y() - r, r * 2, r * 2),
                       Qt::AlignCenter, kLabels[i]);
        }
    }

    // ── Legend (bottom-right corner) ─────────────────────────────────────────
    const qreal lr = 5.0;
    const qreal lx = fr.right()  - 52;
    const qreal ly = fr.bottom() - 28;
    p.setPen(Qt::NoPen); p.setBrush(QColor(0, 0, 0, 150));
    p.drawRoundedRect(QRectF(lx - 4, ly - 4, 52, 24), 3, 3);
    p.setBrush(QColor(220, 50,  50));  p.drawEllipse(QPointF(lx + lr,      ly + 8), lr, lr);
    p.setBrush(QColor(40,  110, 220)); p.drawEllipse(QPointF(lx + lr + 26, ly + 8), lr, lr);
    QFont lf; lf.setPixelSize(8); p.setFont(lf); p.setPen(Qt::white);
    p.drawText(QRectF(lx + lr * 2 + 1, ly + 2, 18, 12), Qt::AlignLeft | Qt::AlignVCenter, "Red");
    p.drawText(QRectF(lx + lr * 2 + 27, ly + 2, 18, 12), Qt::AlignLeft | Qt::AlignVCenter, "Blu");
}

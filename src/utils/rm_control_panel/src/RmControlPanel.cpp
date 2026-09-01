#include "RmControlPanel.h"
#include <QApplication>
#include <QCloseEvent>
#include <QDateTime>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFutureWatcher>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QSpinBox>
#include <QScrollBar>
#include <QSet>
#include <QSplitter>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QTextDocument>
#include <QRegularExpression>
#include <QtConcurrent/QtConcurrent>
#include <atomic>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <memory>
#include <signal.h>
#include <sys/file.h>
#include <unistd.h>

static const QString ROOT              = "/home/zst/T";
static const QString RUNTIME_PATH      = ROOT + "/config/radar_runtime.yaml";
static const QString SERIAL_PRECHECK   = ROOT + "/src/utils/rm_control_panel/scripts/serial_precheck.sh";
static const QString PRELAUNCH_CLEANUP = ROOT + "/src/utils/rm_control_panel/scripts/prelaunch_cleanup.sh";
static const QString BAG_DEFAULT_DIR   = ROOT + "/bags";

namespace {
constexpr int kLogFlushIntervalMs = 120;
constexpr int kMaxPendingLogs = 3000;
constexpr int kMaxLogBlocks = 3500;
constexpr int kLogFlushBatchSize = 180;

constexpr int kStopSigintWaitMs = 1200;
constexpr int kStopSigtermWaitMs = 1000;
constexpr int kStopSigkillWaitMs = 300;

std::atomic<int> gDroppedLogLines{0};

QString preferredSerialPort()
{
    const QStringList preferredPaths = {
        "/dev/gimbal",
    };
    for (const QString& path : preferredPaths) {
        if (QFileInfo::exists(path)) return path;
    }

    const QStringList serialDirs = {
        "/dev/serial/by-id",
        "/dev/serial/by-path",
    };
    for (const QString& dirPath : serialDirs) {
        QDir dir(dirPath);
        const QFileInfoList entries = dir.entryInfoList(
            QDir::System | QDir::Files | QDir::NoDotAndDotDot,
            QDir::Name
        );
        for (const QFileInfo& entry : entries) {
            if (entry.exists()) return entry.absoluteFilePath();
        }
    }

    QDir devDir("/dev");
    for (const QString& pattern : {QStringLiteral("ttyUSB*"), QStringLiteral("ttyACM*")}) {
        const QStringList entries = devDir.entryList({pattern}, QDir::System | QDir::Readable, QDir::Name);
        if (!entries.isEmpty()) return QString("/dev/%1").arg(entries.first());
    }

    return "/dev/ttyUSB0";
}

QList<pid_t> readProcessChildren(pid_t pid)
{
    QList<pid_t> children;
    QFile file(QString("/proc/%1/task/%1/children").arg(pid));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return children;

    const QString content = QString::fromUtf8(file.readAll()).trimmed();
    for (const QString& part : content.split(' ', Qt::SkipEmptyParts)) {
        bool ok = false;
        const qlonglong child = part.toLongLong(&ok);
        if (ok && child > 0) children.push_back(static_cast<pid_t>(child));
    }
    return children;
}

QList<pid_t> collectProcessTree(pid_t rootPid)
{
    QList<pid_t> ordered;
    if (rootPid <= 0) return ordered;

    QSet<pid_t> visited;
    std::function<void(pid_t)> dfs = [&](pid_t pid) {
        if (pid <= 0 || visited.contains(pid)) return;
        visited.insert(pid);
        const QList<pid_t> children = readProcessChildren(pid);
        for (pid_t child : children) dfs(child);
        ordered.push_back(pid);
    };
    dfs(rootPid);
    return ordered;
}

void signalProcessTree(pid_t rootPid, int signalNo)
{
    const QList<pid_t> pids = collectProcessTree(rootPid);
    for (pid_t pid : pids) {
        if (pid > 1) ::kill(pid, signalNo);
    }
}

void forceKillByPattern(const QStringList& patterns)
{
    for (const QString& pattern : patterns) {
        if (pattern.trimmed().isEmpty()) continue;
        QProcess::execute("bash", {"-lc", QString("pkill -9 -f \"%1\" >/dev/null 2>&1 || true").arg(pattern)});
    }
}

bool isNoisyRuntimeLine(const QString& line)
{
    const QString l = line.toLower();
    return l.contains("forward and decode_kernel_invoker time") ||
           l.contains("detect time:") ||
           l.contains("detecting...") ||
           l.contains("drop stale frame:") ||
           l.contains("throttle detect input:") ||
           l.contains("rtps_transport_shm error");
}

}  // namespace

// ── Constructor / Destructor ──────────────────────────────────────────────────
RmControlPanel::RmControlPanel(QWidget* parent) : QMainWindow(parent)
{
    setWindowTitle("RoboMaster Radar Console");
    resize(1480, 900);
    setMinimumSize(1280, 760);
    runtimeCfg_  = loadYaml(RUNTIME_PATH);
    loadMapProfiles();
    applyTheme();
    buildUI();
    loadToWidgets();
    doRefreshStatus();
    tickClock();
    clockTimer_ = new QTimer(this);
    connect(clockTimer_, &QTimer::timeout, this, &RmControlPanel::tickClock);
    clockTimer_->start(500);
    logFlushTimer_ = new QTimer(this);
    connect(logFlushTimer_, &QTimer::timeout, this, &RmControlPanel::flushLogs);
    logFlushTimer_->start(kLogFlushIntervalMs);
    // ── rclcpp 订阅线程 (实时收 /match_info, 无轮询延迟) ─────────────
    rosNode_ = rclcpp::Node::make_shared("rm_control_panel_monitor");
    if (fieldMap_) fieldMap_->initSubscription(rosNode_);
    subMatchInfo_ = rosNode_->create_subscription<vision_interface::msg::MatchInfo>(
        "/match_info", rclcpp::SensorDataQoS(),
        [this](const vision_interface::msg::MatchInfo::SharedPtr msg) {
            // 带副本将消息投递到 Qt 主线程, 线程安全
            QMetaObject::invokeMethod(this,
                [this, msg]() { updateBattleStatus(*msg); },
                Qt::QueuedConnection);
        });
    rosThread_ = std::thread([this]() { rclcpp::spin(rosNode_); });
    log("[SYSTEM] 控制台启动完成");
}
RmControlPanel::~RmControlPanel() {}

void RmControlPanel::closeEvent(QCloseEvent* e)
{
    if (rosNode_) rclcpp::shutdown();
    if (rosThread_.joinable()) rosThread_.join();
    stopProc(procMatch_,      "MATCH");
    stopProc(procReplay_,     "REPLAY");
    stopProc(procCalib_,      "CALIB");
    stopProc(procMapView_,    "MAP_VIEW");
    e->accept();
}

// ── Theme ─────────────────────────────────────────────────────────────────────
void RmControlPanel::applyTheme()
{
    qApp->setStyleSheet(R"(
QMainWindow,QWidget{background:#0a0f16;color:#dfe8f5;font-family:"DejaVu Sans";font-size:11pt;}
QGroupBox{background:#111a26;border:1px solid #2a3a4d;border-radius:4px;margin-top:10px;
  padding:12px;font-size:12pt;font-weight:bold;color:#99b5d1;}
QGroupBox::title{subcontrol-origin:margin;left:10px;padding:0 4px;}
QTabWidget::pane{background:#111a26;border:1px solid #2a3a4d;}
QTabBar::tab{background:#1a2432;color:#a9bdd4;padding:8px 14px;font-weight:bold;}
QTabBar::tab:selected{background:#243547;color:#e8f0fb;}
QPushButton{background:#1d2a39;color:#e6eef8;border:1px solid #344a63;
  border-radius:3px;padding:8px;font-weight:bold;}
QPushButton:hover{background:#26384b;color:#fff;}
QPushButton#primary{background:#255f45;color:#e8fff4;border-color:#347f5c;}
QPushButton#primary:hover{background:#2f7758;}
QPushButton#danger{background:#5a2d2d;color:#ffe2e2;border-color:#8f4a4a;}
QPushButton#danger:hover{background:#7a3d3d;}
QComboBox{background:#1a2432;color:#d3deeb;border:1px solid #2a3a4d;border-radius:3px;padding:4px 8px;}
QComboBox QAbstractItemView{background:#1a2432;color:#d3deeb;selection-background-color:#243547;}
QLineEdit{background:#1a2432;color:#d3deeb;border:1px solid #2a3a4d;border-radius:3px;padding:4px 8px;}
QTextEdit{background:#0d141d;color:#c6d8ea;border:none;font-family:"DejaVu Sans Mono";font-size:10pt;}
QCheckBox{color:#d3deeb;spacing:8px;}
QSplitter::handle{background:#2a3a4d;}
QScrollBar:vertical{background:#0d141d;width:8px;}
QScrollBar::handle:vertical{background:#2a3a4d;border-radius:4px;}
QProgressBar{background:#0d141d;border:1px solid #2a3a4d;border-radius:2px;height:12px;text-align:center;font-size:8pt;color:#c6d8ea;}
QProgressBar::chunk{border-radius:2px;}
    )");
}

// ── UI ────────────────────────────────────────────────────────────────────────
void RmControlPanel::buildUI()
{
    auto* central = new QWidget(this); setCentralWidget(central);
    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(14,14,14,14); root->setSpacing(8);

    auto* hdr = new QWidget; auto* hdrL = new QHBoxLayout(hdr); hdrL->setContentsMargins(0,0,0,0);
    auto* ttl = new QLabel("RoboMaster Radar Console");
    ttl->setStyleSheet("font-size:23pt;font-weight:bold;color:#dfe8f5;");
    clockLabel_ = new QLabel("--:--:--");
    clockLabel_->setStyleSheet("font-family:'DejaVu Sans Mono';font-size:11pt;color:#9fb4cb;");
    hdrL->addWidget(ttl); hdrL->addStretch(); hdrL->addWidget(clockLabel_);
    root->addWidget(hdr);

    auto* sub = new QLabel("赛前参数、地图切换、串口预检、一键开赛与回放。UI 仅负责控制，不参与算法主链计算。");
    sub->setStyleSheet("color:#7f93aa;"); root->addWidget(sub);

    auto* spl = new QSplitter(Qt::Horizontal); spl->setHandleWidth(4);
    root->addWidget(spl, 1);
    auto* lw = new QWidget; auto* ll = new QVBoxLayout(lw); ll->setContentsMargins(8,8,8,8);
    auto* rw = new QWidget; auto* rl = new QVBoxLayout(rw); rl->setContentsMargins(8,8,8,8);
    spl->addWidget(lw); spl->addWidget(rw);
    spl->setStretchFactor(0,3); spl->setStretchFactor(1,2);
    buildLeftPanel(lw); buildRightPanel(rw);
}

void RmControlPanel::buildLeftPanel(QWidget* p)
{
    auto* ml = qobject_cast<QVBoxLayout*>(p->layout());
    auto* sc = makeCard(p, "运行状态");
    auto* g  = new QGridLayout; g->setColumnStretch(1,1);
    addStatusRow(g,0,"比赛进程",  matchStatusLbl_);
    addStatusRow(g,1,"回放进程",  replayStatusLbl_);
    addStatusRow(g,2,"标定进程",  calibStatusLbl_);
    addStatusRow(g,3,"最近配置应用",lastApplyLbl_);
    lastApplyLbl_->setText("从未应用");
    static_cast<QVBoxLayout*>(sc->layout())->addLayout(g);
    ml->addWidget(sc);

    auto* nb = new QTabWidget;
    ml->addWidget(nb, 1);
    auto* t0=new QWidget; auto* t3=new QWidget; auto* t4=new QWidget;
    nb->addTab(t4,"战  况"); nb->addTab(t0,"赛前基础"); nb->addTab(t3,"开赛检查");
    buildBattleTab(t4); buildBasicTab(t0); buildChecklistTab(t3);

    // 地图档案 / 回放参数: 控件仍构建 (loadForm/collectForm 会用), 但不显示
    // 比赛中不需要改这些, 减少 UI 噪音; 如需调整, 直接改 yaml.
    auto* hiddenMap   = new QWidget; hiddenMap->hide();
    auto* hiddenReplay= new QWidget; hiddenReplay->hide();
    buildMapTab(hiddenMap);
    buildReplayTab(hiddenReplay);

    // ── 左下: 实时战场地图 (大幅可见, 填补底部空白) ─────────────────
    auto* fieldCard = makeCard(p, "实时战场地图  (/kalman_detect → /resolve_result_camera_only)");
    auto* fieldLayout = static_cast<QVBoxLayout*>(fieldCard->layout());
    fieldMap_ = new FieldMapWidget(p);
    fieldMap_->setMinimumHeight(380);   // 比之前 200 大近一倍
    fieldLayout->addWidget(fieldMap_, 1);
    ml->addWidget(fieldCard, 1);        // 拉伸占满剩余空间
}

void RmControlPanel::buildRightPanel(QWidget* p)
{
    auto* ml = qobject_cast<QVBoxLayout*>(p->layout());
    auto* ac = makeCard(p, "控制操作");
    auto* al = new QVBoxLayout;

    auto mkRow=[&](const QList<QPair<QString,QString>>& defs)->QWidget*{
        auto* row=new QWidget; auto* rl=new QHBoxLayout(row); rl->setContentsMargins(0,0,0,0);
        for(auto&[lbl,id]:defs){auto* b=new QPushButton(lbl);if(!id.isEmpty())b->setObjectName(id);
            b->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred);rl->addWidget(b);}
        return row;
    };
    auto* r1=mkRow({{"应用全部配置",""},{"串口预检",""}});
    auto* r2=mkRow({{"启动比赛 match.launch","primary"},{"停止比赛","danger"}});
    auto* r3=mkRow({{"启动回放 bag.launch",""},{"停止回放","danger"}});
    auto* r4=mkRow({{"启动实时标定",""},{"启动回放标定",""}});
    auto* r5=mkRow({{"停止标定","danger"},{"标定后回比赛","primary"}});
    auto* r6=mkRow({{"打开小地图窗口",""},{"关闭监控窗口","danger"}});
    auto* r7=mkRow({{"显示传统命令",""}});
    for(auto* r:{r1,r2,r3,r4,r5,r6,r7}) al->addWidget(r);
    auto* ht=new QLabel("你可以只用这个 UI 完成开赛流程：设置参数 -> 应用 -> 启动比赛。");
    ht->setStyleSheet("color:#86a0bb;font-size:9pt;"); al->addWidget(ht);
    static_cast<QVBoxLayout*>(ac->layout())->addLayout(al);
    ml->addWidget(ac);

    auto btns=[](QWidget* row){return row->findChildren<QPushButton*>();};
    connect(btns(r1)[0],&QPushButton::clicked,this,&RmControlPanel::onApplyAll);
    connect(btns(r1)[1],&QPushButton::clicked,this,&RmControlPanel::onSerialPrecheck);
    connect(btns(r2)[0],&QPushButton::clicked,this,&RmControlPanel::onStartMatch);
    connect(btns(r2)[1],&QPushButton::clicked,this,&RmControlPanel::onStopMatch);
    connect(btns(r3)[0],&QPushButton::clicked,this,&RmControlPanel::onStartReplay);
    connect(btns(r3)[1],&QPushButton::clicked,this,&RmControlPanel::onStopReplay);
    connect(btns(r4)[0],&QPushButton::clicked,this,&RmControlPanel::onStartCalibration);
    connect(btns(r4)[1],&QPushButton::clicked,this,&RmControlPanel::onStartCalibrationFromBag);
    connect(btns(r5)[0],&QPushButton::clicked,this,&RmControlPanel::onStopCalibration);
    connect(btns(r5)[1],&QPushButton::clicked,this,&RmControlPanel::onCalibrateAndMatch);
    connect(btns(r6)[0],&QPushButton::clicked,this,&RmControlPanel::onOpenMapWindow);
    connect(btns(r6)[1],&QPushButton::clicked,this,&RmControlPanel::onCloseMonitorWindows);
    connect(btns(r7)[0],&QPushButton::clicked,this,&RmControlPanel::onShowManualCommands);

    // 设备状态 (相机心跳)
    auto* mc=makeCard(p,"设备状态"); auto* mml=static_cast<QVBoxLayout*>(mc->layout());
    auto* camRow=new QWidget; auto* camHl=new QHBoxLayout(camRow); camHl->setContentsMargins(0,0,0,0);
    auto* camTl=new QLabel("相机"); camTl->setFixedWidth(48);
    camTl->setStyleSheet("color:#a9bdd4;");
    cameraStatusLbl_=new QLabel("(加载中...)"); cameraStatusLbl_->setAlignment(Qt::AlignCenter);
    cameraStatusLbl_->setMinimumHeight(40);
    cameraStatusLbl_->setStyleSheet("background:#0d141d;color:#8fa5bc;padding:6px;");
    camHl->addWidget(camTl); camHl->addWidget(cameraStatusLbl_,1); mml->addWidget(camRow);
    auto* bcam=new QPushButton("刷新相机心跳");
    connect(bcam,&QPushButton::clicked,this,&RmControlPanel::onRefreshCameraHeartbeat);
    mml->addWidget(bcam);
    ml->addWidget(mc);

    // Log card
    auto* lc=makeCard(p,"运行日志");
    logText_=new QTextEdit; logText_->setReadOnly(true); logText_->setMinimumHeight(200);
    logText_->setUndoRedoEnabled(false);
    logText_->document()->setMaximumBlockCount(kMaxLogBlocks);
    static_cast<QVBoxLayout*>(lc->layout())->addWidget(logText_);
    ml->addWidget(lc,1);
}

// ── Tab Builders ──────────────────────────────────────────────────────────────
void RmControlPanel::buildBasicTab(QWidget* p)
{
    auto* l=new QVBoxLayout(p); l->setContentsMargins(12,12,12,12);

    // ── 必要字段 (赛前 / 中场切换) ─────────────────────────────────────
    auto* g=new QGridLayout; g->setColumnStretch(1,1);
    teamCombo_       =addComboRow(g,0,"队伍",      {"1 蓝方","0 红方"});
    lidarEnableCombo_=addComboRow(g,1,"雷达链路",  {"1 开启","0 关闭"});
    mapDebugCombo_   =addComboRow(g,2,"地图调试",  {"0 关闭","1 开启"});
    serialModeCombo_ =addComboRow(g,3,"串口模式",  {"real 实串口","virtual 虚拟(/dev/null)"});
    serialPortEdit_  =addEntryRow(g,4,"串口");
    autoRecordCombo_ =addComboRow(g,5,"自动录包",  {"1 开启","0 关闭"});
    recordDirEdit_   =addEntryRow(g,6,"录包目录");
    recordPrefixEdit_=addEntryRow(g,7,"录包前缀");
    connect(serialModeCombo_, &QComboBox::currentTextChanged, this, [this](const QString& text) {
        if (valFromCombo(text) != "real") return;
        const QString current = serialPortEdit_->text().trimmed();
        if (current.isEmpty() || current == "/dev/null") {
            serialPortEdit_->setText(preferredSerialPort());
        }
    });

    // ── 已隐藏字段 (保留对象给 loadForm/collectForm 使用, UI 不显示) ────
    // 用户反馈: 不再分白天黑夜 / self_frame 永远全局系 / 波特率永远 115200
    cameraCombo_    = new QComboBox(p); cameraCombo_->addItems({"day","night"});
    cameraCombo_->setCurrentText("day"); cameraCombo_->hide();
    selfFrameCombo_ = new QComboBox(p); selfFrameCombo_->addItems({"1 己方系","0 全局系"});
    selfFrameCombo_->setCurrentText("0 全局系"); selfFrameCombo_->hide();
    serialBaudEdit_ = new QLineEdit(p); serialBaudEdit_->setText("115200"); serialBaudEdit_->hide();

    l->addLayout(g);

    // ── 相机参数实时调节面板 ───────────────────────────────────────────
    auto* camBox = new QGroupBox("相机参数 (实时生效, 无需重启)", p);
    auto* camLayout = new QGridLayout(camBox); camLayout->setColumnStretch(1,1);

    cameraExposureSpin_ = new QSpinBox(camBox);
    cameraExposureSpin_->setRange(100, 50000);
    cameraExposureSpin_->setSingleStep(500);
    cameraExposureSpin_->setSuffix(" μs");
    cameraExposureSpin_->setValue(5000);
    cameraExposureSpin_->setToolTip("曝光时间 (微秒). 暗场→调大, 强光→调小. 0 = 自动.");
    camLayout->addWidget(new QLabel("曝光"), 0, 0);
    camLayout->addWidget(cameraExposureSpin_, 0, 1);

    cameraGainSpin_ = new QDoubleSpinBox(camBox);
    cameraGainSpin_->setRange(0.0, 20.0);
    cameraGainSpin_->setSingleStep(0.5);
    cameraGainSpin_->setDecimals(1);
    cameraGainSpin_->setValue(10.0);
    cameraGainSpin_->setToolTip("数字增益 (dB). 噪声敏感→调小, 暗场→调大. 0 = 自动.");
    camLayout->addWidget(new QLabel("增益"), 1, 0);
    camLayout->addWidget(cameraGainSpin_, 1, 1);

    auto* camApplyBtn = new QPushButton("应用 (推送到 /camera_node)", camBox);
    camApplyBtn->setObjectName("primary");
    connect(camApplyBtn, &QPushButton::clicked, this, &RmControlPanel::onApplyCameraParams);
    camLayout->addWidget(camApplyBtn, 2, 0, 1, 2);

    cameraApplyStatus_ = new QLabel("(未应用)", camBox);
    cameraApplyStatus_->setStyleSheet("color:#86a0bb;font-size:9pt;");
    camLayout->addWidget(cameraApplyStatus_, 3, 0, 1, 2);

    l->addWidget(camBox);

    auto* hint = new QLabel(
        "比赛中调相机参数: 直接改上面滑块 → 应用. 立即生效, 不会重启检测.");
    hint->setWordWrap(true);
    hint->setStyleSheet("color:#86a0bb;font-size:9pt;");
    l->addWidget(hint);
    l->addStretch();
}

void RmControlPanel::buildMapTab(QWidget* p)
{
    auto* l=new QVBoxLayout(p); l->setContentsMargins(12,12,12,12);
    auto* g=new QGridLayout; g->setColumnStretch(1,1);
    profileCombo_ =addComboRow(g,0,"地图档案",  {});
    mapYamlEdit_  =addEntryRow(g,1,"map_yaml");
    mapImageEdit_ =addEntryRow(g,2,"map_image");
    mapPointsEdit_=addEntryRow(g,3,"map_points");
    mapPcdEdit_   =addEntryRow(g,4,"map_pcd");
    mapWidthEdit_ =addEntryRow(g,5,"地图宽");
    mapHeightEdit_=addEntryRow(g,6,"地图高");
    connect(profileCombo_,&QComboBox::currentTextChanged,this,&RmControlPanel::onProfileChanged);
    auto* ops=new QWidget; auto* oh=new QHBoxLayout(ops); oh->setContentsMargins(0,0,0,0);
    auto* ba=new QPushButton("应用档案到运行配置");
    auto* bs=new QPushButton("保存当前为档案");
    connect(ba,&QPushButton::clicked,this,&RmControlPanel::onApplyProfile);
    connect(bs,&QPushButton::clicked,this,&RmControlPanel::onSaveProfile);
    oh->addWidget(ba); oh->addWidget(bs);
    auto* pt=new QLabel("地图缩略预览（静态底图，不含实时点位）"); pt->setStyleSheet("color:#d3deeb;");
    mapPreviewLbl_=new QLabel("(预览加载中)");
    mapPreviewLbl_->setAlignment(Qt::AlignCenter);
    mapPreviewLbl_->setMinimumHeight(140);
    mapPreviewLbl_->setStyleSheet("background:#0d141d;color:#a9bdd4;");
    l->addLayout(g); l->addWidget(ops); l->addWidget(pt); l->addWidget(mapPreviewLbl_,1);
}

void RmControlPanel::buildReplayTab(QWidget* p)
{
    auto* l=new QVBoxLayout(p); l->setContentsMargins(12,12,12,12);
    auto* g=new QGridLayout; g->setColumnStretch(1,1);
    rosbagCombo_       =addComboRow(g,0,"rosbag路径",      {});
    showImageCombo_    =addComboRow(g,1,"显示画面",        {"0 关闭","1 开启"});
    useBagTimingCombo_ =addComboRow(g,2,"按时间戳",        {"1 开启","0 关闭"});
    replayRateEdit_    =addEntryRow(g,3,"回放倍率");
    maxSleepMsEdit_    =addEntryRow(g,4,"最大等待ms");
    decodeImageCombo_  =addComboRow(g,5,"解码图像",        {"1 开启","0 关闭"});
    publishCompCombo_  =addComboRow(g,6,"发布压缩图",      {"1 开启","0 关闭"});
    passthroughEdit_   =addEntryRow(g,7,"透传话题(逗号分隔)");
    auto* hint=new QLabel("建议比赛时不改此页。离线回放可在此平衡真实节奏与 CPU 占用。");
    hint->setStyleSheet("color:#86a0bb;font-size:9pt;");
    auto* bo=new QWidget; auto* boh=new QHBoxLayout(bo); boh->setContentsMargins(0,0,0,0);
    auto* br=new QPushButton("刷新包列表");
    auto* bb=new QPushButton("浏览文件/目录");
    auto* ba=new QPushButton("归档到中心目录");
    connect(br,&QPushButton::clicked,this,&RmControlPanel::onRefreshRosbagCandidates);
    connect(bb,&QPushButton::clicked,this,&RmControlPanel::onPickRosbagPath);
    connect(ba,&QPushButton::clicked,this,&RmControlPanel::onArchiveSelectedBag);
    boh->addWidget(br); boh->addWidget(bb); boh->addWidget(ba);
    l->addLayout(g); l->addWidget(bo); l->addWidget(hint); l->addStretch();
}

void RmControlPanel::buildBattleTab(QWidget* p)
{
    auto* l = new QVBoxLayout(p); l->setContentsMargins(10,10,10,10); l->setSpacing(6);

    // ── 比赛状态条 ──────────────────────────────────────────────────────
    auto* topRow = new QWidget; auto* tr = new QHBoxLayout(topRow); tr->setContentsMargins(0,0,0,0);
    gameTimeLbl_  = new QLabel("-- s");
    gameTimeLbl_->setStyleSheet("font-family:'DejaVu Sans Mono';font-size:18pt;font-weight:bold;color:#e8f0fb;");
    gameStateLbl_ = new QLabel("待机");
    gameStateLbl_->setStyleSheet("font-size:10pt;color:#86a0bb;padding-left:12px;");
    battlePollLbl_= new QLabel("未连接");
    battlePollLbl_->setStyleSheet("font-size:8pt;color:#5a7a9a;");
    tr->addWidget(gameTimeLbl_); tr->addWidget(gameStateLbl_); tr->addStretch();
    tr->addWidget(battlePollLbl_);
    l->addWidget(topRow);

    // ── HP 表 ────────────────────────────────────────────────────────────
    static const char* kNames[] = {"\u82f1\u96c4","\u5de5\u7a0b","\u6b65\u51753","\u6b65\u51754","\u6b65\u51755","\u54e8\u5175","\u524d\u54e8","\u57fa\u5730"};
    auto* hpGrid = new QWidget; auto* hg = new QGridLayout(hpGrid);
    hg->setContentsMargins(0,0,0,0); hg->setSpacing(3);
    hg->setColumnStretch(1,1); hg->setColumnStretch(4,1);

    auto* hdrSelf  = new QLabel("己方"); hdrSelf->setStyleSheet("color:#80ff88;font-weight:bold;");
    auto* hdrEnemy = new QLabel("敌方"); hdrEnemy->setStyleSheet("color:#ff8080;font-weight:bold;");
    hg->addWidget(hdrSelf,  0, 0, 1, 3);
    hg->addWidget(hdrEnemy, 0, 3, 1, 3);

    for (int i = 0; i < 8; ++i) {
        auto mkBar = [](QProgressBar*& bar, QLabel*& val, bool isSelf) {
            bar = new QProgressBar; bar->setRange(0,600); bar->setValue(600);
            bar->setTextVisible(false); bar->setFixedHeight(16);
            val = new QLabel("255");
            val->setFixedWidth(38); val->setStyleSheet("color:#c6d8ea;font-size:10pt;");
            const char* chunk = isSelf ? "#4caf6f" : "#cf5050";
            bar->setStyleSheet(QString("QProgressBar::chunk{background:%1;}").arg(chunk));
        };
        auto* nl = new QLabel(kNames[i]); nl->setFixedWidth(44);
        nl->setStyleSheet("color:#9fb4cb;font-size:10pt;");
        mkBar(selfHpBar_[i],  selfHpVal_[i],  true);
        mkBar(enemyHpBar_[i], enemyHpVal_[i], false);
        auto* sep = new QLabel("|"); sep->setFixedWidth(10);
        sep->setStyleSheet("color:#2a3a4d;");
        hg->addWidget(nl,             i+1, 0);
        hg->addWidget(selfHpBar_[i],  i+1, 1);
        hg->addWidget(selfHpVal_[i],  i+1, 2);
        hg->addWidget(sep,            i+1, 3, Qt::AlignCenter);
        hg->addWidget(enemyHpBar_[i], i+1, 4);
        hg->addWidget(enemyHpVal_[i], i+1, 5);
    }
    l->addWidget(hpGrid);

    // ── 标记 & 飞镟 ───────────────────────────────────────────────────────
    auto* botRow = new QWidget; auto* br = new QHBoxLayout(botRow); br->setContentsMargins(0,4,0,0);
    auto* markBox = new QGroupBox("标记状态"); auto* ml2 = new QHBoxLayout(markBox);
    ml2->setContentsMargins(6,12,6,6);
    static const char* kMarkNames[] = {"\u82f1\u96c4","\u5de5\u7a0b","\u51753","\u51754","\u54e8\u5175"};
    for (int i = 0; i < 5; ++i) {
        markLbl_[i] = new QLabel(kMarkNames[i]);
        markLbl_[i]->setAlignment(Qt::AlignCenter);
        markLbl_[i]->setFixedSize(44, 28);
        markLbl_[i]->setStyleSheet("background:#1a2432;color:#5a7a9a;border:1px solid #2a3a4d;border-radius:3px;font-size:10pt;");
        ml2->addWidget(markLbl_[i]);
    }
    auto* dartBox = new QGroupBox("\u98de\u955e");
    auto* dl = new QVBoxLayout(dartBox); dl->setContentsMargins(6,12,6,6);
    dartLbl_ = new QLabel("暂无数据"); dartLbl_->setStyleSheet("color:#86a0bb;font-size:10pt;");
    dl->addWidget(dartLbl_);
    br->addWidget(markBox,2); br->addWidget(dartBox,1);
    l->addWidget(botRow);

    auto* hint = new QLabel("注: HP已将原始 uint16 压缩至 0-255 (参考串口解析); 0=死亡, 255=满血或超过上限");
    hint->setStyleSheet("color:#5a7a9a;font-size:8pt;"); hint->setWordWrap(true);
    l->addWidget(hint);
    l->addStretch();
}

void RmControlPanel::updateBattleStatus(vision_interface::msg::MatchInfo msg)
{
    // 直接操作结构体字段, 无需解析字符串, 延迟 <50ms
    const int mtime     = static_cast<int>(msg.match_time);
    const int prog      = static_cast<int>(msg.game_progress);
    const int selfColor = static_cast<int>(msg.self_color);
    const int selfBase  = (selfColor == 2) ? 0 : 8;
    const int enemyBase = (selfColor == 2) ? 8 : 0;
    const int dartT     = static_cast<int>(msg.dart_remaining_time);
    const int dartSel   = static_cast<int>(msg.dart_selected_target);

    if (gameTimeLbl_)
        gameTimeLbl_->setText(mtime >= 0
            ? QString("%1 s").arg(mtime)
            : QString("%1 s倒计时").arg(-mtime));
    static const char* kProg[] = {"未开始","准备","自检","倒计时","比赛中","已结束"};
    if (gameStateLbl_)
        gameStateLbl_->setText(prog >= 0 && prog <= 5 ? kProg[prog] : "");

    auto setHp = [](QProgressBar* bar, QLabel* lbl, int v) {
        bar->setValue(v);
        lbl->setText(QString::number(v));
        const char* col = (v == 0) ? "#444" : (v < 150) ? "#cf5050" : (v < 300) ? "#d4a843" : "#4caf6f";
        bar->setStyleSheet(QString("QProgressBar::chunk{background:%1;}").arg(col));
    };
    for (int i = 0; i < 8; ++i) {
        if (selfBase  + i < 16 && selfHpBar_[i])
            setHp(selfHpBar_[i],  selfHpVal_[i],  msg.robot_hp[selfBase  + i]);
        if (enemyBase + i < 16 && enemyHpBar_[i])
            setHp(enemyHpBar_[i], enemyHpVal_[i], msg.robot_hp[enemyBase + i]);
    }
    for (int i = 0; i < 5; ++i) {
        if (!markLbl_[i]) continue;
        const bool active = (i < static_cast<int>(msg.marks.size()) && msg.marks[i] != 0);
        markLbl_[i]->setStyleSheet(active
            ? "background:#7a3d00;color:#ffc060;border:1px solid #c06000;border-radius:3px;font-size:8pt;"
            : "background:#1a2432;color:#5a7a9a;border:1px solid #2a3a4d;border-radius:3px;font-size:8pt;");
    }
    static const char* kDartTgt[] = {"无","前哨","基地","基地x2"};
    if (dartLbl_) dartLbl_->setText(
        QString("剩余: %1s | 目标: %2").arg(dartT)
            .arg(dartSel >= 0 && dartSel <= 3 ? kDartTgt[dartSel] : "未知"));
    if (battlePollLbl_) battlePollLbl_->setText(
        QDateTime::currentDateTime().toString("hh:mm:ss") + " 实时");
}

void RmControlPanel::buildChecklistTab(QWidget* p)
{
    auto* l=new QVBoxLayout(p); l->setContentsMargins(12,12,12,12); l->setSpacing(10);
    chkLidar_ =new QCheckBox("雷达点云频率正常（仅雷达模式必检，纯相机模式可忽略）");    chkLidar_->setChecked(true);
    chkSerial_=new QCheckBox("串口权限/占用已检查"); chkSerial_->setChecked(true);
    chkCalib_ =new QCheckBox("外参与标定文件已确认"); chkCalib_->setChecked(true);
    chkRecord_=new QCheckBox("录包开启且含图像");    chkRecord_->setChecked(true);
    chkTeam_  =new QCheckBox("队伍与相机档位正确");  chkTeam_->setChecked(true);
    for(auto* c:{chkLidar_,chkSerial_,chkCalib_,chkRecord_,chkTeam_}) l->addWidget(c);
    auto* bs=new QPushButton("检查通过后启动比赛"); bs->setObjectName("primary");
    connect(bs,&QPushButton::clicked,this,&RmControlPanel::onStartMatch);
    preflightLbl_=new QLabel("● 赛前自检: 待检测");
    preflightLbl_->setStyleSheet("background:#111a26;color:#d3deeb;padding-left:8px;");
    preflightLbl_->setMinimumSize(180,34);
    auto* bc=new QPushButton("一键赛前自检");
    connect(bc,&QPushButton::clicked,this,&RmControlPanel::onRunPreflightCheck);
    l->addWidget(bs); l->addWidget(preflightLbl_); l->addWidget(bc); l->addStretch();
}

// ── Widget Helpers ────────────────────────────────────────────────────────────
QGroupBox* RmControlPanel::makeCard(QWidget*, const QString& title)
{
    auto* c=new QGroupBox(title);
    auto* vl=new QVBoxLayout(c); vl->setContentsMargins(12,16,12,12);
    return c;
}
void RmControlPanel::addStatusRow(QGridLayout* g, int row, const QString& name, QLabel*& lbl)
{
    auto* nl=new QLabel(name); nl->setStyleSheet("color:#d3deeb;"); nl->setFixedWidth(120);
    lbl=new QLabel("--"); lbl->setStyleSheet("color:#d3deeb;");
    g->addWidget(nl,row,0,Qt::AlignLeft); g->addWidget(lbl,row,1,Qt::AlignLeft);
}
QComboBox* RmControlPanel::addComboRow(QGridLayout* g, int row, const QString& title, const QStringList& vals)
{
    auto* lbl=new QLabel(title); lbl->setStyleSheet("color:#d3deeb;"); lbl->setFixedWidth(130);
    auto* cb=new QComboBox; cb->addItems(vals);
    g->addWidget(lbl,row,0,Qt::AlignLeft); g->addWidget(cb,row,1);
    return cb;
}
QLineEdit* RmControlPanel::addEntryRow(QGridLayout* g, int row, const QString& title)
{
    auto* lbl=new QLabel(title); lbl->setStyleSheet("color:#d3deeb;"); lbl->setFixedWidth(130);
    auto* e=new QLineEdit;
    g->addWidget(lbl,row,0,Qt::AlignLeft); g->addWidget(e,row,1);
    return e;
}

// ── YAML I/O ──────────────────────────────────────────────────────────────────
YAML::Node RmControlPanel::loadYaml(const QString& path)
{
    if (!QFile::exists(path)) return YAML::Node();
    try { return YAML::LoadFile(path.toStdString()); } catch(...) { return YAML::Node(); }
}
void RmControlPanel::saveYaml(const QString& path, YAML::Node data)
{
    try {
        YAML::Emitter out; out << data;
        std::ofstream ofs(path.toStdString()); ofs << out.c_str() << "\n";
    } catch(const std::exception& e) { log(QString("[ERROR] saveYaml: %1").arg(e.what())); }
}
void RmControlPanel::loadMapProfiles()
{
    mapProfiles_.clear();
    auto m=runtimeCfg_["map"]; YAML::Node d;
    auto s=[&](const char* k,const char* def)->std::string{return m[k]?m[k].as<std::string>():def;};
    d["map_yaml"]  =s("map_yaml",  "config/map/map.yaml");
    d["map_image"] =s("map_image", "config/map/map.jpg");
    d["map_points"]=s("map_points","config/map/map_points.yaml");
    d["map_pcd"]   =s("map_pcd",   "config/map/map.pcd");
    d["width"]     =m["width"]  ?m["width"].as<double>() :28.0;
    d["height"]    =m["height"] ?m["height"].as<double>():15.0;
    mapProfiles_["rm_default"]=d;
}

// ── Widgets ↔ Config ──────────────────────────────────────────────────────────
void RmControlPanel::loadToWidgets()
{
    runtimeCfg_ = loadYaml(RUNTIME_PATH);
    YAML::Node pm = runtimeCfg_["pre_match"];
    teamCombo_->setCurrentText(pm["team"]&&pm["team"].as<int>()==1?"1 蓝方":"0 红方");
    cameraCombo_->setCurrentText(pm["camera"]?QString::fromStdString(pm["camera"].as<std::string>()):"day");
    selfFrameCombo_->setCurrentText(pm["self_frame"]&&pm["self_frame"].as<int>()==1?"1 己方系":"0 全局系");
    lidarEnableCombo_->setCurrentText(pm["lidar_enable"]&&pm["lidar_enable"].as<int>()==0?"0 关闭":"1 开启");
    mapDebugCombo_->setCurrentText(pm["enable_map_debug"]&&pm["enable_map_debug"].as<int>()==1?"1 开启":"0 关闭");
    serialPortEdit_->setText(pm["serial_port"]?QString::fromStdString(pm["serial_port"].as<std::string>()):preferredSerialPort());
    serialBaudEdit_->setText(pm["serial_baud"]?QString::number(pm["serial_baud"].as<int>()):"115200");
    const int serialEnable = pm["serial_enable"] ? pm["serial_enable"].as<int>() : 1;
    const QString serialPort = serialPortEdit_->text().trimmed();
    serialModeCombo_->setCurrentText((serialEnable == 0 || serialPort == "/dev/null") ? "virtual 虚拟(/dev/null)" : "real 实串口");
    autoRecordCombo_->setCurrentText(pm["enable_auto_record"]&&pm["enable_auto_record"].as<int>()==0?"0 关闭":"1 开启");
    recordDirEdit_->setText(pm["record_dir"]?QString::fromStdString(pm["record_dir"].as<std::string>()):"/home/zst/T/bags");
    recordPrefixEdit_->setText(pm["record_prefix"]?QString::fromStdString(pm["record_prefix"].as<std::string>()):"match");

    QStringList names=mapProfiles_.keys();
    profileCombo_->blockSignals(true);
    profileCombo_->clear(); profileCombo_->addItems(names);
    profileCombo_->blockSignals(false);
    if(!names.isEmpty()){ profileCombo_->setCurrentText(names.first()); fillProfileFields(mapProfiles_[names.first()]); }

    auto bag=runtimeCfg_["bag"];
    rosbagCombo_->setCurrentText(bag["rosbag_file"]?QString::fromStdString(bag["rosbag_file"].as<std::string>()):"bags/latest");
    showImageCombo_->setCurrentText(bag["show_image"]&&bag["show_image"].as<bool>()?"1 开启":"0 关闭");
    useBagTimingCombo_->setCurrentText(bag["use_bag_timing"]&&!bag["use_bag_timing"].as<bool>()?"0 关闭":"1 开启");
    replayRateEdit_->setText(bag["replay_rate"]?QString::number(bag["replay_rate"].as<double>()):"1.0");
    maxSleepMsEdit_->setText(bag["max_sleep_ms"]?QString::number(bag["max_sleep_ms"].as<int>()):"300");
    decodeImageCombo_->setCurrentText(bag["decode_compressed_image"]&&!bag["decode_compressed_image"].as<bool>()?"0 关闭":"1 开启");
    publishCompCombo_->setCurrentText(bag["publish_compressed_image"]&&!bag["publish_compressed_image"].as<bool>()?"0 关闭":"1 开启");
    if(bag["passthrough_topics"]){
        QStringList t; for(const auto& x:bag["passthrough_topics"]) t<<QString::fromStdString(x.as<std::string>());
        passthroughEdit_->setText(t.join(", "));
    } else passthroughEdit_->setText("/livox/imu, /tf, /tf_static");
    onRefreshRosbagCandidates();
    onUpdateMapPreview();
}

void RmControlPanel::fillProfileFields(const YAML::Node& prof)
{
    auto s=[&](const char* k,const char* d)->QString{return prof[k]?QString::fromStdString(prof[k].as<std::string>()):d;};
    mapYamlEdit_  ->setText(s("map_yaml",  "config/map/map.yaml"));
    mapImageEdit_ ->setText(s("map_image", "config/map/map.jpg"));
    mapPointsEdit_->setText(s("map_points","config/map/map_points.yaml"));
    mapPcdEdit_   ->setText(s("map_pcd",   "config/map/map.pcd"));
    mapWidthEdit_ ->setText(prof["width"] ?QString::number(prof["width"].as<double>()) :"28.0");
    mapHeightEdit_->setText(prof["height"]?QString::number(prof["height"].as<double>()):"15.0");
}

FormData RmControlPanel::collectForm()
{
    FormData f;
    f.team        =valFromCombo(teamCombo_->currentText()).toInt();
    f.camera      =cameraCombo_->currentText().trimmed();
    f.selfFrame   =valFromCombo(selfFrameCombo_->currentText()).toInt();
    f.lidarEnable =valFromCombo(lidarEnableCombo_->currentText()).toInt();
    f.mapDebug    =valFromCombo(mapDebugCombo_->currentText()).toInt();
    f.serialBaud  =serialBaudEdit_->text().trimmed().toInt();
    QString sm    =valFromCombo(serialModeCombo_->currentText());
    f.serialPort  =(sm=="virtual")?"/dev/null":serialPortEdit_->text().trimmed();
    f.serialEnable=(sm=="virtual")?0:1;
    f.autoRecord  =valFromCombo(autoRecordCombo_->currentText()).toInt();
    f.recordDir   =recordDirEdit_->text().trimmed();
    f.recordPrefix=recordPrefixEdit_->text().trimmed();
    f.mapYaml     =mapYamlEdit_->text().trimmed();
    f.mapImage    =mapImageEdit_->text().trimmed();
    f.mapPoints   =mapPointsEdit_->text().trimmed();
    f.mapPcd      =mapPcdEdit_->text().trimmed();

    bool ok = false;
    const double mapWidth = mapWidthEdit_->text().trimmed().toDouble(&ok);
    f.mapWidth = (ok && mapWidth > 0.1) ? mapWidth : 28.0;
    if (!ok || mapWidth <= 0.1) log("[CONFIG] map_width 非法，自动回退 28.0");

    ok = false;
    const double mapHeight = mapHeightEdit_->text().trimmed().toDouble(&ok);
    f.mapHeight = (ok && mapHeight > 0.1) ? mapHeight : 15.0;
    if (!ok || mapHeight <= 0.1) log("[CONFIG] map_height 非法，自动回退 15.0");

    f.useBagTiming=valFromCombo(useBagTimingCombo_->currentText()).toInt();
    f.showImage   =valFromCombo(showImageCombo_->currentText()).toInt();
    f.rosbagFile  =rosbagCombo_->currentText().trimmed();

    ok = false;
    const double replayRate = replayRateEdit_->text().trimmed().toDouble(&ok);
    f.replayRate = (ok && replayRate > 0.0) ? replayRate : 1.0;
    if (!ok || replayRate <= 0.0) log("[CONFIG] replay_rate 非法，自动回退 1.0");

    ok = false;
    const int maxSleepMs = maxSleepMsEdit_->text().trimmed().toInt(&ok);
    f.maxSleepMs = (ok && maxSleepMs >= 0) ? maxSleepMs : 300;
    if (!ok || maxSleepMs < 0) log("[CONFIG] max_sleep_ms 非法，自动回退 300");

    f.decodeImage =valFromCombo(decodeImageCombo_->currentText()).toInt();
    f.publishComp =valFromCombo(publishCompCombo_->currentText()).toInt();
    for(auto& s:passthroughEdit_->text().split(",")){ s=s.trimmed(); if(!s.isEmpty()) f.passthrough<<s; }
    return f;
}

// ── Apply ─────────────────────────────────────────────────────────────────────
void RmControlPanel::doApplyAll(bool skipGuard)
{
    if(!skipGuard&&actionInProgress_){log("[SYSTEM] 操作过快，上一条仍在处理");return;}
    if(!skipGuard) setActionBusy(true);
    try{
        FormData f=collectForm();
        runtimeCfg_=loadYaml(RUNTIME_PATH);
        runtimeCfg_["pre_match"]["team"]              =f.team;
        runtimeCfg_["pre_match"]["camera"]            =f.camera.toStdString();
        runtimeCfg_["pre_match"]["self_frame"]        =f.selfFrame;
        runtimeCfg_["pre_match"]["lidar_enable"]      =f.lidarEnable;
        runtimeCfg_["pre_match"]["serial_port"]       =f.serialPort.toStdString();
        runtimeCfg_["pre_match"]["serial_enable"]     =f.serialEnable;
        runtimeCfg_["pre_match"]["serial_baud"]       =f.serialBaud;
        runtimeCfg_["pre_match"]["enable_map_debug"]  =f.mapDebug;
        runtimeCfg_["pre_match"]["enable_auto_record"]=f.autoRecord;
        runtimeCfg_["pre_match"]["record_dir"]        =f.recordDir.toStdString();
        runtimeCfg_["pre_match"]["record_prefix"]     =f.recordPrefix.toStdString();
        YAML::Node topics;
        for(auto& t:QStringList{"/livox/lidar","/compressed_image","/match_info"}) topics.push_back(t.toStdString());
        runtimeCfg_["pre_match"]["record_topics"]=topics;
        saveYaml(RUNTIME_PATH,runtimeCfg_);
        doApplyProfile();
        if (f.serialEnable == 1) {
            if (f.lidarEnable == 1) {
                log("[CONFIG] 串口上报链路: 雷达主链(/radar2sentry)优先，纯相机(/resolve_result)为备用");
            } else {
                log("[CONFIG] 串口上报链路: 纯相机(/resolve_result)备用链已启用，可直接测试串口");
            }
        }
        lastApplyLbl_->setText(QDateTime::currentDateTime().toString("HH:mm:ss"));
        log("[CONFIG] 已同步到 radar_runtime.yaml");
    } catch(const std::exception& e){ QMessageBox::critical(this,"错误",QString("应用配置失败: %1").arg(e.what())); }
    if(!skipGuard) QTimer::singleShot(600,this,[this]{setActionBusy(false);});
}

void RmControlPanel::doApplyProfile()
{
    FormData f=collectForm();
    YAML::Node oldCfg = loadYaml(RUNTIME_PATH);
    YAML::Node newCfg;

    if (oldCfg["pre_match"]) newCfg["pre_match"] = oldCfg["pre_match"];
    if (oldCfg["calibration"]) newCfg["calibration"] = oldCfg["calibration"];
    if (oldCfg["runtime"]) newCfg["runtime"] = oldCfg["runtime"];
    if (oldCfg["serial"]) newCfg["serial"] = oldCfg["serial"];

    YAML::Node mapNode;
    mapNode["map_yaml"] = f.mapYaml.toStdString();
    mapNode["map_image"] = f.mapImage.toStdString();
    mapNode["map_points"] = f.mapPoints.toStdString();
    mapNode["map_pcd"] = f.mapPcd.toStdString();
    mapNode["width"] = f.mapWidth;
    mapNode["height"] = f.mapHeight;
    newCfg["map"] = mapNode;

    const YAML::Node oldBag = oldCfg["bag"];
    YAML::Node bagNode;
    bagNode["rosbag_file"] = (f.rosbagFile.isEmpty() ? QString("bags/latest") : f.rosbagFile).toStdString();
    bagNode["loop_playback"] = oldBag["loop_playback"] ? oldBag["loop_playback"].as<bool>() : true;
    bagNode["show_image"] = (bool)f.showImage;
    bagNode["decode_compressed_image"] = (bool)f.decodeImage;
    bagNode["publish_compressed_image"] = (bool)f.publishComp;
    bagNode["legacy_mode"] = oldBag["legacy_mode"] ? oldBag["legacy_mode"].as<bool>() : true;
    bagNode["legacy_cycle_ms"] = oldBag["legacy_cycle_ms"] ? oldBag["legacy_cycle_ms"].as<int>() : 100;
    const bool enableLidarPipeline = true;
    bagNode["enable_lidar_pipeline"] = enableLidarPipeline;
    bagNode["enable_serial"] = oldBag["enable_serial"] ? oldBag["enable_serial"].as<bool>() : false;
    bagNode["serial_dry_run"] = oldBag["serial_dry_run"] ? oldBag["serial_dry_run"].as<bool>() : true;
    bagNode["serial_use_resolve_fallback"] = false;
    bagNode["publish_match_info"] = false;
    newCfg["bag"] = bagNode;

    runtimeCfg_ = newCfg;
    saveYaml(RUNTIME_PATH, runtimeCfg_);
    log("[CONFIG] 已写入 radar_runtime.yaml (最小参数集)");
    log("[CONFIG] 回放默认使用激光雷达链(enable_lidar_pipeline=1)，关闭旧 bag /match_info 回放，关闭纯相机串口 fallback");
}

// ── Action Slots ──────────────────────────────────────────────────────────────
void RmControlPanel::onApplyAll()       { doApplyAll(false); }
void RmControlPanel::onCalibrateAndMatch(){ onStopCalibration(); onStartMatch(); }

void RmControlPanel::onApplyProfile()
{
    try{ doApplyProfile(); }
    catch(const std::exception& e){ QMessageBox::critical(this,"错误",QString("应用地图/回放参数失败: %1").arg(e.what())); }
}

void RmControlPanel::onSaveProfile()
{
    QString name=profileCombo_->currentText().trimmed();
    if(name.isEmpty()){QMessageBox::warning(this,"提示","请先填写档案名");return;}
    FormData f=collectForm();
    YAML::Node prof;
    prof["map_yaml"]  =f.mapYaml.toStdString();  prof["map_image"] =f.mapImage.toStdString();
    prof["map_points"]=f.mapPoints.toStdString(); prof["map_pcd"]   =f.mapPcd.toStdString();
    prof["width"]=f.mapWidth; prof["height"]=f.mapHeight;
    mapProfiles_[name]=prof;
    runtimeCfg_["map"]=prof;
    saveYaml(RUNTIME_PATH,runtimeCfg_);
    QStringList ns=mapProfiles_.keys();
    profileCombo_->blockSignals(true);
    profileCombo_->clear(); profileCombo_->addItems(ns); profileCombo_->setCurrentText(name);
    profileCombo_->blockSignals(false);
    log(QString("[PROFILE] 已保存地图档案: %1").arg(name));
}

void RmControlPanel::onProfileChanged(const QString& name)
{
    auto it=mapProfiles_.find(name);
    if(it!=mapProfiles_.end()){fillProfileFields(it.value());onUpdateMapPreview();log("[PROFILE] 切换: "+name);}
}

void RmControlPanel::onSerialPrecheck()
{
    if(!QFile::exists(SERIAL_PRECHECK)){log("[CHECK] 未找到 serial_precheck.sh");return;}
    runCmd("source /opt/ros/jazzy/setup.bash && bash "+SERIAL_PRECHECK,"CHECK");
    log("[CHECK] 已启动串口预检");
}

void RmControlPanel::onStartMatch()
{
    if(actionInProgress_){log("[MATCH] 上一条操作仍在处理");return;}
    if(procMatch_&&procMatch_->state()!=QProcess::NotRunning){log("[MATCH] 比赛进程已在运行");return;}

    if (procReplay_ && procReplay_->state() != QProcess::NotRunning) {
        log("[MATCH] 检测到回放进程，先停止回放以释放资源");
        onStopReplay();
    }
    if (procCalib_ && procCalib_->state() != QProcess::NotRunning) {
        log("[MATCH] 检测到标定进程，先停止标定以释放资源");
        onStopCalibration();
    }

    runPrelaunchCleanup("match");

    setActionBusy(true); doApplyAll(true);
    procMatch_=runCmd("source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
                      "ros2 launch /home/zst/T/src/tdt_vision/launch/match.launch.py","MATCH");
    log("[MATCH] 已启动 match.launch.py");
    QTimer::singleShot(600,this,[this]{setActionBusy(false);});
}
void RmControlPanel::onStopMatch()
{
    stopProc(procMatch_, "MATCH");
    onCloseMonitorWindows();
    doRefreshStatus();
}

void RmControlPanel::onStopReplay()
{
    stopProc(procReplay_, "REPLAY");
    onCloseMonitorWindows();
    doRefreshStatus();
}

void RmControlPanel::onStopCalibration()
{
    stopProc(procCalib_, "CALIB");
    doRefreshStatus();
}

void RmControlPanel::onStartReplay()
{
    if(actionInProgress_){log("[REPLAY] 上一条操作仍在处理");return;}
    if(procReplay_&&procReplay_->state()!=QProcess::NotRunning){log("[REPLAY] 回放进程已在运行");return;}

    if (procMatch_ && procMatch_->state() != QProcess::NotRunning) {
        log("[REPLAY] 检测到比赛进程，先停止比赛以释放资源");
        onStopMatch();
    }
    if (procCalib_ && procCalib_->state() != QProcess::NotRunning) {
        log("[REPLAY] 检测到标定进程，先停止标定以释放资源");
        onStopCalibration();
    }

    runPrelaunchCleanup("replay");

    setActionBusy(true); doApplyAll(true);
    FormData f=collectForm();
    const QString showImage = (f.showImage == 1) ? "true" : "false";
    const QString cmd =
        "source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
        "ros2 launch tdt_vision bag.launch.py show_image:=" + showImage;
    procReplay_=runCmd(cmd, "REPLAY");
    log("[REPLAY] 已启动 bag.launch.py");
    QTimer::singleShot(600,this,[this]{setActionBusy(false);});
}

void RmControlPanel::onStartCalibration()
{
    if(procCalib_&&procCalib_->state()!=QProcess::NotRunning){log("[CALIB] 标定进程已在运行");return;}

    if (procMatch_ && procMatch_->state() != QProcess::NotRunning) {
        log("[CALIB] 检测到比赛进程，先停止比赛以释放资源");
        onStopMatch();
    }
    if (procReplay_ && procReplay_->state() != QProcess::NotRunning) {
        log("[CALIB] 检测到回放进程，先停止回放以释放资源");
        onStopReplay();
    }

    runPrelaunchCleanup("calib");

    doApplyAll(true);
    procCalib_=runCmd("source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
                      "ros2 launch tdt_vision calib.launch.py","CALIB");
    log("[CALIB] 已启动实时标定 calib.launch.py");
}

void RmControlPanel::onStartCalibrationFromBag()
{
    if(procCalib_&&procCalib_->state()!=QProcess::NotRunning){log("[CALIB] 标定进程已在运行");return;}

    if (procMatch_ && procMatch_->state() != QProcess::NotRunning) {
        log("[CALIB] 检测到比赛进程，先停止比赛以释放资源");
        onStopMatch();
    }
    if (procReplay_ && procReplay_->state() != QProcess::NotRunning) {
        log("[CALIB] 检测到回放进程，先停止回放以释放资源");
        onStopReplay();
    }

    runPrelaunchCleanup("calib");

    doApplyAll(true);
    procCalib_=runCmd("source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
                      "ros2 launch tdt_vision calib_bag.launch.py","CALIB");
    log("[CALIB] 已启动回放标定 calib_bag.launch.py");
}

void RmControlPanel::onOpenMapWindow()
{
    if (procMapView_) {
        if (procMapView_->state() != QProcess::NotRunning) {
            // QProcess may still report running when launch state is stale; verify with system process list.
            const int alive = QProcess::execute(
                "bash",
                {"-lc", "pgrep -f \"debug_map map.launch.py|install/debug_map/lib/.*/debug_map\" >/dev/null 2>&1"});
            if (alive == 0) {
                log("[VIEW] 小地图窗口已在运行");
                return;
            }
            log("[VIEW] 检测到小地图状态残留，已重置后重新启动");
        }
        procMapView_->deleteLater();
        procMapView_ = nullptr;
    }
    FormData f=collectForm();
    QString sc=(f.team==1)?"0":"2";
    // debug_map consumes /kalman_detect /resolve_result in global frame.
    // Force global-frame display to avoid double mirroring (blue/replay appears flipped).
    QString isf="false";
    procMapView_=runCmd("source /opt/ros/jazzy/setup.bash && source install/setup.bash && "
                        "ros2 launch debug_map map.launch.py input_is_self_frame:="+isf+" self_color_override:="+sc,"MAP_VIEW");
    tryPlaceWindow({"rviz", "map", "debug"}, 960, 620, 920, 420);
    log("[VIEW] 已打开小地图调试窗口（按全局坐标显示，避免红蓝镜像）");
}

void RmControlPanel::onCloseMonitorWindows()
{
    stopProc(procMapView_,"MAP_VIEW");
}

void RmControlPanel::onShowManualCommands()
{
    const QString serialPort = preferredSerialPort();
    log("[MANUAL] 传统流程命令如下（UI 异常时直接用）：");
    log("[MANUAL] source /opt/ros/jazzy/setup.bash && source /home/zst/T/install/setup.bash");
    log(QString("[MANUAL] # 推荐稳定串口路径: %1").arg(serialPort));
    log("[MANUAL] # 激光雷达链路（与原项目一致，分开启动）");
    log("[MANUAL] ros2 launch livox_ros2_driver livox_lidar_launch.py");
    log("[MANUAL] ros2 launch dynamic_cloud lidar.launch.py input_is_self_frame:=false self_color_override:=0");
    log(QString("[MANUAL] ros2 launch /home/zst/T/src/tdt_vision/launch/match.launch.py team:=1 camera:=day self_frame:=1 port:=%1 baud:=115200 map:=1").arg(serialPort));
    log("[MANUAL] ros2 launch debug_map map.launch.py input_is_self_frame:=false self_color_override:=0");
    log("[MANUAL] ros2 launch tdt_vision calib.launch.py");
    log("[MANUAL] ros2 launch tdt_vision calib_bag.launch.py");
    log("[MANUAL] ros2 launch tdt_vision bag.launch.py show_image:=false");
    log("[MANUAL] ros2 launch tdt_vision bag.launch.py show_image:=false enable_lidar_pipeline:=true enable_serial:=true serial_dry_run:=true");
    log("[MANUAL] # 纯相机串口测试（自动走 /resolve_result 备用链）");
    log(QString("[MANUAL] ros2 launch /home/zst/T/src/tdt_vision/launch/match.launch.py lidar_enable:=0 serial_enable:=1 port:=%1 baud:=115200").arg(serialPort));
    log("[MANUAL] # 比赛雷达链路（主链 /radar2sentry）");
    log(QString("[MANUAL] ros2 launch /home/zst/T/src/tdt_vision/launch/match.launch.py lidar_enable:=1 serial_enable:=1 port:=%1 baud:=115200").arg(serialPort));
}

void RmControlPanel::onRefreshRosbagCandidates()
{
    QStringList cands;
    auto bag=runtimeCfg_["bag"];
    QString def=bag["rosbag_file"]?QString::fromStdString(bag["rosbag_file"].as<std::string>()):"bags/latest";
    if(!def.isEmpty()) cands<<def;
    QDir bd(BAG_DEFAULT_DIR);
    if(bd.exists()){
        QDirIterator it(BAG_DEFAULT_DIR,{"metadata.yaml"},QDir::Files,QDirIterator::Subdirectories);
        while(it.hasNext()){it.next();cands<<QFileInfo(it.filePath()).absolutePath();}
        QDirIterator it2(BAG_DEFAULT_DIR,{"*.db3"},QDir::Files,QDirIterator::Subdirectories);
        while(it2.hasNext()){it2.next();cands<<it2.filePath();}
    }
    QStringList dedup; QSet<QString> seen;
    for(auto& c:cands) if(!seen.contains(c)){seen.insert(c);dedup<<c;}
    rosbagCandidates_=dedup.mid(0,200);
    QString cur=rosbagCombo_->currentText();
    rosbagCombo_->blockSignals(true);
    rosbagCombo_->clear(); rosbagCombo_->addItems(rosbagCandidates_);
    rosbagCombo_->setCurrentText(!cur.isEmpty()?cur:(!rosbagCandidates_.isEmpty()?rosbagCandidates_.first():""));
    rosbagCombo_->blockSignals(false);
}

void RmControlPanel::onPickRosbagPath()
{
    QString c=QFileDialog::getOpenFileName(this,"选择 rosbag (mcap 目录或 .db3 文件)",
        QDir(BAG_DEFAULT_DIR).exists()?BAG_DEFAULT_DIR:ROOT,"ROS bag (metadata.yaml *.db3);;mcap dir (metadata.yaml);;sqlite (*.db3);;All files (*)");
    if(c.isEmpty()) c=QFileDialog::getExistingDirectory(this,"或选择 rosbag 目录",
        QDir(BAG_DEFAULT_DIR).exists()?BAG_DEFAULT_DIR:ROOT);
    if(!c.isEmpty()){rosbagCombo_->setCurrentText(c);log("[REPLAY] 已选择 rosbag: "+c);}
}

void RmControlPanel::onArchiveSelectedBag()
{
    QString src=rosbagCombo_->currentText().trimmed();
    if(src.isEmpty()){log("[REPLAY] 未选择 rosbag，无法归档");return;}
    QDir(ROOT).mkpath("bags");
    QString base=QFileInfo(src.endsWith("/")?src.chopped(1):src).fileName();
    if(base.isEmpty()) base="rosbag";
    QString link=BAG_DEFAULT_DIR+"/"+QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")+"_"+base;
    QFile::remove(link);
    if(QFile::link(src,link)){log("[REPLAY] 已归档(软链): "+link+" -> "+src);onRefreshRosbagCandidates();}
    else log("[REPLAY] 归档失败");
}

void RmControlPanel::onApplyCameraParams()
{
    if (!cameraExposureSpin_ || !cameraGainSpin_ || !rosNode_) {
        if (cameraApplyStatus_) cameraApplyStatus_->setText("初始化未完成");
        return;
    }
    const int    exposure = cameraExposureSpin_->value();
    const double gain     = cameraGainSpin_->value();

    cameraApplyStatus_->setStyleSheet("color:#d8c060;font-size:9pt;");
    cameraApplyStatus_->setText(QString("应用中... exposure=%1μs gain=%2").arg(exposure).arg(gain, 0, 'f', 1));

    // 异步推送到 /camera_node, 避免阻塞 Qt 主线程
    auto* w = new QFutureWatcher<QString>(this);
    connect(w, &QFutureWatcher<QString>::finished, this, [this, w, exposure, gain]{
        const QString r = w->result();
        if (r == "ok") {
            cameraApplyStatus_->setStyleSheet("color:#4cbf72;font-size:9pt;");
            cameraApplyStatus_->setText(
                QString("✓ 已生效  exposure=%1μs  gain=%2").arg(exposure).arg(gain, 0, 'f', 1));
            log(QString("[CAM] 应用参数: exposure=%1us gain=%2").arg(exposure).arg(gain, 0, 'f', 1));
        } else {
            cameraApplyStatus_->setStyleSheet("color:#d66f6f;font-size:9pt;");
            cameraApplyStatus_->setText("✗ 失败: " + r);
            log("[CAM] 应用失败: " + r);
        }
        w->deleteLater();
    });
    w->setFuture(QtConcurrent::run([exposure, gain]() -> QString {
        // 直接用 ROS2 同步 parameters client. /camera_node 已实现 on_set_param
        // 动态回调, 设置后 backend 实时生效, 不重启采集线程.
        // 注意: 必须新建一个临时节点, 不能复用主监听节点 rosNode_.
        // SyncParametersClient 内部会构造 SingleThreadedExecutor 并 add_node,
        // 而 rosNode_ 已经被 rosThread_ 中的 rclcpp::spin() 占用,
        // 一个节点不能同时属于两个 executor, 会触发
        // "failed to create guard condition: context is not valid" 报错.
        try {
            if (!rclcpp::ok()) return "rclcpp 上下文未初始化";
            auto tmpNode = rclcpp::Node::make_shared(
                "rm_control_panel_param_client_" +
                std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
            auto client = std::make_shared<rclcpp::SyncParametersClient>(
                tmpNode, "camera_node");
            if (!client->wait_for_service(std::chrono::seconds(2))) {
                return "/camera_node 不在线 (相机节点未启动?)";
            }
            std::vector<rclcpp::Parameter> params = {
                rclcpp::Parameter("hik.exposure_time", exposure),
                rclcpp::Parameter("hik.gain",          gain),
            };
            auto results = client->set_parameters(params, std::chrono::seconds(2));
            for (size_t i = 0; i < results.size(); ++i) {
                if (!results[i].successful) {
                    return QString::fromStdString(
                        "param " + std::to_string(i) + ": " + results[i].reason);
                }
            }
            return "ok";
        } catch (const std::exception& e) {
            return QString("exception: ") + e.what();
        }
    }));
}

void RmControlPanel::onRefreshCameraHeartbeat()
{
    auto* w=new QFutureWatcher<QString>(this);
    connect(w,&QFutureWatcher<QString>::finished,this,[this,w]{
        QString r=w->result();
        if(r=="ok"){
            cameraStatusLbl_->setStyleSheet("background:#0d141d;color:#4cbf72;padding:8px;");
            cameraStatusLbl_->setText("● 在线  话题: /camera_image\n已收到图像消息");
            log("[CHECK] 相机状态正常");
        } else{
            cameraStatusLbl_->setStyleSheet("background:#0d141d;color:#d66f6f;padding:8px;");
            cameraStatusLbl_->setText("● 异常  话题: /camera_image\n"+r);
            log("[CHECK] 相机状态异常: "+r);
        }
        w->deleteLater();
    });
    w->setFuture(QtConcurrent::run([]{
        QProcess p;
        p.start("bash",{"-lc","source /opt/ros/jazzy/setup.bash && source /home/zst/T/install/setup.bash && "
                               "timeout 3s ros2 topic echo /camera_image --field header --once"});
        p.waitForFinished(5000);
        QString o=p.readAllStandardOutput()+p.readAllStandardError();
        if (p.exitCode()==0 && o.contains("stamp")) return QString("ok");
        if (o.contains("Could not determine the type")) return QString("未检测到 /camera_image 发布，请先启动比赛或回放");
        if (o.contains("does not appear to be published yet")) return QString("相机话题暂未发布");
        return o.trimmed().left(120);
    }));
}

void RmControlPanel::onUpdateMapPreview()
{
    QString mp=resolvePath(mapImageEdit_->text());
    if(mp.isEmpty()||!QFile::exists(mp)){
        mapPreviewLbl_->setPixmap(QPixmap());
        mapPreviewLbl_->setText("(未找到 map_image)");
        return;
    }
    QPixmap pix(mp);
    if(pix.isNull()){
        mapPreviewLbl_->setPixmap(QPixmap());
        mapPreviewLbl_->setText("(图片读取失败)");
        return;
    }
    mapPreviewPix_=pix.scaledToWidth(520,Qt::SmoothTransformation);
    mapPreviewLbl_->setPixmap(mapPreviewPix_);
    mapPreviewLbl_->setText("");
    if (fieldMap_) fieldMap_->loadBackground(mp);
}

void RmControlPanel::onRunPreflightCheck()
{
    if(preflightLbl_) preflightLbl_->setText("● 赛前自检: 检测中");
    runtimeCfg_=loadYaml(RUNTIME_PATH);
    YAML::Node rt=runtimeCfg_; FormData f=collectForm();
    auto* w=new QFutureWatcher<QStringList>(this);
    connect(w,&QFutureWatcher<QStringList>::finished,this,[this,w]{
        QStringList notes=w->result();
        bool hasErr =notes.filter(QRegularExpression("^ERR")).size()>0;
        bool hasWarn=notes.filter(QRegularExpression("^WARN")).size()>0;
        QString color=hasErr?"#d66f6f":hasWarn?"#d1a34a":"#4cbf72";
        QString txt  =hasErr?"失败":hasWarn?"警告":"通过";
        if(preflightLbl_){preflightLbl_->setStyleSheet("background:#111a26;color:"+color+";padding-left:8px;");
            preflightLbl_->setText("● 赛前自检: "+txt);}
        for(auto& n:notes) log("[PRECHECK] "+n);
        if(notes.isEmpty()) log("[PRECHECK] 赛前自检通过");
        w->deleteLater();
    });
    w->setFuture(QtConcurrent::run([rt,f]()->QStringList{
        QStringList notes;
        if (f.lidarEnable == 1) {
            QProcess p;
            p.start("bash", {"-lc",
                "source /opt/ros/jazzy/setup.bash && source /home/zst/T/install/setup.bash && "
                "timeout 4s ros2 topic hz /livox/lidar"});
            p.waitForFinished(6000);
            QString o = p.readAllStandardOutput();
            double rate = -1;
            for (auto& line : o.split("\n")) {
                if (line.contains("average rate:")) {
                    bool ok;
                    double r = line.split(":").last().trimmed().split(" ").first().toDouble(&ok);
                    if (ok) rate = r;
                }
            }
            if (rate < 0) notes << "ERR: /livox/lidar 无数据";
            else if (rate < 5.0) notes << QString("WARN: /livox/lidar 频率偏低(%1Hz)").arg(rate, 0, 'f', 1);
        } else {
            notes << "INFO: 当前雷达链路关闭，串口将使用纯相机 /resolve_result 备用链";
        }
        {QProcess p; p.start("bash",{"-lc",
             "source /opt/ros/jazzy/setup.bash && source /home/zst/T/install/setup.bash && "
             "timeout 4s ros2 topic hz /match_info"});
         p.waitForFinished(6000); QString o=p.readAllStandardOutput(); double rate=-1;
         for(auto& line:o.split("\n")) if(line.contains("average rate:")){
             bool ok; double r=line.split(":").last().trimmed().split(" ").first().toDouble(&ok); if(ok) rate=r;}
         if(rate<0) notes<<"WARN: /match_info 暂无数据";}
        if(f.serialEnable==0) notes<<"WARN: 串口为虚拟模式";
        auto res=[](const QString& p)->QString{
            if(p.isEmpty()||QFileInfo(p).isAbsolute()) return p; return "/home/zst/T/"+p;};
        auto cal=rt["calibration"];
        QString cp=cal["camera_params"]?QString::fromStdString(cal["camera_params"].as<std::string>()):"config/camera_params.yaml";
        QString om=cal["out_matrix"]   ?QString::fromStdString(cal["out_matrix"].as<std::string>())   :"config/out_matrix.yaml";
        if(!QFile::exists(res(cp))) notes<<"ERR: camera_params 文件缺失";
        if(!QFile::exists(res(om))) notes<<"ERR: out_matrix 文件缺失";
        return notes;
    }));
}

// ── Process Management ────────────────────────────────────────────────────────
void RmControlPanel::runPrelaunchCleanup(const QString& mode)
{
    if (!QFile::exists(PRELAUNCH_CLEANUP)) {
        log("[CLEANUP] 脚本缺失，跳过启动前清理");
        return;
    }

    QProcess cleanup;
    cleanup.setWorkingDirectory(ROOT);
    cleanup.setProcessChannelMode(QProcess::MergedChannels);
    cleanup.start("bash", {"-lc", "bash " + PRELAUNCH_CLEANUP + " " + mode});
    if (!cleanup.waitForFinished(8000)) {
        cleanup.kill();
        log("[CLEANUP] 启动前清理超时，继续启动");
        return;
    }

    const QString output = QString::fromUtf8(cleanup.readAll()).trimmed();
    for (const QString& line : output.split('\n', Qt::SkipEmptyParts)) {
        log("[CLEANUP] " + line);
    }
}

QProcess* RmControlPanel::runCmd(const QString& cmd, const QString& tag)
{
    auto* proc=new QProcess(this);
    proc->setWorkingDirectory(ROOT);
    proc->setProcessChannelMode(QProcess::MergedChannels);
    connect(proc,&QProcess::readyReadStandardOutput,this,[this,proc,tag]{
        while(proc->canReadLine()){
            QString line=QString::fromUtf8(proc->readLine()).trimmed();
            if (line.size() > 1400) line = line.left(1400) + "...(truncated)";
            if ((tag == "MATCH" || tag == "REPLAY") && isNoisyRuntimeLine(line)) continue;
            if(!line.isEmpty()) log(QString("[%1] %2").arg(tag,line));
        }
    });
    connect(proc,QOverload<int,QProcess::ExitStatus>::of(&QProcess::finished),
            this,[this,proc,tag](int code,QProcess::ExitStatus){
        log(QString("[%1] 进程退出 code=%2").arg(tag).arg(code));
        if(tag=="MATCH"   && procMatch_==proc) procMatch_=nullptr;
        if(tag=="REPLAY"  && procReplay_==proc) procReplay_=nullptr;
        if(tag=="CALIB"   && procCalib_==proc) procCalib_=nullptr;
        if(tag=="MAP_VIEW"&& procMapView_==proc) procMapView_=nullptr;
        doRefreshStatus();
        proc->deleteLater();
    });
    proc->start("bash",{"-lc",cmd});
    return proc;
}

void RmControlPanel::stopProc(QProcess*& proc, const QString& tag)
{
    QProcess* target = proc;
    if (!target || target->state() == QProcess::NotRunning) {
        log(QString("[%1] 进程未运行").arg(tag));
        return;
    }

    const qint64 rootPid = target->processId();
    if (rootPid > 0) {
        log(QString("[%1] 停止中: pid=%2").arg(tag).arg(rootPid));
        signalProcessTree(static_cast<pid_t>(rootPid), SIGINT);
    } else {
        target->terminate();
    }

    if (target->state() != QProcess::NotRunning &&
        !target->waitForFinished(kStopSigintWaitMs)) {
        log(QString("[%1] SIGINT 超时，升级 SIGTERM").arg(tag));
        target->terminate();
    }

    if (target->state() != QProcess::NotRunning &&
        !target->waitForFinished(kStopSigtermWaitMs)) {
        log(QString("[%1] SIGTERM 超时，升级 SIGKILL").arg(tag));
        target->kill();
    }

    if (target->state() != QProcess::NotRunning &&
        !target->waitForFinished(kStopSigkillWaitMs)) {
        target->kill();
        target->waitForFinished(500);
    }

    // Replay/match can leave orphan windows in stress conditions; force a scoped cleanup.
    if (tag == "REPLAY") {
        forceKillByPattern({
            "bag.launch.py",
            "rosbag_image_view",
            "component_container --ros-args -r __node:=camera_detector_container",
            "map_server",
            "configure_map_server"
        });
    } else if (tag == "MATCH") {
        forceKillByPattern({
            "match.launch.py",
            "component_container --ros-args -r __node:=camera_detector_container"
        });
    } else if (tag == "CALIB") {
        forceKillByPattern({
            "calib.launch.py",
            "calib_bag.launch.py",
            "radar_calib_node"
        });
    } else if (tag == "MAP_VIEW") {
        forceKillByPattern({
            "debug_map map.launch.py",
            "install/debug_map/lib/.*/debug_map"
        });
    }

    log(QString("[%1] 已停止").arg(tag));
    if (target == proc) {
        target->deleteLater();
        proc = nullptr;
    }
}

// ── Timers / Status ───────────────────────────────────────────────────────────
void RmControlPanel::tickClock()
{
    if(clockLabel_) clockLabel_->setText(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"));
}

void RmControlPanel::flushLogs()
{
    QStringList chunk;
    int dropped = 0;
    {
        QMutexLocker lk(&logMutex_);
        if (!pendingLogs_.isEmpty()) {
            const int n = qMin(kLogFlushBatchSize, pendingLogs_.size());
            chunk = pendingLogs_.mid(0, n);
            pendingLogs_ = pendingLogs_.mid(n);
        }
        dropped = gDroppedLogLines.exchange(0);
    }

    if (chunk.isEmpty() && dropped <= 0) return;
    for (const QString& m : chunk) logText_->append(m);
    if (dropped > 0) logText_->append(QString("[SYSTEM] 日志限流: 已丢弃 %1 行，保障 UI 响应").arg(dropped));
    logText_->verticalScrollBar()->setValue(logText_->verticalScrollBar()->maximum());
}

void RmControlPanel::doRefreshStatus()
{
    auto st=[](QProcess* p){return p&&p->state()!=QProcess::NotRunning?"运行中":"未启动";};
    matchStatusLbl_ ->setText(st(procMatch_));
    replayStatusLbl_->setText(st(procReplay_));
    calibStatusLbl_ ->setText(st(procCalib_));
}

// ── Helpers ───────────────────────────────────────────────────────────────────
void RmControlPanel::tryPlaceWindow(const QStringList& keywordList, int x, int y, int w, int h, int retries)
{
    auto attempt = std::make_shared<std::function<void(int)>>();
    *attempt = [this, keywordList, x, y, w, h, attempt](int remaining) {
        if (remaining <= 0) return;

        QProcess check;
        check.start("bash", {"-lc", "command -v wmctrl >/dev/null 2>&1"});
        check.waitForFinished(500);
        if (check.exitCode() != 0) return;

        QProcess list;
        list.start("bash", {"-lc", "wmctrl -l"});
        if (!list.waitForFinished(1000)) {
            QTimer::singleShot(500, this, [remaining, attempt]() { (*attempt)(remaining - 1); });
            return;
        }

        const QStringList lines = QString::fromUtf8(list.readAllStandardOutput()).split('\n', Qt::SkipEmptyParts);
        QString targetId;
        for (const QString& line : lines) {
            const QString lower = line.toLower();
            bool match = false;
            for (const QString& key : keywordList) {
                if (lower.contains(key.toLower())) {
                    match = true;
                    break;
                }
            }
            if (match) {
                targetId = line.split(' ', Qt::SkipEmptyParts).value(0).trimmed();
                break;
            }
        }

        if (!targetId.isEmpty()) {
            QProcess::execute(
                "bash",
                {"-lc", QString("wmctrl -i -r %1 -e 0,%2,%3,%4,%5").arg(targetId).arg(x).arg(y).arg(w).arg(h)});
            return;
        }

        QTimer::singleShot(500, this, [remaining, attempt]() { (*attempt)(remaining - 1); });
    };

    (*attempt)(retries);
}

QString RmControlPanel::valFromCombo(const QString& text){ return text.trimmed().split(" ").first(); }
QString RmControlPanel::resolvePath(const QString& path){
    QString p=path.trimmed(); if(p.isEmpty()) return "";
    return QFileInfo(p).isAbsolute()?p:ROOT+"/"+p;
}
void RmControlPanel::log(const QString& msg){
    QMutexLocker lk(&logMutex_);
    if (pendingLogs_.size() >= kMaxPendingLogs) {
        gDroppedLogLines.fetch_add(1);
        return;
    }
    pendingLogs_ << msg;
}
void RmControlPanel::setActionBusy(bool busy){ actionInProgress_=busy; }

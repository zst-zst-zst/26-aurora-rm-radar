#pragma once
#include <QMainWindow>
#include <QLabel>
#include <QComboBox>
#include <QLineEdit>
#include <QPushButton>
#include <QTextEdit>
#include <QCheckBox>
#include <QGroupBox>
#include <QProcess>
#include <QTimer>
#include <QStringList>
#include <QMap>
#include <QPixmap>
#include <QMutex>
#include <QGridLayout>
#include <QProgressBar>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <vision_interface/msg/match_info.hpp>
#include <thread>
#include "FieldMapWidget.h"

struct FormData {
    int     team        = 1;
    QString camera      = "day";
    int     selfFrame   = 1;
    int     lidarEnable = 1;
    int     mapDebug    = 0;
    QString serialPort  = "/dev/gimbal";
    int     serialEnable= 1;
    int     serialBaud  = 115200;
    int     autoRecord  = 1;
    QString recordDir   = "/home/zst/T/bags";
    QString recordPrefix= "match";
    QString mapYaml     = "config/map/map.yaml";
    QString mapImage    = "config/map/map.jpg";
    QString mapPoints   = "config/map/map_points.yaml";
    QString mapPcd      = "config/map/map.pcd";
    double  mapWidth    = 28.0;
    double  mapHeight   = 15.0;
    int     useBagTiming= 1;
    int     showImage   = 0;
    QString rosbagFile  = "bags/latest";
    double  replayRate  = 1.0;
    int     maxSleepMs  = 300;
    int     decodeImage = 1;
    int     publishComp = 1;
    QStringList passthrough;
};

class RmControlPanel : public QMainWindow {
    Q_OBJECT
public:
    explicit RmControlPanel(QWidget* parent = nullptr);
    ~RmControlPanel() override;
protected:
    void closeEvent(QCloseEvent*) override;
private slots:
    void onApplyAll();
    void onSerialPrecheck();
    void onStartMatch();
    void onStopMatch();
    void onStartReplay();
    void onStopReplay();
    void onStartCalibration();
    void onStartCalibrationFromBag();
    void onStopCalibration();
    void onCalibrateAndMatch();
    void onShowManualCommands();
    void onOpenMapWindow();
    void onCloseMonitorWindows();
    void onApplyProfile();
    void onSaveProfile();
    void onProfileChanged(const QString&);
    void onRefreshRosbagCandidates();
    void onPickRosbagPath();
    void onArchiveSelectedBag();
    void onRefreshCameraHeartbeat();
    void onApplyCameraParams();
    void onUpdateMapPreview();
    void onRunPreflightCheck();
    void tickClock();
    void flushLogs();
    void doRefreshStatus();
    void updateBattleStatus(vision_interface::msg::MatchInfo msg);
private:
    void buildUI();
    void buildLeftPanel(QWidget*);
    void buildRightPanel(QWidget*);
    void buildBasicTab(QWidget*);
    void buildMapTab(QWidget*);
    void buildReplayTab(QWidget*);
    void buildChecklistTab(QWidget*);
    void buildBattleTab(QWidget*);
    void applyTheme();
    QGroupBox*  makeCard(QWidget* parent, const QString& title);
    QComboBox*  addComboRow(QGridLayout*, int row, const QString& title, const QStringList& vals);
    QLineEdit*  addEntryRow(QGridLayout*, int row, const QString& title);
    void        addStatusRow(QGridLayout*, int row, const QString& name, QLabel*& lbl);
    YAML::Node  loadYaml(const QString& path);
    void        saveYaml(const QString& path, YAML::Node data);
    void        loadMapProfiles();
    void        loadToWidgets();
    void        fillProfileFields(const YAML::Node&);
    FormData    collectForm();
    void        doApplyAll(bool skipGuard);
    void        doApplyProfile();
    void        runPrelaunchCleanup(const QString& mode);
    QProcess*   runCmd(const QString& cmd, const QString& tag);
    void        stopProc(QProcess*& proc, const QString& tag);
    QString     valFromCombo(const QString& text);
    QString     resolvePath(const QString& path);
    void        log(const QString& msg);
    void        setActionBusy(bool busy);
    void        tryPlaceWindow(const QStringList& keywordList, int x, int y, int w, int h, int retries = 12);

    YAML::Node               runtimeCfg_;
    QMap<QString, YAML::Node>mapProfiles_;
    QStringList              rosbagCandidates_;
    bool                     actionInProgress_ = false;

    QProcess* procMatch_      = nullptr;
    QProcess* procReplay_     = nullptr;
    QProcess* procCalib_      = nullptr;
    QProcess* procMapView_    = nullptr;

    QLabel* matchStatusLbl_  = nullptr;
    QLabel* replayStatusLbl_ = nullptr;
    QLabel* calibStatusLbl_  = nullptr;
    QLabel* lastApplyLbl_    = nullptr;
    QLabel* clockLabel_      = nullptr;

    QComboBox* teamCombo_       = nullptr;
    QComboBox* cameraCombo_     = nullptr;
    QComboBox* selfFrameCombo_  = nullptr;
    QComboBox* lidarEnableCombo_= nullptr;
    QComboBox* mapDebugCombo_   = nullptr;
    QComboBox* serialModeCombo_ = nullptr;
    QLineEdit* serialPortEdit_  = nullptr;
    QLineEdit* serialBaudEdit_  = nullptr;
    QComboBox* autoRecordCombo_ = nullptr;
    QLineEdit* recordDirEdit_   = nullptr;
    QLineEdit* recordPrefixEdit_= nullptr;

    QComboBox* profileCombo_  = nullptr;
    QLineEdit* mapYamlEdit_   = nullptr;
    QLineEdit* mapImageEdit_  = nullptr;
    QLineEdit* mapPointsEdit_ = nullptr;
    QLineEdit* mapPcdEdit_    = nullptr;
    QLineEdit* mapWidthEdit_  = nullptr;
    QLineEdit* mapHeightEdit_ = nullptr;
    QLabel*    mapPreviewLbl_ = nullptr;

    QComboBox* rosbagCombo_       = nullptr;
    QComboBox* showImageCombo_    = nullptr;
    QComboBox* useBagTimingCombo_ = nullptr;
    QLineEdit* replayRateEdit_    = nullptr;
    QLineEdit* maxSleepMsEdit_    = nullptr;
    QComboBox* decodeImageCombo_  = nullptr;
    QComboBox* publishCompCombo_  = nullptr;
    QLineEdit* passthroughEdit_   = nullptr;

    // 相机参数实时调节 (无需重启相机节点, 走 ROS2 set_parameter)
    QSpinBox*       cameraExposureSpin_ = nullptr;
    QDoubleSpinBox* cameraGainSpin_     = nullptr;
    QLabel*         cameraApplyStatus_  = nullptr;

    QCheckBox* chkLidar_  = nullptr;
    QCheckBox* chkSerial_ = nullptr;
    QCheckBox* chkCalib_  = nullptr;
    QCheckBox* chkRecord_ = nullptr;
    QCheckBox* chkTeam_   = nullptr;
    QLabel*    preflightLbl_ = nullptr;

    QLabel*         cameraStatusLbl_ = nullptr;
    FieldMapWidget* fieldMap_         = nullptr;
    QTextEdit* logText_         = nullptr;

    QTimer*  clockTimer_     = nullptr;
    QTimer*  logFlushTimer_  = nullptr;
    QPixmap  mapPreviewPix_;
    QMutex   logMutex_;
    QStringList pendingLogs_;

    // ── ROS2 实时订阅 (rclcpp 线程) ─────────────────────────────────
    rclcpp::Node::SharedPtr rosNode_;
    rclcpp::Subscription<vision_interface::msg::MatchInfo>::SharedPtr subMatchInfo_;
    std::thread                 rosThread_;

    // ── 战况面板 ────────────────────────────────────────────────────────────
    QLabel*        gameTimeLbl_    = nullptr;
    QLabel*        gameStateLbl_   = nullptr;
    QProgressBar*  selfHpBar_[8]   = {};
    QLabel*        selfHpVal_[8]   = {};
    QProgressBar*  enemyHpBar_[8]  = {};
    QLabel*        enemyHpVal_[8]  = {};
    QLabel*        markLbl_[5]     = {};
    QLabel*        dartLbl_        = nullptr;
    QLabel*        battlePollLbl_  = nullptr;
};

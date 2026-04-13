#ifndef hwa_qt_mainwindow_h
#define hwa_qt_mainwindow_h
#include "hwa_qt_insworker.h"
#include <QMainWindow>
#include <QFormLayout>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QFile>
#include <QFileDialog>
#include <QTextStream>
#include <QTimer>
#include <QProcess>
#include <QDir>
#include <QApplication>
#include <QMetaObject>
#include <QScrollArea>
#include <QMessageBox>
#include <QLibrary>
#include <QDateTime>
#include <QString>
#include <QStringList>
#include <memory>
#include <vector>
#include <map>
#include <string>
#include <mutex>
#include <filesystem>
#include "hwa_qt_imagereader.h"
#include "hwa_qt_imageshow.h"
#include "hwa_qt_imagedownloader.h"
#include "hwa_qt_imagetracker.h"
#include "hwa_qt_featuremanager.h"
#include "hwa_qt_visworker.h"
#ifndef Q_MOC_RUN
#include "hwa_set_all.h"
#include "hwa_base_glob.h"
#include "hwa_vis_yolo_worker.h"
#endif

using namespace hwa_base;
using namespace hwa_set;
using namespace hwa_vis;
using namespace hwa_ins;

namespace hwa_qt {
    using Mutex = std::map<int, std::shared_ptr<std::mutex>>;
    using VW = std::map<int, std::shared_ptr<cv::VideoWriter>>;
    using LV = std::map<int, std::shared_ptr<QLabel>>;
    using Mstr = std::map<int, QString>;
    using Mimg = std::map<int, ImgMap>;
    using Mcond = std::map<int, std::shared_ptr<base_cond_var>>;
    using Myolo = std::map<int, std::shared_ptr<vis_yolo_v8ov>>;

    enum VType {
        MP4,
        AVI
    };

    class MainWindow : public QMainWindow {
        Q_OBJECT
    public:
        explicit MainWindow(std::shared_ptr<hwa_set::set_msf> _gset, QWidget* parent = nullptr);
        ~MainWindow();

    signals:
        void start_image_readers();
        void stop_image_readers();
        void start_yolo_workers();
        void stop_yolo_workers();
        void start_image_showers();
        void stop_image_showers();
        void start_image_downloader();
        void stop_image_downloader();
        void start_ins_worker();
        void stop_ins_worker();
        void start_feature_managers();
        void stop_feature_managers();
        void start_tracker();
        void stop_tracker();

    private slots:
        void onInitAll();
        void onStart();
        void onStop();
        void onRecordStart();
        void onRecordStop();
        void onDownloadStart();
        void onDownloadStop();
        void onTrackStart();
        void onTrackStop();
        void onInsStart();
        void onInsStop();
        void onVisStart();
        void onVisStop();
        void onQuit();
        void onNewPose(double timestamp, const Eigen::Quaterniond& q,
            const Triple& t);

    private:
        void setupUi();
        void ConnectThreads();
        void createNavGroupBox();
        void KillThread();
        VType str2vtp(std::string s);

    private:
        int csize;
        std::stringstream dir;
        bool isFirst;
        std::shared_ptr<hwa_set::set_msf> gset;
        std::unique_ptr<QPushButton> btnInit_{};
        std::unique_ptr<QPushButton> btnStart_{};
        std::unique_ptr<QPushButton> btnStop_{};
        std::unique_ptr<QPushButton> btnQuit_{};
        std::unique_ptr<QPushButton> btnStartRecord_{};
        std::unique_ptr<QPushButton> btnEndRecord_{};
        std::unique_ptr<QPushButton> btnStartSeqdownload_{};
        std::unique_ptr<QPushButton> btnEndSeqdownload_{};
        std::unique_ptr<QPushButton> btnStartTrack_{};
        std::unique_ptr<QPushButton> btnEndTrack_{};
        std::unique_ptr<QPushButton> btnStartIns_{};
        std::unique_ptr<QPushButton> btnStopIns_{};
        std::unique_ptr<QPushButton> btnStartVis_{};
        std::unique_ptr<QPushButton> btnStopVis_{};
        std::unique_ptr<QGroupBox> navBox{};
        std::unique_ptr<QLabel> Time{};
        std::unique_ptr<QLabel> PosText{};
        std::unique_ptr<QLabel> labX{};
        std::unique_ptr<QLabel> labY{};
        std::unique_ptr<QLabel> labZ{};
        std::unique_ptr<QLabel> QuatText{};
        std::unique_ptr<QLabel> labQ0{};
        std::unique_ptr<QLabel> labQ1{};
        std::unique_ptr<QLabel> labQ2{};
        std::unique_ptr<QLabel> labQ3{};

        Mcond condition_coder;
        Mcond condition_shower;
        Mstr VideoPath;
        LV labelVideo_{};
        LV vislabelVideo_{};
        VW vw_;
        Mutex codermutex_;
        Mutex showmutex_;
        Mutex vwmutex_;
        Mutex trackmutex_;
        Mimg imagepresenter;
        Mimg imagecoder;
        Mimg imagedownloader;
        MMpts pointmap;
        Myolo yolo;

        std::map<int, QThread>  CoderThreads;
        std::map<int, std::unique_ptr<qt_image_reader>> HC;

        std::map<int, QThread>  ShowThreads;
        std::map<int, std::unique_ptr<qt_image_shower>> Showers;

        std::map<int, QThread>  YoloThreads;
        std::map<int, std::unique_ptr<vis_yolo_worker>> yoloworkers;

        std::map<int, QThread>  FeatureThreads;
        std::map<int, std::unique_ptr<qt_feature_manager>> featuremanagers;

        QThread  DownloadThread;
        std::unique_ptr<qt_image_downloader> Downloader;

        std::map<int, QThread> visThreads;
        std::map<int, std::unique_ptr<qt_vis_worker>> visworkers;

        std::map<int, QThread> visshowThreads;
        std::map<int, std::unique_ptr<qt_image_shower>> visshowworkers;

        QThread  TrackerThread;
        std::unique_ptr<qt_img_tracker> Tracker;

        QThread  insThread;
        std::unique_ptr<qt_ins_worker> InsWorker;

        std::vector<int> cameralist;
        std::vector<int> camskip;
        std::map<int, USBInformation> usbInfo;
        std::vector<std::string> devicepathlist;
        std::vector<int> framerate;
        std::vector<std::pair<int, int>> framesize;
        std::vector<std::string> usbname;

        VType videotype = MP4;
        bool m_init = false;
        const QString btnStyle1 =
            "QPushButton{"
            "  width:180px; height:40px;"
            "  font: bold 14px \"Microsoft YaHei\";"
            "  margin:6px; padding:10px;"
            "  background-color:#0078D4;"
            "  color:#FFFFFF;"
            "  border:none;"
            "  border-radius:4px;"
            "}"
            "QPushButton:hover{"
            "  background-color:#106EBE;"
            "}"
            "QPushButton:pressed{"
            "  background-color:#005A9E;"
            "}";

        const QString btnStyle2 =
            "QPushButton{"
            "  width:180px; height:40px;"
            "  font: bold 14px \"Microsoft YaHei\";"
            "  margin:6px; padding:10px;"
            "  background-color:#FF5722;"
            "  color:#FFFFFF;"
            "  border:none;"
            "  border-radius:4px;"
            "}"
            "QPushButton:hover{"
            "  background-color:#106EBE;"
            "}"
            "QPushButton:pressed{"
            "  background-color:#005A9E;"
            "}";

        const QString btnStyle3 =
            "QPushButton{"
            "  width:180px; height:40px;"
            "  font: bold 14px \"Microsoft YaHei\";"
            "  margin:6px; padding:10px;"
            "  background-color:#4CAF50;"
            "  color:#FFFFFF;"
            "  border:none;"
            "  border-radius:4px;"
            "}"
            "QPushButton:hover{"
            "  background-color:#388E3C;"
            "}"
            "QPushButton:pressed{"
            "  background-color:#2E7D32;"
            "}";
    };
}

#endif


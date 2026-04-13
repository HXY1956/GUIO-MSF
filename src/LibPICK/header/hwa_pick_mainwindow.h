#pragma once
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
#include "hwa_set_proc.h"
#include "hwa_base_glob.h"
#include "hwa_pick_imagereader.h"
#include "hwa_pick_imagedownloader.h"
#include "hwa_pick_imageshow.h"

namespace hwa_pick {

    using Mutex = std::map<int, std::shared_ptr<std::mutex>>;
    using VW = std::map<int, std::shared_ptr<cv::VideoWriter>>;
    using LV = std::map<int, std::shared_ptr<QLabel>>;
    using Mstr = std::map<int, QString>;

    class MainWindow : public QMainWindow {
        Q_OBJECT
    public:
        explicit MainWindow(std::shared_ptr<hwa_set::set_base> _gset, QWidget* parent = nullptr);
        ~MainWindow();

    signals:
        void requestStart();
        void requestStop();

        void start_image_readers();
        void stop_image_readers();
        void start_image_showers();
        void stop_image_showers();
        void start_image_downloader();
        void stop_image_downloader();

    private slots:
        void onInitAll();
        void onStart();
        void onStop();
        void onDownloadStart();
        void onDownloadStop();
        void onQuit();

    private:
        void setupUi();
        void ConnectThreads();
        void KillThread();

    private:
        int csize;
        std::stringstream dir;
        bool isFirst;
        std::unique_ptr<QPushButton> btnStart_{};
        std::unique_ptr<QPushButton> btnStop_{};
        std::unique_ptr<QPushButton> btnQuit_{};
        std::unique_ptr<QPushButton> btnStartSeqdownload_{};
        std::unique_ptr<QPushButton> btnEndSeqdownload_{};

        LV labelVideo_{};

        std::map<int, QThread>  CoderThreads;
        std::map<int, std::unique_ptr<qt_image_reader>> HC;

        std::map<int, QThread>  ShowThreads;
        std::map<int, std::unique_ptr<qt_image_shower>> Showers;

        QThread  DownloadThread;
        std::unique_ptr<qt_image_downloader> Downloader;

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
    };
}



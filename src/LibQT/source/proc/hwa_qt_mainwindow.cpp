#include "hwa_qt_mainwindow.h"

namespace hwa_qt {

    MainWindow::MainWindow(std::shared_ptr<hwa_set::set_msf> _gset, QWidget* parent) : QMainWindow(parent), gset(_gset), isFirst(true)
    {
        onInitAll();
        setupUi();
    }

    MainWindow::~MainWindow() {
        onStop();
    }

    void MainWindow::ConnectThreads() {

        for (int i = 0; i < csize; ++i) {
            // Coder
            HC[i]->moveToThread(&CoderThreads[i]);
            connect(HC[i].get(), &qt_image_reader::NewImage,
                yoloworkers[i].get(), &vis_yolo_worker::NewImage,
                Qt::QueuedConnection);
            connect(HC[i].get(), &qt_image_reader::NewImage2Download,
                Downloader.get(), &qt_image_downloader::NewImage,
                Qt::QueuedConnection);
            connect(this, &MainWindow::start_image_readers,
                HC[i].get(), &qt_image_reader::startWork);
            connect(this, &MainWindow::stop_image_readers,
                HC[i].get(), &qt_image_reader::stopWork);

            // Detector
            yoloworkers[i]->moveToThread(&YoloThreads[i]);
            connect(yoloworkers[i].get(), &vis_yolo_worker::NewImage2Show,
                Showers[i].get(), &qt_image_shower::NewImage,
                Qt::QueuedConnection);
            connect(yoloworkers[i].get(), &vis_yolo_worker::NewPts,
                Tracker.get(), &qt_img_tracker::NewPts,
                Qt::QueuedConnection);
            connect(this, &MainWindow::start_yolo_workers,
                yoloworkers[i].get(), &vis_yolo_worker::startWork);
            connect(this, &MainWindow::stop_yolo_workers,
                yoloworkers[i].get(), &vis_yolo_worker::stopWork);

            // Shower
            Showers[i]->moveToThread(&ShowThreads[i]);
            connect(this, &MainWindow::start_image_showers,
                Showers[i].get(), &qt_image_shower::startWork);
            connect(this, &MainWindow::stop_image_showers,
                Showers[i].get(), &qt_image_shower::stopWork);
            connect(Showers[i].get(), &qt_image_shower::NewPixmap,
                labelVideo_[i].get(), [l = labelVideo_[i].get()](const QPixmap& px) {
                    l->setPixmap(px.scaled(l->size(),
                    Qt::KeepAspectRatio,
                    Qt::SmoothTransformation));
                }, Qt::QueuedConnection);
        }
        // Downloader
        Downloader->moveToThread(&DownloadThread);
        connect(this, &MainWindow::start_image_downloader,
            Downloader.get(), &qt_image_downloader::startWork);
        connect(this, &MainWindow::stop_image_downloader,
            Downloader.get(), &qt_image_downloader::stopWork);

        // TRACKER
        Tracker->moveToThread(&TrackerThread);
        connect(this, &MainWindow::start_tracker,
            Tracker.get(), &qt_img_tracker::startWork);
        connect(this, &MainWindow::stop_tracker,
            Tracker.get(), &qt_img_tracker::stopWork);
        connect(Tracker.get(), &qt_img_tracker::NewPose,
            InsWorker.get(), &qt_ins_worker::Accept_tracker,
            Qt::QueuedConnection);

        // INS
        InsWorker->moveToThread(&insThread);
        connect(this, &MainWindow::start_ins_worker,
            InsWorker.get(), &qt_ins_worker::startWork);
        connect(this, &MainWindow::stop_ins_worker,
            InsWorker.get(), &qt_ins_worker::stopWork);
        connect(InsWorker.get(), &qt_ins_worker::NewPose,
            this, &MainWindow::onNewPose,
            Qt::QueuedConnection);

        for (auto i : cameralist) {
            // VIS
            visworkers[i]->moveToThread(&visThreads[i]);
            connect(InsWorker.get(), &qt_ins_worker::NewIns,
                visworkers[i].get(), &qt_vis_worker::Accept_ins,
                Qt::QueuedConnection);
            connect(visworkers[i].get(), &qt_vis_worker::NewUpdate,
                InsWorker.get(), &qt_ins_worker::Accept_vis,
                Qt::QueuedConnection);
            connect(visworkers[i].get(), &qt_vis_worker::SharedCamAttr,
                InsWorker.get(), &qt_ins_worker::AcceptSharedCamAttr,
                Qt::QueuedConnection);

            // Feature Manager
            featuremanagers[i]->moveToThread(&FeatureThreads[i]);
            connect(this, &MainWindow::start_feature_managers,
                featuremanagers[i].get(), &qt_feature_manager::startWork);
            connect(this, &MainWindow::stop_feature_managers,
                featuremanagers[i].get(), &qt_feature_manager::stopWork);
            connect(featuremanagers[i].get(), &qt_feature_manager::NewMap,
                visworkers[i].get(), &qt_vis_worker::Accept_pts,
                Qt::QueuedConnection);
            connect(featuremanagers[i].get(), &qt_feature_manager::NewImg,
                visshowworkers[i].get(), &qt_image_shower::NewImage,
                Qt::QueuedConnection);

            visshowworkers[i]->moveToThread(&visshowThreads[i]);
            connect(this, &MainWindow::start_image_showers,
                visshowworkers[i].get(), &qt_image_shower::startWork);
            connect(this, &MainWindow::stop_image_showers,
                visshowworkers[i].get(), &qt_image_shower::stopWork);
            connect(visshowworkers[i].get(), &qt_image_shower::NewPixmap,
                vislabelVideo_[i].get(), [l = vislabelVideo_[i].get()](const QPixmap& px) {
                    l->setPixmap(px.scaled(l->size(),
                    Qt::KeepAspectRatio,
                    Qt::SmoothTransformation));
                }, Qt::QueuedConnection);
        }
    }

    void MainWindow::KillThread() {
        DownloadThread.quit(); DownloadThread.wait();
        TrackerThread.quit(); TrackerThread.wait();
        emit stop_tracker();
        emit stop_image_downloader();
        for (int i = 0; i < csize; i++) {
            CoderThreads[i].quit(); CoderThreads[i].wait();
            ShowThreads[i].quit(); ShowThreads[i].wait();
            yoloworkers[i]->SetTrack(false); YoloThreads[i].quit(); YoloThreads[i].wait();
            emit stop_image_readers();
            emit stop_image_showers();
            emit stop_yolo_workers();

            labelVideo_[i]->clear();
            labelVideo_[i]->setText("关闭画面");
            std::lock_guard<std::mutex> lock(*vwmutex_[i]);
            if (vw_[i]->isOpened()) {
                vw_[i]->release();
                qInfo() << "视频已保存到: " << VideoPath[i];
            }
        }
    }

    VType MainWindow::str2vtp(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(),
            [](unsigned char c) { return std::toupper(c); });
        if (s == "MP4") return MP4;
        if (s == "AVI") return AVI;
        return AVI;
    }

    void MainWindow::onNewPose(double timestamp, const Eigen::Quaterniond& q,
        const Triple& t)
    {
        Time->setText(QString::number(timestamp, 'f', 3) + "(s)");
        labQ0->setText(QString::number(q.w(), 'f', 3));
        labQ1->setText(QString::number(q.x(), 'f', 3));
        labQ2->setText(QString::number(q.y(), 'f', 3));
        labQ3->setText(QString::number(q.z(), 'f', 3));
        labX->setText(QString::number(t.x(), 'f', 3));
        labY->setText(QString::number(t.y(), 'f', 3));
        labZ->setText(QString::number(t.z(), 'f', 3));
    }

    void MainWindow::createNavGroupBox()
    {
        navBox = std::make_unique<QGroupBox>("载体实时位姿");
        auto* lay = new QFormLayout(navBox.get());

        Time = std::make_unique<QLabel>("0.000(s)");
        PosText = std::make_unique<QLabel>("");
        labX = std::make_unique<QLabel>("0.000");
        labY = std::make_unique<QLabel>("0.000");
        labZ = std::make_unique<QLabel>("0.000");
        QuatText = std::make_unique<QLabel>("");
        labQ0 = std::make_unique<QLabel>("1.000");
        labQ1 = std::make_unique<QLabel>("0.000");
        labQ2 = std::make_unique<QLabel>("0.000");
        labQ3 = std::make_unique<QLabel>("0.000");

        lay->addRow("Time [s]:", Time.get());
        lay->addRow("Pos (m):", PosText.get());
        lay->addRow("X [m]:", labX.get());
        lay->addRow("Y [m]:", labY.get());
        lay->addRow("Z [m]:", labZ.get());
        lay->addRow("Rotation:", QuatText.get());
        lay->addRow("q0 (w):", labQ0.get());
        lay->addRow("q1 (x):", labQ1.get());
        lay->addRow("q2 (y):", labQ2.get());
        lay->addRow("q3 (z):", labQ3.get());

        navBox->setStyleSheet(
            "QLabel {"
            "   color : red;"
            "   font : bold 14px;"
            "   padding : 4px 0px;"
            "}");
    }


    void MainWindow::setupUi() {
        auto central = new QWidget(this);
        auto mainLayout = new QVBoxLayout(central);
        auto downLayout = new QHBoxLayout();

        auto trackerLayout = new QHBoxLayout();
        auto visLayout = new QHBoxLayout();
        auto down2Layout = new QVBoxLayout();

        auto btnGrid = new QGridLayout();

        int row = 0, col = 0;
        const int Cols = 7;
        auto addBtn = [&](QPushButton* btn) {
            btnGrid->addWidget(btn, row, col);
            if (++col == Cols) { col = 0; ++row; }
            };
        auto makeBtn = [&](const QString& text) {
            auto btn = std::make_unique<QPushButton>(text, this);
            btn->setStyleSheet(btnStyle1);
            return btn;
            };

        btnInit_ = makeBtn("初始化");
        btnStart_ = makeBtn("开启检测");
        btnStop_ = makeBtn("结束检测");
        btnStartRecord_ = makeBtn("开始录制");
        btnEndRecord_ = makeBtn("结束录制");
        btnStartSeqdownload_ = makeBtn("采同步图");
        btnEndSeqdownload_ = makeBtn("停止采集");
        btnStartTrack_ = makeBtn("开始跟踪");
        btnEndTrack_ = makeBtn("停止跟踪");
        btnStartIns_ = makeBtn("开启INS");
        btnStopIns_ = makeBtn("关闭INS");
        btnStartVis_ = makeBtn("开启VIS");
        btnStopVis_ = makeBtn("关闭VIS");
        btnQuit_ = makeBtn("退出程序");

        addBtn(btnInit_.get());
        addBtn(btnStart_.get());
        addBtn(btnStop_.get());
        addBtn(btnStartRecord_.get());
        addBtn(btnEndRecord_.get());
        addBtn(btnStartSeqdownload_.get());
        addBtn(btnEndSeqdownload_.get());
        addBtn(btnStartTrack_.get());
        addBtn(btnEndTrack_.get());
        addBtn(btnStartIns_.get());
        addBtn(btnStopIns_.get());
        addBtn(btnStartVis_.get());
        addBtn(btnStopVis_.get());
        addBtn(btnQuit_.get());

        auto btnBar = new QWidget;
        btnBar->setLayout(btnGrid);
        btnBar->setFixedHeight(200);
        btnBar->setFixedWidth(1080);

        createNavGroupBox();
        navBox->setFixedHeight(300);
        navBox->setFixedWidth(200);
        downLayout->addWidget(navBox.get());

        for (int i = 0; i < csize; i++) {
            labelVideo_[i]->setAlignment(Qt::AlignCenter);
            labelVideo_[i]->setText("相机画面");
            labelVideo_[i]->setFixedWidth(400);
            labelVideo_[i]->setFixedHeight(300);
            labelVideo_[i]->setScaledContents(true);
            trackerLayout->addWidget(labelVideo_[i].get());
        }
        for (auto i : cameralist) {
            vislabelVideo_[i]->setAlignment(Qt::AlignCenter);
            vislabelVideo_[i]->setText("相机画面");
            vislabelVideo_[i]->setFixedWidth(400);
            vislabelVideo_[i]->setFixedHeight(300);
            vislabelVideo_[i]->setScaledContents(true);
            visLayout->addWidget(vislabelVideo_[i].get());
        }
        down2Layout->addLayout(trackerLayout);
        down2Layout->addLayout(visLayout);
        downLayout->addLayout(down2Layout);

        mainLayout->addWidget(btnBar);
        mainLayout->addLayout(downLayout);
        setCentralWidget(central);
        this->setFixedSize(1080, 900);

        connect(btnInit_.get(), &QPushButton::clicked, this, &MainWindow::onInitAll);
        connect(btnStart_.get(), &QPushButton::clicked, this, &MainWindow::onStart);
        connect(btnStop_.get(), &QPushButton::clicked, this, &MainWindow::onStop);
        connect(btnStartRecord_.get(), &QPushButton::clicked, this, &MainWindow::onRecordStart);
        connect(btnEndRecord_.get(), &QPushButton::clicked, this, &MainWindow::onRecordStop);
        connect(btnStartSeqdownload_.get(), &QPushButton::clicked, this, &MainWindow::onDownloadStart);
        connect(btnEndSeqdownload_.get(), &QPushButton::clicked, this, &MainWindow::onDownloadStop);
        connect(btnStartTrack_.get(), &QPushButton::clicked, this, &MainWindow::onTrackStart);
        connect(btnEndTrack_.get(), &QPushButton::clicked, this, &MainWindow::onTrackStop);
        connect(btnQuit_.get(), &QPushButton::clicked, this, &MainWindow::onQuit);
        connect(btnStartIns_.get(), &QPushButton::clicked, this, &MainWindow::onInsStart);
        connect(btnStopIns_.get(), &QPushButton::clicked, this, &MainWindow::onInsStop);
        connect(btnStartVis_.get(), &QPushButton::clicked, this, &MainWindow::onVisStart);
        connect(btnStopVis_.get(), &QPushButton::clicked, this, &MainWindow::onVisStop);
    }

    void MainWindow::onInitAll() {
        if (m_init) return;
        m_init = true;
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::tm now_tm;
        localtime_s(&now_tm, &now_c);
        dir << "dataset\\data_" << std::put_time(&now_tm, "%Y-%m-%d_%H-%M-%S");
        std::filesystem::create_directories(dir.str().c_str());

        MV_CC_Initialize();
        MV_CC_DEVICE_INFO_LIST stDeviceList;

        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        csize = stDeviceList.nDeviceNum;
        for (int i = 0; i < csize; ++i) {
            yolo[i] = std::make_shared<vis_yolo_v8ov>(gset);
            vwmutex_[i] = std::make_shared<std::mutex>();
            vw_[i] = std::make_shared<cv::VideoWriter>();
            labelVideo_[i] = std::make_shared<QLabel>();
            yoloworkers[i] = std::make_unique<vis_yolo_worker>(gset, i, yolo[i]);
            HC[i] = std::make_unique<qt_image_reader>(i); 
            HC[i]->SetDevideInfo(stDeviceList.pDeviceInfo[i]);
            HC[i]->Init();
            Showers[i] = std::make_unique<qt_image_shower>(i, vwmutex_[i], vw_[i]);
            Showers[i]->SetFormat(dynamic_cast<set_tracker*>(gset.get())->type());
        }

        Downloader = std::make_unique<qt_image_downloader>();

        Tracker = std::make_unique<qt_img_tracker>(gset);
        if (csize > 0) Tracker->SetSize(HC[0]->HC->GetSize());
        if (csize >= 2) {
            cv::Mat map11, map12, map21, map22, Q;
            Tracker->ComputeRectificationMaps(map11, map12, map21, map22, Q);
            HC[0]->SetMap(map11, map12);
            HC[1]->SetMap(map21, map22);
            Tracker->SetQ(Q);
        }

        InsWorker = std::make_unique<qt_ins_worker>(gset, dir.str());
        int cam_number = dynamic_cast<set_vis*>(gset.get())->group_number();
        for (int i = 0; i < cam_number; i++) {
            devicepathlist.push_back(dynamic_cast<set_vis*>(gset.get())->gst_path(i));
            framerate.push_back(dynamic_cast<set_vis*>(gset.get())->freq(i));
            framesize.push_back(std::make_pair<int,int>(dynamic_cast<set_vis*>(gset.get())->cam0_resolution(i)[0], dynamic_cast<set_vis*>(gset.get())->cam0_resolution(i)[1]));
            usbname.push_back(dynamic_cast<set_vis*>(gset.get())->cam_group_name(i));
            camskip.push_back(dynamic_cast<set_vis*>(gset.get())->cam_update_skip(i));
        }
        cameralist = dynamic_cast<set_vis*>(gset.get())->camera_list();

        for (auto i : cameralist) {
            usbInfo[i] = USBInformation(devicepathlist[i], framerate[i], framesize[i], usbname[i], dir.str(), i, camskip[i]);
            visworkers[i] = std::make_unique<qt_vis_worker>(gset.get(), i);
            visshowworkers[i] = std::make_unique<qt_image_shower>(i);
            visshowworkers[i]->SetFormat(dynamic_cast<set_tracker*>(gset.get())->type());
            vislabelVideo_[i] = std::make_shared<QLabel>();
            featuremanagers[i] = std::make_unique<qt_feature_manager>(gset.get(), usbInfo[i]);
        }
        ConnectThreads();
        ProjectStartTime = std::chrono::duration<double>(
            std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    }

    void MainWindow::onStart() {
        if (!HC[0] || !Showers[0] || !yoloworkers[0]) {
            qCritical() << "Init Failed";
            return;
        };
        btnStart_->setStyleSheet(btnStyle2);
        for (int i = 0; i < csize; i++) {
            CoderThreads[i].start();
            ShowThreads[i].start();
            YoloThreads[i].start();
        }
        emit start_image_readers();
        emit start_image_showers();
        emit start_yolo_workers();
    }

    void MainWindow::onStop() {
        if (!HC[0] || !Showers[0] || !yoloworkers[0]) {
            qCritical() << "Init Failed";
            return;
        };
        btnStart_->setStyleSheet(btnStyle1);
        btnStartTrack_->setStyleSheet(btnStyle1);
        btnStartRecord_->setStyleSheet(btnStyle1);
        onRecordStop();
        KillThread();
    }

    void MainWindow::onRecordStart() {
        btnStartRecord_->setStyleSheet(btnStyle2);
        QString s1 = videotype == AVI ? ".avi" : ".mp4";
        std::string s2 = videotype == AVI ? "XVID" : "mp4v";
        QString videoDir_ = "record";
        for (int i = 0; i < csize; i++) {
            std::lock_guard<std::mutex> lock(*vwmutex_[i]);
            std::stringstream Dir_;
            Dir_ << videoDir_.toStdString() << "\\Cam" << std::to_string(i);
            std::filesystem::create_directories(Dir_.str().c_str());
            QString fileName = QDateTime::currentDateTime()
                .toString("yyyy-MM-dd_hh-mm-ss")
                + s1;
            VideoPath[i] = QString::fromStdString(Dir_.str() + "\\") + fileName;
            vw_[i]->open(VideoPath[i].toStdString(), cv::VideoWriter::fourcc(s2[0], s2[1], s2[2], s2[3]),
                10, HC[i]->HC->GetSize(), dynamic_cast<set_tracker*>(gset.get())->type() == "BGR");
            vw_[i]->set(cv::VIDEOWRITER_PROP_QUALITY, 95);
            if (!vw_[i]->isOpened()) {
                QMessageBox::critical(this, "错误", "无法创建视频文件");
            }
            else {
                qInfo() << "******** 视频已开始录制 ********";
            }
        }
    }

    void MainWindow::onRecordStop() {
        btnStartRecord_->setStyleSheet(btnStyle1);
        for (int i = 0; i < csize; i++) {
            std::lock_guard<std::mutex> lock(*vwmutex_[i]);
            if (vw_[i]->isOpened()) {
                vw_[i]->release();
                qInfo() << "视频已保存到: " << VideoPath[i];
            }
        }
    }

    void MainWindow::onDownloadStart() {
        btnStartSeqdownload_->setStyleSheet(btnStyle2);
        std::stringstream Dir_;
        Dir_ << "Images\\" << QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss").toStdString();
        std::filesystem::create_directories(Dir_.str().c_str());
        Downloader->SetDir(Dir_.str());
        for (int i = 0; i < csize; i++) {
            HC[i]->SetDownload(true);
        }
        DownloadThread.start();
        emit start_image_downloader();
    }

    void MainWindow::onDownloadStop() {
        btnStartSeqdownload_->setStyleSheet(btnStyle1);
        for (int i = 0; i < csize; i++) {
            HC[i]->SetDownload(false);
        }
        emit stop_image_downloader();
        DownloadThread.quit(); DownloadThread.wait();
    }

    void MainWindow::onTrackStart() {
        btnStartTrack_->setStyleSheet(btnStyle2);
        if (csize < 2) {
            std::cerr << "Cam Number < 2, Can't Track" << std::endl;
            return;
        }
        for (int i = 0; i < csize; i++) {
            yoloworkers[i]->SetTrack(true);
        }
        Tracker->SetTrack(true);
        TrackerThread.start();
        emit start_tracker();
    }

    void MainWindow::onTrackStop() {
        btnStartTrack_->setStyleSheet(btnStyle1);
        for (int i = 0; i < csize; i++) {
            yoloworkers[i]->SetTrack(false);
        }
        Tracker->SetTrack(false);
        emit stop_tracker();
        TrackerThread.quit(); TrackerThread.wait();
    }

    void MainWindow::onInsStart() {
        btnStartIns_->setStyleSheet(btnStyle2);
        insThread.start();
        emit start_ins_worker();
    }

    void MainWindow::onInsStop() {
        btnStartIns_->setStyleSheet(btnStyle1);
        emit stop_ins_worker();
        insThread.quit(); insThread.wait();
    }

    void MainWindow::onVisStart() {
        btnStartVis_->setStyleSheet(btnStyle2);
        for (auto i : cameralist) {
            FeatureThreads[i].start();
            visThreads[i].start();
            visshowThreads[i].start();
            emit start_feature_managers();
            emit start_image_showers();
        }
    }

    void MainWindow::onVisStop() {
        btnStartVis_->setStyleSheet(btnStyle1);
        for (auto i : cameralist) {
            FeatureThreads[i].quit(); FeatureThreads[i].wait();
            visThreads[i].quit(); visThreads[i].wait();
            visshowThreads[i].quit(); visshowThreads[i].wait();
            emit stop_feature_managers();
            emit stop_image_showers();
        }
    }

    void MainWindow::onQuit() {
        close();
    }
}

#include "hwa_pick_mainwindow.h"
using namespace hwa_pick;

MainWindow::MainWindow(std::shared_ptr<hwa_set::set_base> _gset, QWidget* parent) : QMainWindow(parent), isFirst(true)
{
    onInitAll();
    setupUi();
}

MainWindow::~MainWindow() {
    onStop();
}

void MainWindow::ConnectThreads(){

    for (int i = 0; i < csize; ++i) {
        // Coder
        HC[i]->moveToThread(&CoderThreads[i]);
        connect(HC[i].get(), &qt_image_reader::NewImage,
            Showers[i].get(), &qt_image_shower::NewImage,
            Qt::QueuedConnection);
        connect(this, &MainWindow::start_image_readers,
            HC[i].get(), &qt_image_reader::startWork);
        connect(this, &MainWindow::stop_image_readers,
            HC[i].get(), &qt_image_reader::stopWork);

        // Shower
        Showers[i]->moveToThread(&ShowThreads[i]);
        connect(Showers[i].get(), &qt_image_shower::NewImage2Download,
            Downloader.get(), &qt_image_downloader::NewImage,
            Qt::QueuedConnection);
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
}

void MainWindow::KillThread() {
    DownloadThread.quit(); DownloadThread.wait();
    emit stop_image_downloader();
    for (int i = 0; i < csize; i++) {
        CoderThreads[i].quit(); CoderThreads[i].wait();
        ShowThreads[i].quit(); ShowThreads[i].wait();
        emit stop_image_readers();
        emit stop_image_showers();

        labelVideo_[i]->clear();
        labelVideo_[i]->setText("关闭画面");
    }
}

void MainWindow::setupUi() {
    auto central = new QWidget(this);
    auto mainLayout = new QVBoxLayout(central);
    auto downLayout = new QHBoxLayout();

    auto btnGrid = new QGridLayout();

    int row = 0, col = 0;
    const int Cols = 5;
    auto addBtn = [&](QPushButton* btn) {
        btnGrid->addWidget(btn, row, col);
        if (++col == Cols) { col = 0; ++row;}
        };
    auto makeBtn = [&](const QString& text) {
        auto btn = std::make_unique<QPushButton>(text, this);
        btn->setStyleSheet(btnStyle1);
        return btn;
        };

    btnStart_ = makeBtn("开启检测");
    btnStop_ = makeBtn("结束检测");
    btnStartSeqdownload_ = makeBtn("采同步图");
    btnEndSeqdownload_ = makeBtn("停止采集");
    btnQuit_ = makeBtn("退出程序");

    addBtn(btnStart_.get());
    addBtn(btnStop_.get());
    addBtn(btnStartSeqdownload_.get());
    addBtn(btnEndSeqdownload_.get());
    addBtn(btnQuit_.get());

    auto btnBar = new QWidget;
    btnBar->setLayout(btnGrid);
    btnBar->setFixedHeight(100);
    btnBar->setFixedWidth(800);

    for (int i = 0; i < csize; i++) {
        labelVideo_[i]->setAlignment(Qt::AlignCenter);
        labelVideo_[i]->setText("相机画面");
        labelVideo_[i]->setFixedWidth(400);
        labelVideo_[i]->setFixedHeight(300);
        labelVideo_[i]->setScaledContents(true);
        downLayout->addWidget(labelVideo_[i].get());
    }

    mainLayout->addWidget(btnBar);
    mainLayout->addLayout(downLayout);
    setCentralWidget(central);
    this->setFixedSize(900, 450);

    connect(btnStart_.get(), &QPushButton::clicked, this, &MainWindow::onStart);
    connect(btnStop_.get(), &QPushButton::clicked, this, &MainWindow::onStop);
    connect(btnStartSeqdownload_.get(), &QPushButton::clicked, this, &MainWindow::onDownloadStart);
    connect(btnEndSeqdownload_.get(), &QPushButton::clicked, this, &MainWindow::onDownloadStop);
    connect(btnQuit_.get(), &QPushButton::clicked, this, &MainWindow::onQuit);
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
        labelVideo_[i] = std::make_shared<QLabel>();
        HC[i] = std::make_unique<qt_image_reader>(i);
        HC[i]->SetDevideInfo(stDeviceList.pDeviceInfo[i]);
        HC[i]->Init();
        Showers[i] = std::make_unique<qt_image_shower>(i);
        Showers[i]->SetFormat("GRAY");
    }
    Downloader = std::make_unique<qt_image_downloader>();

    ConnectThreads();
    ProjectStartTime = std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

void MainWindow::onStart() {
    btnStart_->setStyleSheet(btnStyle2);
    for (int i = 0; i < csize; i++) {
        CoderThreads[i].start();
        ShowThreads[i].start();
    }
    emit start_image_readers();
    emit start_image_showers();
}

void MainWindow::onStop() {
    btnStart_->setStyleSheet(btnStyle1);
    KillThread();
}

void MainWindow::onDownloadStart() {
    btnStartSeqdownload_->setStyleSheet(btnStyle2);
    std::stringstream Dir_;
    Dir_ << "Images\\" << QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss").toStdString();
    std::filesystem::create_directories(Dir_.str().c_str());
    Downloader->SetDir(Dir_.str());
    for (int i = 0; i < csize; i++) {
        Showers[i]->SetDownload(true);
    }
    DownloadThread.start();
    emit start_image_downloader();
}

void MainWindow::onDownloadStop() {
    btnStartSeqdownload_->setStyleSheet(btnStyle1);
    for (int i = 0; i < csize; i++) {
        Showers[i]->SetDownload(false);
    }
    emit stop_image_downloader();
    DownloadThread.quit(); DownloadThread.wait();
}

void MainWindow::onQuit() {
    close();
}
#include <QApplication>
#include "hwa_qt_mainwindow.h"

int main(int argc, char* argv[]) {
    std::shared_ptr<hwa_set::set_msf> gset = std::make_shared<hwa_set::set_msf>();
    gset->arg(argc, argv, true, false);
    QApplication app(argc, argv);
    hwa_qt::MainWindow w(gset);
    w.show();
    return app.exec();
}
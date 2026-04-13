#include <QApplication>
#include "hwa_pick_MainWindow.h"
using namespace hwa_pick;

int main(int argc, char* argv[]) {
    std::shared_ptr<hwa_set::set_proc> gset = std::make_shared<hwa_set::set_proc>();
    gset->arg(argc, argv, true, false);
    QApplication app(argc, argv);
    hwa_pick::MainWindow w(gset);
    w.show();
    return app.exec();
}
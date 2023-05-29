#include "mainwindow.h"
#include <QApplication>
#include <opencv2/opencv.hpp>
#include "graycode.h"
#include "findnew.h"
using namespace cv;
using namespace std;
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}

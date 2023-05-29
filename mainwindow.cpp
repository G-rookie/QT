#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qfiledialog.h>
#include <QTime>
#include <QtCore/qrandom.h>
#include <QFileDialog>
#include <QImage>
#include <QRadioButton>
#include <iostream>
#include "findnew.h"
#include "graycode.h"
#include "instance.h"
#include  "DoubleGrayCode.h"
#include "calibration.h"
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->load_left_pic, &QAction::triggered, this, &MainWindow::load_left_picAction);
    connect(ui->load_right_pic, &QAction::triggered, this, &MainWindow::load_right_picAction);
    connect(ui->load_left_point, &QAction::triggered, this, &MainWindow::load_left_pointAction);
    connect(ui->load_right_point, &QAction::triggered, this, &MainWindow::load_right_pointAction);
    connect(ui->load_double_point, &QAction::triggered, this, &MainWindow::load_double_pointAction);


    m_pOpenglWidget = new MyQOpenglWidget(ui->openGLWidget);
    m_pOpenglWidget->resize(ui->openGLWidget->width(),ui->openGLWidget->height());

    m_pOpenglWidget2 = new MyQOpenglWidget(ui->openGLWidget_4);
    m_pOpenglWidget2->resize(ui->openGLWidget_4->width(),ui->openGLWidget_4->height());

    m_pOpenglWidget3 = new MyQOpenglWidget(ui->openGLWidget_6);
    m_pOpenglWidget3->resize(ui->openGLWidget_6->width(),ui->openGLWidget_6->height());

    instance = new Instance();
}

MainWindow::~MainWindow()
{
    delete ui;
}


//加载左图片
void MainWindow::load_left_picAction()
{
    QString runPath = "D:/environment/qt-5.14.2/qtopenglwidget-master/scan/camera";//默认路径
    QString file_name = QFileDialog::getOpenFileName(this,QStringLiteral("选择文件"),runPath,"Text Files(*.txt)",nullptr,QFileDialog::DontResolveSymlinks);
    String left_file_name = file_name.toStdString();
   ifstream fin(left_file_name);
   string imgPath;
  int cnt = 0;
   while (getline(fin, imgPath))
   {
       cnt++;
       if(cnt == 1)
       {
            // 显示白光图片
            QImage img;
            QString qImgPath = QString::fromStdString(imgPath);
            img.load(qImgPath);
            img.scaled(ui->label_L->width(), ui->label_L->height(), Qt::KeepAspectRatio);
            ui->label_L->setPixmap(QPixmap::fromImage(img));
       }
   }
}

//加载右图片
void MainWindow::load_right_picAction()
{
     qDebug() << instance->right_pic_fileName;
}

//加载左点云
void MainWindow::load_left_pointAction()
{
    QString runPath = "D:/environment/qt-5.14.2/qtopenglwidget-master/restruct";//默认路径
    QString file_name = QFileDialog::getOpenFileName(this,QStringLiteral("选择文件"),runPath,"Files(*.asc)",nullptr,QFileDialog::DontResolveSymlinks);
    std::vector<QVector3D> cloud ;
    QString qfile = file_name;
    cloud= ReadVec3PointCloudASC(qfile);
    QTime startTime = QTime::currentTime();
    m_pOpenglWidget->showPointCloud(cloud);
    qDebug() << "showPointCloud =" << startTime.msecsTo(QTime::currentTime()) << "ms";

}

//加载右点云
void MainWindow::load_right_pointAction()
{
    QString runPath = "D:/environment/qt-5.14.2/qtopenglwidget-master/restruct";//默认路径
    QString file_name = QFileDialog::getOpenFileName(this,QStringLiteral("选择文件"),runPath,"Files(*.asc)",nullptr,QFileDialog::DontResolveSymlinks);
    std::vector<QVector3D> cloud ;
    QString qfile = file_name;
    cloud= ReadVec3PointCloudASC(qfile);
    //cloud = testData(180*10000);
    QTime startTime = QTime::currentTime();
    m_pOpenglWidget2->showPointCloud(cloud);
    qDebug() << "showPointCloud =" << startTime.msecsTo(QTime::currentTime()) << "ms";
}

//加载双目点云
void MainWindow::load_double_pointAction()
{
    QString runPath = "D:/environment/qt-5.14.2/qtopenglwidget-master/restruct";//默认路径
    QString file_name = QFileDialog::getOpenFileName(this,QStringLiteral("选择文件"),runPath,"Files(*.asc)",nullptr,QFileDialog::DontResolveSymlinks);
    std::vector<QVector3D> cloud ;
    QString qfile = file_name;
    cloud= ReadVec3PointCloudASC(qfile);
    QTime startTime = QTime::currentTime();
    m_pOpenglWidget3->showPointCloud(cloud);
    qDebug() << "showPointCloud =" << startTime.msecsTo(QTime::currentTime()) << "ms";

}

void MainWindow::on_reconstruct_clicked()
{
    if(ui->single_res->isChecked())
    {
        cout << "single start..." << endl;
        findNew find;
        find.match_R();
        cout << "single success" << endl;
    }
    if(ui->double_res->isChecked())
    {
        cout << "double start..." << endl;
        Calibration CamCal;
        CamCal.loadCalibrationParameters();
        cout << CamCal.leftRMS << "       " << CamCal.rightRMS << endl;
        DoubleGrayCode grayCode;
        grayCode.decodeGrayCodeandPhase(CamCal);
    }
}

void MainWindow::on_pushButton_openfile_clicked()
{
    std::vector<QVector3D> cloud ;
    //QString qfile = "E:\\test\\test.asc";
    QString qfile = "D:\\bun000_Structured.asc";
    cloud= ReadVec3PointCloudASC(qfile);
    //cloud = testData(180*10000);
    QTime startTime = QTime::currentTime();
    m_pOpenglWidget->showPointCloud(cloud);
    qDebug() << "showPointCloud =" << startTime.msecsTo(QTime::currentTime()) << "ms";
}

std::vector<QVector3D> MainWindow::ReadVec3PointCloudASC(QString path)
{
    std::vector<QVector3D> cloud;
    QFile file(path);
    if (!file.open(QFile::ReadOnly | QIODevice::Text))
    {
        qDebug() << "There is no asc file" ;
        return cloud;
    }
    QTextStream in(&file);
    QString ramData = in.readAll();
    QStringList list = ramData.split("\n");
    QStringList listline;
    cloud.resize(list.count()-1);
    for (int i = 0; i < list.count() - 1; i++)
    {
        listline = list.at(i).split(" ");
        if(listline.size()>=3)
        {
            cloud[i].setX((listline.at(0).toFloat()));
            cloud[i].setY((listline.at(1).toFloat()));
            cloud[i].setZ((listline.at(2).toFloat()));
        }
    }
    return cloud;
}

std::vector<QVector3D> MainWindow::testData(int pointsNum)
{
    std::vector<QVector3D> cloud;
    for (int i = 0; i < pointsNum; i++)
    {
        cloud.push_back(randomVec3f());
    }
    return cloud;
}

QVector3D MainWindow::randomVec3f()
{
    return QVector3D((float)(QRandomGenerator::global()->bounded(100)) / 2.0f -
            (float)(QRandomGenerator::global()->bounded(100)) / 2.0f,
            (float)(QRandomGenerator::global()->bounded(100)) / 2.0f -
            (float)(QRandomGenerator::global()->bounded(100)) / 2.0f,
            (float)(QRandomGenerator::global()->bounded(100)) / 10.0f -
            (float)(QRandomGenerator::global()->bounded(100)) / 10.0f);
}

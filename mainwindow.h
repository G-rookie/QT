#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "myqopenglwidget.h"
#include "instance.h"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_openfile_clicked();
    //加载左图片按钮
    void load_left_picAction();
    //加载右图片按钮
    void load_right_picAction();
    //加载左点云按钮
    void load_left_pointAction();
    //加载右点云按钮
    void load_right_pointAction();
    //加载双目点云按钮
    void load_double_pointAction();

    //点云重建按钮
    void on_reconstruct_clicked();


private:
    Ui::MainWindow *ui;
    MyQOpenglWidget* m_pOpenglWidget; //左点云窗口
    MyQOpenglWidget* m_pOpenglWidget2; //右点云窗口
    MyQOpenglWidget* m_pOpenglWidget3; //双目点云窗口

    Instance* instance;

    std::vector<QVector3D> ReadVec3PointCloudASC(QString path);
    std::vector<QVector3D> testData(int pointsNum);
    QVector3D randomVec3f();
};

#endif // MAINWINDOW_H

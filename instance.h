#ifndef INSTANCE_H
#define INSTANCE_H
#include<string>
#include<QString>

using namespace std;

class Instance
{
public:
    Instance();

    QString left_pic_fileName;
    QString right_pic_fileName;
    QString left_pointCloud_fileName;
    QString right_pointCloud_fileName;
    QString double_pointCloud_fileName;
};

#endif // INSTANCE_H

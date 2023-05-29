#include "DoubleGrayCode.h"
#include "math.h"
#include <opencv2/core/core_c.h>


using namespace std;
using namespace cv;
DoubleGrayCode::DoubleGrayCode()
{
}

DoubleGrayCode::~DoubleGrayCode()
{
}

/*********************************************************************************************************/

unsigned int DoubleGrayCode::binaryToGray(unsigned int num)
{
    return (num >> 1) ^ num;
}
/*********************************************************************************************************/

unsigned int DoubleGrayCode::grayToBinary(unsigned int num, unsigned numBits)
{
    for (unsigned shift = 1; shift < numBits; shift <<= 1) {
        num ^= num >> shift;
    }
    return num;
}

/*********************************************************************************************************/

unsigned int DoubleGrayCode::graytoDecimal(unsigned int x)
{
    int i;
    for (i = 0; (1 << i) < sizeof(x) * 8; i++)
    {
        x ^= x >> (1 << i);
    }
    return x;
}
/*********************************************************************************************************/
void DoubleGrayCode::generation_gray()
{
    unsigned int gray_value[GrayNbits_];
    unsigned char gray_value_bit[GrayNbits_][GrayNbits];

    for (int i = 0; i != GrayNbits_; i++)
    {
        gray_value[i] = i;
    }

    for (int i = 0; i != GrayNbits_; i++)
    {
        gray_value[i] = binaryToGray(gray_value[i]);
    }

    for (int i = 0; i != GrayNbits_; i++)
    {
        for (int j = 0; j != PrjWidth; j++)
        {
            gray_value_bit[i][j] = gray_value[i] % 2;
            if (gray_value_bit[i][j] == 1)
                gray_value_bit[i][j] = 255;
            gray_value[i] = gray_value[i] / 2;
        }
    }
    Mat normal = Mat::zeros(PrjHeight, PrjWidth, CV_8UC1);
    Mat abnormal = Mat::zeros(PrjHeight, PrjWidth, CV_8UC1);

    String left_output = "F:/";
    String right_output = "F:/";

    for (int order = 0; order != GrayNbits; order++)
    {
        for (int i = 0; i != PrjHeight; i++)
        {
            for (int j = 0; j != PrjWidth; j++)
            {
                normal.at<unsigned char>(i, j) = gray_value_bit[j][order];
                abnormal.at<unsigned char>(i, j) = 255 - gray_value_bit[j][order];
            }
        }
        imwrite(left_output + "gray" + to_string(order * 2 + 2) + ".bmp", normal);
        imwrite(right_output + "gray" + to_string(order * 2 + 3) + ".bmp", abnormal);
    }

    for (int i = 0; i != PrjHeight; i++)
    {
        for (int j = 0; j != PrjWidth; j++)
        {
            normal.at<unsigned char>(i, j) = 0;
            abnormal.at<unsigned char>(i, j) = 255;
        }
    }
    imwrite(left_output + "gray" + to_string(0) + ".bmp", normal);
    imwrite(right_output + "gray" + to_string(1) + ".bmp", abnormal);
}
/************************************************************************************************************/
void DoubleGrayCode::generation_phase()
{
    Mat normal = Mat::zeros(PrjHeight, PrjWidth, CV_8UC1);
    Mat abnormal = Mat::zeros(PrjHeight, PrjWidth, CV_8UC1);
    String left_output = "F:/";
    String right_output = "F:/";

    unsigned int gray_value[GrayNbits_];
    unsigned char gray_value_bit[GrayNbits_][GrayNbits];

    for (int i = 0; i != GrayNbits_; i++)
    {
        gray_value[i] = i;
    }

    for (int i = 0; i != GrayNbits_; i++)
    {
        gray_value[i] = binaryToGray(gray_value[i]);
    }

    for (int i = 0; i != GrayNbits_; i++)
    {
        for (int j = 0; j != 8; j++)
        {
            gray_value_bit[i][j] = gray_value[i] % 2;
            if (gray_value_bit[i][j] == 1)
                gray_value_bit[i][j] = 255;
            gray_value[i] = gray_value[i] / 2;
        }
    }


    for (int order = 0; order != GrayNbits; order++)
    {
        for (int i = 0; i != PrjHeight; i++)
        {
            for (int j = 0; j != PrjWidth; j++)
            {
                normal.at<unsigned char>(i, j) = gray_value_bit[j / 4][order];
                abnormal.at<unsigned char>(i, j) = 255 - gray_value_bit[j / 4][order];

            }
        }
        imwrite(left_output + "gray" + to_string(order * 2 + 2) + ".bmp", normal);
        imwrite(right_output + "gray" + to_string(order * 2 + 3) + ".bmp", abnormal);
    }

    for (int steps = 0; steps < 10; steps++) {
        for (int i = 0; i != PrjHeight; i++)
        {
            for (int j = 0; j != PrjWidth; j++)
            {
                normal.at<unsigned char>(i, j) = 120 + 120 * sin(2 * CV_PI * j / 8 + CV_PI / 5 * steps);
            }
        }
        char index = (char)(steps + 48);
        imwrite(left_output + "phase" + index + ".bmp", normal);
    }


    for (int i = 0; i != PrjHeight; i++)
    {
        for (int j = 0; j != PrjWidth; j++)
        {
            normal.at<unsigned char>(i, j) = 0;
            abnormal.at<unsigned char>(i, j) = 255;
        }
    }
    imwrite(left_output + "gray" + to_string(0) + ".bmp", normal);
    imwrite(right_output + "gray" + to_string(1) + ".bmp", abnormal);
}

/************************************************************************************************************/
void DoubleGrayCode::decodeGrayCode(Calibration& cameracalibration)
{
    int threshold = 5;
    ifstream fin_left("D:/environment/qt-5.14.2/qtopenglwidget-master/scan/camera/double_data/leftPic.txt");
    ifstream fin_right("D:/environment/qt-5.14.2/qtopenglwidget-master/scan/camera/double_data/leftPic.txt");
    string imgName_left;
    string imgName_right;
    imgs_left.clear();
    imgs_right.clear();
    ifstream finL("D:/environment/qt-5.14.2/qtopenglwidget-master/scan/camera/double_data/leftPic.txt");
    ifstream finR("D:/environment/qt-5.14.2/qtopenglwidget-master/scan/camera/double_data/leftPic.txt");
    string imgLName;
    string imgRName;
    while (getline(finL, imgLName) && getline(finR, imgRName))
    {
        Mat TestL = imread(imgLName, 0);
        Mat TestR = imread(imgRName, 0);
        // 极限校正
        stereoRectify(cameracalibration.left_cam_Intrinsic_matrix, cameracalibration.left_cam_dist, cameracalibration.right_cam_Intrinsic_matrix, cameracalibration.right_cam_dist, cameracalibration.imageSize, cameracalibration.R, cameracalibration.T, cameracalibration.R1, cameracalibration.R2, cameracalibration.P1, cameracalibration.P2, cameracalibration.Q, 0);
        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(cameracalibration.left_cam_Intrinsic_matrix, cameracalibration.left_cam_dist, cameracalibration.R1, cameracalibration.P1, cameracalibration.imageSize, CV_16SC2, map11, map12);
        initUndistortRectifyMap(cameracalibration.right_cam_Intrinsic_matrix, cameracalibration.right_cam_dist, cameracalibration.R2, cameracalibration.P2, cameracalibration.imageSize, CV_16SC2, map21, map22);
        Mat img1r, img2r;
        remap(TestL, img1r, map11, map12, INTER_LINEAR);
        remap(TestR, img2r, map21, map22, INTER_LINEAR);
        imgs_left.push_back(img1r);
        imgs_right.push_back(img2r);
    }

    static Mat decoded_left = Mat_<int>(2000, 2000);
    static Mat mask = Mat_<int>(2000, 2000);


    static char decimal_left[2000][2000];
    static int mask_left[2000][2000];
    static int mask_right[2000][2000];


    for (int m = 0; m < 2000; m++)
    {
        for (int n = 0; n < 2000; n++)
        {
            if (imgs_left[1].at<unsigned char>(m, n) - imgs_left[0].at<unsigned char>(m, n) < threshold)
            {
                mask_left[m][n] = 0;
            }
            else
            {
                mask_left[m][n] = 1;
            }

        }

    }


    for (int m = 0; m < 2000; m++)
    {
        for (int n = 0; n < 2000; n++)
        {
            if (mask_left[m][n] == 1)
            {
                decimal_left[m][n] = 0;
                for (int i = 1; i <= 10; i++)
                {
                    if (imgs_left[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_left[2 * i].at<unsigned char>(m, n) < -threshold)
                    {
                        decimal_left[m][n] = decimal_left[m][n] + (int)pow(2, i - 1);
                    }
                }
                decoded_left.at<int>(m, n) = graytoDecimal(decimal_left[m][n]);
            }

        }

    }
    /*******************************************************************************************************************************/
    static Mat decoded_right = Mat_<int>(2000, 2000);
    static char decimal_right[2000][2000];



    for (int m = 0; m < 2000; m++)
    {
        for (int n = 0; n < 2000; n++)
        {
            if (imgs_right[1].at<unsigned char>(m, n) - imgs_right[0].at<unsigned char>(m, n) < threshold)
            {
                mask_right[m][n] = 0;

            }
            else
            {
                mask_right[m][n] = 1;

            }
        }
    }


    for (int m = 0; m < 2000; m++)
    {
        for (int n = 0; n < 2000; n++)
        {
            if (mask_right[m][n] == 1)
            {
                decimal_right[m][n] = 0;
                for (int i = 1; i <= 10; i++)
                {
                    if (imgs_right[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_right[2 * i].at<unsigned char>(m, n) < -threshold)
                    {
                        decimal_right[m][n] = decimal_right[m][n] + (int)pow(2, i - 1);
                    }

                }
                decoded_right.at<int>(m, n) = graytoDecimal(decimal_right[m][n]);
            }
        }
    }
    /*************************************************************************************************************************************/

    static Mat shicha = Mat_<int>(2000, 2000);
    int kstart = 0;

    for (int m = 0; m < 2000; m++)
    {
        kstart = 0;
        for (int n = 0; n < 2000; n++)
        {
            for (int i = kstart; i < n; i++)
            {
                if (mask_left[m][n] == 1 && mask_right[m][i] == 1 && decoded_left.at<int>(m, n) == decoded_right.at<int>(m, i))
                {
                    shicha.at<int>(m, n) = n - i;
                    kstart = i;
                    break;
                }
                else {
                    shicha.at<int>(m, n) = 1;
                }
            }
        }


    }

    /**********************************************************************************************/
    Point left_camera_coordinates;
    Mat point_xyz;
    ofstream fs_point_txt;
    fs_point_txt.open("E:/2021.3.17gray.txt");
    reprojectImageTo3D(shicha, point_xyz, cameracalibration.Q, false, -1);

    for (int i = 0; i < 2000; i++)
    {
        for (int j = 0; j < 2000; j++)
        {
            if (1 == 1)
            {
                left_camera_coordinates.x = i; left_camera_coordinates.y = j;

                fs_point_txt << point_xyz.at<Vec3f>(left_camera_coordinates)[0] << ' ' << point_xyz.at<Vec3f>(left_camera_coordinates)[1] << ' ' << point_xyz.at<Vec3f>(left_camera_coordinates)[2] << ' ' << endl;
            }
        }
    }
}
/************************************************************************************************************/


void DoubleGrayCode::decodeGrayCodeandPhase(Calibration& cameracalibration)
{

    int threshold = 10;
    string imgName_left;
    string imgName_right;
    static Mat decoded_right_low = Mat_<int>(2000, 2000);
    static unsigned int decimal_right_low[2000][2000];
    static Mat decoded_right_high = Mat_<int>(2000, 2000);
    static unsigned int decimal_right_high[2000][2000];
    static Mat decoded_left_phase = Mat_<double>(2000, 2000);
    static Mat decoded_right_phase = Mat_<double>(2000, 2000);
    static double phase_temp[2000][2000];
    static Mat decoded_left_low = Mat_<int>(2000, 2000);
    static Mat decoded_left_high = Mat_<int>(2000, 2000);
    static unsigned int decimal_left_low[2000][2000];
    static unsigned int decimal_left_high[2000][2000];
    static int mask_left[2000][2000];
    static int mask_right[2000][2000];
    Mat shicha = Mat_<float>(2000, 2000);
    imgs_left.clear();
    imgs_right.clear();
    ifstream finL("D:/environment/qt-5.14.2/qtopenglwidget-master/scan/camera/double_data/leftPic.txt");
    ifstream finR("D:/environment/qt-5.14.2/qtopenglwidget-master/scan/camera/double_data/rightPic.txt");
    string imgLName;
    string imgRName;
    //极限校正
    stereoRectify(cameracalibration.left_cam_Intrinsic_matrix, cameracalibration.left_cam_dist, cameracalibration.right_cam_Intrinsic_matrix, cameracalibration.right_cam_dist, cameracalibration.imageSize, cameracalibration.R, cameracalibration.T, cameracalibration.R1, cameracalibration.R2, cameracalibration.P1, cameracalibration.P2, cameracalibration.Q, 0);
    Mat map11, map12, map21, map22;
    initUndistortRectifyMap(cameracalibration.left_cam_Intrinsic_matrix, cameracalibration.left_cam_dist, cameracalibration.R1, cameracalibration.P1, cameracalibration.imageSize, CV_16SC2, map11, map12);
    initUndistortRectifyMap(cameracalibration.right_cam_Intrinsic_matrix, cameracalibration.right_cam_dist, cameracalibration.R2, cameracalibration.P2, cameracalibration.imageSize, CV_16SC2, map21, map22);
    while (getline(finL, imgLName) && getline(finR, imgRName))  
    {
        cout << imgLName  << "    " << imgRName<< endl;
        Mat TestL = imread(imgLName, 0);
        Mat TestR = imread(imgRName, 0);
        Mat img1r, img2r;
        remap(TestL, img1r, map11, map12, INTER_LINEAR);
        remap(TestR, img2r, map21, map22, INTER_LINEAR);
        imwrite(imgLName, img1r);
        imwrite(imgRName, img2r);
        imgs_left.push_back(img1r);
        imgs_right.push_back(img2r);
    }

    for (int m = 0; m < 2000; m++)
    {
        for (int n = 0; n < 2000; n++)
        {
            if (imgs_left[1].at<unsigned char>(m, n) - imgs_left[0].at<unsigned char>(m, n) < threshold)
            {
                mask_left[m][n] = 0;
            }
            else
            {
                mask_left[m][n] = 1;
            }
        }
    }


    for (int m = 0; m < 2000; m++)
    {
        for (int n = 0; n < 2000; n++)
        {
            if (mask_left[m][n] == 1)
            {
                decimal_left_high[m][n] = 0;
                for (int i = 1; i <= 8; i++)
                {
                    if (imgs_left[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_left[2 * i].at<unsigned char>(m, n) < -threshold)
                    {
                        decimal_left_high[m][n] = decimal_left_high[m][n] + (int)pow(2, i - 1);
                    }

                }
                decoded_left_high.at<int>(m, n) = graytoDecimal(decimal_left_high[m][n]);
                if (decoded_left_high.at<int>(m, n) % 2 == 0)  decoded_left_high.at<int>(m, n) = decoded_left_high.at<int>(m, n) / 2;
                else decoded_left_high.at<int>(m, n) = decoded_left_high.at<int>(m, n) / 2 + 1;
            }

        }

    }

    for (int m = 0; m < 2000; m++)
    {
        for (int n = 0; n < 2000; n++)
        {
            if (1 == 1)
            {
                decimal_left_low[m][n] = 0;
                for (int i = 2; i <= 8; i++)
                {
                    if (imgs_left[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_left[2 * i].at<unsigned char>(m, n) < -threshold)
                    {
                        decimal_left_low[m][n] = decimal_left_low[m][n] + (int)pow(2, i - 2);
                    }
                }
                decoded_left_low.at<int>(m, n) = graytoDecimal(decimal_left_low[m][n]);

            }
        }
    }


    for (int m = 0; m < 2000; m++)
    {
        for (int n = 0; n < 2000; n++)
        {
            if (imgs_right[1].at<unsigned char>(m, n) - imgs_right[0].at<unsigned char>(m, n) < threshold)
            {
                mask_right[m][n] = 0;
            }
            else
            {
                mask_right[m][n] = 1;
            }
        }

    }

    for (int m = 0; m < 2000; m++)
    {
        for (int n = 0; n < 2000; n++)
        {
            if (mask_right[m][n] == 1)
            {
                decimal_right_high[m][n] = 0;
                for (int i = 1; i <= 8; i++)
                {
                    if (imgs_right[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_right[2 * i].at<unsigned char>(m, n) < -threshold)
                    {
                        decimal_right_high[m][n] = decimal_right_high[m][n] + (int)pow(2, i - 1);
                    }
                }
                decoded_right_high.at<int>(m, n) = graytoDecimal(decimal_right_high[m][n]);
                if (decoded_right_high.at<int>(m, n) % 2 == 0)  decoded_right_high.at<int>(m, n) = decoded_right_high.at<int>(m, n) / 2;
                else decoded_right_high.at<int>(m, n) = decoded_right_high.at<int>(m, n) / 2 + 1;
            }
        }
    }

    for (int m = 0; m < 2000; m++)
    {
        for (int n = 0; n < 2000; n++)
        {
            if (1 == 1)
            {
                decimal_right_low[m][n] = 0;
                for (int i = 2; i <= 8; i++)
                {
                    if (imgs_right[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_right[2 * i].at<unsigned char>(m, n) < -threshold)
                    {
                        decimal_right_low[m][n] = decimal_right_low[m][n] + (int)pow(2, i - 2);
                    }
                }
                decoded_right_low.at<int>(m, n) = graytoDecimal(decimal_right_low[m][n]);

            }
        }

    }


    for (int m = 0; m < 2000; m++)
    {

        for (int n = 0; n < 2000; n++)
        {
            if (mask_left[m][n] == 1) {
                //  phase_temp[m][n] = atan2(imgs_left[19].at<unsigned char>(m, n)-imgs_left[21].at<unsigned char>(m, n),imgs_left[18].at<unsigned char>(m, n)-imgs_left[20].at<unsigned char>(m, n));
                phase_temp[m][n] = -atan2((imgs_left[18].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 0) + imgs_left[19].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 1) + imgs_left[20].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 2) + imgs_left[21].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 3) + imgs_left[22].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 4) + imgs_left[23].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 5) + imgs_left[24].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 6) + imgs_left[25].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 7)), (imgs_left[18].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 0) + imgs_left[19].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 1) + imgs_left[20].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 2) + imgs_left[21].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 3) + imgs_left[22].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 4) + imgs_left[23].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 5) + imgs_left[24].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 6) + imgs_left[25].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 7)));

                if (phase_temp[m][n] < -CV_PI / 2) {
                    decoded_left_phase.at<double>(m, n) = 2 * CV_PI * decoded_left_high.at<int>(m, n) + phase_temp[m][n];
                }
                else if (phase_temp[m][n] > CV_PI / 2) {
                    decoded_left_phase.at<double>(m, n) = 2 * CV_PI * decoded_left_high.at<int>(m, n) + phase_temp[m][n] - 2 * CV_PI;
                }
                else {
                    decoded_left_phase.at<double>(m, n) = 2 * CV_PI * decoded_left_low.at<int>(m, n) + phase_temp[m][n];
                }
            }
            else decoded_left_phase.at<double>(m, n) = -100;


        }
    }


    for (int m = 0; m < 2000; m++)
    {

        for (int n = 0; n < 2000; n++)
        {
            if (mask_right[m][n] == 1)
            {
                // phase_temp[m][n] = atan2(imgs_right[19].at<unsigned char>(m, n)-imgs_right[21].at<unsigned char>(m, n),imgs_right[18].at<unsigned char>(m, n)-imgs_right[20].at<unsigned char>(m, n));
                phase_temp[m][n] = -atan2((imgs_right[18].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 0) + imgs_right[19].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 1) + imgs_right[20].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 2) + imgs_right[21].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 3) + imgs_right[22].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 4) + imgs_right[23].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 5) + imgs_right[24].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 6) + imgs_right[25].at<unsigned char>(m, n) * sin(0.25 * CV_PI * 7)), (imgs_right[18].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 0) + imgs_right[19].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 1) + imgs_right[20].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 2) + imgs_right[21].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 3) + imgs_right[22].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 4) + imgs_right[23].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 5) + imgs_right[24].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 6) + imgs_right[25].at<unsigned char>(m, n) * cos(0.25 * CV_PI * 7)));

                if (phase_temp[m][n] < -CV_PI / 2) {
                    decoded_right_phase.at<double>(m, n) = 2 * CV_PI * decoded_right_high.at<int>(m, n) + phase_temp[m][n];
                }
                else if (phase_temp[m][n] > CV_PI / 2) {
                    decoded_right_phase.at<double>(m, n) = 2 * CV_PI * decoded_right_high.at<int>(m, n) + phase_temp[m][n] - 2 * CV_PI;
                }
                else {
                    decoded_right_phase.at<double>(m, n) = 2 * CV_PI * decoded_right_low.at<int>(m, n) + phase_temp[m][n];
                }
            }
            else decoded_right_phase.at<double>(m, n) = -100;
        }
    }

    imwrite("D:/environment/qt-5.14.2/qtopenglwidget-master/result/doubleLeft.jpg", decoded_left_phase * 255 / 750);
    imwrite("D:/environment/qt-5.14.2/qtopenglwidget-master/result/doubleRight.jpg", decoded_right_phase * 255 / 750);



    for (int m = 1; m < 2000 - 1; m++)
    {
        for (int n = 1; n < 2000 - 1; n++)
        {
            shicha.at<float>(m, n) = 0.5;
            if (mask_left[m][n] == 1) {
                for (int i = 1; i <= n - 1; i++)
                {
                    // (m,n) (m, i)
                    if (mask_right[m][i] && mask_right[m][i - 1] && mask_right[m][i + 1] > 0 && decoded_left_phase.at<double>(m, n) - decoded_right_phase.at<double>(m, i - 1) > 0 && decoded_left_phase.at<double>(m, n) - decoded_right_phase.at<double>(m, i + 1) < 0)
                    {
                        if (fabs(decoded_left_phase.at<double>(m, n) - decoded_right_phase.at<double>(m, i)) < 0.8 && fabs(decoded_left_phase.at<double>(m, n) - decoded_right_phase.at<double>(m, i - 1)) < 0.8 && fabs(decoded_left_phase.at<double>(m, n) - decoded_right_phase.at<double>(m, i + 1)) < 0.8) {
                            //二次函数拟合
                            Mat A = (Mat_<double>(3, 3) << (i - 1) * (i - 1), (i - 1), 1, (i) * (i), (i), 1, (i + 1) * (i + 1), (i + 1), 1);
                            Mat B = (Mat_<double>(3, 1) << decoded_right_phase.at<double>(m, i - 1), decoded_right_phase.at<double>(m, i), decoded_right_phase.at<double>(m, i + 1));
                            Mat C;
                            solve(A, B, C, CV_LU);
                            double index_left = (-C.at<double>(1, 0) - sqrt(C.at<double>(1, 0) * C.at<double>(1, 0) - 4 * C.at<double>(0, 0) * (C.at<double>(2, 0) - decoded_left_phase.at<double>(m, n)))) / 2 / C.at<double>(0, 0);
                            double index_right = (-C.at<double>(1, 0) + sqrt(C.at<double>(1, 0) * C.at<double>(1, 0) - 4 * C.at<double>(0, 0) * (C.at<double>(2, 0) - decoded_left_phase.at<double>(m, n)))) / 2 / C.at<double>(0, 0);
                            if ((index_left > i - 1) && (index_left < i + 1)) {
                                shicha.at<float>(m, n) = (float)(n - index_left);
                                break;
                            }
                            else {
                                shicha.at<float>(m, n) = (float)(n - index_right);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }


    Mat disparity8U;
    shicha.convertTo(disparity8U, CV_8U, 255.0 / 750);
    imwrite("D:/environment/qt-5.14.2/qtopenglwidget-master/result/shicha.jpg", disparity8U);
    Point left_camera_coordinates;
    Mat point_xyz;
    ofstream fs_point_txt;
    fs_point_txt.open("D:/environment/qt-5.14.2/qtopenglwidget-master/result/pointcloud.txt");
    reprojectImageTo3D(shicha, point_xyz, cameracalibration.Q, false, -1);
    for (int i = 1; i < 2000 - 1; i++)
    {

        for (int j = 1; j < 2000 - 1; j++)
        {
            if (fabs(shicha.at<float>(j, i - 1) - shicha.at<float>(j, i)) < 20 && fabs(shicha.at<float>(j, i + 1) - shicha.at<float>(j, i)) < 20 && fabs(shicha.at<float>(j + 1, i) - shicha.at<float>(j, i)) < 20 && fabs(shicha.at<float>(j - 1, i) - shicha.at<float>(j, i)) < 20 && shicha.at<float>(j, i) > 0.6) {

                left_camera_coordinates.x = i; left_camera_coordinates.y = j;
                fs_point_txt << point_xyz.at<Vec3f>(left_camera_coordinates)[0] << ' ' << point_xyz.at<Vec3f>(left_camera_coordinates)[1] << ' ' << point_xyz.at<Vec3f>(left_camera_coordinates)[2] << ' ' << '\n';

            }
        }
    }
    fs_point_txt.close();
}

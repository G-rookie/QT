#pragma once
#include <opencv2\opencv.hpp>
#include<iostream>
#include <fstream>
#include "calibration.h"
#include <vector>

using namespace std;

#define GrayNbits 8
#define GrayNbits_ 228
#define PrjHeight 1140
#define PrjWidth  912

class DoubleGrayCode
{
public:
    DoubleGrayCode();
    ~DoubleGrayCode();
	unsigned int binaryToGray(unsigned int num);
	unsigned int grayToBinary(unsigned int num, unsigned numBits);
	void generation_gray();
	void generation_phase();
	unsigned int graytoDecimal(unsigned int x);
    void decodeGrayCode(Calibration& cameracalibration);
    void decodeGrayCodeandPhase(Calibration& cameracalibration);
	vector<cv::Mat>imgs_left;
	vector<cv::Mat>imgs_right;
};


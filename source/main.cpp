#include <opencv2/opencv.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>

#include "math_headers.h"

using namespace cv;

int HUGE_NUM = 10000;

void show_point(cv::Mat* mat, int i, int j);

int main(int argc, char** argv) 
{
	cv::Mat src, hsv, mask;

	src = cv::imread("../../image/Sim0000.png");
	cv::cvtColor(src, hsv, CV_BGR2HSV);

	//int offset = 50;
	int offset_low = 160;
	int offset_high = -140;

	//inRange(hsv, Scalar(31-offset, 165 - offset, 130 - offset), Scalar(51 + offset, 185 + offset, 150 + offset), mask);
	//inRange(hsv, Scalar(160 - offset_low, 233 - offset_low, 214 - offset_low), Scalar(160+ offset_high, 233+ offset_high, 214+ offset_high), mask);
	inRange(hsv, Scalar(82, 82, 82), Scalar(255, 255, 255), mask);

	// search corner
	EigenVector2i CornerTL, CornerTR, CornerBL, CornerBR;
	ScalarType LNormMIN = 10000.0;	ScalarType RNormMIN = 10000.0;
	ScalarType LNormMAX = 0.0;		ScalarType RNormMAX = 0.0;

	CornerTL[0] = HUGE_NUM; CornerTL[1] = HUGE_NUM;
	CornerTR[0] = HUGE_NUM; CornerTR[1] = HUGE_NUM;
	CornerBR[0] = 0;        CornerBR[1] = 0;
	CornerBL[0] = 0;        CornerBL[1] = 0;

	for (int i = 0; i < src.rows; i++) // rows
	{
		for (int j = 0; j < src.cols; j++) // cols
		{
			Vec3b bgr = src.at<Vec3b>(i,j);

			// skip judge
			bool skip_flag = true;

			if (bgr != cv::Vec3b(0, 0, 0)) { skip_flag = false; }

			// search corner
			if (skip_flag == false)
			{
				// comp norm
				int cols = src.cols;
				ScalarType LNorm = sqrt( (i*i) + (j*j));
				ScalarType RNorm = sqrt((i*i) + ((j-cols)*(j-cols)));


				if (LNorm < LNormMIN)
				{
					LNormMIN = LNorm;
					CornerTL[0] = i;	CornerTL[1] = j;
				}
				else if (LNorm > LNormMAX)
				{
					LNormMAX = LNorm;
					CornerBR[0] = i;	CornerBR[1] = j;
				}

				if (RNorm < RNormMIN)
				{
					RNormMIN = RNorm;
					CornerTR[0] = i;	CornerTR[1] = j;
				}
				else if (RNorm > RNormMAX)
				{
					RNormMAX = RNorm;
					CornerBL[0] = i;	CornerBL[1] = j;
				}
			}
		}
	}

	//CornerTR[0] = CornerTL[0];	CornerTR[1] = CornerBR[1];
	//CornerBL[0] = CornerBR[0];  CornerBL[1] = CornerTL[1];


	std::cout << "CornerTL" << std::endl;	std::cout << CornerTL << std::endl;
	std::cout << "CornerTR" << std::endl;	std::cout << CornerTR << std::endl;
	std::cout << "CornerBL" << std::endl;	std::cout << CornerBL << std::endl;
	std::cout << "CornerBR" << std::endl;	std::cout << CornerBR << std::endl;

	
	show_point(&src, CornerTL[0], CornerTL[1]);
	show_point(&src, CornerTR[0], CornerTR[1]);
	show_point(&src, CornerBL[0], CornerBL[1]);
	show_point(&src, CornerBR[0], CornerBR[1]);


	cv::namedWindow("src", cv::WINDOW_AUTOSIZE);
	imshow("src", src);
	cv::namedWindow("mask", cv::WINDOW_AUTOSIZE);
	imshow("mask", mask);

	waitKey(0);


	//-----------------------------------------------------------------------------

	//Rect roi(0,0, 1857,1057);
	//cv::Mat src, dst, mask, bg, fg;

	//// input image as grey scale
	//src = cv::imread("../../image/shape0.png");
	//// if failed return -1
	//if (src.empty())
	//{
	//	std::cerr << "Failed to open image file." << std::endl;
	//	return -1;
	//}

	//cv::grabCut(src, dst, roi, bg, fg, 1, GC_INIT_WITH_RECT);
	//cv::compare(dst, GC_PR_FGD, mask, CMP_EQ);

	//src.copyTo(dst, mask);

	//cv::namedWindow("src", cv::WINDOW_AUTOSIZE);
	//imshow("src", src);
	//cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
	//imshow("dst", dst);
	//waitKey();


	//-----------------------------------------------------------------------------

	//// input image as grey scale
	//cv::Mat src = cv::imread("../../image/shape0.png", cv::IMREAD_GRAYSCALE);

	//// if failed return -1
	//if (src.empty())
	//{
	//	std::cerr << "Failed to open image file." << std::endl;
	//	return -1;
	//}

	//// binalize
	//cv::Mat bin;
	//cv::threshold(src, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

	//// make image for label
	//cv::Mat labelImage(src.size(), CV_32S);

	//// labeling
	//int nLabels = cv::connectedComponents(bin, labelImage, 8);

	//// decide color for rendering
	//std::vector<cv::Vec3b> colors(nLabels);
	//colors[0] = cv::Vec3b(0, 0, 0);
	//for (int label = 0; label < nLabels; ++label)
	//{
	//	colors[label] = cv::Vec3b((rand()&255),(rand()&255),(rand()&255));
	//}

	//// render result
	//cv::Mat dst(src.size(), CV_8UC3);
	//for (int y = 0; y < dst.rows; ++y)
	//{
	//	for (int x = 0; x < dst.cols; ++x)
	//	{
	//		int label = labelImage.at<int>(y, x);
	//		cv::Vec3b &pixel = dst.at<cv::Vec3b>(y, x);
	//		pixel = colors[label];
	//	}
	//}
	//cv::namedWindow("Source", cv::WINDOW_AUTOSIZE);
	//cv::imshow("Source", src);

	//cv::namedWindow("Connected Components", cv::WINDOW_AUTOSIZE);
	//cv::imshow("Connected Components", dst);

	//cv::waitKey(0 );

	//-----------------------------------------------------------------------------


	//Mat im = imread("../../image/Desert.jpg");

	//if (im.data == NULL) 
	//{	
	//	std::cout << " imput  fail" << std::endl;
	//	return -1; 
	//}

	//// make window
	//namedWindow("desert", CV_WINDOW_AUTOSIZE);
	//imshow("desert", im);

	//
	//waitKey(0);
	//destroyWindow("desert");

	//imwrite("../../image/output.bmp", im);

	return 0;
}

void show_point(cv::Mat* mat,int i, int j)
{
	Vec3b color_bgr = (0,0,255);
	int size = 3;

	for (int height = i-size; height < i+size; height++)
	{
		Vec3b* ptr = mat->ptr<Vec3b>(height);

		for (int width = j-size; width < j+size; width++)
		{
			ptr[width] = color_bgr;
		}
	}
}
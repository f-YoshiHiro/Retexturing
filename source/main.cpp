#include <opencv2/opencv.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>

#include "math_headers.h"

// ----------name space---------- //
using namespace cv;

// -----------global---------- //
int HUGE_NUM = 10000;

cv::Mat src, hsv, mask;

EigenVector2i CornerTL, CornerTR, CornerBL, CornerBR;
EigenMatrixXi V, T;

std::vector<Point> vertVec;


// ----------functions---------- //
void show_point(cv::Mat* mat, int i, int j);

void gen_mesh(EigenMatrixXi* Pmat, EigenMatrixXi* Imat, int NUM_Y, int NUM_X);

void draw_tri(Point p1, Point p2, Point p3, Scalar color=(Vec3b(255,0,255)), int thickness=2);
void draw_mesh(EigenMatrixXi* V, EigenMatrixXi* T, int NUM_Y, int NUM_X);

// ----------main---------- //
int main(int argc, char** argv) 
{
	//cv::Mat src, hsv, mask;

	src = cv::imread("../../image/Sim0000.png");
	cv::cvtColor(src, hsv, CV_BGR2HSV);

	//int offset = 50;
	int offset_low = 160;
	int offset_high = -140;

	//inRange(hsv, Scalar(31-offset, 165 - offset, 130 - offset), Scalar(51 + offset, 185 + offset, 150 + offset), mask);
	//inRange(hsv, Scalar(160 - offset_low, 233 - offset_low, 214 - offset_low), Scalar(160+ offset_high, 233+ offset_high, 214+ offset_high), mask);
	inRange(hsv, Scalar(82, 82, 82), Scalar(255, 255, 255), mask);

	// search corner
	//EigenVector2i CornerTL, CornerTR, CornerBL, CornerBR;
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

	// generate mesh
	gen_mesh(&V, &T, 5, 5);

	//show_point(&src, CornerTL[0], CornerTL[1]);
	//show_point(&src, CornerTR[0], CornerTR[1]);
	//show_point(&src, CornerBL[0], CornerBL[1]);
	//show_point(&src, CornerBR[0], CornerBR[1]);

	Point p1, p2;
	p1.x = CornerTL[1]; p1.y = CornerTL[0];
	p2.x = CornerBR[1]; p2.y = CornerBR[0];

	// line test
	//line(src, p1, p2, Scalar(Vec3b(255, 0, 255)), 2, 8, 0);

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

// Pmat:pos mat
// Imat:Tri index mat
// i:row #vert
// j:col #vert
void gen_mesh(EigenMatrixXi* Pmat, EigenMatrixXi* Imat, int NUM_Y, int NUM_X)
{
	Pmat->resize(NUM_Y*NUM_X,2);
	Pmat->setZero();

	Imat->resize(2*(NUM_Y-1)*(NUM_X-1), 3);
	Imat->setZero();

	//std::cout << "V.rows:" << Pmat->rows() << std::endl;
	//std::cout << "T.rows:" << Imat->rows() << std::endl;

	int dely, delx;
	dely = (double)(CornerBL[0] - CornerTL[0])/(double)(NUM_Y-1.0);
	delx = (double)(CornerTR[1] - CornerTL[1])/(double)(NUM_X-1.0);

	std::cout << "dely:" << dely << std::endl;
	std::cout << "delx:" << delx << std::endl;

	// generate vert
	for (int y = 0; y < NUM_Y; y++)
	{
		for (int x = 0; x < NUM_X; x++)
		{
			//std::cout << "(" << y << "," << x << ")" << std::endl;

			(*Pmat)((NUM_X*y)+x, 0) = CornerTL[0] + (y*dely); //y
			(*Pmat)((NUM_X*y)+x, 1) = CornerTL[1] + (x*delx); //x

			//show_point(&src, CornerTL[0] + (y*dely), CornerTL[1] + (x*delx));
		}
	}

	// generate tri
	for (int y = 0; y < NUM_Y-1; y++)
	{
		for (int x = 0; x < NUM_X-1; x++)
		{
			int triIndex;

			triIndex = ( (((NUM_X - 1)*y) + x) * 2 )+0;
			(*Imat)(triIndex, 0) = (NUM_X*y)     + x;
			(*Imat)(triIndex, 1) = (NUM_X*y)     + (x+1);
			(*Imat)(triIndex, 2) = (NUM_X*(y+1)) + (x+1);

			triIndex = ((((NUM_X - 1)*y) + x) * 2) + 1;
			(*Imat)(triIndex, 0) = (NUM_X*y)     + x;
			(*Imat)(triIndex, 2) = (NUM_X*(y+1)) + (x + 1);
			(*Imat)(triIndex, 1) = (NUM_X*(y+1)) + x;
		}
	}

	// show vert
	for (int y = 0; y < NUM_Y; y++)
	{
		for (int x = 0; x < NUM_X; x++)
		{
			show_point(&src, (*Pmat)((NUM_X*y) + x, 0), (*Pmat)((NUM_X*y) + x, 1));
		}
	}

	// show Tri
	for (int i = 0; i < Imat->rows(); i++)
	{
		Point p1, p2, p3;
		int   i1, i2, i3;

		i1 = (*Imat)(i, 0);
		i2 = (*Imat)(i, 1);
		i3 = (*Imat)(i, 2);

		p1.x = (*Pmat)(i1, 1);	p1.y = (*Pmat)(i1, 0);
		p2.x = (*Pmat)(i2, 1);	p2.y = (*Pmat)(i2, 0);
		p3.x = (*Pmat)(i3, 1);	p3.y = (*Pmat)(i3, 0);

		draw_tri(p1, p2, p3);
	}



	// set


	/*for (int height = 0; height < i; height++)
	{
		for (int width = 0; width < j; width++)
		{
			if(height==0 && width==0)
			{ 
				(*Pmat)(0, 0) = CornerTL[0];
				(*Pmat)(0, 1) = CornerTL[1];
			}
			else
			{
				double iwight = (double)width / (double)j;
				double jwight = (double)height / (double)i;

				EigenVector2i ivec, jvec;

				ivec[0] = ((1.0 - iwight)*(double)Lvec[0]) + (iwight*(double)Lvec[0]);
				ivec[1] = ((1.0 - iwight)*(double)Lvec[1]) + (iwight*(double)Lvec[1]);

				jvec[0] = ((1.0 - jwight)*(double)Tvec[0]) + (jwight*(double)Bvec[0]);
				jvec[1] = ((1.0 - jwight)*(double)Tvec[1]) + (jwight*(double)Bvec[1]);

				(*Pmat)((height*j) + width, 0) = ;
				(*Pmat)((height*j) + width, 1) = ;
			}

		}
	}*/
}


// Draw Triangle 
void draw_tri(Point p1, Point p2, Point p3, Scalar color, int thickness)
{
	line(src, p1, p2, color, thickness, 8, 0);
	line(src, p2, p3, color, thickness, 8, 0);
	line(src, p3, p1, color, thickness, 8, 0);
}

// Draw Mesh
void draw_mesh(EigenMatrixXi* V, EigenMatrixXi* T, int NUM_Y, int NUM_X)
{
	for (int y = 0; y < NUM_Y - 1; y++)
	{
		for (int x = 0; x < NUM_X - 1; x++)
		{
			int triIndex;
			Point p1, p2, p3, p4;

			triIndex = ((((NUM_X - 1)*y) + x) * 2) + 0;
			

			triIndex = ((((NUM_X - 1)*y) + x) * 2) + 1;
			
		}
	}
}
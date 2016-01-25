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
Mat mat00H, mat19H;
Mat Gmat00H;

int HUGE_NUM = 10000;

int num_X = 5;
int num_Y = 5;

std::vector<Point> vertVec;


// ----------functions---------- //
void load_images();

void search_corners(Mat* mat, Point* TL, Point* TR, Point* BL, Point* BR);

void gen_mesh(EigenMatrixXi* V, EigenMatrixXi* T, Point TL, Point TR, Point BL, Point BR, int NUM_Y, int NUM_X);

void draw_vert(Mat* mat, EigenMatrixXi* V, int radius=3, Scalar color = (Vec3b(255, 0, 0)));
void draw_tri(Mat* mat, Point p1, Point p2, Point p3, Scalar color=(Vec3b(255,0,255)), int thickness=1);
void draw_mesh(Mat* mat, EigenMatrixXi* V, EigenMatrixXi* T, int NUM_Y, int NUM_X);

void compute_GlobalTransformation(const Mat* mat_old, const Mat* mat_new, EigenVector2i d);

unsigned char compute_GDerivativeX(const Mat* mat, int y, int x, int size = 3);
unsigned char compute_GDerivativeY(const Mat* mat, int y, int x, int size = 3);
unsigned char compute_GDerivativeT(const Mat* mat_old, const Mat* mat_new, int y, int x, int size = 3);

Vec3b compute_DerivativeX(const Mat* mat, int y, int x, int size = 3);
Vec3b compute_DerivativeY(const Mat* mat, int y, int x, int size = 3);
Vec3b compute_DerivativeT(const Mat* mat_old, const Mat* mat_new, int y, int x, int size = 3);

// ----------main---------- //
int main(int argc, char** argv) 
{
	// variable
	Point TL, TR, BL, BR;	// corners
	EigenMatrixXi V;	// vertices
	EigenMatrixXi T;	// triangles

	// load image
	load_images();

	// search corner
	search_corners(&Gmat00H, &TL, &TR, &BL, &BR);

	// generate mesh
	gen_mesh(&V, &T, TL, TR, BL, BR, num_Y, num_X);

	// draw
	draw_mesh(&Gmat00H, &V, &T, num_Y, num_X);
	draw_vert(&Gmat00H, &V);

	cv::namedWindow("src", cv::WINDOW_AUTOSIZE);
	imshow("src", mat00H);

	cv::namedWindow("gray", cv::WINDOW_AUTOSIZE);
	imshow("gray", Gmat00H);

	{
		//// test del
		//Mat matDelY(Size(Gmat00H.cols, Gmat00H.rows), CV_8UC1);

		//for (int y = 1; y < Gmat00H.rows-1; y++)
		//{
		//	for (int x = 1; x < Gmat00H.cols-1; x++)
		//	{
		//		//std::cout << y << "," << x << std::endl;
		//		matDelY.at<unsigned char>(y, x) = compute_GDerivativeY(&Gmat00H, y, x);
		//	}
		//}
		//cv::namedWindow("dely", cv::WINDOW_AUTOSIZE);
		//imshow("dely", matDelY);

		//Mat matDelX(Size(Gmat00H.cols, Gmat00H.rows), CV_8UC1);

		//for (int y = 1; y < Gmat00H.rows - 1; y++)
		//{
		//	for (int x = 1; x < Gmat00H.cols - 1; x++)
		//	{
		//		//std::cout << y << "," << x << std::endl;
		//		matDelX.at<unsigned char>(y, x) = compute_GDerivativeX(&Gmat00H, y, x);
		//	}
		//}
		//cv::namedWindow("delx", cv::WINDOW_AUTOSIZE);
		//imshow("delx", matDelX);
	}

	waitKey(0);

	return 0;
}

#pragma region functions
// -------------------------------------------------- //
// Load Images
// -------------------------------------------------- //
void load_images()
{
	mat00H = cv::imread("../../image/Sim0000H.png");
	mat19H = cv::imread("../../image/Sim0019H.png");

	Gmat00H = cv::imread("../../image/Sim0000H.png", IMREAD_GRAYSCALE);
}

// -------------------------------------------------- //
// Search Corners
//
// mat:image
// TL: top left
// TR: top right
// BL: bottom left
// BR: bottom right
// -------------------------------------------------- //
void search_corners(Mat* mat, Point* TL, Point* TR, Point* BL, Point* BR)
{
	// init
	TL->y = HUGE_NUM;	TL->x = HUGE_NUM;
	TR->y = HUGE_NUM;	TR->x = HUGE_NUM;
	BR->y = 0;			BR->x = 0;
	BL->y = 0;			BL->x = 0;

	ScalarType LNormMIN = 10000.0;	ScalarType RNormMIN = 10000.0;
	ScalarType LNormMAX = 0.0;		ScalarType RNormMAX = 0.0;

	for (int y = 0; y < mat->rows; y++) // rows
	{
		for (int x = 0; x < mat->cols; x++) // cols
		{
			unsigned char pixel = mat->at<unsigned char>(y, x);

			// skip judge
			bool skip_flag = true;

			if (pixel != (unsigned char)0) { skip_flag = false; }

			// search corner
			if (skip_flag == false)
			{
				// comp norm
				int cols = mat->cols;
				ScalarType LNorm = sqrt((y*y) + (x*x));
				ScalarType RNorm = sqrt((y*y) + ((x - cols)*(x - cols)));


				if (LNorm < LNormMIN)
				{
					LNormMIN = LNorm;
					TL->y = y;	TL->x = x;
				}
				else if (LNorm > LNormMAX)
				{
					LNormMAX = LNorm;
					BR->y = y;	BR->x = x;
				}

				if (RNorm < RNormMIN)
				{
					RNormMIN = RNorm;
					TR->y = y;	TR->x = x;
				}
				else if (RNorm > RNormMAX)
				{
					RNormMAX = RNorm;
					BL->y = y;	BL->x = x;
				}
			}
		}
	}
	std::cout << "Corners" << std::endl;
	std::cout << "TL:" << TL->y << "," << TL->x <<std::endl;
	std::cout << "TR:" << TR->y << "," << TR->x <<std::endl;
	std::cout << "BL:" << BL->y << "," << BL->x <<std::endl;
	std::cout << "BR:" << BR->y << "," << BR->x <<std::endl;

	int rudius = 3;
	//circle(*mat, *TL, rudius, (Vec3b(255, 0, 0)), -1, 8, 0);
	//circle(*mat, *TR, rudius, (Vec3b(255, 0, 0)), -1, 8, 0);
	//circle(*mat, *BL, rudius, (Vec3b(255, 0, 0)), -1, 8, 0);
	//circle(*mat, *BR, rudius, (Vec3b(255, 0, 0)), -1, 8, 0);
}

// -------------------------------------------------- //
// Generate Mesh
//
// Pmat:pos mat
// Imat:Tri index mat
// i:row #vert
// j:col #vert
// -------------------------------------------------- //
void gen_mesh(EigenMatrixXi* V, EigenMatrixXi* T, Point TL, Point TR, Point BL, Point BR, int NUM_Y, int NUM_X)
{
	V->resize(NUM_Y*NUM_X,2);
	V->setZero();

	T->resize(2*(NUM_Y-1)*(NUM_X-1), 3);
	T->setZero();

	//std::cout << "V.rows:" << Pmat->rows() << std::endl;
	//std::cout << "T.rows:" << Imat->rows() << std::endl;

	int dely, delx;
	dely = (double)(BL.y - TL.y)/(double)(NUM_Y-1.0);
	delx = (double)(TR.x - TL.x)/(double)(NUM_X-1.0);

	std::cout << "dely:" << dely << std::endl;
	std::cout << "delx:" << delx << std::endl;

	// generate vert
	for (int y = 0; y < NUM_Y; y++)
	{
		for (int x = 0; x < NUM_X; x++)
		{
			//std::cout << "(" << y << "," << x << ")" << std::endl;

			(*V)((NUM_X*y)+x, 0) = TL.y + (y*dely); //y
			(*V)((NUM_X*y)+x, 1) = TL.x + (x*delx); //x

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
			(*T)(triIndex, 0) = (NUM_X*y)     + x;
			(*T)(triIndex, 1) = (NUM_X*y)     + (x+1);
			(*T)(triIndex, 2) = (NUM_X*(y+1)) + (x+1);

			triIndex = ((((NUM_X - 1)*y) + x) * 2) + 1;
			(*T)(triIndex, 0) = (NUM_X*y)     + x;
			(*T)(triIndex, 2) = (NUM_X*(y+1)) + (x + 1);
			(*T)(triIndex, 1) = (NUM_X*(y+1)) + x;
		}
	}
}

// -------------------------------------------------- //
// Draw Vertices
// -------------------------------------------------- //
void draw_vert(Mat* mat,EigenMatrixXi* V, int radius, Scalar color)
{
	for (int i = 0; i < V->rows(); i++)
	{
		Point p;
		p.x = (*V)(i, 1);	p.y = (*V)(i, 0);

		circle(*mat, p, radius, color, -1, 8, 0);
	}
}

// -------------------------------------------------- //
// Draw Triangle 
// -------------------------------------------------- //
void draw_tri(Mat* mat, Point p1, Point p2, Point p3, Scalar color, int thickness)
{
	line(*mat, p1, p2, color, thickness, 8, 0);
	line(*mat, p2, p3, color, thickness, 8, 0);
	line(*mat, p3, p1, color, thickness, 8, 0);
}

// -------------------------------------------------- //
// Draw Mesh
// -------------------------------------------------- //
void draw_mesh(Mat* mat, EigenMatrixXi* V, EigenMatrixXi* T, int NUM_Y, int NUM_X)
{
	for (int i = 0; i < T->rows(); i++)
	{
		Point p1, p2, p3;
		int   i1, i2, i3;

		i1 = (*T)(i, 0);	i2 = (*T)(i, 1);	i3 = (*T)(i, 2);

		p1.x = (*V)(i1, 1);	p1.y = (*V)(i1, 0);
		p2.x = (*V)(i2, 1);	p2.y = (*V)(i2, 0);
		p3.x = (*V)(i3, 1);	p3.y = (*V)(i3, 0);

		draw_tri(mat, p1, p2, p3);
	}
}

// -------------------------------------------------- //
// Compute Global Transformation
// -------------------------------------------------- //
void compute_GlobalTransformation(const Mat* mat_old, const Mat* mat_new, EigenVector2i d)
{
	if(((mat_old->rows)!=(mat_new->rows))|| ((mat_old->cols) != (mat_new->cols)))
	{
		std::cerr << "mat size is different compute_GlobalTransformation" << std::endl;
	}

	std::vector<SparseMatrixTriplet> a_triplets;	a_triplets.clear();

	for (int y = 1; y < mat_old->rows-1; y++)
	{
		for (int x = 1; x < mat_old->cols-1; x++)
		{
			Vec3b delY = compute_DerivativeY(mat_new, y, x);
			Vec3b delX = compute_DerivativeX(mat_new, y, x);
			Vec3b zero;

			/*if ((delY!=(Vec3b)(0,0,0))&&(delX!=(Vec3b)(0,0,0)))
			{

			}*/
		}
	}


}

// -------------------------------------------------- //
// Compute Grayscale Derivative X
// -------------------------------------------------- //
unsigned char compute_GDerivativeX(const Mat* mat, int y, int x, int size)
{
	unsigned char pixel_p = mat->at<unsigned char>(y, x + 1);
	unsigned char pixel_m = mat->at<unsigned char>(y, x - 1);

	unsigned char value = pixel_p - pixel_m;

	return value;
}

// -------------------------------------------------- //
// Compute Grayscale Derivative Y
// -------------------------------------------------- //
unsigned char compute_GDerivativeY(const Mat* mat, int y, int x, int size)
{
	unsigned char pixel_p = mat->at<unsigned char>(y + 1, x);
	unsigned char pixel_m = mat->at<unsigned char>(y - 1, x);

	unsigned char value = pixel_p - pixel_m;

	return value;
}

// -------------------------------------------------- //
// Compute Grayscale Derivative T
// -------------------------------------------------- //
unsigned char compute_GDerivativeT(const Mat* mat_old, const Mat* mat_new, int y, int x, int size)
{
	unsigned char pixel_old = mat_old->at<unsigned char>(y, x);
	unsigned char pixel_new = mat_new->at<unsigned char>(y, x);

	unsigned char value = pixel_new - pixel_old;

	return value;
}

// -------------------------------------------------- //
// Compute Derivative X
// -------------------------------------------------- //
Vec3b compute_DerivativeX(const Mat* mat, int y, int x, int size)
{
	Vec3b pixel_p = mat->at<Vec3b>(y, x+1);
	Vec3b pixel_m = mat->at<Vec3b>(y, x-1);

	Vec3b value = pixel_p - pixel_m;

	return value;
}

// -------------------------------------------------- //
// Compute Derivative Y
// -------------------------------------------------- //
Vec3b compute_DerivativeY(const Mat* mat, int y, int x, int size)
{
	Vec3b pixel_p = mat->at<Vec3b>(y+1, x);
	Vec3b pixel_m = mat->at<Vec3b>(y-1, x);

	Vec3b value = pixel_p - pixel_m;

	return value;
}

// -------------------------------------------------- //
// Compute Derivative T
// -------------------------------------------------- //
Vec3b compute_DerivativeT(const Mat* mat_old, const Mat* mat_new, int y, int x, int size)
{
	Vec3b pixel_old = mat_old->at<Vec3b>(y, x);
	Vec3b pixel_new = mat_new->at<Vec3b>(y, x);

	Vec3b value = pixel_new - pixel_old;

	return value;
}
#pragma endregion
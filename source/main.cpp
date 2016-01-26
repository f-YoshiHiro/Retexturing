#include <opencv2/opencv.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>

#include "math_headers.h"
#include "triangle.h"
#include "pixel.h"
#include "mesh.h"

// ----------name space---------- //
using namespace cv;

// -----------global---------- //
Mat mat00H;
Mat Gmat00H, Gmat19H;
Mat Gmat00M;

int num_X = 5;
int num_Y = 5;

// ----------functions---------- //
void load_images();

// core func
void compute_InterporationWeight(EigenMatrixXi* V, EigenMatrixXi* T, std::vector<EigenMatrixXs>* BVec, Point TL, Point TR, Point BL, Point BR);
void compute_GlobalTransformation(const Mat* mat_old, const Mat* mat_new, Vec2b d);

// mathematical func
unsigned char compute_GDerivativeX(const Mat* mat, int y, int x, int size = 3);
unsigned char compute_GDerivativeY(const Mat* mat, int y, int x, int size = 3);
unsigned char compute_GDerivativeT(const Mat* mat_old, const Mat* mat_new, int y, int x, int size = 3);

Vec3b compute_DerivativeX(const Mat* mat, int y, int x, int size = 3);
Vec3b compute_DerivativeY(const Mat* mat, int y, int x, int size = 3);
Vec3b compute_DerivativeT(const Mat* mat_old, const Mat* mat_new, int y, int x, int size = 3);

void compute_BarycentricCoordinates(ScalarType* w1, ScalarType* w2, ScalarType* w3, Point t1, Point t2, Point t3, Point p);

int cross_product(Point p1, Point p2);

// ----------main---------- //
int main(int argc, char** argv) 
{
	// variable
	//EigenMatrixXs B;	// weight
	std::vector<EigenMatrixXs> BVec; // weight
	Vec2b d;

	// load image
	load_images();

	// generate mesh
	Mesh mesh(&Gmat00H, num_X, num_Y);
	//compute_InterporationWeight(&V, &T, &BVec, TL, TR, BL, BR);

	// core
	//compute_GlobalTransformation(&Gmat00H, &Gmat19H, d);

	// draw
	mesh.draw_mesh(&Gmat00H);
	mesh.draw_vert(&Gmat00H);

	cv::namedWindow("src", cv::WINDOW_AUTOSIZE);
	imshow("src", mat00H);

	cv::namedWindow("G00", cv::WINDOW_AUTOSIZE);
	imshow("G00", Gmat00H);

	cv::namedWindow("G19", cv::WINDOW_AUTOSIZE);
	imshow("G19", Gmat19H);

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

	Gmat00H = cv::imread("../../image/Sim0000H.png", IMREAD_GRAYSCALE);
	Gmat19H = cv::imread("../../image/Sim0019H.png", IMREAD_GRAYSCALE);

	Gmat00M = cv::imread("../../image/Sim0000M.png", IMREAD_GRAYSCALE);
}

// -------------------------------------------------- //
// Compute Interporation Weight
// -------------------------------------------------- //
//void compute_InterporationWeight(EigenMatrixXi* V, EigenMatrixXi* T, std::vector<EigenMatrixXs>* BVec, Point TL, Point TR, Point BL, Point BR)
//{
//	int PixelNum_Y = BL.y - TL.y;	// ignore bottom line
//	int PixelNum_X = TR.x - TL.x;	// ignore right line
//
//	BVec->clear();
//
//		// for debag
//		//int u_count = 0;	int d_count = 0;
//
//	// loop for tri
//	for (int i = 0; i < TriVec.size(); i++)
//	{
//		int i1 = TriVec[i].get_i1();
//		int i2 = TriVec[i].get_i2();
//		int i3 = TriVec[i].get_i3();
//
//		Point p1, p2, p3;
//		p1.y = (*V)(i1, 0);		p1.x = (*V)(i1, 1);
//		p2.y = (*V)(i2, 0);		p2.x = (*V)(i2, 1);
//		p3.y = (*V)(i3, 0);		p3.x = (*V)(i3, 1);
//
//		int min_y = min(p2.y, p3.y);
//		int min_x = min(p1.x, p3.x);
//		int max_y = min(p1.y, p2.y);
//		int max_x = min(p2.x, p3.x);
//
//		for (int y = p1.y; y <= p3.y; y++)
//		{
//			for (int x = p1.x; x <= p2.x; x++)
//			{
//				Point pixel;
//				pixel.y = y;
//				pixel.x = x;
//
//				Point v12(p2 - p1);
//				Point v23(p3 - p2);
//				Point v31(p1 - p3);
//				Point v1P(pixel - p1);
//				Point v2P(pixel - p2);
//				Point v3P(pixel - p3);
//
//				int Cross121P = cross_product(v12, v1P);
//				int Cross232P = cross_product(v23, v2P);
//				int Cross313P = cross_product(v31, v3P);
//				//std::cout << "121P:" << Cross121P << std::endl;
//				//std::cout << "232P:" << Cross232P << std::endl;
//				//std::cout << "313P:" << Cross313P << std::endl;
//
//				if ((Cross121P>=0) && (Cross232P>=0) && (Cross313P>=0))
//				{
//					ScalarType w1, w2, w3;
//					compute_BarycentricCoordinates(&w1,&w2,&w3,p1,p2,p3,pixel);
//					Pixel pixel(pixel, w1, w2, w3);
//					TriVec[i].push_back_pixel(pixel);
//				}
//			}
//		}
//	}
//
//		// for debag
//		//std::cout << "U_NUM:" << u_count << std::endl;
//		//std::cout << "D_NUM:" << d_count << std::endl;
//
//		/*for (int i = 0; i < TriVec.size(); i++)
//		{
//			std::cout << "Tri["<< i << "]:" << TriVec[i].get_PixelNum() << std::endl;
//		}*/
//
//		for (int j = 0; j < TriVec[0].get_PixelNum(); ++j) 
//		{
//			Point pos = TriVec[0].get_PixelPos(j);
//			circle(Gmat00H, pos, 0, Scalar(Vec3b(200, 0, 0)), -1, 8, 0);
//		}
//
//		Point p1, p2, p3;
//		int i = 0;
//		p1.y = (*V)((*T)(i, 0), 0);		p1.x = (*V)((*T)(i, 0), 1);
//		p2.y = (*V)((*T)(i, 1), 0);		p2.x = (*V)((*T)(i, 1), 1);
//		p3.y = (*V)((*T)(i, 2), 0);		p3.x = (*V)((*T)(i, 2), 1);
//		//circle(Gmat00H, p1, 5, Scalar(Vec3b(0, 0, 0)), -1, 8, 0);
//		//circle(Gmat00H, p2, 5, Scalar(Vec3b(0, 0, 0)), -1, 8, 0);
//		//circle(Gmat00H, p3, 5, Scalar(Vec3b(0, 0, 0)), -1, 8, 0);
//}

// -------------------------------------------------- //
// Compute Global Transformation
// -------------------------------------------------- //
void compute_GlobalTransformation(const Mat* mat_old, const Mat* mat_new, Vec2b d)
{
	if(((mat_old->rows)!=(mat_new->rows))|| ((mat_old->cols) != (mat_new->cols)))
	{
		std::cerr << "mat size is different compute_GlobalTransformation" << std::endl;
	}

	// variable
	SparseMatrix A, ATA;
	VectorX b, x;
	std::vector<unsigned char> bVec;				bVec.clear();
	std::vector<SparseMatrixTriplet> a_triplets;	a_triplets.clear();
	Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper> llt_solver;
	int count = 0;

	for (int y = 1; y < mat_old->rows-1; y++)
	{
		for (int x = 1; x < mat_old->cols-1; x++)
		{
			unsigned char delY = compute_GDerivativeY(mat_new, y, x);
			unsigned char delX = compute_GDerivativeX(mat_new, y, x);
			unsigned char zero = 0;

			if ((delY != zero)&&(delX != zero))
			{
				unsigned char delT = compute_GDerivativeT(mat_old, mat_new, y, x);
				bVec.push_back(delT);

				a_triplets.push_back(SparseMatrixTriplet(count, 1, (double)delY));
				a_triplets.push_back(SparseMatrixTriplet(count, 0, (double)delX));

				count += 1;
			}
		}
	}

	A.resize(count, 2);
	A.setFromTriplets(a_triplets.begin(), a_triplets.end());

	b.resize(count);
	for (int i = 0; i < b.size(); ++i)
	{
		b(i) = (double)bVec[i];
	}
	b = (-1)*(A.transpose())*b;

	ATA = A.transpose() * A;
	factorizeDirectSolverLLT(ATA, llt_solver);

	x.resize(2);
	x = llt_solver.solve(b);

	std::cout << "x=" << std::endl;
	std::cout << x << std::endl;
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

// -------------------------------------------------- //
// Compute Barycentric Coorsinates
//
// weight: w1, w2, w3
// tri   : t1, t2, t3
// point : p
// -------------------------------------------------- //
void compute_BarycentricCoordinates(ScalarType* w1, ScalarType* w2, ScalarType* w3, Point t1, Point t2, Point t3, Point p)
{
	ScalarType denominator = ((t2.y-t3.y)*(t1.x-t3.x)) + ((t3.x-t2.x)*(t1.y-t3.y));

	*w1 = (((t2.y-t3.y)*(p.x-t3.x)) + ((t3.x-t2.x)*(p.y-t3.y))) / denominator;
	*w2 = (((t3.y-t1.y)*(p.x-t3.x)) + ((t1.x-t3.x)*(p.y-t3.y))) / denominator;
	*w3 = 1.0 - *w1 - *w2;
}

// -------------------------------------------------- //
// Compute Cross Product
// -------------------------------------------------- //
int cross_product(Point p1, Point p2)
{
	return ((p1.x*p2.y) - (p1.y*p2.x));
}
#pragma endregion
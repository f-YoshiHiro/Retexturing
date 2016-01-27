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

Mat matTest;

int num_X = 5;
int num_Y = 5;

// ----------functions---------- //
void load_images();

// core func
void compute_GlobalTransformation(const Mat* mat_old, const Mat* mat_new, Vec2b d);

// mathematical func
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
	//EigenMatrixXs B;	// weight
	std::vector<EigenMatrixXs> BVec; // weight
	Vec2b d;

	// load image
	load_images();

	// generate mesh
	Mesh mesh(&Gmat00H, num_X, num_Y);
	
		// test
		for (int j = 0; j < mesh.get_Tri(1).get_PixelNum(); ++j)
		{
			int index = mesh.get_Tri(1).get_PixelIndex(j);
			cv::Point pos = mesh.get_Pixel(index).get_pos();
			circle(Gmat00H, pos, 0, Scalar(Vec3b(200, 0, 0)), -1, 8, 0);
		}

		for (int i = 0; i < mesh.get_TriNum(); i++)
		{
			/*int i1, i2, i3;
			i1 = mesh.get_Tri(i).get_i1();	i2 = mesh.get_Tri(i).get_i2();	i3 = mesh.get_Tri(i).get_i3();
			std::cout << "i1" << i1  << std::endl;
			std::cout << "i2" << i2  << std::endl;
			std::cout << "i3" << i3  << std::endl;

			Point p1, p2, p3;
			p1 = mesh.get_Vert(i1);	p2 = mesh.get_Vert(i2);	p3 = mesh.get_Vert(i3);

			for (int j = 0; j < mesh.get_Tri(i).get_PixelNum(); j++)
			{
				int index;
				index = mesh.get_Tri(i).get_PixelIndex(j);

				ScalarType w1, w2, w3;
				w1 = mesh.get_Pixel(index).get_w1();
				w2 = mesh.get_Pixel(index).get_w2();
				w3 = mesh.get_Pixel(index).get_w3();

				Point p = (w1*p1) + (w2*p2) + (w3*p3);

				circle(matTest, p, 0, Scalar(Vec3b(200, 0, 0)), -1, 8, 0);
			}*/
		}

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

	cv::namedWindow("Test", cv::WINDOW_AUTOSIZE);
	imshow("Test", matTest);

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

	matTest = cv::imread("../../image/Sim0000H.png", IMREAD_GRAYSCALE);
}

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
#pragma endregion
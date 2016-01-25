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

// ----------name space---------- //
using namespace cv;

// -----------global---------- //
Mat mat00H;
Mat Gmat00H, Gmat19H;
Mat Gmat00M;

int HUGE_NUM = 10000;

int num_X = 5;
int num_Y = 5;

std::vector<Triangle> TriVec;

//std::vector<Point> vertVec;


// ----------functions---------- //
void load_images();

void search_corners(Mat* mat, Point* TL, Point* TR, Point* BL, Point* BR);
void gen_mesh(EigenMatrixXi* V, EigenMatrixXi* T, Point TL, Point TR, Point BL, Point BR, int NUM_Y, int NUM_X);

// draw func
void draw_vert(Mat* mat, EigenMatrixXi* V, int radius=2, Scalar color = (Vec3b(255, 0, 0)));
void draw_tri(Mat* mat, Point p1, Point p2, Point p3, Scalar color=(Vec3b(255,0,255)), int thickness=1);
void draw_mesh(Mat* mat, EigenMatrixXi* V, EigenMatrixXi* T, int NUM_Y, int NUM_X);

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
	Point TL, TR, BL, BR;	// corners
	EigenMatrixXi V;	// vertices
	EigenMatrixXi T;	// triangles
	//EigenMatrixXs B;	// weight
	std::vector<EigenMatrixXs> BVec; // weight
	Vec2b d;

	// load image
	load_images();

	// search corner
	search_corners(&Gmat00H, &TL, &TR, &BL, &BR);

	// generate mesh
	gen_mesh(&V, &T, TL, TR, BL, BR, num_Y, num_X);
	compute_InterporationWeight(&V, &T, &BVec, TL, TR, BL, BR);

	// core
	//compute_GlobalTransformation(&Gmat00H, &Gmat19H, d);

	// draw
	draw_mesh(&Gmat00H, &V, &T, num_Y, num_X);
	draw_vert(&Gmat00H, &V);

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
	TriVec.clear();
	for (int y = 0; y < NUM_Y-1; y++)
	{
		for (int x = 0; x < NUM_X-1; x++)
		{
			int triIndex;

			// upper tri
			triIndex = ( (((NUM_X - 1)*y) + x) * 2 )+0;
			(*T)(triIndex, 0) = (NUM_X*y)     + x;
			(*T)(triIndex, 1) = (NUM_X*y)     + (x+1);
			(*T)(triIndex, 2) = (NUM_X*(y+1)) + (x+1);
			Triangle u_tri(triIndex, (NUM_X*y) + x, (NUM_X*y) + (x + 1), (NUM_X*(y + 1)) + (x + 1));
			TriVec.push_back(u_tri);

			// down tri
			triIndex = ((((NUM_X - 1)*y) + x) * 2) + 1;
			(*T)(triIndex, 0) = (NUM_X*y)     + x;
			(*T)(triIndex, 1) = (NUM_X*(y+1)) + (x + 1);
			(*T)(triIndex, 2) = (NUM_X*(y+1)) + x;
			Triangle d_tri(triIndex, (NUM_X*y) + x, (NUM_X*(y + 1)) + (x + 1), (NUM_X*(y + 1)) + x);
			TriVec.push_back(d_tri);
		}
	}
	std::cout << "Num TriVec" << TriVec.size() << std::endl;
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
// Compute Interporation Weight
// -------------------------------------------------- //
void compute_InterporationWeight(EigenMatrixXi* V, EigenMatrixXi* T, std::vector<EigenMatrixXs>* BVec, Point TL, Point TR, Point BL, Point BR)
{
	int PixelNum_Y = BL.y - TL.y;	// ignore bottom line
	int PixelNum_X = TR.x - TL.x;	// ignore right line

	BVec->clear();

		// for debag
		//int u_count = 0;	int d_count = 0;

	// loop for tri
	for (int i = 0; i < TriVec.size(); i++)
	{
		int i1 = TriVec[i].get_i1();
		int i2 = TriVec[i].get_i2();
		int i3 = TriVec[i].get_i3();

		Point p1, p2, p3;
		p1.y = (*V)(i1, 0);		p1.x = (*V)(i1, 1);
		p2.y = (*V)(i2, 0);		p2.x = (*V)(i2, 1);
		p3.y = (*V)(i3, 0);		p3.x = (*V)(i3, 1);

		int min_y = min(p2.y, p3.y);
		int min_x = min(p1.x, p3.x);
		int max_y = min(p1.y, p2.y);
		int max_x = min(p2.x, p3.x);

		for (int y = p1.y; y <= p3.y; y++)
		{
			for (int x = p1.x; x <= p2.x; x++)
			{
				Point pixel;
				pixel.y = y;
				pixel.x = x;

				Point v12(p2 - p1);
				Point v23(p3 - p2);
				Point v31(p1 - p3);
				Point v1P(pixel - p1);
				Point v2P(pixel - p2);
				Point v3P(pixel - p3);

				int Cross121P = cross_product(v12, v1P);
				int Cross232P = cross_product(v23, v2P);
				int Cross313P = cross_product(v31, v3P);
				//std::cout << "121P:" << Cross121P << std::endl;
				//std::cout << "232P:" << Cross232P << std::endl;
				//std::cout << "313P:" << Cross313P << std::endl;

				if ((Cross121P>=0) && (Cross232P>=0) && (Cross313P>=0))
				{
					ScalarType w1, w2, w3;
					compute_BarycentricCoordinates(&w1,&w2,&w3,p1,p2,p3,pixel);
					Pixel pixel(pixel, w1, w2, w3);
					TriVec[i].push_back_pixel(pixel);
				}
			}
		}
	}

		// for debag
		//std::cout << "U_NUM:" << u_count << std::endl;
		//std::cout << "D_NUM:" << d_count << std::endl;

		for (int i = 0; i < TriVec.size(); i++)
		{
			std::cout << "Tri["<< i << "]:" << TriVec[i].get_PixelNum() << std::endl;
		}

		for (int j = 0; j < TriVec[0].get_PixelNum(); ++j) 
		{
			Point pos = TriVec[0].get_PixelPos(j);
			circle(Gmat00H, pos, 0, Scalar(Vec3b(200, 0, 0)), -1, 8, 0);
		}

		Point p1, p2, p3;
		int i = 0;
		p1.y = (*V)((*T)(i, 0), 0);		p1.x = (*V)((*T)(i, 0), 1);
		p2.y = (*V)((*T)(i, 1), 0);		p2.x = (*V)((*T)(i, 1), 1);
		p3.y = (*V)((*T)(i, 2), 0);		p3.x = (*V)((*T)(i, 2), 1);
		//circle(Gmat00H, p1, 5, Scalar(Vec3b(0, 0, 0)), -1, 8, 0);
		//circle(Gmat00H, p2, 5, Scalar(Vec3b(0, 0, 0)), -1, 8, 0);
		//circle(Gmat00H, p3, 5, Scalar(Vec3b(0, 0, 0)), -1, 8, 0);
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
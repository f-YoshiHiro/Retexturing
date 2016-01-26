#pragma once

#include "mesh.h"

Mesh::Mesh(cv::Mat* mat, int _num_X, int _num_Y)
{
	num_X = _num_X;	num_Y = _num_Y;
	corner_flag = false;
	detect_corners(mat);
	init();
}

Mesh::Mesh(cv::Point _TL, cv::Point _TR, cv::Point _BL, cv::Point _BR, int _num_X, int _num_Y)
{
	TL = _TL;	TR = _TR;	BL = _BL;	BR = _BR;
	num_X = _num_X;	num_Y = _num_Y;
	corner_flag = true;
	init();
}

void Mesh::init() 
{
	gen_vert();
	gen_tri();
}

void Mesh::detect_corners(cv::Mat* mat)
{
	if (corner_flag == false)
	{
		std::cout << "Corners Detecting..." << std::endl;

		// init
		int HUGE_NUM = 10000;
		TL.y = HUGE_NUM;	TL.x = HUGE_NUM;
		TR.y = HUGE_NUM;	TR.x = HUGE_NUM;
		BR.y = 0;			BR.x = 0;
		BL.y = 0;			BL.x = 0;

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
						TL.y = y;	TL.x = x;
					}
					else if (LNorm > LNormMAX)
					{
						LNormMAX = LNorm;
						BR.y = y;	BR.x = x;
					}

					if (RNorm < RNormMIN)
					{
						RNormMIN = RNorm;
						TR.y = y;	TR.x = x;
					}
					else if (RNorm > RNormMAX)
					{
						RNormMAX = RNorm;
						BL.y = y;	BL.x = x;
					}
				}
			}
		}

		corner_flag = true;

		std::cout << "TL:" << TL.y << "," << TL.x << std::endl;
		std::cout << "TR:" << TR.y << "," << TR.x << std::endl;
		std::cout << "BL:" << BL.y << "," << BL.x << std::endl;
		std::cout << "BR:" << BR.y << "," << BR.x << std::endl;
	}
}

void Mesh::gen_vert()
{
	if (corner_flag == false){ std::cerr << "Please Detect Corner" << std::endl; }

	std::cout << "Vertices Generating... " << std::endl;

	VertVec.clear();

	int dely, delx;
	dely = (double)(BL.y - TL.y) / (double)(num_Y - 1.0);
	delx = (double)(TR.x - TL.x) / (double)(num_X - 1.0);

	for (int y = 0; y < num_Y; y++)
	{
		for (int x = 0; x < num_X; x++)
		{
			//std::cout << "(" << y << "," << x << ")" << std::endl;
			cv::Point vert;
			vert.y = TL.y + (y*dely); //y
			vert.x = TL.x + (x*delx); //x

			VertVec.push_back(vert);
		}
	}

	std::cout << "#Vert: " << VertVec.size() <<std::endl;
}

void Mesh::gen_tri()
{
	if (corner_flag == false) { std::cerr << "Please Detect Corner" << std::endl; }

	std::cout << "Triangles Generating..." << std::endl;

	TriVec.clear();

	for (int y = 0; y < num_Y - 1; y++)
	{
		for (int x = 0; x < num_X - 1; x++)
		{
			int triIndex;

			// upper tri
			triIndex = ((((num_X - 1)*y) + x) * 2) + 0;
			Triangle u_tri(triIndex, (num_X*y) + x, (num_X*y) + (x + 1), (num_X*(y + 1)) + (x + 1));
			TriVec.push_back(u_tri);

			// down tri
			triIndex = ((((num_X - 1)*y) + x) * 2) + 1;
			Triangle d_tri(triIndex, (num_X*y) + x, (num_X*(y + 1)) + (x + 1), (num_X*(y + 1)) + x);
			TriVec.push_back(d_tri);
		}
	}
	std::cout << "#Tri:" << TriVec.size() << std::endl;
}

// -------------------------------------------------- //
// Draw All Vertices
// -------------------------------------------------- //
void Mesh::draw_vert(cv::Mat* mat, int radius, cv::Scalar color)
{
	for (int i = 0; i < VertVec.size(); i++)
	{
		circle(*mat, VertVec[i], radius, color, -1, 8, 0);
	}
}

// -------------------------------------------------- //
// Draw i-th Triangle 
// -------------------------------------------------- //
void Mesh::draw_tri(cv::Mat* mat, int _i, cv::Scalar color, int thickness)
{
	int i1 = TriVec[_i].get_i1();
	int i2 = TriVec[_i].get_i2();
	int i3 = TriVec[_i].get_i3();

	cv::Point v1 = VertVec[i1];
	cv::Point v2 = VertVec[i2];
	cv::Point v3 = VertVec[i3];

	line(*mat, v1, v2, color, thickness, 8, 0);
	line(*mat, v2, v3, color, thickness, 8, 0);
	line(*mat, v3, v1, color, thickness, 8, 0);
}

// -------------------------------------------------- //
// Draw Mesh
// -------------------------------------------------- //
void Mesh::draw_mesh(cv::Mat* mat)
{
	for (int i = 0; i < TriVec.size(); i++)
	{
		draw_tri(mat, i);
	}
}
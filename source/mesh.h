#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


#include <iostream>
#include <vector>

#include "math_headers.h"
#include "pixel.h"
#include "triangle.h"

class Mesh
{
public:
	Mesh() {}
	Mesh(cv::Mat* mat,int _num_X, int _num_Y);
	Mesh(cv::Point _TL, cv::Point _TR, cv::Point _BL, cv::Point _BR, int _num_X, int _num_Y);
	~Mesh() {}

   
protected:
	bool corner_flag;

protected:
	cv::Point TL, TR, BL, BR;
	int num_X, num_Y;			// #devide (#x, y vert)

	std::vector<cv::Point> VertVec;
	std::vector<Triangle>  TriVec;
	std::vector<std::vector<Pixel>> PixelVecVec;

public:	
// core func
	void init();

	void detect_corners(cv::Mat* mat);
	void gen_vert();
	void gen_tri();

public:	
// get methods
	cv::Point get_Vert(int _i) { return VertVec[_i]; }

public:	
// draw func
	void draw_vert(cv::Mat* mat, int radius = 2, cv::Scalar color = (cv::Vec3b(255, 0, 0)));
	void draw_tri(cv::Mat* mat, int _i, cv::Scalar color = (cv::Vec3b(255, 0, 255)), int thickness = 1);
	void draw_mesh(cv::Mat* mat);
};

#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


#include <iostream>
#include <vector>

#include "math_headers.h"
#include "pixel.h"

typedef enum
{
	TRI_TYPE_UP,
	TRI_TYPE_DOWN,
	TRI_TYPE_TOTAL_NUM

} TriType;

class Pixel;

class Triangle
{
public:
	Triangle() {}
	Triangle(int _index, int _i1, int _i2, int _i3) { index = _index, i1 = _i1;	i2 = _i2; i3 = _i3; PixelVec.clear(); }
	~Triangle() {}

protected:
	int index;
	int i1, i2, i3;
	TriType type;
	std::vector<Pixel> PixelVec;

public:
	int get_i1() { return i1; }
	int get_i2() { return i2; }
	int get_i3() { return i3; }
	int get_PixelNum() { return PixelVec.size(); }
	cv::Point get_PixelPos(int i) { return PixelVec[i].get_pos(); }
	
	void set_type(TriType _type) { type = _type; }

	void push_back_pixel(Pixel _pixel) { PixelVec.push_back(_pixel); }
};
#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


#include <iostream>
#include <vector>

#include "math_headers.h"

class Pixel
{
public:
	Pixel(){}
	Pixel(int _y, int _x) { pos.y = _y; pos.x = _x; }
	Pixel(cv::Point _p) { pos = _p; }
	Pixel(cv::Point _p, ScalarType _w1, ScalarType _w2, ScalarType _w3) { pos = _p; w1 = _w1; w2 = _w2; w3 = _w3; }
	~Pixel() {}

protected:
	cv::Point pos;
	int tri_index;
	ScalarType w1, w2, w3;

public:
// core func
	void compute_BarycentricCoordinates(cv::Point t1, cv::Point t2, cv::Point t3);

public:
// set methods
	void set_w1(ScalarType _w1) { w1 = _w1; }
	void set_w2(ScalarType _w2) { w2 = _w2; }
	void set_w3(ScalarType _w3) { w3 = _w3; }

	void set_tri_index(int _i) { tri_index = _i; }

public:
// get methods
	cv::Point get_pos() { return pos; }
	ScalarType get_w1() { return w1; }
	ScalarType get_w2() { return w2; }
	ScalarType get_w3() { return w3; }
};
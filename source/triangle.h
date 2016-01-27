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
	Triangle(int _index, int _i1, int _i2, int _i3) { index = _index, i1 = _i1;	i2 = _i2; i3 = _i3; PixelIndexVec.clear(); }
	~Triangle() {}

protected:
	int index;
	int i1, i2, i3;
	TriType type;
	std::vector<int> PixelIndexVec;

public:
// core func

public:
// get methods
	int get_i1() { return i1; }
	int get_i2() { return i2; }
	int get_i3() { return i3; }
	int get_PixelNum() { return PixelIndexVec.size(); }
	int get_PixelIndex(int _i) { return PixelIndexVec[_i]; }
	
public:
// set methods
	void set_type(TriType _type) { type = _type; }
	void set_pixel_index(int _i) { PixelIndexVec.push_back(_i); }
};
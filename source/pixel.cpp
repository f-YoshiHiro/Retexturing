#include "pixel.h"

void Pixel::compute_BarycentricCoordinates(cv::Point t1, cv::Point t2, cv::Point t3)
{
	ScalarType denominator = (ScalarType)((t2.y - t3.y)*(t1.x - t3.x)) + ((t3.x - t2.x)*(t1.y - t3.y));

	w1 = (ScalarType)(((t2.y - t3.y)*(pos.x - t3.x)) + ((t3.x - t2.x)*(pos.y - t3.y))) / denominator;
	w2 = (ScalarType)(((t3.y - t1.y)*(pos.x - t3.x)) + ((t1.x - t3.x)*(pos.y - t3.y))) / denominator;
	w3 = 1.0 - w1 - w2;
}
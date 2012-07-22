#include <opencv2/opencv.hpp>

cv::VideoCapture cap;
cap.open(0);
cap>>imgRGB;


std::vector<int> qualityType;
qualityType.push_back(CV_IMWRITE_JPEG_QUALITY);
qualityType.push_back(100);


std::string s;
std::stringstream out;
out << "./bob.jpg";
s = out.str();


imwrite( s, imgRGB, qualityType);

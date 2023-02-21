#ifndef POLY1D_HPP
#define POLY1D_HPP

#include <opencv2/opencv.hpp>
#include <vector>

class poly1d {
public:
    poly1d(const std::vector<cv::Point> &points, int degree);

    std::vector<cv::Point> getUniformPoints(double dist = 15) const;

    double operator() (double x) const;

    double getError() const;

    const std::vector<double>& getCoeffs() const;

private:
    std::vector<double> coeffs;
    double error;
    double minX, maxX, minY, maxY;
    double calculateError(const std::vector<cv::Point>& points, const std::vector<double>& coeffs);
};

std::vector<std::vector<cv::Point>> process_image(cv::Mat image);

#endif

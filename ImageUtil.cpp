#include "ImageUtil.hpp"
#include <cmath>
#include <iostream>

poly1d::poly1d(const std::vector<cv::Point>& points, int degree) : coeffs(degree + 1) {
    degree = std::min((double)degree, (double)points.size() - 1);
    cv::Mat_<double> x_mat(points.size(), degree + 1), y_mat(points.size(), 1);
    for (int i = 0; i < points.size(); ++i) {
        for (int j = 0; j <= degree; ++j) {
            x_mat(i, j) = std::pow(points[i].x, j);
        }
        y_mat(i, 0) = points[i].y;
    }

    cv::Mat_<double> p;
    cv::solve(x_mat, y_mat, p, cv::DECOMP_NORMAL | cv::DECOMP_LU);

    for (int i = 0; i <= degree; ++i) {
        coeffs[i] = p(i, 0);
    }
    error = calculateError(points, coeffs);

    minX = points[0].x;
    maxX = points[0].x;
    minY = points[0].y;
    maxY = points[0].y;
    for (auto point : points) {
        minX = std::min(minX, (double)point.x);
        maxX = std::max(maxX, (double)point.x);
        minY = std::min(minY, (double)point.y);
        maxY = std::max(maxY, (double)point.y);
    }
}

std::vector<cv::Point> poly1d::getUniformPoints(double dist) const {
    std::vector<cv::Point> uniformPoints;
    double x = minX;
    while (x < maxX) {
        double y = operator()(x);
        uniformPoints.emplace_back(x, y);
        x += dist;
    }
    return uniformPoints;
}

double poly1d::operator()(double x) const {
    double y = coeffs[0];
    for (int i = 1; i < coeffs.size(); ++i) {
        y += coeffs[i] * std::pow(x, i);
    }
    return y;
}

double poly1d::getError() const {
    return error;
}

const std::vector<double>& poly1d::getCoeffs() const {
    return coeffs;
}

double poly1d::calculateError(const std::vector<cv::Point>& points, const std::vector<double>& coeffs) {
    double sum_squared_error = 0;
    for (int i = 0; i < points.size(); ++i) {
        double y_predicted = coeffs[0];
        for (int j = 1; j < coeffs.size(); ++j) {
            y_predicted += coeffs[j] * std::pow(points[i].x, j);
        }
        sum_squared_error += std::pow(y_predicted - points[i].y, 2);
    }
    return sum_squared_error;
}

std::vector<std::vector<cv::Point>> process_image(cv::Mat image) {
    // Dilate and erode to eliminate noise and connect intermittent lane lines
    int kernel_size = 8;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::dilate(image, image, kernel, cv::Point(-1, -1), 5);
    cv::erode(image, image, kernel, cv::Point(-1,-1), 5);

    // Find contours in the image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> lane_lines;
    for (auto& contour : contours) {
        // std::cout << "Type : " << typeid(contour).name() << " with length : " << contour.size() << std::endl;
        // Extract x and y values from the contour
        if (contour.size() >= 2) {
          std::vector<cv::Point> points;
          for(int i = 0; i< contour.size(); i+=1) {
            points.push_back(contour[i]);
          }
          lane_lines.push_back(points);
        }
    }

    // Fit the polynomial, and return uniform points list
    std::vector<std::vector<cv::Point>> results;
    for (auto & lane : lane_lines) {
        poly1d f = poly1d(lane, 5);
        // std::cout << f << std::endl;
        auto points = f.getUniformPoints();
        if (points.size() >= 2){
            results.push_back(points);
        }
    }

    return results;
}
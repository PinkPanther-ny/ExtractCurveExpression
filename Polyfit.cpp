#include "Polyfit.hpp"
#include <cmath>
#include <iostream>

// Constructor for the poly1d class that takes a vector of cv::Point objects and a degree
poly1d::poly1d(const std::vector<cv::Point>& points, int degree) : coeffs(degree + 1) {
    // Calculate the degree of the polynomial to fit, based on the input degree and number of points
    degree = std::min((double)degree, (double)points.size() - 1);

    // Create the coefficient matrix X and the target matrix Y
    cv::Mat_<double> x_mat(points.size(), degree + 1), y_mat(points.size(), 1);
    for (int i = 0; i < points.size(); ++i) {
        // Fill in the X matrix with powers of x up to the specified degree
        for (int j = 0; j <= degree; ++j) {
            x_mat(i, j) = std::pow(points[i].x, j);
        }
        // Fill in the Y matrix with the y values of the input points
        y_mat(i, 0) = points[i].y;
    }

    // Solve for the coefficients of the polynomial fit using the X and Y matrices
    cv::Mat_<double> p;
    cv::solve(x_mat, y_mat, p, cv::DECOMP_NORMAL | cv::DECOMP_LU);

    // Store the coefficients in the coeffs vector
    for (int i = 0; i <= degree; ++i) {
        coeffs[i] = p(i, 0);
    }

    // Calculate the sum squared error of the polynomial fit
    error = calculateError(points, coeffs);

    // Store the minimum and maximum x and y values of the input points
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

// Function to get a vector of cv::Point objects with uniform spacing along the fitted polynomial
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

// Overloaded function call operator that evaluates the polynomial at a given x value
double poly1d::operator()(double x) const {
    double y = coeffs[0];
    for (int i = 1; i < coeffs.size(); ++i) {
        y += coeffs[i] * std::pow(x, i);
    }
    return y;
}

// Function to get the sum squared error of the polynomial fit
double poly1d::getError() const {
    return error;
}

// Function to get a constant reference to the coefficients of the polynomial
const std::vector<double>& poly1d::getCoeffs() const {
    return coeffs;
}

// Function to calculate the sum squared error of the polynomial fit
double poly1d::calculateError(const std::vector<cv::Point>& points, const std::vector<double>& coeffs) {
    double sum_squared_error = 0;
    for (int i = 0; i < points.size(); ++i) {
        // Evaluate the polynomial at each x value of the input points
        double y_predicted = coeffs[0];
        for (int j = 1; j < coeffs.size(); ++j) {
            y_predicted += coeffs[j] * std::pow(points[i].x, j);
        }
        // Add the squared difference between the predicted and actual y values to the sum
        sum_squared_error += std::pow(y_predicted - points[i].y, 2);
    }
    return sum_squared_error;
}

// Function to segment an image into lane lines and fit polynomials to each lane line
std::vector<std::vector<cv::Point>> seg_to_points(cv::Mat image) {
    // Dilate and erode the image to remove noise and connect intermittent lane lines
    int kernel_size = 8;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::dilate(image, image, kernel, cv::Point(-1, -1), 5);
    cv::erode(image, image, kernel, cv::Point(-1,-1), 5);

    // Find contours in the image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> lane_lines;
    for (auto& contour : contours) {
        // Extract x and y values from the contour and store them as a vector of cv::Point objects
        if (contour.size() >= 2) {
            std::vector<cv::Point> points;
            for(int i = 0; i< contour.size(); i+=1) {
                points.push_back(contour[i]);
            }
            lane_lines.push_back(points);
        }
    }

    // Fit a polynomial to each lane line and return a vector of vectors of cv::Point objects
    std::vector<std::vector<cv::Point>> results;
    for (auto & lane : lane_lines) {
        poly1d f = poly1d(lane, 5);
        auto points = f.getUniformPoints();
        if (points.size() >= 2){
            results.push_back(points);
        }
    }

    return results;
}
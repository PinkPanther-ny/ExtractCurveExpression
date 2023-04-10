#include "Polyfit.hpp"
#include <cmath>
#include <iostream>

// Constructor for the Poly1D class that takes a vector of cv::Point objects and a degree
Poly1D::Poly1D(const std::vector<cv::Point> &points, int degree) : coeffs_(degree + 1) {
    // Calculate the degree of the polynomial to fit, based on the input degree and number of points
    degree = std::min((double) degree, (double) points.size() - 1);

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
        coeffs_[i] = p(i, 0);
    }

    // Calculate the sum squared error of the polynomial fit
    error_ = calculateError(points, coeffs_);

    // Store the minimum and maximum x and y values of the input points
    min_x_ = points[0].x;
    max_x_ = points[0].x;
    min_y_ = points[0].y;
    max_y_ = points[0].y;
    for (const auto& point: points) {
        min_x_ = std::min(min_x_, (double) point.x);
        max_x_ = std::max(max_x_, (double) point.x);
        min_y_ = std::min(min_y_, (double) point.y);
        max_y_ = std::max(max_y_, (double) point.y);
    }
}

// Function to get a vector of cv::Point objects with uniform spacing along the fitted polynomial
std::vector<cv::Point> Poly1D::getUniformPoints(double dist) const {
    std::vector<cv::Point> uniform_points;
    double x = min_x_;
    while (x < max_x_) {
        double y = operator()(x);
        uniform_points.emplace_back(x, y);
        x += dist;
    }
    return uniform_points;
}

// Overloaded function call operator that evaluates the polynomial at a given x value
double Poly1D::operator()(double x) const {
    double y = coeffs_[0];
    for (int i = 1; i < coeffs_.size(); ++i) {
        y += coeffs_[i] * std::pow(x, i);
    }
    return y;
}

// Function to get the sum squared error of the polynomial fit
double Poly1D::getError() const {
    return error_;
}

// Function to get a constant reference to the coefficients of the polynomial
const std::vector<double> &Poly1D::getCoeffs() const {
    return coeffs_;
}

// Function to calculate the sum squared error of the polynomial fit
double Poly1D::calculateError(const std::vector<cv::Point> &points, const std::vector<double> &coeffs) {
    double sum_squared_error = 0;
    for (const auto & point : points) {
        // Evaluate the polynomial at each x value of the input points
        double y_predicted = coeffs[0];
        for (int j = 1; j < coeffs.size(); ++j) {
            y_predicted += coeffs[j] * std::pow(point.x, j);
        }
        // Add the squared difference between the predicted and actual y values to the sum
        sum_squared_error += std::pow(y_predicted - point.y, 2);
    }
    return sum_squared_error;
}

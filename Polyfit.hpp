#ifndef POLYFIT_HPP
#define POLYFIT_HPP

#include <opencv2/opencv.hpp>
#include <vector>

// Declare the Poly1D class
class Poly1D {
public:
    // Constructor that takes a vector of cv::Point objects and a degree
    Poly1D(const std::vector<cv::Point> &points, int degree);

    // Returns a vector of cv::Point objects with a uniform spacing
    std::vector<cv::Point> getUniformPoints(double dist = 15) const;

    // Overloaded function call operator that evaluates the polynomial at a given x value
    double operator() (double x) const;

    // Returns the sum squared error of the polynomial fit
    double getError() const;

    // Returns a constant reference to the coefficients of the polynomial
    const std::vector<double>& getCoeffs() const;

private:
    // Vector to store the coefficients of the polynomial fit
    std::vector<double> coeffs_;

    // Stores the sum squared error of the polynomial fit
    double error_;

    // Stores the minimum and maximum x and y values of the input points
    double min_x_, max_x_, min_y_, max_y_;

    // Helper function to calculate the sum squared error of the polynomial fit
    double calculateError(const std::vector<cv::Point>& points, const std::vector<double>& coeffs);
};

#endif

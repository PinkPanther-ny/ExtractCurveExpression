#ifndef POLYFIT_HPP
#define POLYFIT_HPP

#include <opencv2/opencv.hpp>
#include <vector>

// Declare the poly1d class
class poly1d {
public:
    // Constructor that takes a vector of cv::Point objects and a degree
    poly1d(const std::vector<cv::Point> &points, int degree);

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
    std::vector<double> coeffs;

    // Stores the sum squared error of the polynomial fit
    double error;

    // Stores the minimum and maximum x and y values of the input points
    double minX, maxX, minY, maxY;

    // Helper function to calculate the sum squared error of the polynomial fit
    double calculateError(const std::vector<cv::Point>& points, const std::vector<double>& coeffs);
};

// Function to convert a segmented image to a vector of vectors of cv::Point objects
std::vector<std::vector<cv::Point>> seg_to_points(cv::Mat image);

#endif

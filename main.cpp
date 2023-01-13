#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/ml.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <cmath>


class poly1d {
public:
    poly1d(const std::vector<cv::Point> &points, int degree) : coeffs(degree+1) {
        degree = std::min((double) degree, (double) points.size()-1);
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
            minX = std::min(minX, (double) point.x);
            maxX = std::max(maxX, (double) point.x);
            minY = std::min(minY, (double) point.y);
            maxY = std::max(maxY, (double) point.y);
        }
    }

    std::vector<cv::Point> getUniformPoints(double dist = 15) const {
        std::vector<cv::Point> uniformPoints;
        double x = minX;
        while (x < maxX) {
            double y = operator()(x);
            uniformPoints.emplace_back(x, y);
            x += dist;
        }
        return uniformPoints;
    }

    double operator() (double x) const {
        double y = coeffs[0];
        for (int i = 1; i < coeffs.size(); ++i) {
            y += coeffs[i] * std::pow(x, i);
        }
        return y;
    }
    double getError() const {
        return error;
    }
    const std::vector<double>& getCoeffs() const {
        return coeffs;
    }

    friend std::ostream& operator<<(std::ostream& os, const poly1d& f) {
        os << "Degree " << f.coeffs.size() - 1 << " polynomial" << std::endl;
        os << "Coefficients: [";
        for (int i = 0; i < f.coeffs.size(); ++i) {
            os << f.coeffs[i];
            if (i < f.coeffs.size() - 1) {
                os << ", ";
            }
        }
        os << "]" << std::endl;
        os << "Error: " << f.error << std::endl;
        os << "minX: " << f.minX << " maxX: " << f.maxX << " minY: " << f.minY << " maxY: " << f.maxY << std::endl;
        return os;
    }

private:
    std::vector<double> coeffs;
    double error;
    double minX, maxX, minY, maxY;
    double calculateError(const std::vector<cv::Point>& points, const std::vector<double>& coeffs) {
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
};

std::vector<std::vector<cv::Point>> process_image(cv::Mat image) {
    // Dilate and erode to eliminate noise and connect intermittent lane lines
    int kernel_size = 8;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::dilate(image, image, kernel, cv::Point(-1,-1), 5);
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

cv::Scalar randomColor(cv::RNG& rng)
{
    int b = rng.uniform(0, 256);
    int g = rng.uniform(0, 256);
    int r = rng.uniform(0, 256);
    return cv::Scalar(b, g, r);
}

int main(int argc, char const *argv[]) {
    // Check for correct number of command line arguments
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <image_path>" << std::endl;
        return -1;
    }
    // Read image name from command line argument
    std::string image_path = argv[1];
    cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    cv::Mat color_image;
    cv::cvtColor(image, color_image, cv::COLOR_GRAY2BGR);

    // Process the image to extract fitted lane lines
    std::vector<std::vector<cv::Point>> results = process_image(image);

    // Simple visualization
    cv::RNG rng(0xFFFFFFFF);
    for (auto & lane : results) {
        cv::Scalar color = randomColor(rng);
        // std::cout << lane.size() << std::endl;
        for (int i = 0; i < lane.size() - 1; i++) {
            
            // Draw circle of the fitted lane line
            cv::line(color_image, lane[i], lane[i+1], color, 2);
            cv::circle(color_image, lane[i], 5, color, -1);
        }
        cv::circle(color_image, lane[lane.size()-1], 5, color, -1);
    }
    cv::imwrite("output.png", color_image);

    return 0;
}

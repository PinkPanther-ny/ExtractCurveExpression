#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

std::vector<double> fitPolynomial(std::vector<cv::Point> points, int degree)
{
    // Create the design matrix
    cv::Mat X = cv::Mat::ones(points.size(), degree+1, CV_64F);
    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < degree; j++)
        {
            X.at<double>(i, j+1) = pow(points[i].x, degree-j);
        }
    }
    cv::Mat y = cv::Mat::ones(points.size(), 1, CV_64F);
    for (int i = 0; i < points.size(); i++)
    {
        y.at<double>(i, 0) = points[i].y;
    }
    // Fit the polynomial using the method of least squares
    cv::Mat p = (X.t() * X).inv() * X.t() * y;
    // Return the coefficients in the desired order
    std::vector<double> coeffs;
    for (int i = 0; i <= degree; i++)
    {
        coeffs.push_back(p.at<double>(degree-i, 0));
    }
    return coeffs;
}


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
        std::cout << "Type : " << typeid(contour).name() << " with length : " << contour.size() << std::endl;
        // Extract x and y values from the contour
        if (contour.size() > 2) {
          std::vector<cv::Point> points;
          int step = 1;
          for(int i = 0; i< contour.size(); i+=step) {
            points.push_back(contour[i]);
          }
          lane_lines.push_back(points);
        }
    }
    return lane_lines;
}


int main(int argc, char const *argv[]) {
    cv::Mat image = cv::imread("im.png", cv::IMREAD_GRAYSCALE);
    cv::Mat color_image;
    cv::cvtColor(image, color_image, cv::COLOR_GRAY2BGR);
    // Process the image to extract fitted lane lines
    std::vector<std::vector<cv::Point>> lane_lines = process_image(image);

    for (auto & lane : lane_lines) {
        for (int i = 0; i < lane.size() - 1; i++) {
            cv::line(color_image, lane[i], lane[i + 1], cv::Scalar(0, 0, 255), 2);
            cv::circle(color_image, lane[i], 3, cv::Scalar(0, 255, 0), -1);
        }
        cv::line(color_image, lane[lane.size() - 1], lane[0], cv::Scalar(0, 0, 255), 2);
        cv::circle(color_image, lane[0], 3, cv::Scalar(128, 128, 128), -1);
        cv::circle(color_image, lane[lane.size() - 1], 3, cv::Scalar(255, 255, 0), -1);
    }

    // cv::imshow("Image", color_image);
    // cv::waitKey(0);
    cv::imwrite("output.png", color_image);
    return 0;
}

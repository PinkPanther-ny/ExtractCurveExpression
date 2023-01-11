#include <opencv2/opencv.hpp>
#include <iostream>
std::vector<std::vector<cv::Point>> process_image(cv::Mat image) {
    // Dilate and erode to eliminate noise and connect intermittent lane lines
    int kernel_size = 8;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::dilate(image, image, kernel, cv::Point(-1,-1), 5);
    cv::erode(image, image, kernel, cv::Point(-1,-1), 5);

    // Find contours in the image
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<std::vector<cv::Point>> lane_lines;
    for (auto& contour : contours) {
        // Extract x and y values from the contour
        lane_lines.push_back(contour);
    }
    
    return lane_lines;
}

int main(int argc, char const *argv[]) {
    cv::Mat image = cv::imread("im.png", cv::IMREAD_GRAYSCALE);

    // Process the image to extract fitted lane lines
    std::vector<std::vector<cv::Point>> lane_lines = process_image(image);

    for (auto & lane : lane_lines) {
        std::cout << "Type : " << typeid(lane).name() << " with length : " << lane.size() << std::endl;
    }
    return 0;
}

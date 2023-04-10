#include <opencv2/opencv.hpp>
#include "Polyfit.hpp"

cv::Scalar randomColor(cv::RNG& rng)
{
    int b = rng.uniform(0, 256);
    int g = rng.uniform(0, 256);
    int r = rng.uniform(0, 256);
    return cv::Scalar(b, g, r);
}


/**
 * @brief 从分割输出的二值图片提取出路肩线的表达式，以及点
 * @param image 分割输出的二值图
 * @return 图中包含的多条路肩线上的点的像素坐标
 */
static std::vector<std::vector<cv::Point>> segToPoints(cv::Mat image) {
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
        Poly1D f = Poly1D(lane, 5);
        auto points = f.getUniformPoints();
        if (points.size() >= 2){
            results.push_back(points);
        }
    }

    return results;
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
    std::vector<std::vector<cv::Point>> results = segToPoints(image);

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

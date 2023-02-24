#include <opencv2/opencv.hpp>
#include "Polyfit.hpp"

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
    std::vector<std::vector<cv::Point>> results = seg_to_points(image);

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

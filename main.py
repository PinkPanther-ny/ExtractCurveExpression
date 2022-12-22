import argparse
import cv2
import numpy as np
import os

def process_image(image):
    # Dilate and erode to eliminate noise and connect intermittent lane lines
    kernel_size = 8
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    image = cv2.dilate(image, kernel, iterations=5)
    image = cv2.erode(image, kernel, iterations=5)

    # Find contours in the image
    contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Fit each contour with a polynomial curve using np.polyfit()
    lane_lines = []
    for contour in contours:
        # Extract x and y values from the contour
        x, y = contour.T[:,0]

        # Fit a polynomial curve to the contour
        curve_fit = np.polyfit(x, y, 5)

        # Extract the start and end x values from the curve fit
        start_x = min(x)
        end_x = max(x)

        # Add the start and end x values and the curve fit to the list of lane lines
        lane_lines.append((start_x, end_x, np.poly1d(curve_fit)))
    
    return lane_lines

def draw_fitted_lines(image, lane_lines):
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    
    black_board = np.zeros((image.shape[0], image.shape[1], 3), np.uint8)
    black_board_no_text = np.zeros((image.shape[0], image.shape[1], 3), np.uint8)
    
    # Re-draw the fitted lane lines on the original image
    for lane_line in lane_lines:
        start_x, end_x, curve_fit = lane_line
        y_values = curve_fit(range(start_x, end_x))
        y_values = curve_fit(range(start_x, end_x))
        points = np.array([range(start_x, end_x), y_values]).T.reshape(-1, 1, 2)
        cv2.polylines(black_board_no_text, np.int32([points]), isClosed=False, color=(255, 255, 255))
        
        # Draw the curve expression string next to the lane line
        cv2.polylines(black_board, np.int32([points]), isClosed=False, color=(255, 255, 255))
        curve_expression_str = " ".join([f"{'+' if coeff >=0 else ''}{coeff:.1e}x^{exp}" for exp, coeff in enumerate(curve_fit)])
        cv2.putText(black_board, curve_expression_str, (end_x, int(y_values[-1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
    return image, black_board_no_text, black_board

if __name__ == '__main__':
    
    DEFAULT_INPUT_DIR = "data/mask"
    DEFAULT_OUTPUT_DIR = "data/out"

    # Create the directory if it does not exist
    if not os.path.exists(DEFAULT_OUTPUT_DIR):
        os.makedirs(DEFAULT_OUTPUT_DIR)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_dir', type=str, default=DEFAULT_INPUT_DIR, help='Directory path for input images')
    parser.add_argument('--output_dir', type=str, default=DEFAULT_OUTPUT_DIR, help='Directory path for output images')
    args = parser.parse_args()
    
    # Get the list of input images
    input_dir = args.input_dir
    input_images = os.listdir(input_dir)
    
    # Process each image and save the result to the output directory
    for im in input_images:

        # Read the image from a file
        image = cv2.imread(os.path.join(input_dir, im), cv2.IMREAD_GRAYSCALE)

        # Process the image to extract fitted lane lines
        lane_lines = process_image(image)
        result = draw_fitted_lines(image, lane_lines)
        
        # Save the result to a new file
        print(os.path.join(args.output_dir, im))
        cv2.imwrite(os.path.join(args.output_dir, im), cv2.vconcat(result))
       

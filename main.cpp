#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::VideoCapture cap(0); // Open the default camera
    if (!cap.isOpened()) {
        throw std::runtime_error("Error opening your webcam");
    }

    std::string asciiChars = " .\":-=+*#&%8@$BW\MNQ█"; // ASCII characters from lightest to darkest

    cv::Mat frame;
    cv::namedWindow("ASCII Art", cv::WINDOW_AUTOSIZE); // Create a window for display.

    while (true) {
        cap >> frame; // Capture each frame
        if (frame.empty()) {
            break;
        }

        // Convert to grayscale
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Resize the image
        cv::Mat resized;
        int scale = 5;
        cv::resize(gray, resized, cv::Size(), 1.0/scale, 1.0/scale, cv::INTER_LINEAR);

        // Create an image to display ASCII Art
        cv::Mat asciiImage = cv::Mat::zeros(resized.rows * 15, resized.cols * 10, CV_8UC3); // Adjust the size

        // Convert pixels to ASCII and draw on the image
        for (int i = 0; i < resized.rows; i++) {
            for (int j = 0; j < resized.cols; j++) {
                float pixelValue = resized.at<uchar>(i, j);
                int index = static_cast<int>((pixelValue / 255.0) * (asciiChars.length() - 1));
                std::string character(1, asciiChars[index]);
                cv::putText(asciiImage, character, cv::Point(j * 10, i * 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            }
        }

        // Display ASCII Art in the created window
        cv::imshow("ASCII Art", asciiImage);

        // Break loop on 'q' key press
        if (cv::waitKey(30) == 113) { // ASCII value for 'q' is 113
            break;
        }
    }
    return 0;
}

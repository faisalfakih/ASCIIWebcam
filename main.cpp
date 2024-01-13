#include <CoreAudio/CoreAudio.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <cmath>

bool toggleFingertips = true;
int boxSize = 300;

void setMacVolume(float volume) {
    // Get the default audio device
    AudioDeviceID defaultDevice;
    UInt32 size = sizeof(AudioDeviceID);
    AudioObjectPropertyAddress defaultDeviceProperty = {
            kAudioHardwarePropertyDefaultOutputDevice,
            kAudioObjectPropertyScopeGlobal,
            kAudioObjectPropertyElementMaster
    };
    AudioObjectGetPropertyData(kAudioObjectSystemObject, &defaultDeviceProperty, 0, NULL, &size, &defaultDevice);

    // Set the volume for the default audio device
    Float32 newVolume = volume;
    AudioObjectPropertyAddress volumeProperty = {
            kAudioDevicePropertyVolumeScalar,
            kAudioDevicePropertyScopeOutput,
            kAudioObjectPropertyElementMaster
    };
    AudioObjectSetPropertyData(defaultDevice, &volumeProperty, 0, NULL, sizeof(Float32), &newVolume);
}

float calculateDistance(const cv::Point& pt1, const cv::Point& pt2) {
    int dx = pt2.x - pt1.x;
    int dy = pt2.y - pt1.y;
    return std::sqrt(dx * dx + dy * dy);
}

void find_fingertips(cv::Mat& frame) {
    // ROI
    cv::Rect roi(100, 100, boxSize, boxSize);
    cv::Mat imgROI = frame(roi);

    // Convert to grayscale
    cv::Mat gray;
    cv::cvtColor(imgROI, gray, cv::COLOR_BGR2GRAY);

    // Apply Gaussian blur
    cv::GaussianBlur(gray, gray, cv::Size(35, 35), 0);

    // Threshold the image
    cv::Mat thresh;
    cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY_INV + cv::THRESH_OTSU);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Point> max_contour;
    double max_area = 0.0;
    for (auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area > max_area) {
            max_area = area;
            max_contour = contour;
        }
    }

    if (!max_contour.empty()) {
        // Find convex hull
        std::vector<int> hull_indices;
        cv::convexHull(max_contour, hull_indices);

        // Find convexity defects
        std::vector<cv::Vec4i> defects;
        cv::convexityDefects(max_contour, hull_indices, defects);

        cv::Point ptStart, ptEnd, ptFar;
        float maxDepth = 0;
        for (const auto& defect : defects) {
            float depth = defect[3] / 256.0f;
            if (depth > maxDepth) {
                maxDepth = depth;
                ptStart = max_contour[defect[0]];
                ptEnd = max_contour[defect[1]];
                ptFar = max_contour[defect[2]];
            }
        }

        // Draw the defect with max depth
        if (maxDepth > 0) {
            cv::line(imgROI, ptStart, ptEnd, cv::Scalar(0, 255, 0), 2);
            cv::circle(imgROI, ptStart, 10, cv::Scalar(0, 0, 255), 2);
            cv::circle(imgROI, ptEnd, 10, cv::Scalar(255, 0, 0), 2);

            // Calculate distance between fingertips
            float distance = calculateDistance(ptStart, ptEnd);

            // Set the mac volume based on the distance
            float volume = distance / std::sqrt(boxSize * boxSize + boxSize * boxSize);
            setMacVolume(volume);

            // Draw distance on the image
            std::stringstream ss;
            ss << "Distance: " << distance << " px";
            cv::putText(frame, ss.str(), cv::Point(frame.cols - 350, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);

            // Convert volume to string
            std::stringstream vol_ss;
            vol_ss << "Volume: " << volume * 100 << "%";
            std::string vol_str = vol_ss.str();

            // Draw volume on the image
            cv::putText(frame, vol_str, cv::Point(30, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
        }

        // Draw ROI rectangle on the original image
        cv::rectangle(frame, roi, cv::Scalar(0, 255, 0), 2);
    }
}

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cout << "Could not open the webcam" << std::endl;
        return -1;
    }

    bool toggleKeyPressed = false;
    while (true) {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) {
            std::cout << "Could not capture a frame" << std::endl;
            break;
        }

        if (toggleFingertips) {
            find_fingertips(frame);
        }

        cv::namedWindow("Webcam", cv::WINDOW_NORMAL);
        cv::imshow("Webcam", frame);

        int key = cv::waitKey(1);
        if (key == 'q') {
            break;
        } else if (key == 't' && !toggleKeyPressed) {
            toggleKeyPressed = true;
            toggleFingertips = !toggleFingertips;
        } else if (key != 't') {
            toggleKeyPressed = false;
        }
    }

    cap.release();

    return 0;
}

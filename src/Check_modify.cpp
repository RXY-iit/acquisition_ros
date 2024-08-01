#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Function to calculate the histogram
void calcHistogram(const Mat& image, Mat& hist) {
    // Parameters for the histogram calculation
    int histSize = 256;    // Number of bins
    float range[] = { 0, 256 }; // Range of intensity values
    const float* histRange = { range };

    // Calculate the histogram
    calcHist(&image, 1, 0, Mat(), hist, 1, &histSize, &histRange);
}

// Function to draw and display the histogram with coordinate annotations
void drawHistogram(const Mat& hist) {
    int histHeight = 400;
    int histWidth = 512;
    Mat histImage(histHeight + 50, histWidth + 50, CV_8UC1, Scalar(255)); // Increased size for annotations
    normalize(hist, hist, 0, histHeight, NORM_MINMAX);

    int histSize = hist.rows;
    int binWidth = cvRound((double)histWidth / histSize);
    for (int i = 1; i < histSize; i++) {
        line(histImage,
             Point(binWidth * (i - 1) + 25, histHeight - cvRound(hist.at<float>(i - 1)) + 25),
             Point(binWidth * i + 25, histHeight - cvRound(hist.at<float>(i)) + 25),
             Scalar(0), 2);
    }

    // Draw x-axis and y-axis
    line(histImage, Point(25, 25), Point(25, histHeight + 25), Scalar(0), 2);
    line(histImage, Point(25, histHeight + 25), Point(histWidth + 25, histHeight + 25), Scalar(0), 2);

    // Annotate x-axis
    for (int i = 0; i <= 255; i += 51) { // 5 ticks
        line(histImage, Point(binWidth * i + 25, histHeight + 25), Point(binWidth * i + 25, histHeight + 30), Scalar(0), 2);
        putText(histImage, to_string(i), Point(binWidth * i + 20, histHeight + 45), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0), 1);
    }

    // Annotate y-axis
    for (int i = 0; i <= histHeight; i += histHeight / 4) { // 5 ticks
        line(histImage, Point(20, histHeight - i + 25), Point(25, histHeight - i + 25), Scalar(0), 2);
        putText(histImage, to_string(i), Point(0, histHeight - i + 30), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0), 1);
    }

    imshow("Histogram", histImage);
}

// Function to check for over-exposure and modify the image if needed
bool isOverExposed(const Mat& hist, Mat& image, int threshold = 255, double fraction = 0.01) {
    float totalPixels = sum(hist)[0];
    float overExposedPixels = hist.at<float>(threshold);

    // Debug
    float temp_overexposedValue = overExposedPixels / totalPixels;
    cout << "overExposedPixels / totalPixels is : " << temp_overexposedValue << endl;

    // Check if the fraction of over-exposed pixels is greater than the given fraction
    if (overExposedPixels / totalPixels > fraction) {
        // Modify the image to reduce over-exposure
        // image.setTo(threshold - 50, image == threshold); // Reduce intensity of over-exposed pixels
        return true;
    }
    return false;
}

int main(int argc, char** argv) {
    // Load an image
    // Mat image = imread("/home/ruan-x/rsworkSpace/src/acquisition/output/Brightness_125_3.jpg", IMREAD_GRAYSCALE);
    Mat image = imread("/home/ruan-x/Pictures/17_22_37_368.pgm", IMREAD_GRAYSCALE);

    if (image.empty()) {
        cout << "Could not open or find the image" << endl;
        return -1;
    }

    // Display the original image
    imshow("Original Image", image);

    // Calculate the histogram
    Mat hist;
    calcHistogram(image, hist);

    // Check for over-exposure and modify the image if needed
    bool overExposed = isOverExposed(hist, image);
    if (overExposed) {
        cout << "The image is over-exposed." << endl;
    } else {
        cout << "The image is not over-exposed." << endl;
    }

    // Draw and display the histogram
    drawHistogram(hist);

    waitKey(0); // Wait for a keystroke in the window
    return 0;
}

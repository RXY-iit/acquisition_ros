#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

Mat gammaCorrection(const Mat& image, double gamma) {
    Mat result;
    Mat lut(1, 256, CV_8UC1);

    for (int i = 0; i < 256; i++) {
        lut.at<uchar>(i) = saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }

    LUT(image, lut, result);
    return result;
}

Mat calculateHistogram(const Mat& image) {
    // Calculate the histogram
    Mat hist;
    int histSize = 256;
    float range[] = { 0, 256 };
    const float* histRange = { range };
    calcHist(&image, 1, 0, Mat(), hist, 1, &histSize, &histRange);

    // Normalize the histogram
    int histHeight = 256;
    Mat histImage(histHeight, histSize, CV_8UC1, Scalar(255));

    normalize(hist, hist, 0, histHeight, NORM_MINMAX);

    // Draw the histogram
    for (int i = 1; i < histSize; i++) {
        line(histImage, Point((i - 1), histHeight - cvRound(hist.at<float>(i - 1))),
             Point(i, histHeight - cvRound(hist.at<float>(i))),
             Scalar(0), 2);
    }

    return histImage;
}

Mat combineImagesWithHistograms(const Mat& original, const Mat& equalized, const Mat& gammaCorrected, const Mat& clahe, const Mat& enhanced) {
    // Calculate histograms
    Mat histOriginal = calculateHistogram(original);
    Mat histEqualized = calculateHistogram(equalized);
    Mat histGammaCorrected = calculateHistogram(gammaCorrected);
    Mat histCLAHE = calculateHistogram(clahe);
    Mat histEnhanced = calculateHistogram(enhanced);

    // Define the result image size
    int imageHeight = original.rows;
    int imageWidth = original.cols;
    int histHeight = 256*2;
    int histWidth = 256*3;

    // Ensure histograms have the correct size
    resize(histOriginal, histOriginal, Size(histWidth, histHeight));
    resize(histEqualized, histEqualized, Size(histWidth, histHeight));
    resize(histGammaCorrected, histGammaCorrected, Size(histWidth, histHeight));
    resize(histCLAHE, histCLAHE, Size(histWidth, histHeight));
    resize(histEnhanced, histEnhanced, Size(histWidth, histHeight));

    // Create a result image with appropriate size
    Mat result(imageHeight * 2, imageWidth * 5, CV_8UC1, Scalar(255));

    // Copy images to result
    original.copyTo(result(Rect(0, 0, imageWidth, imageHeight)));
    equalized.copyTo(result(Rect(imageWidth, 0, imageWidth, imageHeight)));
    gammaCorrected.copyTo(result(Rect(imageWidth * 2, 0, imageWidth, imageHeight)));
    clahe.copyTo(result(Rect(imageWidth * 3, 0, imageWidth, imageHeight)));
    enhanced.copyTo(result(Rect(imageWidth * 4, 0, imageWidth, imageHeight)));

    // Copy histograms to result
    histOriginal.copyTo(result(Rect(20, imageHeight+ 20, histWidth, histHeight)));
    histEqualized.copyTo(result(Rect(imageWidth+ 20, imageHeight+ 20, histWidth, histHeight)));
    histGammaCorrected.copyTo(result(Rect(imageWidth * 2+ 20, imageHeight + 20, histWidth, histHeight)));
    histCLAHE.copyTo(result(Rect(imageWidth* 3+ 20, imageHeight+ 20, histWidth, histHeight)));
    histEnhanced.copyTo(result(Rect(imageWidth* 4+ 20, imageHeight+ 20, histWidth, histHeight)));

    // // Add text labels for histograms
    putText(result, "Original", Point(20, imageHeight+histHeight+ 90), FONT_HERSHEY_SIMPLEX, 2, Scalar(0), 1);
    putText(result, "Equalized", Point(imageWidth+20, imageHeight+histHeight+ 90), FONT_HERSHEY_SIMPLEX, 2, Scalar(0), 1);
    putText(result, "Gamma Corrected", Point(imageWidth*2+20, imageHeight+histHeight+ 90), FONT_HERSHEY_SIMPLEX, 2, Scalar(0), 1);
    putText(result, "CLAHE", Point(imageWidth*3+20, imageHeight+histHeight+ 90), FONT_HERSHEY_SIMPLEX, 2, Scalar(0), 1);
    putText(result, "Combine CLAHE and Gamma", Point(imageWidth*4+20, imageHeight+histHeight+ 90), FONT_HERSHEY_SIMPLEX, 2, Scalar(0), 1);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Create a result image with appropriate size
    // Mat result(imageHeight * 5+10, imageWidth * 2+10, CV_8UC1, Scalar(255));

    // // Copy images and histograms to result
    // original.copyTo(result(Rect(0, 0, imageWidth, imageHeight)));
    // equalized.copyTo(result(Rect(0, imageHeight, imageWidth, imageHeight)));
    // gammaCorrected.copyTo(result(Rect(0, imageHeight * 2, imageWidth, imageHeight)));
    // clahe.copyTo(result(Rect(0, imageHeight, imageWidth*3, imageHeight)));
    // enhanced.copyTo(result(Rect(0, imageHeight, imageWidth*4, imageHeight)));

    // histOriginal.copyTo(result(Rect(imageWidth, 0, histWidth, histHeight)));
    // histEqualized.copyTo(result(Rect(imageWidth, imageHeight, histWidth, histHeight)));
    // histGammaCorrected.copyTo(result(Rect(imageWidth, imageHeight*2, histWidth, histHeight)));
    // histCLAHE.copyTo(result(Rect(imageWidth, imageHeight * 3, histWidth, histHeight)));
    // histEnhanced.copyTo(result(Rect(imageWidth, imageHeight * 4, histWidth, histHeight)));

    // // Add text labels for histograms
    // putText(result, "Original", Point(imageWidth + histWidth / 4, imageHeight), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0), 1);
    // putText(result, "Equalized", Point(imageWidth+ histWidth / 4, imageHeight*2- 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0), 1);
    // putText(result, "Gamma Corrected", Point(imageWidth + histWidth / 4, imageHeight * 3  - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0), 1);
    // putText(result, "CLAHE", Point(imageWidth + histWidth / 4, imageHeight * 4  - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0), 1);
    // putText(result, "Enhanced", Point(imageWidth + histWidth / 4, imageHeight * 5  - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0), 1);


    return result;
}




int main() {

    string folderPath = "/home/ruan-x/Videos";
    vector<String> imagePaths;
    // glob(folderPath + "/*.jpg", imagePaths, false);
    // glob(folderPath + "/*.png", imagePaths, false);
    glob(folderPath + "/*.pgm", imagePaths, false); 

    size_t currentIndex = 0;
    while (currentIndex < imagePaths.size()) {
        // Load the image
        Mat image = imread(imagePaths[currentIndex], IMREAD_GRAYSCALE);
        if (image.empty()) {
            cout << "Could not open or find the image: " << imagePaths[currentIndex] << endl;
            currentIndex++;
            continue;
        }

        // imshow("Original Image", image);
        
        /////*******************Method1****************************//////
        // Method1: Apply Histogram Equalization
        Mat equalizedImage;
        equalizeHist(image, equalizedImage);

        // Display equalized images
        // imshow("Equalized Image", equalizedImage);

        /////*******************Method2****************************//////
        // Apply Gamma Correction
        double gamma = 2.2; // Adjust this value as needed
        Mat correctedImage = gammaCorrection(image, gamma);

        // Display the corrected images
        // imshow("Gamma Corrected Image", correctedImage);

        /////*******************Method3****************************//////
        // Apply CLAHE
        Ptr<CLAHE> clahe = createCLAHE();
        clahe->setClipLimit(4.0); // Adjust this value as needed
        Mat claheImage;
        clahe->apply(image, claheImage);

        // Display the CLAHE images
        // imshow("CLAHE Image", claheImage);
        
        /////*******************Method4****************************//////
        // combine methods: applying CLAHE followed by gamma correction.
        // double gamma = 2.2; // use previous one
        Mat correctedImage2 = gammaCorrection(claheImage, gamma);

        // Display the original and enhanced images
        // imshow("Enhanced Image", correctedImage2);

        // Combine all images and histograms into one result image
        Mat result = combineImagesWithHistograms(image, equalizedImage, correctedImage, claheImage, correctedImage2);

        // Display the result image
        imshow("Result Image", result);

        // Save the result image
        // string outputPath = "/home/ruan-x/rsworkSpace/src/acquisition/analysis/Picture_enhence_1.png";
        // imwrite(outputPath, result);
        char key = waitKey(0);
        if (key == ' ') {
            currentIndex++;
        } else if (key == 's') {
            string outputPath = "/home/ruan-x/rsworkSpace/src/acquisition/analysis/Picture_enhence_Night_" + to_string(currentIndex) + ".png";
            imwrite(outputPath, result);
            cout << "Result saved to " << outputPath << endl;
        }
    }

    return 0;
}

#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <opencv2/opencv.hpp> // OpenCV library for video saving and image display
#include <iostream>
#include <vector>
#include <algorithm>
#include <sstream>
#include <ctime> // for time function
#include <thread>  // Include this for std::this_thread::sleep_for
#include <chrono>  // Include this for std::chrono functions
#include <ctime>  // for time function

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace std;

class CameraNode
{
public:
    CameraNode()
        : recording_(false),
          exposure_time_(5000.0), // Default values
          gain_(2.0),
          fps_(30.0),
        //   fps_(5.0),
          savePath_("/home/ruan-x/midTask/src/keyboard_acq/video/"),
          current_mode_("Auto Mode")
    {
        // Initialize ROS node
        nh_ = ros::NodeHandle();

        // Initialize Spinnaker
        system_ = System::GetInstance();
        camList_ = system_->GetCameras();

        if (camList_.GetSize() == 0)
        {
            ROS_ERROR("No camera detected.");
            camList_.Clear();
            system_->ReleaseInstance();
            ros::shutdown();
        }

        cam_ = camList_.GetByIndex(0);
        cam_->Init();

        // Set initial condition to auto mode
        EnableAutoExposureAndGain();

        // Start video recording
        StartRecording();

        // Subscribe to keyboard topic
        sub_ = nh_.subscribe("keyboard", 30, &CameraNode::keyboardCallback, this);
        // sub_ = nh_.subscribe("keyboard", 5, &CameraNode::keyboardCallback, this);

        // Initialize the start time for frame rate control
        start_time_ = std::chrono::high_resolution_clock::now();
    }

    ~CameraNode()
    {
        if (recording_)
        {
            StopRecording();
        }
        cam_->DeInit();
        cam_ = nullptr;
        camList_.Clear();
        system_->ReleaseInstance();
    }

    void StartRecording()
    {
        width_ = cam_->Width.GetValue();
        height_ = cam_->Height.GetValue();
        string videoFileName = savePath_ + "output.avi";

        videoWriter_.open(videoFileName, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps_, cv::Size(width_, height_));
        if (!videoWriter_.isOpened())
        {
            ROS_ERROR("Could not open the video file for writing.");
            ros::shutdown();
        }

        cam_->BeginAcquisition();
        recording_ = true;
        ROS_INFO("Recording started...");
    }

    void StopRecording()
    {
        videoWriter_.release();
        cam_->EndAcquisition();
        ROS_INFO("Recording stopped.");
    }

void keyboardCallback(const std_msgs::Char::ConstPtr& msg)
{
    // Get current image and value it
    ImagePtr image = cam_->GetNextImage(); // Timeout after 1000ms
    bool overExposed = false; // Initialize as false
    cv::Mat cvImage;

    if (image->IsIncomplete())
    {
        ROS_WARN("Image incomplete with image status %d", image->GetImageStatus());
        image->Release();
        return; // Exit early if the image is incomplete
    }

    try
    {
        unsigned int width = image->GetWidth();
        unsigned int height = image->GetHeight();

        // Convert ImagePtr to cv::Mat
        unsigned char* pData = static_cast<unsigned char*>(image->GetData());
        cvImage = cv::Mat(cv::Size(width, height), CV_8UC1, pData).clone(); // Clone to ensure ownership

        // Release the image
        image->Release();

        // Calculate the histogram
        cv::Mat hist;
        calcHistogram(cvImage, hist);

        // Check for over-exposure
        overExposed = isOverExposed(hist); // Assuming isOverExposed returns a tuple
        avgBrightness = calcAverageBrightness(cvImage);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error processing image: %s", e.what());
        return; // Exit early on error
    }

    // Update frame_start time
    frame_start = std::chrono::high_resolution_clock::now();

    char input = msg->data;

    switch (input)
    {
    case 'a':
        EnableAutoExposureAndGain();
        current_mode_ = "Auto Mode";
        break;
    case 'n':
        SetExposureAndGain(3000, 25.0); // Adjusted values
        current_mode_ = "Mode 1";
        break;
    case 'm':
        SetExposureAndGain(1500.0, 15.0);
        current_mode_ = "Mode 2";
        break;
    case 'h':
        MinimizeOverexposure(overExposed); // Overexposed parameter removed
        current_mode_ = "His_Bri Mode";
        break;
    case 's':
        SaveImage(cvImage);
        break;
    case 'q':
        ROS_INFO("Quitting...");
        recording_ = false;
        break;
    default:
        ROS_WARN("Invalid input. Please try again.");
        break;
    }

    if (!recording_)
    {
        StopRecording();
        return;
    }

    // Capture and write frame to video file
    CaptureAndWriteFrame(cvImage);
}


private:
    void SetExposureAndGain(double exposureTime, double gain)
    {
        try
        {
            // Turn off automatic exposure and gain
            cam_->ExposureAuto.SetValue(ExposureAuto_Off);
            cam_->GainAuto.SetValue(GainAuto_Off);

            // Ensure the exposure and gain values are within allowed ranges
            double minExposure = cam_->ExposureTime.GetMin();
            double maxExposure = cam_->ExposureTime.GetMax();
            double minGain = cam_->Gain.GetMin();
            double maxGain = cam_->Gain.GetMax();

            exposureTime = min(max(exposureTime, minExposure), maxExposure);
            gain = min(max(gain, minGain), maxGain);

            // Set exposure time and gain
            cam_->ExposureTime.SetValue(exposureTime);
            cam_->Gain.SetValue(gain);

            ROS_INFO("Exposure set to %f and gain set to %f.", exposureTime, gain);
            exposure_time_ = exposureTime;
            gain_ = gain;
        }
        catch (Spinnaker::Exception& e)
        {
            ROS_ERROR("Error: %s", e.what());
        }
    }

    void EnableAutoExposureAndGain()
    {
        try
        {
            cam_->ExposureAuto.SetValue(ExposureAuto_Continuous);
            cam_->GainAuto.SetValue(GainAuto_Continuous);
            
            // Ensure settings are applied
            cam_->ExposureAuto.GetValue(); // Trigger the read to apply settings
            cam_->GainAuto.GetValue();     // Trigger the read to apply settings
            
            exposure_time_ = cam_->ExposureTime.GetValue(); 
            gain_ = cam_->Gain.GetValue(); 
            ROS_INFO("Auto exposure and gain enabled.");
        }
        catch (Spinnaker::Exception& e)
        {
            ROS_ERROR("Error: %s", e.what());
        }
    }

// Function to check for over-exposure and modify the image if needed
bool isOverExposed(const cv::Mat& hist, int threshold = 255, double fraction = 0.01) {
    float totalPixels = cv::sum(hist)[0];
    float overExposedPixels = hist.at<float>(threshold);

    // Calculate the ratio of over-exposed pixels to total pixels
    ratio = overExposedPixels / totalPixels;

    // Check if the fraction of over-exposed pixels is greater than the given fraction
    return ratio > fraction;
}

// Function to calculate average brightness
double calcAverageBrightness(const cv::Mat& image) {
    return cv::mean(image)[0];
}

// Function to calculate the histogram
void calcHistogram(const cv::Mat& image, cv::Mat& hist) {
    int histSize = 256;    // Number of bins
    float range[] = { 0, 256 }; // Range of intensity values
    const float* histRange = { range };

    // Calculate the histogram
    cv::calcHist(&image, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange);
}

void MinimizeOverexposure(bool overExposed)
{
    try
    {
        // PID control parameters
        double kp = 0.1; // Proportional gain
        double ki = 0.01; // Integral gain
        double kd = 0.05; // Derivative gain
        static double previousError = 0.0;
        static double integral = 0.0;

        // Check if the camera is already acquiring
        if (!cam_->IsStreaming())
        {
            cam_->BeginAcquisition();
        }

            // Calculate PID error
            double error = avgBrightness - 100.0;
            integral += error;
            double derivative = error - previousError;
            previousError = error;

            double pidAdjustment = kp * error + ki * integral + kd * derivative;

            if (avgBrightness > 100.0 || overExposed)
            {
                // Significant adjustment
                pidAdjustment *= 2.0;
                ROS_INFO("During Significant adjustment");
            }
            else
            {
                // Small adjustment
                pidAdjustment *= 0.5;
                ROS_INFO("During Small adjustment");
            }

            if (gain_ > cam_->Gain.GetMax() || gain_ < cam_->Gain.GetMin())
            {
                double newExposure = max(cam_->ExposureTime.GetMin(), min(cam_->ExposureTime.GetMax(), exposure_time_ - pidAdjustment));
                SetExposureAndGain(newExposure, gain_);
            }
            else
            {
                double newGain = max(cam_->Gain.GetMin(), min(cam_->Gain.GetMax(), gain_ - pidAdjustment));
                SetExposureAndGain(exposure_time_, newGain);
            }
        
        // Output current average brightness and histogram ratio to the console
            // ROS_INFO("Average Brightness: %f, Overexposure Ratio: %f", avgBrightness, ratio);
        
        // image->Release();
        // cam_->EndAcquisition();
    }
    catch (Spinnaker::Exception& e)
    {
        ROS_ERROR("Error: %s", e.what());
    }
}


void SaveImage(const cv::Mat& image)
{
    try
    {
        // Generate a file name with the current time
        stringstream imgFileName;
        imgFileName << savePath_ << "image_" << time(0) << ".jpg";

        // Save the image using OpenCV
        if (cv::imwrite(imgFileName.str(), image))
        {
            ROS_INFO("Image saved: %s", imgFileName.str().c_str());
        }
        else
        {
            ROS_ERROR("Failed to save the image.");
        }
    }
    catch (const cv::Exception& e)  // Catch OpenCV exceptions
    {
        ROS_ERROR("Error: %s", e.what());
    }
}

void CaptureAndWriteFrame(const cv::Mat& input_image)
{
    try
    {
        // Check if the input image is empty
        if (input_image.empty())
        {
            ROS_ERROR("Input image is empty. Cannot process.");
            return;
        }
        // Convert the input image to BGR if it's grayscale
        cv::Mat frame;
        if (input_image.channels() == 1) {
            cv::cvtColor(input_image, frame, cv::COLOR_GRAY2BGR);
        } else {
            frame = input_image.clone(); // Create a copy of the input image
        }

        // Overlay exposure time, gain, and other information on the frame
        stringstream ss;
        ss << "Current Mode: " << current_mode_ << " Exposure: " << exposure_time_ << "us, Gain: " << gain_ << "dB";

        // Calculate elapsed time
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - start_time_;
        ss << " Time: " << elapsed.count() << "s";

        stringstream bh_info;
        bh_info << std::fixed << std::setprecision(5); // Ensure 5 digits after the decimal point
        bh_info << "Average Brightness: " << avgBrightness << "    Ratio: " << ratio;

        cv::putText(frame, ss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, bh_info.str(), cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        // Write the frame to the video file
        videoWriter_.write(frame);

        // Ensure we maintain the correct frame rate
        auto frame_end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> frame_duration = frame_end - frame_start;
        std::chrono::duration<double> target_duration = std::chrono::duration<double>(1.0 / fps_);
        auto sleep_duration = target_duration - frame_duration;

        if (sleep_duration.count() > 0)
        {
            std::this_thread::sleep_for(sleep_duration);
        // } else {
        //     ROS_WARN("Frame processing time exceeded target frame duration. Video frame rate may be lower than expected.");
        }
    }
    catch (const Spinnaker::Exception& e)
    {
        ROS_ERROR("Spinnaker Error: %s", e.what());
    }
    catch (const cv::Exception& e)
    {
        ROS_ERROR("OpenCV Error: %s", e.what());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Standard Exception: %s", e.what());
    }
}

    ros::NodeHandle nh_;
    ros::Subscriber sub_;

    SystemPtr system_;
    CameraList camList_;
    CameraPtr cam_;
    cv::VideoWriter videoWriter_;
    double exposure_time_;
    double gain_;
    bool recording_;
    double fps_;
    unsigned int width_;
    unsigned int height_;
    string savePath_;
    string current_mode_;
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point frame_start;
    double avgBrightness;
    float ratio;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_node");
    
    CameraNode camera_node;

    ros::spin();

    return 0;
}

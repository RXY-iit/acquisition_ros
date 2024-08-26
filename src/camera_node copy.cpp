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
        frame_start = std::chrono::high_resolution_clock::now();
        char input = msg->data;

        switch (input)
        {
        case 'a':
            EnableAutoExposureAndGain();
            current_mode_ = "Auto Mode";
            break;
        case 'n':
            // SetExposureAndGain(2500, 25.0);
            SetExposureAndGain(3000, 25.0);
            current_mode_ = "Mode 1";
            break;
        case 'm':
            SetExposureAndGain(1500.0, 15.0);
            current_mode_ = "Mode 2";
            break;
        case 'h':
            MinimizeOverexposure();
            current_mode_ = "Histogram Mode";
            break;
        case 's':
            SaveImage();
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
        CaptureAndWriteFrame();
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

    void MinimizeOverexposure()
    {
        // average brightness and histogram value based parameter change
        // thread  brightness: 110     histogram: 0.1     change logic: gain first
        try
        {
            cam_->BeginAcquisition();

            ImagePtr image = cam_->GetNextImage();

            if (image->IsIncomplete())
            {
                ROS_WARN("Image incomplete with image status %d", image->GetImageStatus());
            }
            else
            {
                unsigned int width = image->GetWidth();
                unsigned int height = image->GetHeight();

                unsigned char* pData = (unsigned char*)image->GetData();

                vector<int> histogram(256, 0);

                for (unsigned int i = 0; i < width * height; i++)
                {
                    histogram[pData[i]]++;
                }

                int peakBrightness = max_element(histogram.begin(), histogram.end()) - histogram.begin();

                if (peakBrightness > 200)
                {
                    double newExposure = max(1000.0, exposure_time_ * 0.9);
                    double newGain = max(0.0, gain_ * 0.9);

                    SetExposureAndGain(newExposure, newGain);
                }
            }

            image->Release();
            cam_->EndAcquisition();
        }
        catch (Spinnaker::Exception& e)
        {
            ROS_ERROR("Error: %s", e.what());
        }
    }

    void SaveImage()
    {
        try
        {
            ImagePtr image = cam_->GetNextImage();
            if (!image->IsIncomplete())
            {
                stringstream imgFileName;
                imgFileName << savePath_ << "image_" << time(0) << ".jpg";
                image->Save(imgFileName.str().c_str());
                ROS_INFO("Image saved: %s", imgFileName.str().c_str());
            }
            image->Release();
        }
        catch (Spinnaker::Exception& e)
        {
            ROS_ERROR("Error: %s", e.what());
        }
    }


void CaptureAndWriteFrame()
    {
        try
        {
            
            ImagePtr image = cam_->GetNextImage();
            if (!image->IsIncomplete())
            {
                unsigned char* pData = static_cast<unsigned char*>(image->GetData());
                cv::Mat frame(cv::Size(width_, height_), CV_8UC1, pData);
                cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);

                // Overlay exposure time and gain on the frame
                stringstream ss;
                ss << "Current Mode: " << current_mode_ << " Exposure: " << exposure_time_ << "us, Gain: " << gain_ << "dB";

                // Calculate elapsed time
                auto now = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = now - start_time_;
                ss << " Time: " << elapsed.count() << "s";

                cv::putText(frame, ss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

                // Write the frame to the video file
                videoWriter_.write(frame);
            }
            image->Release();

            // Ensure we maintain the correct frame rate
            auto frame_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> frame_duration = frame_end - frame_start;
            std::chrono::duration<double> target_duration = std::chrono::duration<double>(1.0 / fps_);
            std::this_thread::sleep_for(target_duration - frame_duration);
        }
        catch (Spinnaker::Exception& e)
        {
            ROS_ERROR("Error: %s", e.what());
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
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_node");
    
    CameraNode camera_node;

    ros::spin();

    return 0;
}

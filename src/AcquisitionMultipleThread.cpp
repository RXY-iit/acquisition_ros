#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <mutex>

#ifndef _WIN32
#include <pthread.h>
#endif

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

std::mutex g_mutex;

// Function to save image with thread-safe file naming
void SaveImage(ImagePtr convertedImage, const std::string& filename) {
    std::lock_guard<std::mutex> lock(g_mutex); // Lock mutex to ensure thread-safe file operations
    convertedImage->Save(filename.c_str());
    cout << "Image saved at " << filename << endl;
}

// Thread function to acquire and process images
void AcquireImages(CameraPtr pCam, int threadId) {
    try {
        // Initialize camera
        pCam->Init();

        // Begin acquisition
        pCam->BeginAcquisition();

        const unsigned int k_numImages = 10;
        ImagePtr pResultImage;

        // Process images
        for (unsigned int imageCnt = 0; imageCnt < k_numImages; imageCnt++) {
            // Retrieve next image
            pResultImage = pCam->GetNextImage();

            // Process image (convert to desired format, etc.)
            ImageProcessor processor;
            processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
            ImagePtr convertedImage = processor.Convert(pResultImage, PixelFormat_Mono8);

            // Create unique filename
            stringstream filename;
            filename << "/home/ruan-x/rsworkSpace/src/acquisition/output/";
            filename << "AcquisitionMultipleThread-" << threadId << "-" << imageCnt << ".jpg";

            // Save image (use thread-safe function)
            SaveImage(convertedImage, filename.str());

            // Release image resources
            pResultImage->Release();
        }

        // End acquisition
        pCam->EndAcquisition();
        pCam->DeInit();
    }
    catch (Spinnaker::Exception& e) {
        cout << "Error: " << e.what() << endl;
    }
}

// Main function
int main() {
    // Initialize Spinnaker system
    SystemPtr system = System::GetInstance();

    // Retrieve list of cameras
    CameraList camList = system->GetCameras();
    unsigned int numCameras = camList.GetSize();

    cout << "Number of cameras detected: " << numCameras << endl;

    // Start acquisition threads for each camera
    std::thread thread1(AcquireImages, camList.GetByIndex(0), 1); // Start thread for camera 1
    std::thread thread2(AcquireImages, camList.GetByIndex(1), 2); // Start thread for camera 2

    // Join threads to wait for completion
    thread1.join();
    thread2.join();

    // Clean up
    camList.Clear();
    system->ReleaseInstance();

    cout << "All threads have completed." << endl;

    return 0;
}

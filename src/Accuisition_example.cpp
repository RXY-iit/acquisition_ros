#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <unordered_set>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace std;
using namespace std::chrono;

// Function to calculate the brightness of an image
double CalculateBrightness(const ImagePtr& image)
{
    unsigned char* pData = (unsigned char*)image->GetData();
    int width = image->GetWidth();
    int height = image->GetHeight();
    int numPixels = width * height;
    
    double brightness = 0.0;
    for (int i = 0; i < numPixels; ++i)
    {
        brightness += pData[i];
    }
    return brightness / numPixels;
}

// Function to save an image to disk
void SaveImage(const ImagePtr& image, const std::string& filename)
{
    try
    {
        image->Save(filename.c_str());
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error saving image: " << e.what() << endl;
    }
}

int main(int /*argc*/, char** /*argv*/)
{
    SystemPtr system = System::GetInstance();
    CameraList camList = system->GetCameras();
    unsigned int numCameras = camList.GetSize();

    if (numCameras == 0)
    {
        camList.Clear();
        system->ReleaseInstance();
        cout << "No cameras detected." << endl;
        return -1;
    }

    CameraPtr pCam = camList.GetByIndex(0);

    try
    {
        pCam->Init();
        pCam->BeginAcquisition();

        unordered_set<int> targetBrightnesses = {5,10,20,35,50 ,82 ,85 ,90, 100, 125, 135, 145, 150, 160, 175, 200, 225};
        unordered_set<int> capturedBrightnesses;

        while (capturedBrightnesses.size() < targetBrightnesses.size())
        {
            ImagePtr pResultImage = pCam->GetNextImage();

            if (pResultImage->IsIncomplete())
            {
                cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << endl;
            }
            else
            {
                double currentBrightness = CalculateBrightness(pResultImage);
                // change exposuretime and Gain
                INodeMap& nodeMap = pCam->GetNodeMap();
                CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
                CFloatPtr ptrGain = nodeMap.GetNode("Gain");
                double exposureTime = ptrExposureTime->GetValue();
                double gain = ptrGain->GetValue();

                cout << "Current Brightness: " << currentBrightness << "      Exposuretime: " << exposureTime << "      Gain: " << gain << endl;
                
                ImageProcessor processor;
                // Well-balanced speed and quality.
                // processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
                // Weighted pixel average from different directions.
                processor.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_WEIGHTED_DIRECTIONAL_FILTER);
                int roundedBrightness = static_cast<int>(currentBrightness);
                if (targetBrightnesses.find(roundedBrightness) != targetBrightnesses.end() &&
                    capturedBrightnesses.find(roundedBrightness) == capturedBrightnesses.end())
                {
                    capturedBrightnesses.insert(roundedBrightness);
                    // patern 1
                    std::string filename = "/home/ruan-x/rsworkSpace/src/acquisition/output/Brightness_" + std::to_string(roundedBrightness) + ".jpg";
                    ImagePtr imageCopy = processor.Convert(pResultImage, PixelFormat_Mono8);//8BITに圧縮している
                    // patern 2
                    // std::string filename = "/home/ruan-x/rsworkSpace/src/acquisition/output/Brightness_" + std::to_string(roundedBrightness) + "_12bit.jpg";
                    // ImagePtr imageCopy = processor.Convert(pResultImage, PixelFormat_Mono12p);//16BITに圧縮している
                    SaveImage(imageCopy, filename);
                    cout << "Saved image: " << filename << endl;
                }

                pResultImage->Release();
            }
        }

        pCam->EndAcquisition();
        pCam->DeInit();
    }
    catch (Spinnaker::Exception& e)
    {
        cout << "Error: " << e.what() << endl;
        pCam->EndAcquisition();
        pCam->DeInit();
    }

    camList.Clear();
    system->ReleaseInstance();
    return 0;
}

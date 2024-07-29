#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <fstream>

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
    image->Save(filename.c_str());
}

// Function to get the current time as a string
std::string GetCurrentTimeString()
{
    auto now = system_clock::now();
    auto in_time_t = system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
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

        bool timing = false;
        time_point<system_clock> startTime, endTime;
        vector<ImagePtr> images;

        while (true)
        {
            ImagePtr pResultImage = pCam->GetNextImage();

            if (pResultImage->IsIncomplete())
            {
                cout << "Image incomplete with image status " << pResultImage->GetImageStatus() << endl;
            }
            else
            {
                double currentBrightness = CalculateBrightness(pResultImage);
                cout << "Current Brightness: " << currentBrightness << endl;

                if (currentBrightness < 80 || currentBrightness > 150)
                {
                    if (!timing)
                    {
                        timing = true;
                        startTime = system_clock::now();
                        cout << "Timing started." << endl;
                    }
                    images.push_back(pResultImage);
                }
                else
                {
                    if (timing)
                    {
                        timing = false;
                        endTime = system_clock::now();
                        cout << "Timing ended." << endl;

                        duration<double> elapsedSeconds = endTime - startTime;
                        cout << "Elapsed time: " << elapsedSeconds.count() << " seconds." << endl;

                        // Save images
                        // for (size_t i = 0; i < images.size(); ++i)
                        // {
                        //     std::string filename = "/home/ruan-x/rsworkSpace/src/acquisition/output/Image_" + std::to_string(i) + "_" + GetCurrentTimeString() + ".jpg";
                        //     SaveImage(images[i], filename);
                        //     cout << "Saved image: " << filename << endl;
                            
                        // }

                        images.clear();
                        pResultImage->Release();
                        break;// count only once
                    }
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
    }

    camList.Clear();
    system->ReleaseInstance();
    return 0;
}

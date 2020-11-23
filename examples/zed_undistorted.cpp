// ----> Includes
#include <iostream>
#include <sstream>
#include <string>
#include <cctype>
#include <boost/program_options.hpp>

#include "videocapture.hpp"

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

// Sample includes
#include "calibration.hpp"
// <---- Includes

namespace po = boost::program_options;

void showImage( std::string name, cv::Mat& img, sl_oc::video::RESOLUTION res);

int main(int argc, char** argv) {
    sl_oc::video::VideoParams params;
    std::string resolution;
    std::string fps;
    
    std::string side_camera;

    std::string calibration_file;

    try
    {
        po::options_description desc{"Options"};
        desc.add_options()
            ("help,h", "Help screen")
            ("resolution,r", po::value<std::string>()->default_value("VGA"), "Camera supported resolutions: \nHD2K - 4416x1242\nHD1080 - 3840x1080\nHD720 - 2560x720\nVGA - 1344x376")
            ("fps,f", po::value<std::string>()->default_value("FPS_15"), "Frames per second")
            ("side,s", po::value<std::string>(), "Right/Left")
            ("camera-calib,cc", po::value<std::string>(), "Calibration file path");

        po::variables_map vm;
        store(parse_command_line(argc, argv, desc), vm);
        notify(vm);

        if (vm.count("help"))
        {
            std::cout << desc << std::endl;
            return 0;
        }

        if (vm.count("resolution"))
        {
            resolution = vm["resolution"].as<std::string>();
            if (resolution == "HD2K")
                params.res = sl_oc::video::RESOLUTION::HD2K;
            else if (resolution == "HD1080")
                params.res = sl_oc::video::RESOLUTION::HD1080;
            else if (resolution == "HD720")
                params.res = sl_oc::video::RESOLUTION::HD720;
            else if (resolution == "VGA")
                params.res = sl_oc::video::RESOLUTION::VGA;
            else
            {
                std::cerr << "Can't apply input resolution, need: 'HD2K/HD1080/HD720/VGA'.\nUse -h for more info" << std::endl;
                return 0;
            }   

            std::cout << "Resolution save to: " + resolution << std::endl;
        }

        if (vm.count("fps"))
        {
            fps = vm["fps"].as<std::string>();
            if (fps == "100" || fps == "FPS_100")
                params.fps = sl_oc::video::FPS::FPS_100;
            else if (fps == "60" || fps == "FPS_60")
                params.fps = sl_oc::video::FPS::FPS_60;
            else if (fps == "50" || fps == "FPS_50")
                params.fps = sl_oc::video::FPS::FPS_50;
            else if (fps == "40" || fps == "FPS_40")
                params.fps = sl_oc::video::FPS::FPS_40;
            else if (fps == "30" || fps == "FPS_30")
                params.fps = sl_oc::video::FPS::FPS_30;
            else if (fps == "20" || fps == "FPS_20")
                params.fps = sl_oc::video::FPS::FPS_20;
            else if (fps == "15" || fps == "FPS_15")
                params.fps = sl_oc::video::FPS::FPS_15;
            else if (fps == "10" || fps == "FPS_10")
                params.fps = sl_oc::video::FPS::FPS_10;
            else
            {
                std::cerr << "Can't apply fps, need: '10/15/20/30/40/50/60/100'.\nUse -h for more info" << std::endl;
                return 0;
            }   

            std::cout << "FPS save to: " + fps << std::endl;
        }

        if (vm.count("side"))
        {
            side_camera = vm["side"].as<std::string>();
            if (side_camera == "") 
            {
                std::cout << "Choose side camera" << std::endl;
            }
        }

        if (vm.count("camera-calib"))
        {
            calibration_file = vm["camera-calib"].as<std::string>();
        }
    }
    catch (const po::error &ex)
    {
        std::cerr << ex.what() << std::endl << "Print -h for help\n";
        return 0;
    }

    // ----> Create Video Capture
    sl_oc::video::VideoCapture cap(params);
    if ( !cap.initializeVideo(-1) )
    {
        std::cerr << "Cannot open camera video capture" << std::endl;
        std::cerr << "See verbosity level for more details." << std::endl;

        return EXIT_FAILURE;
    }
    int sn = cap.getSerialNumber();
    std::cout << "Connected to camera sn: " << sn << std::endl;
    // <---- Create Video Capture

    // ----> Frame size
    int w,h;
    cap.getFrameSize(w,h);
    // <---- Frame size

    cv::Mat distCoeff;
    cv::Mat cameraMatrix;

    cv::FileStorage camera_calibration(calibration_file, cv::FileStorage::READ);
    
    if (camera_calibration.isOpened())
    {
        camera_calibration["K"] >> cameraMatrix;
        camera_calibration["D"] >> distCoeff;
        std::cout << " Camera Matrix K: \n" << cameraMatrix << std::endl << std::endl;
        std::cout << " Camera Distortion coeff D: \n" << distCoeff << std::endl << std::endl;
    } else {
        std::cout << "Could not open calibration file, check path";
        return 0;
    }
    cv::Mat frameBGR, raw, undist;

    uint64_t last_ts=0;
    std::string image_name;
    std::string dir_name;

    // Infinite video grabbing loop
    while (1)
    {
        // Get a new frame from camera
        const sl_oc::video::Frame frame = cap.getLastFrame();

        if (frame.data!=nullptr && frame.timestamp!=last_ts)
        {
            last_ts = frame.timestamp;

            // ----> Conversion from YUV 4:2:2 to BGR for visualization
            cv::Mat frameYUV = cv::Mat( frame.height, frame.width, CV_8UC2, frame.data );
            cv::cvtColor(frameYUV,frameBGR,cv::COLOR_YUV2BGR_YUYV);
            // <---- Conversion from YUV 4:2:2 to BGR for visualization

            // ----> Extract left and right images from side-by-sideq
            if (side_camera == "right" || side_camera == "r")
            {
                raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));
            } else if (side_camera == "left" || side_camera == "l") {
                raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
            } else {
                std::cout << "Incorrect chose side of camera" << std::endl;
                return 0;
            }
            showImage("Raw", raw, params.res);

            undistort(raw, undist, cameraMatrix, distCoeff);
            
            showImage("Undistorted", undist, params.res);
        }

    int key = cv::waitKey( 5 );
    if(key=='q' || key=='Q') // Quit
        break;

    }

    return EXIT_SUCCESS;
}

void loadCameraCalibration(cv::Mat &distCoeff)
{

}

// Rescale the images according to the selected resolution to better display them on screen
void showImage( std::string name, cv::Mat& img, sl_oc::video::RESOLUTION res )
{
    cv::Mat resized;
    switch(res)
    {
    default:
    case sl_oc::video::RESOLUTION::VGA:
        resized = img;
        break;
    case sl_oc::video::RESOLUTION::HD720:
        name += " [Resize factor 0.6]";
        cv::resize( img, resized, cv::Size(), 0.6, 0.6 );
        break;
    case sl_oc::video::RESOLUTION::HD1080:
    case sl_oc::video::RESOLUTION::HD2K:
        name += " [Resize factor 0.4]";
        cv::resize( img, resized, cv::Size(), 0.4, 0.4 );
        break;
    }

    cv::imshow( name, resized );
}

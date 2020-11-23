// ----> Includes
#include <iostream>
#include <sstream>
#include <string>
#include <pwd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <boost/program_options.hpp>

#include "videocapture.hpp"

#include "calibration.hpp"

#include <opencv2/opencv.hpp>


namespace po = boost::program_options;

void showImage(std::string name, cv::Mat& img, sl_oc::video::RESOLUTION res);

int main(int argc, char** argv) {
    std::string homedir = getpwuid(getuid())->pw_dir;

    sl_oc::video::VideoParams params;
    std::string resolution;
    std::string fps;
    std::string calibration_flag;

    std::string calibration_file;

    try
    {
        po::options_description desc{"Options"};
        desc.add_options()
            ("help,h", "Help screen")
            ("resolution,r", po::value<std::string>()->default_value("VGA"), "Camera supported resolutions: \nHD2K - 4416x1242\nHD1080 - 3840x1080\nHD720 - 2560x720\nVGA - 1344x376")
            ("fps,f", po::value<std::string>()->default_value("FPS_15"), "Frames per second")
            ("save-dir,sd", po::value<std::string>(), "Main directory for saving the dataset")
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

        if (vm.count("save-dir"))
        {
            homedir = vm["save-dir"].as<std::string>();
            std::cout << "Dataset save directory is set to: " + homedir << std::endl;
        }

        if (vm.count("camera-calib"))
            calibration_flag = vm["camera-calib"].as<std::string>();
        else 
            calibration_flag = "";
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

    cv::Mat map_left_x, map_left_y;
    cv::Mat map_right_x, map_right_y;
    cv::Mat cameraMatrix_left, cameraMatrix_right;

    if (calibration_flag == "d" || calibration_flag == "download")
    {
        unsigned int serial_number = sn;
        if( !downloadCalibrationFile(serial_number, calibration_file) )
        {
            std::cerr << "Could not load calibration file from Stereolabs servers" << std::endl;
            return EXIT_FAILURE;
        }
        std::cout << "Calibration file found. Loading..." << std::endl;
    } else if (calibration_flag == "") 
    {
        std::cout << "Camera works without calibration" << std::endl;
    } else 
    {
        bool calibration = initCalibration(calibration_flag, cv::Size(w/2,h), map_left_x, map_left_y, map_right_x, map_right_y,
                    cameraMatrix_left, cameraMatrix_right);
        if (!calibration)
        {
            std::cout << "Check calibration path. Used: " + calibration_flag << std::endl;
            return 0;
        } else
        {
            std::cout << "Сamera calibration successfully loaded" << std::endl;
            std::cout << " Camera Matrix L: \n" << cameraMatrix_left << std::endl << std::endl;
            std::cout << " Camera Matrix R: \n" << cameraMatrix_right << std::endl << std::endl;
        } 
    }

    cv::Mat frameBGR, left_raw, left_rect, right_raw, right_rect;

    uint64_t last_ts=0;
    int image_counter = 0;
    std::string image_name;
    std::string dir_name;

    dir_name = homedir + "/dataset_zed_images";
    char* dir = new char[dir_name.size() + 1];
    std::strcpy (dir, dir_name.c_str());
    mkdir(dir, 0777);

    for (int dir_counter; dir_counter < 100; dir_counter++)
    {
        dir_name = "00";
        dir_name.replace(dir_name.size() - std::to_string(dir_counter).size(), std::to_string(dir_counter).size(), std::to_string(dir_counter));
        
        dir_name = homedir + "/dataset_zed_images/" + dir_name + "/";
        char* dir = new char[dir_name.size() + 1];
        std::strcpy (dir, dir_name.c_str());

        if (!mkdir(dir, 0777))
        {
            std::string images_dir = dir_name + "image_2";
            dir = new char[images_dir.size() + 1];
            std::strcpy (dir, images_dir.c_str());
            mkdir(dir, 0777);

            images_dir = dir_name + "image_3";
            dir = new char[images_dir.size() + 1];
            std::strcpy (dir, images_dir.c_str());
            mkdir(dir, 0777);

            break;
        }
    }
    std::cout << "Dataset number: " + dir_name << std::endl;


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
            left_raw = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
            right_raw = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));

            showImage("left RAW", left_raw, params.res);
            showImage("right RAW", right_raw, params.res);
            
            int key = cv::waitKey( 5 );
            if(key=='C' || key=='c') 
            {
                image_name = dir_name + "image_2/" + std::to_string(image_counter) + ".png";
                cv::imwrite(image_name, left_raw);

                image_name = dir_name + "image_3/" + std::to_string(image_counter) + ".png";
                cv::imwrite(image_name, right_raw);

                std::cout << "Frame saved: " + std::to_string(image_counter) << "\n";

                image_counter += 1;
            }
        }
    }

    return EXIT_SUCCESS;
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

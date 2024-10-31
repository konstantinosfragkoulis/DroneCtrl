#include <chrono>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <queue>

#include <opencv2/imgcodecs.hpp>

#include "DroneCtrl.hpp"

namespace DroneCtrl {

    Camera::Camera() : capturing(false) {}

    Camera::~Camera() {
        releaseCamera();
    }

    bool Camera::initializeCamera(const std::string& device) {
        cap.open(device);
        if(!cap.isOpened()) {
            fprintf(stderr, "Error: Unable to open camera device %s\n", device.c_str());
            return false;
        }

        capturing = true;
        saving = true;
        captureThread = std::thread(&Camera::captureLoop, this);
        saveThread = std::thread(&Camera::saveLoop, this);
        return true;
    }

    void Camera::captureLoop() {
        while(capturing) {
            cv::Mat tempFrame;
            if(cap.read(tempFrame)) {
                if(tempFrame.empty()) {
                    fprintf(stderr, "Warning: Captured an empty frame.\n");
                    continue;
                }

                std::lock_guard<std::mutex> lock(imageMutex);
                tempFrame.copyTo(frame);
            } else {
                fprintf(stderr, "Failed to capture frame in thread.\n");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void Camera::saveLoop() {
        while(saving) {
            cv::Mat frameToSave;
            std::string filepath;

            {
                std::unique_lock<std::mutex> lock(saveMutex);
                saveCondition.wait(lock, [this] { return !frameQueue.empty() || !saving; });

                if(!saving && frameQueue.empty()) {
                    break;
                }

                frameToSave = frameQueue.front().first;
                filepath = frameQueue.front().second;
                frameQueue.pop();
            }

            try {
                std::filesystem::create_directories("./img/");
            } catch(const std::filesystem::filesystem_error& e) {
                fprintf(stderr, "Error creating directory ./img/: %s\n", e.what());
                continue;
            }

            if(cv::imwrite(filepath, frameToSave)) {
                fprintf(stdout, "Frame saved successfully to %s\n", filepath.c_str());
            } else {
                fprintf(stderr, "Failed to save frame to %s\n", filepath.c_str());
            }
        }
    }

    bool Camera::getFrame(cv::Mat& outFrame) {
        std::lock_guard<std::mutex> lock(imageMutex);
        if(!frame.empty()) {
            frame.copyTo(outFrame);
            return true;
        }
        return false;
    }

    bool Camera::saveFrameToPNG(const std::string& filename) {
        cv::Mat currentFrame;
        if(getFrame(currentFrame)) {
            std::string filepath;

            if(!filename.empty()) {
                filepath = "./img/" + filename;
            } else {
                auto now = std::chrono::system_clock::now();
                std::time_t now_time = std::chrono::system_clock::to_time_t(now);
                std::tm tm_now;

                #ifdef _WIN32
                    localtime_s(&tm_now, &now_time);
                #else
                    localtime_r(&now_time, &tm_now);
                #endif

                std::stringstream ss;
                ss << "./img/" << std::put_time(&tm_now, "%Y%m%d_%H%M%S");

                auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch()) % 1000;
                ss << "_" << std::setw(3) << std::setfill('0') << milliseconds.count();
                ss << ".png";

                filepath = ss.str();
            }

            {
                std::lock_guard<std::mutex> lock(saveMutex);
                frameQueue.push(std::make_pair(currentFrame, filepath));
            }
            saveCondition.notify_one();

            return true;
        } else {
            fprintf(stderr, "No frame available to save.\n");
            return false;
        }
    }

    void Camera::releaseCamera() {
        capturing = false;
        saving = false;
        saveCondition.notify_all();

        if(captureThread.joinable()) {
            captureThread.join();
        }
        if(saveThread.joinable()) {
            saveThread.join();
        }
        if(cap.isOpened()) {
            cap.release();
        }
    }

}
#ifndef DRONECTRL_H
#define DRONECTRL_H

#include <string>
#include <array>
#include <fstream>
#include <sys/mman.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <condition_variable>

#include <opencv2/opencv.hpp>

namespace DroneCtrl {

    extern double dt;
    extern std::string VERSION;
    extern bool running;

    extern bool SET_SPEED; // User inputs desired speed or position
    
    extern float MAX_SPEED;
    extern float MAX_ACCELERATION;

    extern float takeOffHeight;

    class FlightData;
    class MagicData;
    class InputData;
    class IOData;
    class Camera;
    class Physics;
    class Blackbox;
    class StateManager;

    extern FlightData fd;
    extern MagicData md;
    extern InputData id;
    extern IOData io;
    extern Camera camera;
    extern Physics physics;
    extern Blackbox bb;
    extern StateManager sm;

    enum class State {
        Disarmed = 0,
        Grounded = 1,
        TakingOff = 2,
        Flying = 3,
        Landing = 4
    };

    enum class fState {
        Hovering = 0,
        FollowingObject = 1,
        FlyingForward = 2,
        StabilizedHover = 3,
        ManualControl = 4
    };

    // Estimated current state of the drone
    class FlightData {
    public:
        std::array<float, 3> pos = {0, 0, 0};
        std::array<float, 3> v = {0, 0, 0};
        float w = 0.0;
        std::array<float, 3> a = {0, 0, 0};

        float battery = 100.0;
        
        float thrust = 0.0;
        float theta = 0.0; // Pitch angle
        float phi = 0.0; // Roll angle

        int roll = 0; // CRSF values
        int pitch = 0; // CRSF values
        int yaw = 0; // CRSF values
        int throttle = 0; // CRSF values
    };

    class MagicData {
    public:
        std::array<float, 3> targetPos = {0, 0, 0};
        std::array<float, 3> targetV = {0, 0, 0};
        float targetYaw = 0.0;

        std::array<float, 3> dv = {0, 0, 0};

        std::array<float, 3> a = {0, 0, 0};

        std::array<float, 3> F = {0, 0, 0};
        float thrust = 0.0;

        float theta = 0.0; // Pitch angle
        float phi = 0.0; // Roll angle

        double timer = 0.0;
    };

    // Desired state of the drone
    class InputData {
    public:
        std::array<float, 3> pos = {0, 0, 0}; // Not implemented yet
        std::array<float, 3> v = {0, 0, 0};
        float yaw = 0.0;

        std::array<int, 2> objCenter = {0, 0};
    };

    class IOData {
    public:
        void* shmPtr;
        const char* SHM_NAME = "/myshm";
        const size_t SHM_SIZE = 64;
        int values[16];
        int shmFd;

        void* guiShmPtr;
        const char* GUI_SHM_NAME = "/guiSharedMem";
        const size_t GUI_SHM_SIZE = 256;
        const size_t GUI_RX_SIZE = 128;
        const size_t GUI_TX_SIZE = 128;
        float guiRx[32];
        float guiTx[32];
        int guiShmFd;

        IOData();
        ~IOData();

        int initSharedMemory();
        void readSharedMemory();
        void passValues(const int (&values)[16]);
        int closeSharedMemory();

        int initGUISharedMemory();
        void readGUISharedMemory();
        void writeGUISharedMemory();
        int closeGUISharedMemory();
    };

    class Camera {
    public:
        int width = 640;
        int height = 480;
        int widthD2 = width / 2;
        int heightD2 = height / 2;

        Camera();
        ~Camera();

        bool initializeCamera(const std::string& device = "/dev/video2");
        void captureLoop();
        void saveLoop();
        bool getFrame(cv::Mat& outFrame);
        bool saveFrameToPNG(const std::string& filename);
        void releaseCamera();
    
    private:
        cv::VideoCapture cap;
        cv::Mat frame;
        std::thread captureThread;
        std::mutex imageMutex;
        std::atomic<bool> capturing;

        std::thread saveThread;
        std::queue<std::pair<cv::Mat, std::string>> frameQueue;
        std::mutex saveMutex;
        std::condition_variable saveCondition;
        bool saving;
    };

    class Physics {
    public:
        float PI = 3.14159;
        float RHO = 1.225;
        float G = 10;
        float DEG2RAD = PI / 180;
        float RAD2DEG = 180 / PI;
        float MASS = 0.32;
    };

    class Blackbox {
    public:
        ~Blackbox();

        std::string getCurrentTimestamp();
        void writeFlightDataToCSV();
        void initializeCSV(const std::string& filename);
        void closeCSV();
        void initializeBlackbox();
    
    private:
        std::string timestamp;
        std::ofstream flightDataFile;
    };

    class StateManager {
    private:
        State currentState;
        fState currentSubState;

    public:
        StateManager() : currentState(State::Disarmed), currentSubState(fState::Hovering) {}

        void setState(State newState) {
            currentState = newState;
            if(newState != State::Flying) {
                currentSubState = fState::Hovering;
            }
        }

        State getState() const {
            return currentState;
        }

        void setSubState(fState newSubState) {
            if(currentState == State::Flying) {
                currentSubState = newSubState;
            }
        }

        fState getSubState() const {
            return currentSubState;
        }
    };

}

#endif // DRONECTRL_H
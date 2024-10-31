#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <chrono>
#include <iostream>

#include "DroneCtrl.hpp"

namespace DroneCtrl {

    Blackbox::~Blackbox() {
        if(flightDataFile.is_open()) {
            flightDataFile.close();
        }
    }

    std::string Blackbox::getCurrentTimestamp() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
        return ss.str();
    }

    void Blackbox::writeFlightDataToCSV() {
        if (!bb.flightDataFile.is_open()) {
            std::cerr << "Flight data file is not open." << std::endl;
            return;
        }

        // Write the flight data
        bb.flightDataFile << fd.pos[0] << "," << fd.pos[1] << "," << fd.pos[2] << ","
                    << fd.v[0] << "," << fd.v[1] << "," << fd.v[2] << ","
                    << fd.w << ","
                    << fd.a[0] << "," << fd.a[1] << "," << fd.a[2] << ","
                    << fd.battery << ","
                    << fd.thrust << ","
                    << fd.theta << ","
                    << fd.phi << "\n";
    }

    void Blackbox::initializeCSV(const std::string& filename) {
        bb.flightDataFile.open(filename, std::ios::out | std::ios::app);

        if (!bb.flightDataFile.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return;
        }

        // Write the header
        bb.flightDataFile << "pos_x,pos_y,pos_z,v_x,v_y,v_z,w,a_x,a_y,a_z,battery,thrust,theta,phi\n";
    }

    void Blackbox::closeCSV() {
        if (bb.flightDataFile.is_open()) {
            bb.flightDataFile.close();
        }
    }

    void Blackbox::initializeBlackbox() {
        bb.timestamp = getCurrentTimestamp();
        std::string filename = "logs_" + bb.timestamp + ".csv";
        initializeCSV(filename);
    }

}
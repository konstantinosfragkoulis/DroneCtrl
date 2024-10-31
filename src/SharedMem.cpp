#include <cstdio>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <chrono>
#include <thread>

#include "DroneCtrl.hpp"

namespace DroneCtrl {

    IOData::IOData() : shmPtr(nullptr), shmFd(-1), guiShmPtr(nullptr), guiShmFd(-1) {}

    IOData::~IOData() {
        closeSharedMemory();
        closeGUISharedMemory();
    }

    int IOData::initSharedMemory() {
        shmFd = shm_open(SHM_NAME, O_RDWR, 0666);
        if (shmFd == -1) {
            fprintf(stderr, "Failed to open shared memory: %s\n", strerror(errno));
            return 1;
        }

        shmPtr = mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shmFd, 0);
        if (shmPtr == MAP_FAILED) {
            fprintf(stderr, "Failed to map shared memory: %s\n", strerror(errno));
            close(shmFd);
            return 1;
        }

        return 0;
    }

    void IOData::readSharedMemory() {
        std::memcpy(values, shmPtr, SHM_SIZE);
    }

    void IOData::passValues(const int (&newValues)[16]) {
        std::memcpy(shmPtr, newValues, SHM_SIZE);
    }

    int IOData::closeSharedMemory() {
        if (shmPtr != nullptr) {
            if (munmap(shmPtr, SHM_SIZE) == -1) {
                fprintf(stderr, "Failed to unmap shared memory: %s\n", strerror(errno));
            }
            shmPtr = nullptr;
        }

        if (shmFd != -1) {
            if (close(shmFd) == -1) {
                fprintf(stderr, "Failed to close shared memory file descriptor: %s\n", strerror(errno));
                return 1;
            }
            shmFd = -1;
        }

        return 0;
    }

    int IOData::initGUISharedMemory() {
        guiShmFd = shm_open(GUI_SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (guiShmFd == -1) {
            fprintf(stderr, "Failed to create GUI shared memory: %s\n", strerror(errno));
            return 1;
        }

        if (ftruncate(guiShmFd, GUI_SHM_SIZE) == -1) {
            fprintf(stderr, "Failed to set size of GUI shared memory: %s\n", strerror(errno));
            close(guiShmFd);
            guiShmFd = -1;
            return 1;
        }

        guiShmPtr = mmap(0, GUI_SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, guiShmFd, 0);
        if (guiShmPtr == MAP_FAILED) {
            fprintf(stderr, "Failed to map GUI shared memory: %s\n", strerror(errno));
            close(guiShmFd);
            guiShmFd = -1;
            return 1;
        }

        std::memset(guiShmPtr, 0, GUI_SHM_SIZE);
        return 0;
    }

    void IOData::readGUISharedMemory() {
        std::memcpy(guiRx, guiShmPtr, GUI_RX_SIZE);
    }

    void IOData::writeGUISharedMemory() {
        std::memcpy(static_cast<char*>(guiShmPtr) + (GUI_RX_SIZE), guiTx, GUI_TX_SIZE);
    }

    int IOData::closeGUISharedMemory() {
        if (guiShmPtr != nullptr) {
            if (munmap(guiShmPtr, GUI_SHM_SIZE) == -1) {
                fprintf(stderr, "Failed to unmap GUI shared memory: %s\n", strerror(errno));
            }
            guiShmPtr = nullptr;
        }

        if (guiShmFd != -1) {
            if (close(guiShmFd) == -1) {
                fprintf(stderr, "Failed to close GUI shared memory file descriptor: %s\n", strerror(errno));
                return 1;
            }
            guiShmFd = -1;
        }

        return 0;
    }

}
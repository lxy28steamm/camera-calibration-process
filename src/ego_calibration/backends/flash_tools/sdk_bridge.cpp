// Narrow C ABI for the vendor's C++ SDK. Hardware operations are orchestrated in Python.
#include "sunpluscamera.h"
#include <cstdint>
#include <cstdio>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <thread>

struct DeviceInfo {
    uint32_t vid, pid, bcd_device, asic, flash_type, flash_bytes;
};
struct Device {
    SunplusCamera camera;
    DeviceInfo info{};
    std::string path;
    std::filesystem::path usb_port;
    bool rom_verified = false;
};

static thread_local Device* reconnecting = nullptr;

// SDK 9.2510.31.1 calls this public method through its PLT in ResetToROM.
// Its original modalias matcher repeatedly selects an unrelated UVC node,
// leaking descriptors until readdir(nullptr) crashes. Replace only this lookup;
// all reset/read/write commands still use the unmodified vendor SDK.
// Scope the override to our ROM transition and the original physical USB port.
int SunplusCamera::SunplusCamera_InitPath(char*) {
    if (!reconnecting || this != &reconnecting->camera) return -9004;
    namespace fs = std::filesystem;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    do {
        std::error_code ec;
        for (const auto& entry : fs::directory_iterator("/sys/class/video4linux", ec)) {
            auto interface = fs::canonical(entry.path() / "device", ec);
            if (ec || interface.parent_path() != reconnecting->usb_port) continue;
            std::string index, vid, pid;
            std::ifstream(entry.path() / "index") >> index;
            std::ifstream(interface.parent_path() / "idVendor") >> vid;
            std::ifstream(interface.parent_path() / "idProduct") >> pid;
            if (index != "0" || vid != "1bcf" || pid != "0b12") continue;
            std::string node = "/dev/" + entry.path().filename().string();
            int rc = SunplusCamera_Init(node.data());
            if (!rc) {
                info_camera_os info{};
                uint8_t asic = 0;
                if (!Get_information_OS(&info) && !Get_ASICType(&asic) &&
                    info.vid == 0x1bcf && info.pid == 0x0b12 &&
                    info.bcdDevice == 0x0100 && asic == reconnecting->info.asic) {
                    reconnecting->rom_verified = true;
                    std::fprintf(stderr, "ROM connected on original USB port: %s (%s)\n",
                                 reconnecting->usb_port.c_str(), node.c_str());
                    return 0;
                }
            }
            SunplusCamera_UnInit();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (std::chrono::steady_clock::now() < deadline);
    // Throw instead of returning an error: prevent the vendor's fallback from
    // selecting a different camera by VID/PID when the original port is absent.
    throw std::runtime_error("ROM camera did not return on the original USB port");
}

extern "C" {
int sp_open(char* path, void** output) {
    *output = nullptr;
    try {
        auto device = std::make_unique<Device>();
        device->path = path;
        device->usb_port = std::filesystem::canonical(
            std::filesystem::path("/sys/class/video4linux") /
            std::filesystem::path(path).filename() / "device").parent_path();
        int rc = device->camera.SunplusCamera_Init(device->path.data());
        if (rc) return rc;
        info_camera_os info{};
        uint8_t asic = 0;
        if ((rc = device->camera.Get_information_OS(&info))) return rc;
        if ((rc = device->camera.Get_ASICType(&asic))) return rc;
        if ((rc = device->camera.Get_FlashType(&device->info.flash_type))) return rc;
        if ((rc = device->camera.Get_FlashSize(device->info.flash_type, &device->info.flash_bytes))) return rc;
        device->info.vid = info.vid;
        device->info.pid = info.pid;
        device->info.bcd_device = info.bcdDevice;
        device->info.asic = asic;
        *output = device.release();
        return 0;
    } catch (...) { return -9001; }
}

int sp_info(void* handle, DeviceInfo* info) {
    if (!handle || !info) return -9002;
    *info = static_cast<Device*>(handle)->info;
    return 0;
}

int sp_close(void* handle) {
    try { delete static_cast<Device*>(handle); return 0; }
    catch (...) { return -9001; }
}

int sp_read(void* handle, uint32_t sector, uint8_t* buffer) {
    if (!handle || !buffer) return -9002;
    auto* device = static_cast<Device*>(handle);
    if (sector >= device->info.flash_bytes / 4096) return -9003;
    try { return device->camera.Upload_Sector(buffer, static_cast<int>(sector), device->info.flash_type); }
    catch (...) { return -9001; }
}

int sp_write(void* handle, uint32_t sector, uint8_t* buffer) {
    if (!handle || !buffer) return -9002;
    auto* device = static_cast<Device*>(handle);
    if (!device->rom_verified || sector == 0 || sector >= device->info.flash_bytes / 4096) return -9003;
    try { return device->camera.Write_Sector(buffer, static_cast<int>(sector), device->info.flash_type); }
    catch (...) { return -9001; }
}

int sp_rom(void* handle) {
    if (!handle) return -9002;
    auto* device = static_cast<Device*>(handle);
    if (device->info.asic != 110 || device->info.vid != 0x1bcf || device->info.pid != 0x28c4)
        return -9003;
    reconnecting = device;
    device->rom_verified = false;
    try {
        int rc = device->camera.ResetToROM();
        reconnecting = nullptr;
        return rc ? rc : (device->rom_verified ? 0 : -9004);
    } catch (...) { reconnecting = nullptr; return -9004; }
}

int sp_reset(void* handle) {
    if (!handle) return -9002;
    static_cast<Device*>(handle)->rom_verified = false;
    try { return static_cast<Device*>(handle)->camera.Reset(); }
    catch (...) { return -9001; }
}
}

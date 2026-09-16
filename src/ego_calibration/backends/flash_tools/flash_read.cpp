// Read-only Sunplus SDK diagnostic and backup utility. No erase/write/reset APIs.
#include "sunpluscamera.h"
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <unistd.h>

constexpr uint32_t SECTOR_BYTES = 4096;

void check(int rc, const char* operation) {
    if (rc != 0) {
        throw std::runtime_error(std::string(operation) + " failed: " + std::to_string(rc));
    }
}

int main(int argc, char** argv) {
    const bool info_mode = argc == 3 && std::string(argv[1]) == "info";
    const bool sector_mode = argc == 5 && std::string(argv[1]) == "read-sector";
    const bool backup_mode = argc == 4 && std::string(argv[1]) == "backup";
    if (!info_mode && !sector_mode && !backup_mode) {
        std::fprintf(stderr, "Usage:\n  %s info DEVICE\n  %s read-sector DEVICE SECTOR OUTPUT.bin\n  %s backup DEVICE OUTPUT.bin\n", argv[0], argv[0], argv[0]);
        return 2;
    }
    int output = -1;
    std::string output_path;
    try {
        SunplusCamera camera;
        check(camera.SunplusCamera_Init(argv[2]), "SunplusCamera_Init");
        info_camera_os info{};
        uint8_t asic = 0;
        uint32_t flash_type = 0, flash_bytes = 0;
        check(camera.Get_information_OS(&info), "Get_information_OS");
        check(camera.Get_ASICType(&asic), "Get_ASICType");
        check(camera.Get_FlashType(&flash_type), "Get_FlashType");
        check(camera.Get_FlashSize(flash_type, &flash_bytes), "Get_FlashSize");
        std::printf("VID=%04x PID=%04x bcdDevice=%04x ASIC=%u FlashType=0x%08x FlashBytes=%u\n",
                    info.vid, info.pid, info.bcdDevice, asic, flash_type, flash_bytes);
        std::fflush(stdout);
        if (info_mode) return 0;
        if (info.vid != 0x1bcf || info.pid != 0x28c4) {
            throw std::runtime_error("Unexpected device: this utility was verified with USB 1bcf:28c4");
        }
        if (flash_bytes == 0 || flash_bytes % SECTOR_BYTES != 0 || flash_bytes > 32 * 1024 * 1024) {
            throw std::runtime_error("Unsupported flash size; refusing an unbounded read");
        }
        uint32_t first = 0, count = flash_bytes / SECTOR_BYTES;
        if (sector_mode) {
            char* end = nullptr;
            errno = 0;
            const unsigned long value = std::strtoul(argv[3], &end, 0);
            if (errno || *end != '\0' || end == argv[3] || argv[3][0] == '-' || value >= count) {
                throw std::runtime_error("Sector is outside the reported flash capacity");
            }
            first = static_cast<uint32_t>(value);
            count = 1;
        }
        output_path = argv[argc - 1];
        output = open(output_path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
        if (output < 0) throw std::runtime_error(std::string("Create output: ") + std::strerror(errno));
        std::array<uint8_t, SECTOR_BYTES> data, verify;
        for (uint32_t index = first; index < first + count; ++index) {
            data.fill(0xa5);
            verify.fill(0x5a);
            check(camera.Upload_Sector(data.data(), static_cast<int>(index), flash_type), "Upload_Sector");
            check(camera.Upload_Sector(verify.data(), static_cast<int>(index), flash_type), "Upload_Sector verify");
            if (data != verify) throw std::runtime_error("Repeated sector reads differ or SDK returned incomplete data");
            size_t offset = 0;
            while (offset < data.size()) {
                const auto written = write(output, data.data() + offset, data.size() - offset);
                if (written < 0 && errno == EINTR) continue;
                if (written <= 0) throw std::runtime_error("Failed to write backup file");
                offset += static_cast<size_t>(written);
            }
            if ((index + 1) % 16 == 0 || index + 1 == first + count) {
                std::printf("Read and compared %u / %u sectors\n", index - first + 1, count);
                std::fflush(stdout);
            }
        }
        if (fsync(output) != 0) throw std::runtime_error("Cannot flush backup file");
        close(output);
        output = -1;
        std::printf("Saved %u bytes to %s\n", count * SECTOR_BYTES, output_path.c_str());
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        if (output >= 0) {
            close(output);
            unlink(output_path.c_str());
        }
        return 1;
    }
}

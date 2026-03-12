#include "serial_interface.hpp"

#include "user_lib.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <string>

namespace IO
{
    namespace
    {
        constexpr uint8_t kLegacyHeader0 = 0x55;
        constexpr uint8_t kLegacyHeader1 = 0xAA;
        constexpr uint8_t kCh10xHeader0 = 0x5A;
        constexpr uint8_t kCh10xHeader1 = 0xA5;
        constexpr uint8_t kCh10xTagFloat = 0x91;

        struct Ch10xImu91
        {
            uint8_t tag;
            uint8_t id;
            uint8_t reserved;
            int8_t temperature;
            float air_pressure;
            uint32_t system_time;
            float acc_b[3];
            float gyr_b[3];
            float mag_b[3];
            float eul[3];
            float quat[4];
        } __attribute__((packed));

        static_assert(sizeof(Ch10xImu91) == 76, "CH10X IMU91 payload size mismatch");

        uint16_t crc16_ccitt(uint16_t crc, const uint8_t *src, size_t length) {
            for (size_t j = 0; j < length; ++j) {
                crc ^= static_cast<uint16_t>(src[j]) << 8;
                for (int i = 0; i < 8; ++i) {
                    if (crc & 0x8000) {
                        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
                    } else {
                        crc = static_cast<uint16_t>(crc << 1);
                    }
                }
            }
            return crc;
        }

        bool parse_ch10x_payload(const uint8_t *payload, size_t payload_len, Types::ReceivePacket_IMU *out) {
            if (payload_len < sizeof(Ch10xImu91)) {
                return false;
            }

            const uint8_t *pkt_ptr = nullptr;
            for (size_t offset = 0; offset + sizeof(Ch10xImu91) <= payload_len; ++offset) {
                if (payload[offset] == kCh10xTagFloat) {
                    pkt_ptr = payload + offset;
                    break;
                }
            }
            if (pkt_ptr == nullptr) {
                return false;
            }

            Ch10xImu91 pkt{};
            memcpy(&pkt, pkt_ptr, sizeof(Ch10xImu91));
            if (pkt.tag != kCh10xTagFloat) {
                return false;
            }

            // CH10X outputs roll/pitch/yaw in degrees and gyro in deg/s.
            // Convert gyro to the legacy unit (0.001 deg/s) expected by imu.cc.
            constexpr float kRateScale = 1000.0f;

            // If the CH10X installation coordinate differs from the legacy IMU,
            // adjust these signs to match the previous coordinate convention.
            constexpr float kYawSign = 1.0f;
            constexpr float kPitchSign = -1.0f;
            constexpr float kRollSign = 1.0f;

            out->roll = kRollSign * pkt.eul[0];
            out->pitch = -kPitchSign * pkt.eul[1];
            out->yaw = -kYawSign * pkt.eul[2];

            out->roll_v = kRollSign * pkt.gyr_b[1] * kRateScale;
            out->pitch_v = kPitchSign * pkt.gyr_b[0] * kRateScale;
            out->yaw_v = kYawSign * pkt.gyr_b[2] * kRateScale;
            return true;
        }

        struct ImuParseStats
        {
            uint64_t legacy_frames = 0;
            uint64_t ch10x_frames = 0;
            uint64_t ch10x_crc_fail = 0;
            uint64_t ch10x_no_tag = 0;
            uint64_t header_miss = 0;
            std::chrono::steady_clock::time_point last_log = std::chrono::steady_clock::now();
        };

        void dump_hex(const uint8_t *data, size_t len) {
            constexpr size_t kMaxBytes = 96;
            size_t dump_len = len < kMaxBytes ? len : kMaxBytes;
            std::string line;
            line.reserve(dump_len * 3);
            char byte_buf[4];
            for (size_t i = 0; i < dump_len; ++i) {
                std::snprintf(byte_buf, sizeof(byte_buf), "%02X ", data[i]);
                line.append(byte_buf);
            }
            LOG_INFO("IMU_CH10X raw[%zu]: %s\n", dump_len, line.c_str());
        }
    }  // namespace

    Serial_interface::Serial_interface(std::string port_name, int baudrate, int simple_timeout)
        : serial::Serial(port_name, baudrate, serial::Timeout::simpleTimeout(simple_timeout)),
          name(port_name) {
    }

    Serial_interface::~Serial_interface() = default;

    inline void Serial_interface::enumerate_ports() {
        std::vector<serial::PortInfo> devices_found = serial::list_ports();
        auto iter = devices_found.begin();

        while (iter != devices_found.end()) {
            serial::PortInfo device = *iter++;
            LOG_INFO("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str());
        }
    }

    inline int Serial_interface::unpack(uint8_t pkg_id) {
        if (pkg_id == 1) {
            memcpy(buffer, read(sizeof(Types::ReceivePacket_IMU)).c_str(), sizeof(Types::ReceivePacket_IMU));
            UserLib::fromVector(buffer, &imu_pkg);
            callback(imu_pkg);
        } else if (pkg_id == 2) {
            size_t pkg_size = sizeof(Types::ReceivePacket_RC_CTRL);
            memcpy(buffer, read(pkg_size).c_str(), pkg_size);
            
            // printf("Raw Buffer (Hex): \n");
            // for (size_t i = 0; i < pkg_size; ++i) {
            //     printf("%02X ", (unsigned char)buffer[i]); 
            // }
            // printf("\n");
            UserLib::fromVector(buffer, &rc_pkg);
            // printf("mouse_x: %d | mouse_y: %d | mouse_z: %d | mouse_l: %d | mouse_r: %d\n", rc_pkg.mouse_x, rc_pkg.mouse_y, rc_pkg.mouse_z, rc_pkg.mouse_l, rc_pkg.mouse_r);
            callback(rc_pkg);
        } 
       
        return 0;
    }

    void Serial_interface::task() {
        static ImuParseStats stats;
        constexpr bool kCh10xRawDump = false;
        constexpr auto kRawDumpInterval = std::chrono::milliseconds(200);
        static auto last_raw_dump = std::chrono::steady_clock::now();
        constexpr uint8_t kCh10xLenLow = 0x4C;
        constexpr uint8_t kCh10xLenHigh = 0x00;
        constexpr size_t kCh10xFrameSize = 82;  // 2(header) + 2(len) + 2(crc) + 76(payload)
        static std::array<uint8_t, 4> ch10x_window = {};
        static size_t ch10x_window_len = 0;
        while (true) {
            try {
                if (isOpen()) {
                    if (kCh10xRawDump && name.find("IMU_CH10X") != std::string::npos) {
                        uint8_t byte = 0;
                        read(&byte, 1);

                        if (ch10x_window_len < ch10x_window.size()) {
                            ch10x_window[ch10x_window_len++] = byte;
                        } else {
                            ch10x_window[0] = ch10x_window[1];
                            ch10x_window[1] = ch10x_window[2];
                            ch10x_window[2] = ch10x_window[3];
                            ch10x_window[3] = byte;
                        }

                        if (ch10x_window_len >= ch10x_window.size() &&
                            ch10x_window[0] == kCh10xHeader0 &&
                            ch10x_window[1] == kCh10xHeader1 &&
                            ch10x_window[2] == kCh10xLenLow &&
                            ch10x_window[3] == kCh10xLenHigh) {
                            std::array<uint8_t, kCh10xFrameSize> frame = {};
                            frame[0] = ch10x_window[0];
                            frame[1] = ch10x_window[1];
                            frame[2] = ch10x_window[2];
                            frame[3] = ch10x_window[3];

                            size_t remain = kCh10xFrameSize - 4;
                            size_t got = read(frame.data() + 4, remain);
                            if (got == remain) {
                                auto now = std::chrono::steady_clock::now();
                                if (now - last_raw_dump >= kRawDumpInterval) {
                                    dump_hex(frame.data(), frame.size());
                                    last_raw_dump = now;
                                }
                            }
                        }
                        continue;
                    }

                    uint8_t head0 = 0;
                    uint8_t head1 = 0;
                    static bool have_pending = false;
                    static uint8_t pending = 0;

                    if (have_pending) {
                        head0 = pending;
                        have_pending = false;
                    } else {
                        read(&head0, 1);
                    }

                    if (head0 != kLegacyHeader0 && head0 != kCh10xHeader0) {
                        stats.header_miss++;
                        continue;
                    }

                    read(&head1, 1);
                    if (head0 == kLegacyHeader0 && head1 == kLegacyHeader1) {
                        read((uint8_t *)&header, 1);
                        unpack(header);
                        stats.legacy_frames++;
                    } else if (head0 == kCh10xHeader0 && head1 == kCh10xHeader1) {
                        uint8_t len_bytes[2];
                        uint8_t crc_bytes[2];
                        read(len_bytes, 2);
                        read(crc_bytes, 2);
                        uint16_t payload_len = static_cast<uint16_t>(len_bytes[0]) |
                                               (static_cast<uint16_t>(len_bytes[1]) << 8);
                        uint16_t frame_crc = static_cast<uint16_t>(crc_bytes[0]) |
                                             (static_cast<uint16_t>(crc_bytes[1]) << 8);

                        if (payload_len > sizeof(buffer)) {
                            // Drain oversized payload to keep stream aligned.
                            read(payload_len);
                            continue;
                        }

                        read(buffer, payload_len);
                        std::array<uint8_t, 4> header_and_len = {
                            kCh10xHeader0,
                            kCh10xHeader1,
                            len_bytes[0],
                            len_bytes[1],
                        };
                        uint16_t crc = 0;
                        crc = crc16_ccitt(crc, header_and_len.data(), header_and_len.size());
                        crc = crc16_ccitt(crc, buffer, payload_len);
                        if (crc != frame_crc) {
                            stats.ch10x_crc_fail++;
                            continue;
                        }

                        if (parse_ch10x_payload(buffer, payload_len, &imu_pkg)) {
                            callback(imu_pkg);
                            stats.ch10x_frames++;
                        } else {
                            stats.ch10x_no_tag++;
                        }
                    } else {
                        // If the second byte is also a potential header start, keep it.
                        if (head1 == kLegacyHeader0 || head1 == kCh10xHeader0) {
                            pending = head1;
                            have_pending = true;
                        }
                        continue;
                    }

                    auto now = std::chrono::steady_clock::now();
                    if (now - stats.last_log > std::chrono::seconds(2)) {
                        // LOG_INFO(
                        //     "IMU serial stats: legacy=%llu ch10x=%llu crc_fail=%llu no_tag=%llu header_miss=%llu\n",
                        //     static_cast<unsigned long long>(stats.legacy_frames),
                        //     static_cast<unsigned long long>(stats.ch10x_frames),
                        //     static_cast<unsigned long long>(stats.ch10x_crc_fail),
                        //     static_cast<unsigned long long>(stats.ch10x_no_tag),
                        //     static_cast<unsigned long long>(stats.header_miss));
                        stats.last_log = now;
                    }
                } else {
                    enumerate_ports();
                    return;
                }
            } catch (serial::IOException &e) {
                LOG_ERR("serail offline! end program now\n");
                // exit(-1);
            }
        }
    }
}  // namespace IO

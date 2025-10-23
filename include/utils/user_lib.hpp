#pragma once
#include <cmath>
#include <vector>

#include "types.hpp"

namespace UserLib
{
    fp32 rad_format(fp32 ang);
    void sleep_ms(uint32_t dur);

    template<typename T>
    void unpack(T &t, void *ptr) {
        for (size_t i = 0; i < sizeof(T); i++) {
            *((uint8_t *)&t + i) = *((uint8_t *)ptr + i);
        }
    }

    template<typename T>
    std::vector<uint8_t> toVector(T &t) {
        std::vector<uint8_t> ans;
        for (size_t i = 0; i < sizeof(T); i++) {
            ans.push_back(*((uint8_t *)&t + i));
        }
        return ans;
    }

    template<typename T>
    inline void fromVector(const uint8_t *data, T *pkg) {
        for (size_t i = 0; i < sizeof(T); ++i) {
            ((uint8_t *)pkg)[i] = data[i];
        }
    }

    class RealRad
    {
       public:
        void update(fp32 ref);

       public:
        int count = 0;
        fp32 now = 0.0;
        fp32 last = 0.0;
    };
}  // namespace UserLib

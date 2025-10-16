#ifndef LED_H_
#define LED_H_
#include <cstdint>

namespace led
{
    constexpr uint32_t kPATTERN_4_BLIPS_NORMED = 0x0f0f0f0f;
    constexpr uint32_t kPATTERN_3_BLIPS_NORMED = 0x7f03f03f;
    constexpr uint32_t kPATTERN_2_BLIPS_NORMED = 0x7ff003ff;
    int setup();
    void start();

    void show_pattern(uint32_t p, uint32_t tms = 2000);
}
#endif

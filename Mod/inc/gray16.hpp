#ifndef GRAY16_HPP
#define GRAY16_HPP

#include <stdint.h>

// 第五代 16 路灰度传感器的数字量输出协议：
// 查询帧为 57 ID，应答为 75 Data0 Data1 03；合并后的最低位对应第 1 路。
struct Gray16Sample
{
    uint16_t bits;
    uint32_t tick_ms;
    uint32_t sequence;
};

class Gray16Parser
{
public:
    Gray16Parser();
    uint8_t feed(const uint8_t *bytes, uint16_t length, uint32_t tick_ms);
    Gray16Sample sample() const;

private:
    Gray16Sample sample_;
    uint8_t state_;
    uint8_t data0_;
    uint8_t data1_;
    uint32_t last_byte_ms_ = 0;
};

#endif

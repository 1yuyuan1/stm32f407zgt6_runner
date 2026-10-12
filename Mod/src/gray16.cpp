#include "gray16.hpp"

Gray16Parser::Gray16Parser() : sample_{0, 0, 0}, state_(0), data0_(0), data1_(0) {}

uint8_t Gray16Parser::feed(const uint8_t *bytes, uint16_t length, uint32_t tick_ms)
{
    if (bytes == nullptr) return 0;
    if (tick_ms - last_byte_ms_ > 10) state_ = 0;
    last_byte_ms_ = tick_ms;
    uint8_t completed = 0;
    for (uint16_t i = 0; i < length; ++i)
    {
        const uint8_t byte = bytes[i];
        switch (state_)
        {
        case 0:
            if (byte == 0x75) state_ = 1;
            break;
        case 1:
            data0_ = byte;
            state_ = 2;
            break;
        case 2:
            data1_ = byte;
            state_ = 3;
            break;
        default:
            if (byte == 0x03)
            {
                sample_.bits = (uint16_t)data0_ | ((uint16_t)data1_ << 8);
                sample_.tick_ms = tick_ms;
                ++sample_.sequence;
                ++completed;
                state_ = 0;
            }
            else
            {
                // 新帧头也可能是下一帧的起点。
                // 坏帧中出现帧头时保留该起点，以便重新同步；
                // 正常帧的数据字节即使等于帧头，也仍按数据处理。
                if (data0_ == 0x75)
                {
                    data0_ = data1_;
                    data1_ = byte;
                    state_ = 3;
                }
                else if (data1_ == 0x75)
                {
                    data0_ = byte;
                    state_ = 2;
                }
                else state_ = (byte == 0x75) ? 1 : 0;
            }
            break;
        }
    }
    return completed;
}

Gray16Sample Gray16Parser::sample() const
{
    return sample_;
}

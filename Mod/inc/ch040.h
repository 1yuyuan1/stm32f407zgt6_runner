#ifndef CH040_H
#define CH040_H

#include "main.h"
#include <stdint.h>

#define WIT_PI 3.1415926535f

class ch040_t
{
public:
    /**
     * @brief 解析串口收到的数据流
     * @note  适配 WIT 标准协议，只解析 0x55 0x53 欧拉角帧
     */
    void analyze_data(const uint8_t *data, uint16_t len)
    {
        for (uint16_t i = 0; i < len; i++)
        {
            parseByte(data[i]);
        }
    }

    /**
     * @brief 返回 yaw，单位：弧度，范围 -pi ~ pi
     */
    float getYaw()
    {
        float yaw = yaw_raw_rad - yaw_zero_rad;

        if (yaw > WIT_PI)
        {
            yaw -= 2.0f * WIT_PI;
        }
        else if (yaw < -WIT_PI)
        {
            yaw += 2.0f * WIT_PI;
        }

        return yaw;
    }

    /**
     * @brief 返回 yaw，单位：角度，调试用
     */
    float getYawDeg()
    {
        return getYaw() * 180.0f / WIT_PI;
    }

    /**
     * @brief 设置当前 yaw 为 0
     */
    void setYawZero()
    {
        yaw_zero_rad = yaw_raw_rad;
    }

    /**
     * @brief 如果发现车逆时针转 yaw 反而减小，就调用 setYawDir(-1)
     */
    void setYawDir(float dir)
    {
        yaw_dir = (dir >= 0.0f) ? 1.0f : -1.0f;
    }

    bool isUpdated()
    {
        return updated;
    }

    uint32_t validFrameCount() const
    {
        return valid_frame_count;
    }

private:
    uint8_t frame[11] = {0};
    uint8_t frame_index = 0;

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_raw_rad = 0.0f;
    float yaw_zero_rad = 0.0f;
    float yaw_dir = 1.0f;

    bool updated = false;
    uint32_t valid_frame_count = 0;

private:
    static float normalizeRad(float rad)
    {
        if (rad > WIT_PI)
        {
            rad -= 2.0f * WIT_PI;
        }
        else if (rad < -WIT_PI)
        {
            rad += 2.0f * WIT_PI;
        }

        return rad;
    }

    static int16_t bytesToInt16(uint8_t low, uint8_t high)
    {
        return (int16_t)((uint16_t)high << 8 | low);
    }

    static float angleRawToDeg(int16_t raw)
    {
        return (float)raw / 32768.0f * 180.0f;
    }

    void parseByte(uint8_t byte)
    {
        switch (frame_index)
        {
        case 0:
            if (byte == 0x55)
            {
                frame[0] = byte;
                frame_index = 1;
            }
            break;

        case 1:
            if (byte == 0x53)
            {
                frame[1] = byte;
                frame_index = 2;
            }
            else if (byte == 0x55)
            {
                frame[0] = 0x55;
                frame_index = 1;
            }
            else
            {
                frame_index = 0;
            }
            break;

        default:
            frame[frame_index++] = byte;

            if (frame_index >= 11)
            {
                parseAngleFrame(frame);
                frame_index = 0;
            }
            break;
        }
    }

    void parseAngleFrame(const uint8_t *f)
    {
        uint8_t sum = 0;

        for (uint8_t i = 0; i < 10; i++)
        {
            sum += f[i];
        }

        if (sum != f[10])
        {
            return;
        }

        int16_t roll_raw  = bytesToInt16(f[2], f[3]);
        int16_t pitch_raw = bytesToInt16(f[4], f[5]);
        int16_t yaw_raw   = bytesToInt16(f[6], f[7]);

        float roll_deg  = angleRawToDeg(roll_raw);
        float pitch_deg = angleRawToDeg(pitch_raw);
        float yaw_deg   = angleRawToDeg(yaw_raw);

        roll_rad = roll_deg * WIT_PI / 180.0f;
        pitch_rad = pitch_deg * WIT_PI / 180.0f;

        yaw_raw_rad = normalizeRad(yaw_dir * yaw_deg * WIT_PI / 180.0f);

        updated = true;
        ++valid_frame_count;
    }
};

inline ch040_t ch040;

#endif

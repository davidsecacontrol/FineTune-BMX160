#ifndef FINETUNE_BMX160_MASKS
#define FINETUNE_BMX160_MASKS

#include <stdint.h>

namespace FineTuneBMX160
{
        namespace ACCEL
    {

        /** @brief Allowed accelerometer ranges. Values represent mask */
        enum struct RANGE : uint8_t
        {
            G2 = 0b00000011,
            G4 = 0b00000101,
            G8 = 0b00001000,
            G16 = 0b00001100
        };

        /** @brief Allowed accelerometer power modes. Values represent the mask */
        enum struct POWER_MODE : uint8_t
        {
            NORMAL = 0b00010001,
            LOW_POWER = 0b00010010,
            SUSPEND = 0b00010000
        };
        /** @brief Allowed accelerometer ODRs. Values represent the mask */
        enum struct ODR : uint8_t
        {
            Hz25_over_32 = UINT8_C(1),
            Hz25_over_16 = UINT8_C(2),
            Hz25_over_8 = UINT8_C(3),
            Hz25_over_4 = UINT8_C(4),
            Hz25_over_2 = UINT8_C(5),
            Hz25 = UINT8_C(6),
            Hz50 = UINT8_C(7),
            Hz100 = UINT8_C(8),
            Hz200 = UINT8_C(9),
            Hz400 = UINT8_C(10),
            Hz800 = UINT8_C(11),
            Hz1600 = UINT8_C(12)
        };

        /** @brief Accelerometer sensitivity presets*/
        constexpr float SENSITIVITY[] = {
            1.0f / 16384,
            1.0f / 8192,
            1.0f / 4096,
            1.0f / 2048};
    }

    namespace GYRO
    {

        /** @brief Allowed gyroscope ranges. Values represent mask */
        enum struct RANGE : uint8_t
        {
            DPS2000 = 0b00000000,
            DPS1000 = 0b00000001,
            DPS500 = 0b00000010,
            DPS250 = 0b00000011,
            DPS150 = 0b00000100
        };

        /** @brief Allowed gyroscope power modes. Values represent mask */
        enum struct POWER_MODE : uint8_t
        {
            NORMAL = 0b00010101,
            FAST_STARTUP = 0b00010111,
            SUSPEND = 0b00010100
        };

        /** @brief Allowed gyroscope ODRs. Values represent the mask */
        enum struct ODR : uint8_t
        {
            Hz25 = UINT8_C(6),
            Hz50 = UINT8_C(7),
            Hz100 = UINT8_C(8),
            Hz200 = UINT8_C(9),
            Hz400 = UINT8_C(10),
            Hz800 = UINT8_C(11),
            Hz1600 = UINT8_C(12),
            Hz3200 = UINT8_C(13)
        };

        /** @brief Gyroscope sensisitivy presets*/
        constexpr float SENSITIVITY[] = {
            1.0f / 16.4f,
            1.0f / 32.8f,
            1.0f / 65.6f,
            1.0f / 131.2f,
            1.0f / 262.4f
        };
    }

    /**
     * @brief Embedded magnetometer chip inside the BMX160. Can be used via indirect addressing
     *
     */
    namespace MAGN
    {
        /** @brief Magnetometer addresses for indirect addressing */
        enum struct REGISTER : uint8_t
        {
            POWER_MODE = UINT8_C(0x4B),
            REPXY = UINT8_C(0x51),
            REPZ = UINT8_C(0x52)
        };

        namespace PRESETS
        {
            /** @brief Allowed magnetometer preset modes for XY axes. Values represent the mask */
            enum struct REPXY : uint8_t
            {
                LOW_POWER = UINT8_C(0x01),
                REGULAR = UINT8_C(0x04),
                ENHANCED_REGULAR = UINT8_C(0x07),
                HIGH_ACCURACY = UINT8_C(0x17)
            };

            /** @brief Allowed magnetometer preset modes for Z axis. Values represent the mask */
            enum struct REPZ : uint8_t
            {
                LOW_POWER = UINT8_C(0x02),
                REGULAR = UINT8_C(0x0E),
                ENHANCED_REGULAR = UINT8_C(0x1A),
                HIGH_ACCURACY = UINT8_C(0x52)
            };
        }

        /** @brief Allowed magnetometer range*/
        enum struct RANGE : int
        {
            XY_uT1300 = 0,
            Z_uT2500 = 0
        };

        /** @brief Allowed magnetometer power modes*/
        enum struct POWER_MODE : int
        {
            FORCE = 0x02, // Actually not specified in datasheet, shouldn't be employed
            SLEEP = 0x01,
            SUSPEND = 0x00
        };

        /** @brief Magnetometer sensitivity presets*/
        constexpr float SENSITIVITY[] = {
            0.3f};
    }

    /**
     * @brief Interface used for synchronising and using the embedded magnetometer same way as the other sensors.
     *
     */
    namespace MAGN_INTERFACE
    {
        /** @brief Allowed magnetometer interface power modes*/
        enum struct POWER_MODE : int8_t
        {
            NORMAL = 0b00011001,
            LOW_POWER = 0b00011010,
            SUSPEND = 0b00011000
        };

        /** @brief Allwed magnetometer ODRs. ODR settings may depend on perset mode (see Table 11 in datasheet) */
        enum struct ODR : uint8_t
        {
            Hz25_over_32 = UINT8_C(1),
            Hz25_over_16 = UINT8_C(2),
            Hz25_over_8 = UINT8_C(3),
            Hz25_over_4 = UINT8_C(4),
            Hz25_over_2 = UINT8_C(5),
            Hz25 = UINT8_C(6),
            Hz50 = UINT8_C(7),
            Hz100 = UINT8_C(8),
            Hz200 = UINT8_C(9),
            Hz400 = UINT8_C(10),
            Hz800 = UINT8_C(11)
        };

        /** @brief Selected values for reading  */
        enum struct DATA_SIZE : uint8_t
        {
            LSB_X = UINT8_C(0x00),
            X = UINT8_C(0x01),
            XYZ = UINT8_C(0x02),
            XYZ_RHALL = UINT8_C(0x03)
        };

        /** @brief Read offset after data ready, 2.5ms resolution. Recommended setting: 0x00<<2 */
        enum struct READ_OFFSET : uint8_t // Unused, minimum offset selected (0x00)
        {
            ms0 = UINT8_C(0x00 << 2),
            ms2_5 = UINT8_C(0x01 << 2),
            ms5 = UINT8_C(0x02 << 2),
            ms7_5 = UINT8_C(0x03 << 2),
            ms10 = UINT8_C(0x04 << 2),
            ms12_5 = UINT8_C(0x05 << 2),
            ms15 = UINT8_C(0x06 << 2),
            ms17_5 = UINT8_C(0x07 << 2),
            ms20 = UINT8_C(0x08 << 2),
            ms22_5 = UINT8_C(0x09 << 2),
            ms25 = UINT8_C(0x0A << 2),
            ms27_5 = UINT8_C(0x0B << 2),
            ms30 = UINT8_C(0x0C << 2),
            ms32_5 = UINT8_C(0x0D << 2),
            ms35 = UINT8_C(0x0E << 2),
            ms37_5 = UINT8_C(0x0F << 2),
        };

    }

    namespace TIMESTAMPS
    {
        /** @brief Timestamps sensitivity presets in seconds */
        constexpr float SENSITIVITY[] = {
            0.000039f};
    }

    namespace TEMP_SENSOR
    {
        /** @brief Temperature sensor sensitivity presets in ºC*/
        constexpr float SENSITIVITY[] = {
            1.0f / 512.0f};
    }
}

#endif
#include <Wire.h>

#define NORMAL 0x6825
#define LOW_POWER 0x609C
#define LOW_NOISE 0x70DF
#define ULTRA_LOW_NOISE 0x7866
#define NORMAL_T_FIRST 0x6825
#define NORMAL_P_FIRST 0x48A3
#define LOW_POWER_T_FIRST 0x609C
#define LOW_POWER_P_FIRST 0x401A
#define LOW_NOISE_T_FIRST 0x70DF
#define LOW_NOISE_P_FIRST 0x5059
#define ULN_T_FIRST 0x7866
#define ULN_P_FIRST 0x58E0

#define SOFT_RESET 0x805D
#define READ_ID 0xEFC8
#define MOVE_ADDRESS_PTR 0xC595
#define READ_OTP 0xC7F7

#define DEFAULT_I2C_ADDRESS 0x63
#define CHIP_ID 0x08

#define OK 0
#define CRC_FAIL 1

const int MEASUREMENT_DELAYS[4] = {
    7,   // NORMAL (5.6 to 6.3ms)
    2,   // LOW_POWER (1.6 to 1.8ms)
    24,  // LOW_NOISE (20.8 to 23.8ms)
    95   // ULTRA_LOW_NOISE (83.2 to 94.5ms)
};

int sensor_constants[4];

struct SensorConstants {
    float A;
    float B;
    float C;
};

struct measurementValues {
    float raw_pressure;
    float raw_temperature;
    float pressure;
    float temperature;
};

void init_ICP10125() {
    int chip_id = get_chip_id();
    if (chip_id != CHIP_ID) {
        Serial.println("Invalid chip ID for ICP10125!");
    }
    read_otp();
}

void rdwr(int command, int length, int delayms, uint8_t* outData) {
    byte commandArray[2];
    commandArray[0] = (command >> 8) & 0xFF; // High byte
    commandArray[1] = command & 0xFF;        // Low byte

    Wire.beginTransmission(DEFAULT_I2C_ADDRESS);
    Wire.write(commandArray, 2);
    Wire.endTransmission();
    delay(delayms);

    if (length > 0) {
        Wire.requestFrom(DEFAULT_I2C_ADDRESS, length);
        int i = 0;
        while (Wire.available() && i < length) {
            outData[i++] = Wire.read();
        }
    }
}

int get_chip_id() {
    uint8_t result[3];
    rdwr(READ_ID, 3, 0, result);
    return result[0] & 0x3F;
}

void read_otp() {
    byte move_address_ptr[] = {
        (MOVE_ADDRESS_PTR >> 8) & 0xFF,
        MOVE_ADDRESS_PTR & 0xFF,
        0x00, 0x66, 0x9C // Address and CRC
    };

    Wire.beginTransmission(DEFAULT_I2C_ADDRESS);
    Wire.write(move_address_ptr, sizeof(move_address_ptr));
    Wire.endTransmission();

    for (int x = 0; x < 4; x++) {
        uint8_t result[3];
        rdwr(READ_OTP, 3, 0, result);
        if (crc8(result, 2) == result[2]) {
            sensor_constants[x] = (result[0] << 8) | result[1];
        } else {
            Serial.println("CRC failed reading OTP!");
            sensor_constants[x] = 0;
        }
    }
}

void reset() {
    uint8_t dummy[1];
    rdwr(SOFT_RESET, 0, 1, dummy);
}

struct measurementValues measure() {
    int delayms = MEASUREMENT_DELAYS[0];
    uint8_t result[9];
    rdwr(NORMAL, 9, delayms, result);

    measurementValues measurements = {0};

    if (crc8(&result[0], 2) != result[2] ||
        crc8(&result[3], 2) != result[5] ||
        crc8(&result[6], 2) != result[8]) {
        Serial.println("CRC failed on measurement data!");
        return measurements;
    }

    uint16_t T_raw = (result[0] << 8) | result[1];
    uint16_t P_raw = (result[3] << 8) | result[4];
    
    measurements.raw_pressure = P_raw;
    measurements.raw_temperature = T_raw;

    float temperature = -45.0 + (175.0 * T_raw / 65536.0);

    float LUT_lower = 3.5 * (1 << 20);
    float LUT_upper = 11.5;
    float quadr_factor = 1.0 / 16777216.0;
    float offst_factor = 2048.0;

    float t = T_raw - 32768;
    float s1 = LUT_lower + (sensor_constants[0] * t * t) * quadr_factor;
    float s2 = offst_factor * sensor_constants[3] + (sensor_constants[1] * t * t) * quadr_factor;
    float s3 = LUT_upper + (sensor_constants[2] * t * t) * quadr_factor;

    float sensorList[3] = {s1, s2, s3};
    SensorConstants constants = calculate_conversion_constants(sensorList);
    float pressure = constants.A + constants.B / (constants.C + P_raw);

    measurements.temperature = temperature;
    measurements.pressure = pressure;
    return measurements;
}

float calculate_altitude(float pressure) {
    return 44330.0 * (1.0 - pow((pressure / 1013.25), (1.0 / 5.255)));
}

struct SensorConstants calculate_conversion_constants(float *p_LUT) {
    float p_Pa[] = {45000.0, 80000.0, 105000.0};
    SensorConstants constants = {0};

    constants.C = (p_LUT[0] * p_LUT[1] * (p_Pa[0] - p_Pa[1]) +
                   p_LUT[1] * p_LUT[2] * (p_Pa[1] - p_Pa[2]) +
                   p_LUT[2] * p_LUT[0] * (p_Pa[2] - p_Pa[0])) /
                  (p_LUT[2] * (p_Pa[0] - p_Pa[1]) +
                   p_LUT[0] * (p_Pa[1] - p_Pa[2]) +
                   p_LUT[1] * (p_Pa[2] - p_Pa[0]));

    constants.A = (p_Pa[0] * p_LUT[0] - p_Pa[1] * p_LUT[1] -
                   (p_Pa[1] - p_Pa[0]) * constants.C) /
                  (p_LUT[0] - p_LUT[1]);

    constants.B = (p_Pa[0] - constants.A) * (p_LUT[0] + constants.C);

    return constants;
}

uint8_t crc8(uint8_t *data, int length) {
    uint8_t result = 0xFF;
    for (int i = 0; i < length; i++) {
        result ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (result & 0x80) {
                result = (result << 1) ^ 0x31;
            } else {
                result <<= 1;
            }
        }
    }
    return result;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    init_ICP10125();
    reset();
    delay(10);
}

void loop() {
    
    measurementValues values = measure();
    Serial.print("Initial Pressure (raw): ");
    Serial.println(values.raw_pressure);
    
    Serial.print("Initial Temperature (raw): ");
    Serial.println(values.raw_temperature);

    Serial.print("Processed Pressure (hPa): ");
    Serial.println(values.pressure/100);
    Serial.print("Processed Temperature (°C): ");
    Serial.println(values.temperature);

    float altitude = calculate_altitude(values.pressure/100);
    Serial.print("Altitude (m): ");
    Serial.println(altitude);

    Serial.println("--------------------------");
    delay(2000);

}
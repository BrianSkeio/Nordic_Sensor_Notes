
#define BME280_I2C_ADDR_PRIM                      UINT8_C(0x76)
#define BME280_I2C_ADDR_SEC                       UINT8_C(0x77)
#define BME280_REG_CHIP_ID                        UINT8_C(0xD0)
#define BME280_REG_TEMP_CALIB_DATA                UINT8_C(0x88)
#define BME280_REG_PRESS_CALIB_DATA               UINT8_C(0x8E)
#define BME280_REG_HUMIDITY1_CALIB_DATA           UINT8_C(0xA1)
#define BME280_REG_HUMIDITY2_CALIB_DATA           UINT8_C(0xE1)
#define BME280_REG_DATA                           UINT8_C(0xF7)
#define BME280_TEMPERATURE_MSB_REG                0xFA //Temperature MSB
#define BME280_PRESSURE_MSB_REG                   0xF7 //Pressure MSB
#define BME280_RST_REG                            0xE0
#define BME280_CTRL_MEAS_REG                      0xF4
#define BME280_CTRL_HUMIDITY_REG                  0xF2
#define BME280_CONFIG_REG				          0xF5 //Configuration Reg


struct
{
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;

	int32_t ucomp_pressure;
	int32_t ucomp_temperature;
	int32_t ucomp_humidity;

	int32_t t_fine;
} st_SensorCalibration;

struct
{
	float m_fTemperate;
	uint32_t m_uiPressure;
	uint32_t m_uiHumidity;
} st_BME280_SensorMeasurements;

typedef enum
{
    BME280_Stat_SUCCESS,
    BME280_Stat_ID_ERROR,
    BME280_Stat_I2C_ERROR,
    BME280_Stat_INTERNAL_ERROR,
	BME280_Reset_ERROR,
	BME280_Ctrl_Meas_ERROR,
	BME280_Ctrl_Humidity_ERROR,
	BME280_Temp_Comp_Read_ERROR,
	BME280_Pressure_Comp_Read_ERROR,
	BME280_Hum_Part1_Comp_Read_ERROR,
	BME280_Hum_Part2_Comp_Read_ERROR
    //...
} BME280_Status_e;
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

#include "BME280/CBME280.h"
#include "CCS811/CCCS811.h"
#include "VCNL4040/CVCNL4040.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define I2C_NODE DT_NODELABEL(i2c0)

// UUIDs for HRS
#define HRS_SERVICE_UUID BT_UUID_DECLARE_16(0x180D)
#define HRS_MEASUREMENT_UUID BT_UUID_DECLARE_16(0x2A37)
#define DATA_SIZE 20

typedef enum
{
	ERR_SUCCESS,
	ERR_I2C_INTERFACE_NOT_READY,
	ERR_SPI_INTERFACE_NOT_READY,
	ERR_SENSOR,

}ErrorCode_e;

static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
static struct bt_conn *current_conn = NULL;

struct heart_rate_measurement
{
    uint8_t custom_data[DATA_SIZE]; // user define data
};

static struct heart_rate_measurement hr_measurement = {
    .custom_data = {0}, // user define data
};

// GATT Service Definition
BT_GATT_SERVICE_DEFINE(hrs_service,
    BT_GATT_PRIMARY_SERVICE(HRS_SERVICE_UUID),
    BT_GATT_CHARACTERISTIC(HRS_MEASUREMENT_UUID,
        BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        NULL, NULL, NULL),
    BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

#if !defined(CONFIG_BT_EXT_ADV)
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};
#endif

static ATOMIC_DEFINE(state, 2U);

#define STATE_CONNECTED    1U
#define STATE_DISCONNECTED 2U

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err)
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
	else
	{
		printk("Connected\n");
		current_conn = bt_conn_ref(conn);
		(void)atomic_set_bit(state, STATE_CONNECTED);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

	current_conn = NULL;
	(void)atomic_set_bit(state, STATE_DISCONNECTED);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN] = {0};
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.cancel = auth_cancel,
};

#if defined(CONFIG_GPIO)
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS_OKAY(LED0_NODE)
#define HAS_LED 1
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define BLINK_ONOFF K_MSEC(500)

static struct k_work_delayable blink_work;
static bool led_is_on;

static void blink_timeout(struct k_work *work)
{
	led_is_on = !led_is_on;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);
	k_work_schedule(&blink_work, BLINK_ONOFF);
}

static int blink_setup(void)
{
	int err;
	printk("Checking LED device...");
	if (!gpio_is_ready_dt(&led)) 
	{
		printk("failed.\n");
		return -EIO;
	}
	printk("done.\n");

	printk("Configuring GPIO pin...");
	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err)
	{
		printk("failed.\n");
		return -EIO;
	}
	printk("done.\n");

	k_work_init_delayable(&blink_work, blink_timeout);
	return 0;
}

static void blink_start(void)
{
	printk("Start blinking LED...\n");
	led_is_on = false;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);
	k_work_schedule(&blink_work, BLINK_ONOFF);
}

static void blink_stop(void)
{
	struct k_work_sync work_sync;
	printk("Stop blinking LED.\n");
	k_work_cancel_delayable_sync(&blink_work, &work_sync);

	/* Keep LED on */
	led_is_on = true;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);
}
#endif /* LED0_NODE */
#endif /* CONFIG_GPIO */

static void hrs_notify(uint16_t CO2Data, uint16_t tVOCData, 
	uint32_t Humidity, uint32_t Pressure, uint16_t Temperate, 
	uint16_t ProxValue, uint16_t AmbientValue, uint16_t WhiteValue)
{
	hr_measurement.custom_data[0] = CO2Data & 0xFF;
	hr_measurement.custom_data[1] = (CO2Data >> 8) & 0xFF;
	hr_measurement.custom_data[2] = tVOCData & 0xFF;
	hr_measurement.custom_data[3] = (tVOCData >> 8) & 0xFF;
	hr_measurement.custom_data[4] = Humidity & 0xFF;
	hr_measurement.custom_data[5] = (Humidity >> 8) & 0xFF;
	hr_measurement.custom_data[6] = Pressure & 0xFF;
	hr_measurement.custom_data[7] = (Pressure >> 8) & 0xFF;
	hr_measurement.custom_data[8] = (Pressure >> 16) & 0xFF;
	hr_measurement.custom_data[9] = (Pressure >> 24) & 0xFF;
	hr_measurement.custom_data[10] = Temperate & 0xFF;
	hr_measurement.custom_data[11] = (Temperate >> 8) & 0xFF;
	hr_measurement.custom_data[12] = ProxValue & 0xFF;
	hr_measurement.custom_data[13] = (ProxValue >> 8) & 0xFF;
	hr_measurement.custom_data[14] = AmbientValue & 0xFF;
	hr_measurement.custom_data[15] = (AmbientValue >> 8) & 0xFF;
	hr_measurement.custom_data[16] = WhiteValue & 0xFF;
	hr_measurement.custom_data[17] = (WhiteValue >> 8) & 0xFF;

    int err = bt_gatt_notify(current_conn, &hrs_service.attrs[1], &hr_measurement, ARRAY_SIZE(hr_measurement.custom_data));
    if (err)
        printk("Failed to notify heart rate (err %d)\n", err);
    else
        printk("Heart Rate Notification sent with custom data\n");    
}

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
//static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

void Sensor_WriteRegAfterReadCount(uint8_t MainAddr, uint8_t* pRegAddr,uint8_t* pDataBuffer, uint8_t DataSize)
{
	struct i2c_msg msg[2];
	msg[0].buf = (uint8_t*)pRegAddr;
	msg[0].len = 1;
	msg[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	msg[1].buf = (uint8_t *)pDataBuffer;
	msg[1].len = DataSize;
	msg[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP;

	i2c_transfer(i2c_dev, msg, 2, MainAddr); // BME280_I2C_ADDR_SEC
}

void BME280_comp_temperature(void)
{
	uint8_t buffer[3] = {0};
	uint8_t write_reg_buffer[1] = {BME280_TEMPERATURE_MSB_REG};
   	i2c_write_read(i2c_dev, BME280_I2C_ADDR_SEC, write_reg_buffer, 1, buffer, sizeof(buffer));
	//BME280_WriteRegAfterReadCount(write_reg_buffer, buffer, sizeof(buffer));

	st_SensorCalibration.ucomp_temperature = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

	int64_t var1, var2;
	var1 = ((((st_SensorCalibration.ucomp_temperature>>3) - ((int32_t)st_SensorCalibration.dig_T1<<1))) * ((int32_t)st_SensorCalibration.dig_T2)) >> 11;
	var2 = (((((st_SensorCalibration.ucomp_temperature>>4) - ((int32_t)st_SensorCalibration.dig_T1)) * ((st_SensorCalibration.ucomp_temperature>>4) - ((int32_t)st_SensorCalibration.dig_T1))) >> 12) *
	((int32_t)st_SensorCalibration.dig_T3)) >> 14;

	st_SensorCalibration.t_fine = var1 + var2;
	st_BME280_SensorMeasurements.m_fTemperate = (st_SensorCalibration.t_fine * 5 + 128) >> 8;
	st_BME280_SensorMeasurements.m_fTemperate = st_BME280_SensorMeasurements.m_fTemperate / (float)100.0;
}

void BME280_comp_Press(void)
{
	uint8_t buffer[3] = {0};
	uint8_t write_reg_buffer[1] = {BME280_PRESSURE_MSB_REG};
	i2c_write_read(i2c_dev, BME280_I2C_ADDR_SEC, write_reg_buffer, 1, buffer, 3);
	st_SensorCalibration.ucomp_pressure = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);

	int64_t var1, var2, p_acc;
	var1 = ((int64_t)st_SensorCalibration.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)st_SensorCalibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)st_SensorCalibration.dig_P5)<<17);
	var2 = var2 + (((int64_t)st_SensorCalibration.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)st_SensorCalibration.dig_P3)>>8) + ((var1 * (int64_t)st_SensorCalibration.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)st_SensorCalibration.dig_P1)>>33;

	if(var1 == 0) return;
	
	p_acc = 1048576 - st_SensorCalibration.ucomp_pressure;
	p_acc = (((p_acc<<31) - var2)*3125)/var1;
	var1 = (((int64_t)st_SensorCalibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	var2 = (((int64_t)st_SensorCalibration.dig_P8) * p_acc) >> 19;
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)st_SensorCalibration.dig_P7)<<4);

	st_BME280_SensorMeasurements.m_uiPressure = (uint32_t)p_acc/256.0;
}

void BME280_comp_Humidity(void)
{
	uint8_t buffer[2] = {0};
	uint8_t write_reg_buffer[1] = {BME280_HUMIDITY_MSB_REG};
	i2c_write_read(i2c_dev, BME280_I2C_ADDR_SEC, write_reg_buffer, 1, buffer, sizeof(buffer));
	st_SensorCalibration.ucomp_humidity = ((uint32_t)(buffer[0] << 8) | (buffer[1]));

	int32_t var1;
	var1 = (st_SensorCalibration.t_fine - ((int32_t)76800));
	var1 = (((((st_SensorCalibration.ucomp_humidity << 14) - (((int32_t)st_SensorCalibration.dig_H4) << 20) - (((int32_t)st_SensorCalibration.dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)st_SensorCalibration.dig_H6)) >> 10) * (((var1 * ((int32_t)st_SensorCalibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)st_SensorCalibration.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)st_SensorCalibration.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	st_BME280_SensorMeasurements.m_uiHumidity = (var1 >> 12) / 1024.0;
}

bool IsCCS811Ready(void)
{
	uint8_t readBuffData[1] = {0};
	uint8_t write_reg_buffer[1] = {CSS811_HW_ID};
	int iErr = i2c_write_read(i2c_dev, CCS811_ADDR, write_reg_buffer, 1, readBuffData, sizeof(readBuffData));
	if(iErr < 0 && readBuffData[0] == 0x81)
	{
		printf("Read CCS811 chip id failed.\n");
		return false;
	}

	printf("CCS811 id: 0x%02X\n", readBuffData[0]);
	return true;
}

bool BME280_RRegister(uint8_t value, uint8_t* pRetValue)
{
	uint8_t readBuffData[1] = {0};
	uint8_t write_reg_buffer[1] = {value};
	int iErr = i2c_write_read(i2c_dev, BME280_I2C_ADDR_SEC, write_reg_buffer, 1, readBuffData, 1);
	if(iErr < 0)
		return false;

	(*pRetValue) = readBuffData[0];
	return true;
}

void BME280_SetStandbyTime(uint8_t value)
{
	if(value > 7) value = 0; //Error check. Default to 0.5ms
	
	uint8_t controlData;
	BME280_RRegister(BME280_CONFIG_REG, &controlData);
	controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear the 7/6/5 bits
	controlData |= (value << 5); //Align with bits 7/6/5
	i2c_reg_write_byte(i2c_dev, BME280_I2C_ADDR_SEC, BME280_CONFIG_REG, controlData);
}

void BME280_SetFilter(uint8_t filterSetting)
{
	if(filterSetting > 7) filterSetting = 0; //Error check. Default to filter off
	
	uint8_t controlData;
	BME280_RRegister(BME280_CONFIG_REG, &controlData);
	controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear the 4/3/2 bits
	controlData |= (filterSetting << 2); //Align with bits 4/3/2
	i2c_reg_write_byte(i2c_dev, BME280_I2C_ADDR_SEC, BME280_CONFIG_REG, controlData);
}

uint8_t BME280_CheckSampleValue(uint8_t userValue)
{
	switch(userValue) 
	{
		case(0): 
			return 0;
			break; //Valid
		case(1): 
			return 1;
			break; //Valid
		case(2): 
			return 2;
			break; //Valid
		case(4): 
			return 3;
			break; //Valid
		case(8): 
			return 4;
			break; //Valid
		case(16): 
			return 5;
			break; //Valid
		default: 
			return 1; //Default to 1x
			break; //Good
	}
}

uint8_t BME280_GetMode()
{
	uint8_t controlData;
	BME280_RRegister(BME280_CTRL_MEAS_REG, &controlData);
	return(controlData & 3); //Clear bits 7 through 2
}

void BME280_SetMode(uint8_t mode)
{
	if(mode > 3) mode = 0; //Error check. Default to sleep mode
	
	uint8_t controlData;
	BME280_RRegister(BME280_CTRL_MEAS_REG, &controlData);
	controlData &= ~( (1<<1) | (1<<0) ); //Clear the mode[1:0] bits
	controlData |= mode; //Set
	i2c_reg_write_byte(i2c_dev, BME280_I2C_ADDR_SEC, BME280_CTRL_MEAS_REG, controlData);
}

void BME280_SetPressureOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = BME280_CheckSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = BME280_GetMode(); //Get the current mode so we can go back to it at the end
	
	BME280_SetMode(0); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_p bits (4, 3, 2) to overSampleAmount
	uint8_t controlData = 0;
//	BME280_RRegister(BME280_CTRL_MEAS_REG, &controlData);

	uint8_t buffer[1] = {0};
  	uint8_t write_reg_buffer[1] = {BME280_CTRL_MEAS_REG};
	i2c_write_read(i2c_dev, BME280_I2C_ADDR_SEC, write_reg_buffer, 1, buffer, sizeof(buffer));
	//BME280_WriteRegAfterReadCount(write_reg_buffer, buffer, sizeof(buffer));
	controlData = buffer[0];
	
	controlData &= ~( (1<<4) | (1<<3) | (1<<2) ); //Clear bits 432
	controlData |= overSampleAmount << 2; //Align overSampleAmount to bits 4/3/2
	i2c_reg_write_byte(i2c_dev, BME280_I2C_ADDR_SEC, BME280_CTRL_MEAS_REG, controlData);
	
	BME280_SetMode(originalMode); //Return to the original user's choice
}

void BME280_SetHumidityOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = BME280_CheckSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = BME280_GetMode(); //Get the current mode so we can go back to it at the end
	
	BME280_SetMode(0); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_h bits (2, 1, 0) to overSampleAmount
	uint8_t controlData;
	BME280_RRegister(BME280_CTRL_HUMIDITY_REG, &controlData);
	controlData &= ~( (1<<2) | (1<<1) | (1<<0) ); //Clear bits 2/1/0
	controlData |= overSampleAmount << 0; //Align overSampleAmount to bits 2/1/0
	i2c_reg_write_byte(i2c_dev, BME280_I2C_ADDR_SEC, BME280_CTRL_HUMIDITY_REG, controlData);

	BME280_SetMode(originalMode); //Return to the original user's choice
}

void BME280_SetTempOverSample(uint8_t overSampleAmount)
{
	overSampleAmount = BME280_CheckSampleValue(overSampleAmount); //Error check
	
	uint8_t originalMode = BME280_GetMode(); //Get the current mode so we can go back to it at the end
	
	BME280_SetMode(0); //Config will only be writeable in sleep mode, so first go to sleep mode

	//Set the osrs_t bits (7, 6, 5) to overSampleAmount
	uint8_t controlData;
	BME280_RRegister(BME280_CTRL_MEAS_REG, &controlData);
	controlData &= ~( (1<<7) | (1<<6) | (1<<5) ); //Clear bits 765
	controlData |= overSampleAmount << 5; //Align overSampleAmount to bits 7/6/5
	i2c_reg_write_byte(i2c_dev, BME280_I2C_ADDR_SEC, BME280_CTRL_MEAS_REG, controlData);

	BME280_SetMode(originalMode); //Return to the original user's choice
}

uint8_t BME280_Calibration(void)
{
	uint8_t	ReadBuffTemp[6] = {0};
	uint8_t write_reg_buffer[1] = {BME280_REG_TEMP_CALIB_DATA};
	int iErr = i2c_write_read(i2c_dev, BME280_I2C_ADDR_SEC, write_reg_buffer, 1, ReadBuffTemp, sizeof(ReadBuffTemp));
	if(iErr < 0)
	{
		printk("BME280 temp comp read failed.\n");
		return BME280_Temp_Comp_Read_ERROR;
	}

	st_SensorCalibration.dig_T1 = (uint16_t)ReadBuffTemp[0];
	st_SensorCalibration.dig_T1 |= ((uint16_t)ReadBuffTemp[1]) << 8;
	st_SensorCalibration.dig_T2 = (uint16_t)ReadBuffTemp[2];
	st_SensorCalibration.dig_T2 |= ((uint16_t)ReadBuffTemp[3]) << 8;
	st_SensorCalibration.dig_T3 = (uint16_t)ReadBuffTemp[4];
	st_SensorCalibration.dig_T3 |= ((uint16_t)ReadBuffTemp[5]) << 8;
	printk("T1: %d, T2: %d, T3: %d\n", st_SensorCalibration.dig_T1, st_SensorCalibration.dig_T2, st_SensorCalibration.dig_T3);

	uint8_t ReadBuffPress[18] = {0};
	write_reg_buffer[0] = BME280_REG_PRESS_CALIB_DATA;
	iErr = i2c_write_read(i2c_dev, BME280_I2C_ADDR_SEC, write_reg_buffer, 1, ReadBuffPress, 18);
	if(iErr < 0)
	{
		printk("BME280 pressure comp read failed.\n");
		return BME280_Pressure_Comp_Read_ERROR;
	}

	st_SensorCalibration.dig_P1 = ((uint16_t)(ReadBuffPress[0]) + (ReadBuffPress[1] << 8));
	st_SensorCalibration.dig_P2 = ((uint16_t)(ReadBuffPress[2]) + (ReadBuffPress[3] << 8));
	st_SensorCalibration.dig_P3 = ((uint16_t)(ReadBuffPress[4]) + (ReadBuffPress[5] << 8));
	st_SensorCalibration.dig_P4 = ((uint16_t)(ReadBuffPress[6]) + (ReadBuffPress[7] << 8));
	st_SensorCalibration.dig_P5 = ((uint16_t)(ReadBuffPress[8]) + (ReadBuffPress[9] << 8));
	st_SensorCalibration.dig_P6 = ((uint16_t)(ReadBuffPress[10]) + (ReadBuffPress[11] << 8));
	st_SensorCalibration.dig_P7 = ((uint16_t)(ReadBuffPress[12]) + (ReadBuffPress[13] << 8));
	st_SensorCalibration.dig_P8 = ((uint16_t)(ReadBuffPress[14]) + (ReadBuffPress[15] << 8));
	st_SensorCalibration.dig_P9 = ((uint16_t)(ReadBuffPress[16]) + (ReadBuffPress[17] << 8));
	printk("P1: %d, P2: %d, P3: %d\n", st_SensorCalibration.dig_P1, st_SensorCalibration.dig_P2, st_SensorCalibration.dig_P3);
	printk("P4: %d, P5: %d, P6: %d\n", st_SensorCalibration.dig_P4, st_SensorCalibration.dig_P5, st_SensorCalibration.dig_P6);
	printk("P7: %d, P8: %d, P9: %d\n", st_SensorCalibration.dig_P7, st_SensorCalibration.dig_P8, st_SensorCalibration.dig_P9);

	write_reg_buffer[0] = BME280_REG_HUMIDITY1_CALIB_DATA;
	iErr = i2c_write_read(i2c_dev, BME280_I2C_ADDR_SEC, write_reg_buffer, 1, &st_SensorCalibration.dig_H1, 1);
	if(iErr < 0)
	{
		printk("BME280 hum part1 comp read failed.\n");
		return BME280_Hum_Part1_Comp_Read_ERROR;
	}

	uint8_t ReadBuffHum2[7] = {0};
	write_reg_buffer[0] = BME280_REG_HUMIDITY2_CALIB_DATA;
	iErr = i2c_write_read(i2c_dev, BME280_I2C_ADDR_SEC, write_reg_buffer, 1, ReadBuffHum2, 7);
	if(iErr < 0)
	{
		printk("BME280 hum part2 comp read failed.\n");
		return BME280_Hum_Part2_Comp_Read_ERROR;
	}

	st_SensorCalibration.dig_H2 = ReadBuffHum2[0] | (ReadBuffHum2[1] << 8);
	st_SensorCalibration.dig_H3 = ReadBuffHum2[2];
	st_SensorCalibration.dig_H4 = (ReadBuffHum2[3] << 4) | (ReadBuffHum2[4] & 0x0F);
	st_SensorCalibration.dig_H5 = ((ReadBuffHum2[4] >> 4) & 0x0F) | (ReadBuffHum2[5] << 4);
	st_SensorCalibration.dig_H6 = ReadBuffHum2[6];
	printk("H1: %d, H2: %d, H3: %d\n", st_SensorCalibration.dig_H1, st_SensorCalibration.dig_H2, st_SensorCalibration.dig_H3);
	printk("H4: %d, H5: %d, H6: %d\n", st_SensorCalibration.dig_H4, st_SensorCalibration.dig_H5, st_SensorCalibration.dig_H6);

	return BME280_Stat_SUCCESS;
}

uint16_t VCNL4040_ReadCommand(uint8_t RegisterAddr)
{
	uint8_t buffer[2] = {0};
	uint8_t write_reg_buffer[1] = {RegisterAddr};
	i2c_write_read(i2c_dev, VCNL4040_ADDR, write_reg_buffer, 1, buffer, sizeof(buffer));
//	Sensor_WriteRegAfterReadCount(VCNL4040_ADDR, write_reg_buffer, buffer, 2);

	return (uint16_t)((buffer[1] << 8) | buffer[0]);
}

void VNCL4040_IsConnect(void)
{
	uint8_t uszData[1] = {0};
	i2c_write(i2c_dev, uszData, 0, VCNL4040_ADDR);
}

bool IsVNCL4040Ready(void)
{
	uint8_t VCNL4040_read_buf_id[2] = {0};
	uint8_t write_reg_buffer[1] = {VCNL4040_ID};
	int iErr = i2c_write_read(i2c_dev, VCNL4040_ADDR, write_reg_buffer, 1, VCNL4040_read_buf_id, 2);
	if(iErr < 0)
	{
		printk("Read VCNL4040 chip id failed.\n");
		return false;
	}
	printk("VCNL4040 chip id: 0x%04X\n", (uint16_t)((VCNL4040_read_buf_id[1] << 8) | VCNL4040_read_buf_id[0]));
	return true;
}

bool writeCommand(uint8_t commandAddress, uint16_t value)
{
	uint8_t WriteData[3] = {0};
	WriteData[0] = commandAddress;
	WriteData[1] = (value >> 8); //MSB
	WriteData[2] = (value & 0xFF); //LSB
	int iErr = i2c_write(i2c_dev, WriteData, sizeof(WriteData), VCNL4040_ADDR);
	if(iErr < 0)
		return false;

	return true;
}

bool writeCommandLower(uint8_t commandAddress, uint8_t newValue)
{
  uint16_t commandValue = VCNL4040_ReadCommand(commandAddress);
  commandValue &= 0xFF00; //Remove lower 8 bits
  commandValue |= (uint16_t)newValue; //Mask in
  return (writeCommand(commandAddress, commandValue));
}

bool writeCommandUpper(uint8_t commandAddress, uint8_t newValue)
{
	uint16_t commandValue = VCNL4040_ReadCommand(commandAddress);
	commandValue &= 0x00FF; //Remove lower 8 bits
	commandValue |= (uint16_t)newValue << 8; //Mask in
	return (writeCommand(commandAddress, commandValue));
}

uint8_t VCNL4040_ReadCommandLower(uint8_t commandCode)
{
  uint16_t commandValue = VCNL4040_ReadCommand(commandCode);
  return (commandValue & 0xFF);
}

//Given a command code (address) read the upper byte
uint8_t VCNL4040_ReadCommandUpper(uint8_t commandCode)
{
  uint16_t commandValue = VCNL4040_ReadCommand(commandCode);
  return (commandValue >> 8);
}

void bitMask(uint8_t commandAddress, bool commandHeight, uint8_t mask, uint8_t thing)
{
  	// Grab current register context
  	uint8_t registerContents;

	if (commandHeight == LOWER)
		registerContents = VCNL4040_ReadCommandLower(commandAddress);
	else
		registerContents = VCNL4040_ReadCommandUpper(commandAddress);

	// Zero-out the portions of the register we're interested in
	registerContents &= mask;

	// Mask in new thing
	registerContents |= thing;

  	// Change contents
	if (commandHeight == LOWER)
		writeCommandLower(commandAddress, registerContents);		
	else
		writeCommandUpper(commandAddress, registerContents);
}

void VCNL4040_SetLEDCurrent(uint8_t value)
{
	if(value > 200 - 1) value = VCNL4040_LED_200MA;
	else if(value > 180 - 1) value = VCNL4040_LED_180MA;
	else if(value > 160 - 1) value = VCNL4040_LED_160MA;
	else if(value > 140 - 1) value = VCNL4040_LED_140MA;
	else if(value > 120 - 1) value = VCNL4040_LED_120MA;
	else if(value > 100 - 1) value = VCNL4040_LED_100MA;
	else if(value > 75 - 1) value = VCNL4040_LED_75MA;
	else value = VCNL4040_LED_50MA;

	writeCommand(VCNL4040_PS_MS, value); //Reset by select led current value
	bitMask(VCNL4040_PS_MS, UPPER, VCNL4040_LED_I_MASK, value);
}

void VCNL4040_setIRDutyCycle(uint16_t dutyValue)
{
	if(dutyValue > 320 - 1) dutyValue = VCNL4040_PS_DUTY_320;
	else if(dutyValue > 160 - 1) dutyValue = VCNL4040_PS_DUTY_160;
	else if(dutyValue > 80 - 1) dutyValue = VCNL4040_PS_DUTY_80;
	else dutyValue = VCNL4040_PS_DUTY_40;

	bitMask(VCNL4040_PS_CONF1, LOWER, VCNL4040_PS_DUTY_MASK, dutyValue);
}

void VCNL4040_setProxIntegrationTime(uint8_t timeValue)
{
	if(timeValue > 8 - 1) timeValue = VCNL4040_PS_IT_8T;
	else if(timeValue > 4 - 1) timeValue = VCNL4040_PS_IT_4T;
	else if(timeValue > 3 - 1) timeValue = VCNL4040_PS_IT_3T;
	else if(timeValue > 2 - 1) timeValue = VCNL4040_PS_IT_2T;
	else timeValue = VCNL4040_PS_IT_1T;

	bitMask(VCNL4040_PS_CONF1, LOWER, VCNL4040_PS_IT_MASK, timeValue);
}

void VCNL4040_SetProxResolution(uint8_t resolutionValue)
{
	if(resolutionValue > 16 - 1) resolutionValue = VCNL4040_PS_HD_16_BIT;
	else resolutionValue = VCNL4040_PS_HD_12_BIT;
	
  	bitMask(VCNL4040_PS_CONF2, UPPER, VCNL4040_PS_HD_MASK, resolutionValue);
}

void VCNL4040_EnableSmartPersistance(void)
{
	bitMask(VCNL4040_PS_CONF3, LOWER, VCNL4040_PS_SMART_PERS_MASK, VCNL4040_PS_SMART_PERS_ENABLE);
}

void VCNL4040_PowerOnProximity(void)
{
	bitMask(VCNL4040_PS_CONF1, LOWER, VCNL4040_PS_SD_MASK, VCNL4040_PS_SD_POWER_ON);
}

void VCNL4040_PowerOnAmbient(void)
{
	bitMask(VCNL4040_ALS_CONF, LOWER, VCNL4040_ALS_SD_MASK, VCNL4040_ALS_SD_POWER_ON);
}

void VCNL4040_EnableWhiteChannel(void)
{
	bitMask(VCNL4040_PS_MS, UPPER, VCNL4040_WHITE_EN_MASK, VCNL4040_WHITE_ENABLE);
}

bool ResetCCS811(void)
{
	uint8_t uszData[4] = {0x11, 0xE5, 0x72, 0x8A};
	//uint8_t uszRead_buff[1];
	//memset(uszRead_buff, 0, sizeof(uszRead_buff));

	for(int i=0; i<sizeof(uszData); i++)
	{
		i2c_reg_write_byte(i2c_dev, CCS811_ADDR, CSS811_SW_RESET, uszData[i]);
	}
	
	//int iErr = i2c_write_read(i2c_dev, CCS811_ADDR, uszData, sizeof(uszData), uszRead_buff, 1);
	//if(iErr < 0)
	//{
	//	printf("Reset CCS811 failed.\n");
	//	return false;
	//}

	return true;
}

bool CCS811_CheckForStatusError(void)
{
	uint8_t uszReadValue;
	int iErr = i2c_reg_read_byte(i2c_dev, CCS811_ADDR, CSS811_STATUS, &uszReadValue);
	if(iErr < 0)
	{
		printf("Get status failed.\n");
		return false;
	}

	return (uszReadValue & 1 << 0) == 1? true:false;
}

bool CCS811_CheckAppValid(void)
{
	uint8_t uszValue;
	int iErr = i2c_read(i2c_dev, &uszValue, 1, CCS811_ADDR);
	//int iErr = i2c_reg_read_byte(i2c_dev, CCS811_ADDR, CSS811_STATUS, &uszValue);
	if(iErr < 0)
	{
		printf("Get status failed.\n");
		return false;
	}

	return (uszValue & 1 << 4);
}

uint32_t CCS811_WDataToStartApp(void)
{
	uint8_t uszData[1] = {CSS811_APP_START};

	int iW0ByteErr = i2c_write(i2c_dev, uszData, 1, CCS811_ADDR);
	if(iW0ByteErr < 0)
	{
		printf("Write 0 byte to CCS811 failed.\n");
		return CCS811_Stat_I2C_ERROR;
	}

	return iW0ByteErr;
}

uint32_t CCS811_SetDriveMode(uint8_t usMode)
{
	uint8_t uszReadValue;

	int iErr = i2c_reg_read_byte(i2c_dev, CCS811_ADDR, CSS811_MEAS_MODE, &uszReadValue);
	if(iErr != CCS811_Stat_SUCCESS)
		return iErr;
	
	uszReadValue &= ~(7 << 4);
	uszReadValue |= (usMode << 4);
	iErr = i2c_reg_write_byte(i2c_dev, CCS811_ADDR, CSS811_MEAS_MODE, uszReadValue);

	return iErr;
}

uint32_t CCS811_ReadAlgorithmResults(uint16_t* pCO2Data, uint16_t* ptVOCData)
{
	uint8_t uszWData[1] = {CSS811_ALG_RESULT_DATA};
	uint8_t uszRData[4];

	memset(uszRData, 0, sizeof(uszRData));

	int iErr = i2c_write(i2c_dev, uszWData, 1, CCS811_ADDR);
	if(iErr < 0)
		return CCS811_Stat_I2C_ERROR;

	iErr = i2c_read(i2c_dev, uszRData, sizeof(uszRData), CCS811_ADDR);

	(*pCO2Data) = ((uint16_t)uszRData[0] << 8) | uszRData[1];
	(*ptVOCData) = ((uint16_t)uszRData[2] << 8) | uszRData[3];
	
	return CCS811_Stat_SUCCESS;
}

bool CCS811_DataAvailable(void)
{
	uint8_t uszReadValue;
	int iErr = i2c_reg_read_byte(i2c_dev, CCS811_ADDR, CSS811_STATUS, &uszReadValue);
	if(iErr != 0)
		return false;

	return (uszReadValue & 1 << 3);
}

uint32_t Ctrl_SensorBME280(void)
{
	uint8_t uszGetRetValue;
	if(!BME280_RRegister(BME280_REG_CHIP_ID, &uszGetRetValue))	
	 	return ERR_SENSOR;
	else
		printf("BME280 chip id(0x%02X): 0x%02X\n", BME280_REG_CHIP_ID, uszGetRetValue);

	BME280_Calibration();
	BME280_SetStandbyTime(0);
	BME280_SetFilter(0);
	BME280_SetPressureOverSample(1);
	BME280_SetHumidityOverSample(1);
	BME280_SetTempOverSample(1);
	BME280_SetMode(3);

	if(!BME280_RRegister(BME280_REG_CHIP_ID, &uszGetRetValue))	
	 	return ERR_SENSOR;

//	i2c_reg_write_byte(i2c_dev, BME280_I2C_ADDR_SEC, BME280_RST_REG, 0xB6); //Reset

	uint8_t readBuffData[1] = {0};
	if(!BME280_RRegister(BME280_RST_REG, &uszGetRetValue))
		return BME280_Reset_ERROR;
	else
		printf("BME280 reset(0x%02X): 0x%02X\n", BME280_RST_REG, readBuffData[0]);

	if(!BME280_RRegister(BME280_CTRL_MEAS_REG, &uszGetRetValue))
		return BME280_Ctrl_Meas_ERROR;
	else
		printf("BME280 ctrl meas(0x%02X): 0x%02X\n", BME280_CTRL_MEAS_REG, uszGetRetValue);

	if(!BME280_RRegister(BME280_CTRL_HUMIDITY_REG, &uszGetRetValue))
		return BME280_Ctrl_Humidity_ERROR;
	else
		printf("BME280 ctrl humidity(0x%02X): 0x%02X\n", BME280_CTRL_HUMIDITY_REG, uszGetRetValue);

	printf("Displaying all regs:\n");
	uint8_t uszMemCounter = 0x80;
	for(int i=8; i<16; i++)
	{
		printf("0x%X0:", i);
		for(int k=0; k<16; k++)
		{
			BME280_RRegister(uszMemCounter, &uszGetRetValue);
			printf("%02X ", uszGetRetValue);
			uszMemCounter++;
		}
		printf("\n");
	}

	return ERR_SUCCESS;
}

uint32_t Ctrl_SensorCCS811(void)
{	
	if(!ResetCCS811())
		return CCS811_Reset;

	if(!IsCCS811Ready())
		return ERR_SENSOR;

	volatile uint8_t temp = 0;
	for(uint32_t i = 0; i < 200000; i++) //Spin for a good while
	{
		temp++;
	}

	if(CCS811_CheckForStatusError() == true)
	{
		printf("Failed.\n");
		return CCS811_Stat_INTERNAL_ERROR;
	}

	if(CCS811_CheckAppValid() == false)
		return CCS811_Stat_INTERNAL_ERROR;

	uint32_t ulErrCode = 0;
	ulErrCode = CCS811_WDataToStartApp();
	if(ulErrCode == CCS811_Stat_SUCCESS)
		CCS811_SetDriveMode(1);

	return ERR_SUCCESS;
}

uint32_t Ctrl_SensorVCNL4040(void)
{
	if(!IsVNCL4040Ready())
		return ERR_SENSOR;

	VCNL4040_SetLEDCurrent(200);
	VCNL4040_setIRDutyCycle(40);
	VCNL4040_setProxIntegrationTime(8);
	VCNL4040_SetProxResolution(16);
	VCNL4040_EnableSmartPersistance();
	VCNL4040_PowerOnProximity();

	VCNL4040_PowerOnProximity();
	VCNL4040_PowerOnAmbient();
	VCNL4040_EnableWhiteChannel();
	printk("Set LED and duty cycle pass\n");

	return ERR_SUCCESS;
}

int main(void)
{
	printf("Search device is connect or not.\n");

	int err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized\n");
	bt_conn_auth_cb_register(&auth_cb_display);

#if !defined(CONFIG_BT_EXT_ADV)
	printk("Starting Legacy Advertising (connectable and scannable)\n");
    err = bt_le_adv_start(BT_LE_ADV_CONN_ONE_TIME, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err)
	{
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}
#endif	
	
	printk("Advertising successfully started\n");

#if defined(HAS_LED)
	err = blink_setup();
	if (err)
		return 0;

	blink_start();
#endif /* HAS_LED */

	if(!device_is_ready(i2c_dev))
	{
		printf("I2C mode not ready.\n");
		return ERR_I2C_INTERFACE_NOT_READY;
	}
	else
		printf("I2C mode is ready.\n");	

	i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_FAST)); // Change defaule 100KHz to 400JHz

	uint32_t ulErrCode = ERR_SUCCESS; 
	ulErrCode = Ctrl_SensorBME280();
	if(ulErrCode == ERR_SUCCESS)
		ulErrCode = Ctrl_SensorCCS811();
	if(ulErrCode == ERR_SUCCESS)
		ulErrCode = Ctrl_SensorVCNL4040();

	while (1)
	{
		do
		{
			BME280_comp_Humidity();
			BME280_comp_Press();
			BME280_comp_temperature();

			uint16_t CO2Data, tVOCData = 0;
			if(CCS811_DataAvailable())
				CCS811_ReadAlgorithmResults(&CO2Data, &tVOCData);

			uint16_t proxValue = VCNL4040_ReadCommand(VCNL4040_PS_DATA);
			uint16_t ambientValue = VCNL4040_ReadCommand(VCNL4040_ALS_DATA);
			uint16_t whiteValue = VCNL4040_ReadCommand(VCNL4040_WHITE_DATA);			

			printf("Humidity: %d, Pressure: %d, Temperature: %.2f\n", 
				st_BME280_SensorMeasurements.m_uiHumidity,
				st_BME280_SensorMeasurements.m_uiPressure,
				(double)st_BME280_SensorMeasurements.m_fTemperate);
			printf("CO2: %d, tVOC: %d\n", CO2Data, tVOCData);
			printf("Prox value[%d] Ambient light level[%d] White level[%d]\n", proxValue, ambientValue, whiteValue);
			printf("\n");
			
			if(current_conn)
			{
				hrs_notify(CO2Data, tVOCData, 
					st_BME280_SensorMeasurements.m_uiHumidity,
					st_BME280_SensorMeasurements.m_uiPressure,
					(uint16_t)(st_BME280_SensorMeasurements.m_fTemperate*(float)100.0),
					proxValue, ambientValue, whiteValue);
			}

			if (atomic_test_and_clear_bit(state, STATE_CONNECTED))
			{
				/* Connected callback executed */

				#if defined(HAS_LED)
							blink_stop();
				#endif /* HAS_LED */
			}
			else if (atomic_test_and_clear_bit(state, STATE_DISCONNECTED))
			{
				#if !defined(CONFIG_BT_EXT_ADV)
					printk("Starting Legacy Advertising (connectable and scannable)\n");
					err = bt_le_adv_start(BT_LE_ADV_CONN_ONE_TIME, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
					if (err)
					{
						printk("Advertising failed to start (err %d)\n", err);
						return 0;
					}
				#endif /* CONFIG_BT_EXT_ADV */

				#if defined(HAS_LED)
							blink_start();
				#endif /* HAS_LED */
			}

		} while (false);

		k_msleep(SLEEP_TIME_MS);
	}

	return 0;
}

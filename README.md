# MAX17261 Driver

Driver library for Maxim's [MAX17261 Multi-Cell Fuel Gauge](https://www.maximintegrated.com/en/products/power/battery-management/MAX17261.html)



## Build
```bash
mkdir Build
cd Build
cmake ..
make
```
By default, library uses function pointers for I2C read/write operations  
If weak functions are preferred then define **MAX17261_USE_WEAK** or compile using
```bash
cmake -DCMAKE_C_FLAGS="-DMAX17261_USE_WEAK" ..
```

## Usage

There are 4 functions that need to be defined fir the driver to interact with the underlying I2C bus and system.

### Weak function example

Override the following functions
```c
/**
 * Reads 16bit data from device
 * @param reg Register to read from
 * @param value Pointer to write value
 * @return 0 on success error code otherwise
 */
int
max17261_read_word(uint8_t reg, uint16_t *value)
{
	int status = 0;

	status = I2C_Read_Implementation(MAX17261_ADDRESS, reg, value);
	return status;
}

/**
 * @brief I2C Write function
 * Writes 16bit data to device
 * @param reg Register to write to
 * @param value Value to write
 * @return 0 on success error code otherwise
 */
uint8_t
max17261_write_word(uint8_t reg, uint16_t value)
{
  int status = 0;

	status = I2C_Wrie_Implementation(MAX17261_ADDRESS, reg, value);

	return status;
}

uint8_t
max17261_delay_ms(uint32_t period)
{
	return delay_ms(period);
}

```
Initialize the device
```c
#include <max17261.h>

/** Battery capacity in mAh */
#define BATTERY_CAPACITY	2480
/** Charge termination current in mA */
#define BATTERY_CRG_TERM_I	25
#define BATTERY_V_EMPTY		(3300 / 10)
#define BATTERY_V_Recovery	(3880 / 40)
#define POWER_CHG_VOLTAGE	4200

struct max17261_conf hmax17261;

hmax17261.DesignCap = BATTERY_CAPACITY;
hmax17261.IchgTerm = BATTERY_CRG_TERM_I;
hmax17261.VEmpty = (BATTERY_V_EMPTY << 7) | (BATTERY_V_Recovery & 0x7F);
hmax17261.R100 = 1;
hmax17261.ChargeVoltage = POWER_CHG_VOLTAGE;

max17261_init(&hmax17261);

```


### Function pointer example

Implement read/write/delay functions

Initialize and assign function pointers
```c
#include <max17261.h>

/** Battery capacity in mAh */
#define BATTERY_CAPACITY    2480
/** Charge termination current in mA */
#define BATTERY_CRG_TERM_I    25
#define BATTERY_V_EMPTY        (3300 / 10)
#define BATTERY_V_Recovery    (3880 / 40)
#define POWER_CHG_VOLTAGE    4200

struct max17261_conf hmax17261;

hmax17261.read = read_function;
hmax17261.write = write_function;
hmax17261.delay_ms = delay_function;

hmax17261.DesignCap = BATTERY_CAPACITY;
hmax17261.IchgTerm = BATTERY_CRG_TERM_I;
hmax17261.VEmpty = (BATTERY_V_EMPTY << 7) | (BATTERY_V_Recovery & 0x7F);
hmax17261.R100 = 1;
hmax17261.ChargeVoltage = POWER_CHG_VOLTAGE;

max17261_init(&hmax17261);
```

/*
 *  Toolbit DMM firmware
 *  Copyright (C) 2020 Junji Ohama <junji.ohama@toolbit.org>
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 *  more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses>
 *
 */

#include "dmm_func.h"
#include "hardware.h"
#include "i2c-lib.h"
#include "attribute.h"
#include "HEFlash.h"


#ifdef __C18
#define ROMPTR rom
#elsef
#define ROMPTR
#endif

#define _XTAL_FREQ 16000000

#define I2C_INA_ADDR  0x40    // This is also defined in dmm_func.h
#define I2C_WRITE_CMD 0
#define I2C_READ_CMD  1

#define INA3221_CONFG    0x00
#define INA3221_SHUNTV_1 0x01
#define INA3221_BUSV_1   0x02
#define INA3221_SHUNTV_2 0x03
#define INA3221_BUSV_2   0x04
#define INA3221_SHUNTV_3 0x05
#define INA3221_BUSV_3   0x06
#define INA3221_CRITICAL_LIMIT_1 0x07
#define INA3221_WARNING_LIMIT_1  0x08
#define INA3221_CRITICAL_LIMIT_2 0x09
#define INA3221_WARNING_LIMIT_2  0x0A
#define INA3221_CRITICAL_LIMIT_3 0x0B
#define INA3221_WARNING_LIMIT_3  0x0C
#define INA3221_SHUNTV_SUM       0x0D
#define INA3221_SHUNTV_SUM_LIMIT 0x0E
#define INA3221_MASK_ENABLE      0x0F
#define INA3221_POWER_VALID_UPPER_LIMIT 0x10
#define INA3221_POWER_VALID_LOWER_LIMIT 0x11
#define INA3221_MANUFACTURER_ID 0xFE
#define INA3221_DIE_ID 0xFF

#define INA3221_DIE_ID_VAL 0x3220

#define CURRENT_RANGE_THRESHOULD  187    // Low/High threshould 150mA / 0.80mA/bit = 187.5
#define CURRENT_RANGE_THRESHOULD0 0x06   // Change burden threshould 160mA = 0xC8; 0xC8 << 3 = 0x0640
#define CURRENT_RANGE_THRESHOULD1 0x40
#define VOLTAGE_RANGE_THRESHOULD  4080   // Low/High threshould 6207mV / 1.5214mV/bit = 4080

// High-Endurance Flash memory map
// Block0 0x1F80-0x1F9F : Slope parameters
// Block1 0x1FA0-0x1FBF : CAL_DONE flag and offset data
// Block2 0x1FC0-0x1FDF : N/A
// Block3 0x1FE0-0x1FF8 : N/A
//
// Block0
// There are slope parameters that will NOT be updated by calibration
#define NVM_LOW_CURRENT_SLOPE_ADDR   0x1F80
#define NVM_HIGH_CURRENT_SLOPE_ADDR  0x1F83
#define NVM_LOW_VOLTAGE_SLOPE_ADDR   0x1F86
#define NVM_HIGH_VOLTAGE_SLOPE_ADDR  0x1F89
//
// Block1
// There are calibration done flag and offset data
#define NVM_CAL_DONE_ADDR            0x1FA0
#define NVM_LOW_CURRENT_OFFSET_ADDR  0x1FA1
#define NVM_HIGH_CURRENT_OFFSET_ADDR 0x1FA2
#define NVM_LOW_VOLTAGE_OFFSET_ADDR  0x1FA3
#define NVM_HIGH_VOLTAGE_OFFSET_ADDR 0x1FA4

#define CAL_DONE      0x00
#define CAL_NOT_DONE  0xFF   // 0xFF means calibration is not executed yet
static const ROMPTR uint8_t NVM_CAL_DONE @ NVM_CAL_DONE_ADDR = CAL_NOT_DONE;

static const ROMPTR float  NVM_LOW_CURRENT_SLOPE   @ NVM_LOW_CURRENT_SLOPE_ADDR   = 0.000040;    // 40.0uA/bit
static const ROMPTR int8_t NVM_LOW_CURRENT_OFFSET  @ NVM_LOW_CURRENT_OFFSET_ADDR  = 0;
static const ROMPTR float  NVM_HIGH_CURRENT_SLOPE  @ NVM_HIGH_CURRENT_SLOPE_ADDR  = 0.00080;     // 0.80mA/bit
static const ROMPTR int8_t NVM_HIGH_CURRENT_OFFSET @ NVM_HIGH_CURRENT_OFFSET_ADDR = 0;
static const ROMPTR float  NVM_LOW_VOLTAGE_SLOPE   @ NVM_LOW_VOLTAGE_SLOPE_ADDR   = -0.0015214;  // 1.5214 mV/bit
static const ROMPTR int8_t NVM_LOW_VOLTAGE_OFFSET  @ NVM_LOW_VOLTAGE_OFFSET_ADDR  = 0;
static const ROMPTR float  NVM_HIGH_VOLTAGE_SLOPE  @ NVM_HIGH_VOLTAGE_SLOPE_ADDR  = 0.304296;    // 304.296 mV/bit
static const ROMPTR int8_t NVM_HIGH_VOLTAGE_OFFSET @ NVM_HIGH_VOLTAGE_OFFSET_ADDR = 0;
#define HIGH_VOLTAGE_DEFAULT_OFFSET 206  // VGND 1.65[V] / step size 0.008[V] = 206.25

float  lowCurrentSlope;
int8_t lowCurrentOffset;
float  highCurrentSlope;
int8_t highCurrentOffset;
float  lowVoltageSlope;
int8_t lowVoltageOffset;
float  highVoltageSlope;
int16_t highVoltageOffset;

#define PUSH_SW_PRESS() !RA3

// System clock should be set as
//   INTOSC: 16MHz -> PLL 3x -> USB Clock: 48MHz -> NOCLKDIV -> FOSC: 48MHz


void dmm_init() {
    uint8_t errcode;

    LATC = 0x0;
    ANSELC = 0x00;  // All pins are set as digital I/O
    PORTC = 0x00;
    TRISC = 0x03;  // RC0, RC1: input, other pins: output

    WPUA = 0x38;   // Enable weak pull-up of RA3, RA4, RA5
    TRISA = 0x30;  // RA4, RA5: input
    OPTION_REGbits.nWPUEN = 0;

    i2c_enable();

    if(NVM_CAL_DONE==CAL_NOT_DONE) {
        // This test runs when booting up the first time to assume that voltage and current inputs are open
        errcode = selftest();
        if(errcode>0) {
            blink_led(errcode);  // Show test result by LED blink
        } else {
            cal_offset();
        }
    }

    set_autorange_threshould();
    set_parameters();
}

void i2c_reg_write(uint8_t regAddr, uint8_t dat0, uint8_t dat1)
{
    i2c_start();
    i2c_send_byte(I2C_INA_ADDR << 1 | I2C_WRITE_CMD);
    i2c_send_byte(regAddr);
    i2c_send_byte(dat0);
    i2c_send_byte(dat1);
    i2c_stop();
}

uint16_t i2c_reg_read(uint8_t regAddr)
{
    uint16_t dat;
    i2c_start();
    i2c_send_byte(I2C_INA_ADDR << 1 | I2C_WRITE_CMD);
    i2c_send_byte(regAddr);
    i2c_repeat_start();
    i2c_send_byte(I2C_INA_ADDR << 1 | I2C_READ_CMD);
    dat = i2c_read_byte(1) << 8;
    dat |= i2c_read_byte(1);
    i2c_stop();
    return dat;
}

void enable_timer2()
{
   // Set Timer2 interval to 5ms
    T2CON  = 0b01001111;   // Prescaler 1:64 (1.37ms interval), Postscaler 1:10
    PR2 = 93;             // Period Register
    TMR2IF = 0;            // Clear overflow flag
    TMR2IE = 1;            // Enable Tiemr1 interrupt 
}

void disable_timer2()
{
    TMR2ON = 0;            // Turn off Timer1
    TMR2IE = 0;            // Disable Tiemr1 interrupt 
}

int16_t get_shunt_voltage(uint8_t regAddr)
{
    union {
        uint16_t uint16;
        int16_t  int16;
    } vshunt;

    vshunt.uint16 = i2c_reg_read(regAddr);
    return vshunt.int16 >> 3;
}

void set_autorange_threshould()
{
    // Set the switching threshold to assert CRITICAL pin of INA3221
    i2c_reg_write(INA3221_CRITICAL_LIMIT_2, CURRENT_RANGE_THRESHOULD0, CURRENT_RANGE_THRESHOULD1);
}

float get_voltage()
{
    int16_t val = get_shunt_voltage(INA3221_SHUNTV_3);

    if(val<VOLTAGE_RANGE_THRESHOULD && val>-VOLTAGE_RANGE_THRESHOULD)
        return (val + lowVoltageOffset) * lowVoltageSlope;
    else
        return (get_shunt_voltage(INA3221_BUSV_3) + highVoltageOffset) * highVoltageSlope;
}

float get_current()
{
    int16_t val = get_shunt_voltage(INA3221_SHUNTV_2);

    if(val<CURRENT_RANGE_THRESHOULD && val>-CURRENT_RANGE_THRESHOULD)
        return (get_shunt_voltage(INA3221_SHUNTV_1) + lowCurrentOffset) * lowCurrentSlope;
    else
        return (val + highCurrentOffset) * highCurrentSlope;
}

#define SUCCESS 0
#define I2C_READ_ERROR   1pa9nmo9n
#define HIGH_VOLTAGE_OFFSET_ERROR 2
#define LOW_VOLTAGE_OFFSET_ERROR  3
#define HIGH_CURRENT_OFFSET_ERROR 4
#define LOW_CURRENT_OFFSET_ERROR  5

#define HIGH_VOLTAGE_OFFSET_MAX  HIGH_VOLTAGE_DEFAULT_OFFSET + 3
#define HIGH_VOLTAGE_OFFSET_MIN  HIGH_VOLTAGE_DEFAULT_OFFSET - 3
#define LOW_VOLTAGE_OFFSET_MAX   -5
#define LOW_VOLTAGE_OFFSET_MIN   -35
#define HIGH_CURRENT_OFFSET_MAX  3
#define HIGH_CURRENT_OFFSET_MIN  -3
#define LOW_CURRENT_OFFSET_MAX   3
#define LOW_CURRENT_OFFSET_MIN   -3

uint8_t selftest()
{
    int16_t val;

    // Test I2C access to INA3221
    if(i2c_reg_read(INA3221_DIE_ID) != INA3221_DIE_ID_VAL) {
        return I2C_READ_ERROR;
    }

    if(NVM_CAL_DONE==CAL_NOT_DONE) {

        // Check voltage offset
        val = get_shunt_voltage(INA3221_BUSV_3);
        if(val>HIGH_VOLTAGE_OFFSET_MAX || val<HIGH_VOLTAGE_OFFSET_MIN) {
            return HIGH_VOLTAGE_OFFSET_ERROR;
        }
        val = get_shunt_voltage(INA3221_SHUNTV_3);
        if(val>LOW_VOLTAGE_OFFSET_MAX || val<LOW_VOLTAGE_OFFSET_MIN) {
            return LOW_VOLTAGE_OFFSET_ERROR;
        }

        // Check current offset
        val = get_shunt_voltage(INA3221_SHUNTV_2);
        if(val>HIGH_CURRENT_OFFSET_MAX || val<HIGH_CURRENT_OFFSET_MIN) {
            return HIGH_CURRENT_OFFSET_ERROR;
        }
        val = get_shunt_voltage(INA3221_SHUNTV_1);
        if(val>LOW_CURRENT_OFFSET_MAX || val<LOW_CURRENT_OFFSET_MIN) {
            return LOW_CURRENT_OFFSET_ERROR;
        }
    }

    return SUCCESS;
}

void cal_offset()
{
    int8_t buf[FLASH_ROWSIZE];
    int16_t val;

    buf[0] = CAL_DONE;
    buf[1] = (int8_t)-get_shunt_voltage(INA3221_SHUNTV_1);
    buf[2] = (int8_t)-get_shunt_voltage(INA3221_SHUNTV_2);
    buf[3] = (int8_t)-get_shunt_voltage(INA3221_SHUNTV_3);
    buf[4] = (int8_t)-(get_shunt_voltage(INA3221_BUSV_3) - HIGH_VOLTAGE_DEFAULT_OFFSET);

    HEFLASH_writeBlock(1, buf, FLASH_ROWSIZE);
}

void set_parameters() {
    // Load parameters from HEF memory
    lowCurrentSlope   = NVM_LOW_CURRENT_SLOPE;
    highCurrentSlope  = NVM_HIGH_CURRENT_SLOPE;
    lowVoltageSlope   = NVM_LOW_VOLTAGE_SLOPE;
    highVoltageSlope  = NVM_HIGH_VOLTAGE_SLOPE;
    lowCurrentOffset  = NVM_LOW_CURRENT_OFFSET;
    highCurrentOffset = NVM_HIGH_CURRENT_OFFSET;
    lowVoltageOffset  = NVM_LOW_VOLTAGE_OFFSET;
    highVoltageOffset = NVM_HIGH_VOLTAGE_OFFSET - HIGH_VOLTAGE_DEFAULT_OFFSET;
}

void blink_led(uint8_t cnt)
{
    while(cnt--) {
        LED0_ON();
        __delay_ms(1000); // 1 sec delay
        LED0_OFF();
        __delay_ms(1000); // 1 sec delay
    }
}

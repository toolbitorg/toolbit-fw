/*
 *  ChoppyUSB firmware
 *  Copyright (C) 2024 Junji Ohama <junji.ohama@toolbit.org>
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

#include "choppy_func.h"
#include "hardware.h"
#include "i2c-lib.h"
#include "attribute.h"
//#include "HEFlash.h"
#include "Flash.h"


#ifdef __C18
#define ROMPTR rom
#else
#define ROMPTR
#endif

#define _XTAL_FREQ 16000000

#define I2C_INA_ADDR  0x40
#define I2C_WRITE_CMD 0
#define I2C_READ_CMD  1

#define INA228_CONFIG       0x00
#define INA228_ADC_CONFIG   0x01
#define INA228_SHUNT_CAL    0x02
#define INA228_SHUNT_TEMPCO 0x03
#define INA228_VSHUNT       0x04
#define INA228_VBUS         0x05
#define INA228_DIETEMP      0x06
#define INA228_CURRENT      0x07
#define INA228_POWER        0x08
#define INA228_ENERGY       0x09
#define INA228_CHARGE       0x0A
#define INA228_DIAG_ALRT    0x0B
#define INA228_SOVL         0x0C
#define INA228_SUVL         0x0D
#define INA228_BOVL         0x0E
#define INA228_BUVL         0x0F
#define INA228_TEMP_LIMIT   0x10
#define INA228_PWR_LIMIT    0x11
#define INA228_MANUFACTURER_ID 0x3E
#define INA228_DEVICE_ID       0x3F

#define INA228_MANUFACTURER_ID_VAL 0x5449
#define INA228_DEVICE_ID_VAL       0x2281

// High-Endurance Flash memory map
// Block0 0x1F80-0x1F9F : COLOR
// Block1 0x1FA0-0x1FBF : N/A
// Block2 0x1FC0-0x1FDF : N/A
// Block3 0x1FE0-0x1FF8 : N/A
//
// Block0
#define NVM_COLOR_ADDR   0x1F80

// Color value
#define COLOR_BLACK    0x00
#define COLOR_BROWN    0x01
#define COLOR_RED      0x02
#define COLOR_ORANGE   0x03
#define COLOR_YELLOW   0x04
#define COLOR_GREEN    0x05
#define COLOR_BLUE     0x06
#define COLOR_VIOLET   0x07
#define COLOR_GRAY     0x08
#define COLOR_WHITE    0x09
static const ROMPTR uint8_t NVM_COLOR @ NVM_COLOR_ADDR = COLOR_BLACK;

// Error code
#define SUCCESS             0
#define RID_OPEN            1
#define I2C_READ_ERROR      2
#define RID_ADC_ERROR       3


float  vshuntSlope;
int8_t vshuntOffset;
float  vbusSlope;
int8_t vbusOffset;

#define PUSH_SW_PRESS() !RA3

// System clock should be set as
//   INTOSC: 16MHz -> PLL 3x -> USB Clock: 48MHz -> NOCLKDIV -> FOSC: 48MHz


void choppy_init() {
    uint8_t errcode;

    LATA = 0x0;
    ANSELA = 0x10; // RA4 is set as analog input
    TRISA = 0x30;  // RA    5   4    3    2    1   0
                   //      in  in  in*  N/A  in* in*    *:always in
    WPUA = 0x28;   // Enable weak pull-up of RA3, RA5 and disable RA4 because it has an external pull-up

    LATC = 0x0;
    ANSELC = 0x00; // All pins are set as digital I/O
    TRISC = 0x03;  // RC0, RC1: input, other pins: output
    PORTC = 0x00;
    OPTION_REGbits.nWPUEN = 0;

//  ADCON2 = 0x00;  // No auto-conversion trigger selected
    ADCON2 = 0x50;  // TRIGSEL[6:4] is set as TMR2
    ADCON1 = 0x60;  // Left justified, FOSC/64, VREF+ is connected to VDD
    
    ADCON0 = 0x0D;  // Select AN3(RA4), Enable ADC    
    __delay_ms(1);  // delay
    GO = 1;  // A/D Conversion Start
    
    errcode = set_parameters_by_rid();
    if(errcode>0) {
        blink_led(errcode);  // Show test result by LED blink
    }
    vbusSlope    = 0.0001953125;     // Bus voltage 1 LSB step size: 195.3125[uV/bit];
    
    i2c_enable();    
    errcode = selftest();
    if(errcode>0) {
        blink_led(errcode);  // Show test result by LED blink
    }

    config_monitor_reg();

    // GPIO interrupt setting
    /* Do NOT use RA5 pin as innterrupt
    IOCAP = 0x00;  // Disable positive edge detection
    IOCAN = 0x28;  // Enable RA5 negative edge detection
                   // RA5: ALERT_N is an interrupt pin of INA228
    IOCAF = 0;  // Clear porta flag
    IOCIE = 1;  // Enable IOC
    */

    // Timer1 setting for 2.7ms
    /* Do NOT use timer1
    T1CON  = 0b01010001;   // TMR1CS:  0b01 FOSC
                           // T1CKPS:  0b01 1:2 Prescalaer
                           // T1OSCEN: OFF
                           // nT1SYNC: OFF
                           // TMR1ON:  ON
    TMR1IF = 0;            // Clear overflow flag
    TMR1IE = 1;            // Enable Tiemr1 interrupt
    */      
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

uint24_t i2c_reg_read_3bytes(uint8_t regAddr)
{
    uint24_t dat;
    i2c_start();
    i2c_send_byte(I2C_INA_ADDR << 1 | I2C_WRITE_CMD);
    i2c_send_byte(regAddr);
    i2c_repeat_start();
    i2c_send_byte(I2C_INA_ADDR << 1 | I2C_READ_CMD);
    dat = i2c_read_byte(1) << 8;
    dat = (dat | i2c_read_byte(1)) << 8;
    dat |= i2c_read_byte(1);
    i2c_stop();
    return dat;
}

void config_monitor_reg()
{
    // Configure ADC_CONFIG register of INA228
    // to 0b1011 101 101 101 000
    //   MODE[15:12]:  0xB, Continuous shunt and bus voltage
    //   VBUSCT[11:9]: 0x1, 84us
    //   VSHCT[8-6]:   0x2, 150us
    //   VTCT[5-3]:    0x0, 50us
    //   AVG[2-0]:     0x1, 4
    //  (84us + 150us) x 4 times = 936us < 1ms
    i2c_reg_write(INA228_ADC_CONFIG, 0b10110010, 0b10000001);

    // Configure DIAG_ALRT register of INA228
    // to 0b0100 0000 0000 0001
    //   ALATCH[15]:    0b0, Transparent, alert pin resets to the idel state when the fault has been cleared
    //                  0b1, Latch mode, alert pin remains active until DIAG_ALRT register has been read
    //   CNVR[14]:      0b1, Enables Conversion ready flag on ALERT pin
    //   SLOWALERT[13]: 0b0
    //   APOL[12]:      0b0, Active-low, open-drain
    //   ENERGYOF[11], CHARGEOF[10], MATHOF[9], RESERVED[8]: Read only
    //   TMPOL[7]:   0b0
    //   SHNTOL[6]:  0b0
    //   SHNTUL[5]:  0b0
    //   BUSOL[4]:   0b0
    //   BUSUL[3]:   0b0
    //   POL[2]:     0b0
    //   CNVRF[1]:   0b0, This bit is set to 1 if the conversion is completed
    //   MEMSTAT[0]: 0b1, Normal Operation
    i2c_reg_write(INA228_DIAG_ALRT, 0b11000000, 0b00000001);
}

void enable_timer2()
{
   // Set Timer2 interval to 1ms
    T2CON  = 0b00000111;   // Prescaler 1:64 (1.37ms interval), Postscaler 1:1
    PR2 = 186;             // Period Register
    TMR2IF = 0;            // Clear overflow flag
    TMR2IE = 1;            // Enable Tiemr1 interrupt 
}

void disable_timer2()
{
    TMR2ON = 0;            // Turn off Timer1
    TMR2IE = 0;            // Disable Tiemr1 interrupt 
}

uint16_t check_diag_alert()
{
    return i2c_reg_read(INA228_DIAG_ALRT);  // Clear diagnostic flags and alert registor by reading it
}

int24_t get_shunt_voltage(uint8_t regAddr)
{
    union {
        uint24_t uint24;
        int24_t  int24;
    } vshunt;

    vshunt.uint24 = i2c_reg_read_3bytes(regAddr);
    return vshunt.int24 >> 4;
}

float get_voltage()
{
    int24_t val = get_shunt_voltage(INA228_VBUS);
    return (val + vbusOffset) * vbusSlope;
}

float get_current()
{
    int24_t val = get_shunt_voltage(INA228_VSHUNT);
    return (val + vshuntOffset) * vshuntSlope;
}

uint8_t get_color()
{
    return NVM_COLOR;    // This is same as return FLASH_read(NVM_COLOR_ADDR);
}

void set_color(uint8_t val)
{
    GIE = 0;   // Disable interrupt
    FLASH_erase(NVM_COLOR_ADDR);
    FLASH_write(NVM_COLOR_ADDR, val, 0);
    GIE = 1;   // Enable interrupt
}

uint8_t selftest()
{
    // Test I2C access to INA228
    if(i2c_reg_read(INA228_MANUFACTURER_ID) != INA228_MANUFACTURER_ID_VAL) {
        return I2C_READ_ERROR;
    }

    return SUCCESS;
}

uint8_t set_parameters_by_rid()
{    
    uint8_t errcode = SUCCESS;

    while(GO);    
    uint8_t val = ADRESH;
    
    if(val<=127) {               // Error
        vshuntSlope  = 0.0;
        errcode = RID_ADC_ERROR;
    } else if(val<=129) {        // Shunt R: 10m ohm
        vshuntSlope = 31.250 / 1000000.0;
    } else if(val<=172) {        // Shunt R: 20m ohm
        vshuntSlope = 15.625 / 1000000.0;
    } else if(val<=198) {        // Shunt R: 33m ohm
        vshuntSlope = 9.470 / 1000000.0;
    } else if(val<=213) {        // Shunt R: 47m ohm
        vshuntSlope = 6.649 / 1000000.0;
    } else if(val<=235) {        // Shunt R: 100m ohm
        vshuntSlope = 3.125 / 1000000.0;
    } else if(val<=253) {                    // Shunt R: 500m ohm
        vshuntSlope = 0.625 / 1000000.0;     // 0.625[nA/bit]
    } else {
        vshuntSlope = 0.0;        // There is no target
        errcode = RID_OPEN;
    }     
    
    return errcode;
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

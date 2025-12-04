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

#ifndef DMM_FUNC_H__
#define DMM_FUNC_H__

#include <xc.h>
#include <stdint.h>
#include "i2c-lib.h"

#define I2C_INA_ADDR  0x40
#define NVM_ADDR      0x1FA1
#define NVM_NUM_OF_BYTE_DATA 4

#define TRIGGER_MODE_NONE       0
#define TRIGGER_MODE_NORMAL     1
#define TRIGGER_MODE_CONTINUOUS 2

#define LED0_ON()       RC2 = 1   // PORTCbits.RC2 = 1
#define LED0_OFF()      RC2 = 0   // PORTCbits.RC2 = 0
#define LED0_TOGGLE()   RC2 = !RC2


void dmm_init();
void i2c_reg_write(uint8_t regAddr, uint8_t dat0, uint8_t dat1);
uint16_t i2c_reg_read(uint8_t regAddr);

void enable_timer2();
void disable_timer2();

int16_t get_shunt_voltage(uint8_t regAddr);
void set_autorange_threshould();

float get_voltage();
float get_current();

uint8_t selftest();
void cal_offset();
void set_parameters();
void blink_led(uint8_t cnt);


#endif

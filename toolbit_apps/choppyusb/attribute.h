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

#ifndef ATTRIBUTE_H__
#define	ATTRIBUTE_H__

#ifdef __C18
#define ROMPTR rom
#else
#define ROMPTR
#endif

#define VALUE_LEN 32

#define PROTOCOL_VERSION 0x00  // 7-6bits is effective

//#define byte uint8_t
#define ATTID uint16_t

typedef enum
{
	// Operation Code
	OP_METADATA_GET = 0x10,
	OP_CONFIG_SET,
	OP_ATT_VALUE_SET,
	OP_ATT_VALUE_GET,
	// Event Code
	OP_INTERRUPT_TRANSFER = 0xA0,
	OP_EVT_NOTIFY = 0xA1
} OperationCode;

typedef enum
{
	// Return Code
	RC_FAIL = 0x0,
	RC_OK   = 0x1,
} ReturnCode;

typedef enum
{
    // Toolbit common attribute ID
    ATT_VENDOR_NAME      = 0x0000,
    ATT_PRODUCT_NAME     = 0x0001,
    ATT_PRODUCT_REVISION = 0x0002,
    ATT_PRODUCT_SERIAL   = 0x0003,
    ATT_FIRM_VERSION     = 0x0004,

    // Platform commom attribute ID
    ATT_RESET            = 0x1100,
    ATT_I2C0_DEVICE_ADDR = 0x1400,
    ATT_I2C0_REG_ADDR    = 0x1401,
    ATT_I2C0_RW_1BYTE    = 0x1402,
    ATT_I2C0_RW_2BYTE    = 0x1403,
    ATT_I2C0_RW_3BYTE    = 0x1404,

    // Product specific attribute ID
    ATT_TRIGGER_MODE     = 0x8000,
    ATT_COLOR            = 0x8100,
    ATT_VOLTAGE          = 0x8101,
	ATT_CURRENT          = 0x8102

} AttributionID;

/*
struct Attribution{
    uint16_t id;
    uint8_t permission;
    uint8_t property;
    uint8_t *value;
};
 */

static const ROMPTR char VENDOR_NAME[]      = "Toolbit";
static const ROMPTR char PRODUCT_NAME[]     = "Choppy";
static const ROMPTR char FIRM_VERSION[]     = "0.2";
// 0.1: first distributed version
// 0.2: Support interrupt-driven data transfer

// The serial number provided by 'USB 512-Word DFU Bootloader for PIC16(L)F1454/5/9' as follows as
//      0x81EE: one byte length
//      0x81EF: one byte for string type
//      0x81F0-0x81FF: 8 x 2 bytes for eight character UNICODE string
#define NVM_PRODUCT_SERIAL_ADDR 0x81F0
#define NVM_PRODUCT_SERIAL_SIZE 8
static const ROMPTR uint16_t NVM_PRODUCT_SERIAL[] @ NVM_PRODUCT_SERIAL_ADDR;


#endif	/* ATTRIBUTE_H__ */

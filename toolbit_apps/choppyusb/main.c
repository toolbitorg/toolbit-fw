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
 *  This program is based on USB HID Mouse that is distributed in
 *  the Apache License 2.0 by Alan Ott, Signal 11 Software.
 *
 */

#include "usb.h"
#include <xc.h>
#include <string.h>
#include "usb_config.h"
#include "usb_ch9.h"
#include "usb_hid.h"
#include "hardware.h"
#include "i2c-lib.h"
#include "attribute.h"
#include "choppy_func.h"


uint8_t trigger_mode;


#define TRANSFER_DATA_BUF_SIZE 4
#define TRANSFER_DATA_EMPTY    0
#define TRANSFER_DATA_FULL     TRANSFER_DATA_BUF_SIZE
#define TRANSFER_DATA_DISABLE  255

uint8_t transfer_data_cnt;
uint8_t transfer_data_buf[8*TRANSFER_DATA_BUF_SIZE];
float latest_volt;
float latest_curr;

#ifdef MULTI_CLASS_DEVICE
static uint8_t hid_interfaces[] = {0};
#endif


int main(void) {

    // Set these variables before initialization
    trigger_mode = TRIGGER_MODE_NONE;
    transfer_data_cnt = TRANSFER_DATA_DISABLE;
    
    hardware_init();

    choppy_init();

#ifdef MULTI_CLASS_DEVICE
    hid_set_interface_list(hid_interfaces, sizeof (hid_interfaces));
#endif
    usb_init();

    uint8_t  regAddr = 0;
        
    while (1) {
        if (usb_is_configured() && usb_out_endpoint_has_data(1)) {

            const unsigned char *RxDataBuffer;
            unsigned char *TxDataBuffer = usb_get_in_buffer(1);
            /* Data received from host */

            if (!usb_in_endpoint_halted(1)) {
                /* Wait for EP 1 IN to become free then send. This of
                 * course only works using interrupts. */
                while (usb_in_endpoint_busy(1)) {
                    ;
                }
                
                usb_get_out_buffer(1, &RxDataBuffer);

                uint8_t pcktVer = RxDataBuffer[0] & 0xC0;
                uint8_t pcktLen = RxDataBuffer[0] & 0x2F;

                if (pcktVer == PROTOCOL_VERSION && pcktLen > 1) {

                    uint8_t opcode = RxDataBuffer[1];
                    TxDataBuffer[1] = opcode; // echo back operation code
                    ATTID id = (RxDataBuffer[2] << 8) + RxDataBuffer[3];
                    uint8_t len;

                    switch (opcode) {
                        case OP_ATT_VALUE_GET:
                            TxDataBuffer[2] = RC_OK; // Return OK code
                            TxDataBuffer[0] = PROTOCOL_VERSION;

                            switch (id) {
                                case ATT_VENDOR_NAME:
                                  len = strlen(VENDOR_NAME) + 1; // +1 for NULL
                                  TxDataBuffer[0] |= len + 3; // packet length
                                  memcpy(&TxDataBuffer[3], VENDOR_NAME, len);
                                  break;

                                case ATT_PRODUCT_NAME:
                                    len = strlen(PRODUCT_NAME) + 1; // +1 for NULL
                                    TxDataBuffer[0] |= len + 3; // packet length
                                    memcpy(&TxDataBuffer[3], PRODUCT_NAME, len);
                                    break;

                                case ATT_PRODUCT_REVISION:
                                    TxDataBuffer[0] |= 2 + 3; // packet length
                                    TxDataBuffer[3] = '0';
                                    TxDataBuffer[4] = NULL;
                                    break;

                                case ATT_PRODUCT_SERIAL:
                                    TxDataBuffer[0]  |= NVM_PRODUCT_SERIAL_SIZE + 1 + 3; // packet length  +1 for NULL
                                    // Transfer 8bit char code instead on UNICODE
                                    TxDataBuffer[3] = NVM_PRODUCT_SERIAL[0];
                                    TxDataBuffer[4] = NVM_PRODUCT_SERIAL[1];
                                    TxDataBuffer[5] = NVM_PRODUCT_SERIAL[2];
                                    TxDataBuffer[6] = NVM_PRODUCT_SERIAL[3];
                                    TxDataBuffer[7] = NVM_PRODUCT_SERIAL[4];
                                    TxDataBuffer[8] = NVM_PRODUCT_SERIAL[5];
                                    TxDataBuffer[9] = NVM_PRODUCT_SERIAL[6];
                                    TxDataBuffer[10] = NVM_PRODUCT_SERIAL[7];
                                    TxDataBuffer[11] = NULL;
                                    break;

                                case ATT_FIRM_VERSION:
                                    len = strlen(FIRM_VERSION) + 1; // +1 for NULL
                                    TxDataBuffer[0] |= len + 3; // packet length
                                    memcpy(&TxDataBuffer[3], FIRM_VERSION, len);
                                    break;
                                    
                                case ATT_I2C0_DEVICE_ADDR:
                                    TxDataBuffer[0]  |= 1 + 3; // packet length
                                    TxDataBuffer[3] = I2C_INA_ADDR;  // I2C device address is fixed
                                    break;

                                case ATT_I2C0_RW_2BYTE:
                                    TxDataBuffer[0]  |= 2 + 3; // packet length
                                    uint16_t dat = i2c_reg_read(regAddr);
                                    TxDataBuffer[4] = dat >> 8;
                                    TxDataBuffer[3] = dat;
                                    break;

                                case ATT_I2C0_RW_3BYTE:
                                    TxDataBuffer[0]  |= 3 + 3; // packet length
                                    uint24_t dat2 = i2c_reg_read_3bytes(regAddr);
                                    TxDataBuffer[5] = dat2 >> 16;
                                    TxDataBuffer[4] = dat2 >> 8;
                                    TxDataBuffer[3] = dat2;
                                    break;

                                case ATT_TRIGGER_MODE:
                                    TxDataBuffer[0]  |= 1 + 3; // packet length
                                    //
                                    TxDataBuffer[3] = trigger_mode;
                                    break;

                                case ATT_COLOR:
                                    TxDataBuffer[0]  |= 1 + 3; // packet length
                                    //
                                    TxDataBuffer[3] = get_color();
                                    break;

                                case ATT_VOLTAGE:
                                    TxDataBuffer[0] |= 4 + 3; // packet length
                                    TxDataBuffer[3] = 0x00;
                                    GIE = 0;  // Disable interrupt
                                    memcpy(&TxDataBuffer[4], &latest_volt, 3);
                                    GIE = 1;  // Enable Interrupt
                                    break;

                                case ATT_CURRENT:
                                    TxDataBuffer[0] |= 4 + 3; // packet length
                                    TxDataBuffer[3] = 0x00;
                                    GIE = 0;  // Disable interrupt
                                    memcpy(&TxDataBuffer[4], &latest_curr, 3);
                                    GIE = 1;  // Enable Interrupt
                                    break;

                                default:
                                    TxDataBuffer[0] |= 3; // packet length
                                    TxDataBuffer[2] = RC_FAIL; // Return error code
                                    break;

                            } // end of switch (id)
                            break;

                        case OP_ATT_VALUE_SET:
                            TxDataBuffer[0] = PROTOCOL_VERSION | 3; // packet length
                            TxDataBuffer[2] = RC_OK; // Return OK code

                            switch (id) {

                                case ATT_RESET:
                                    WDTCON = 1;
                                    break;
                                
                                case ATT_I2C0_DEVICE_ADDR:
                                    // Nothing to do because I2C device address is fixed
                                    break;

                                case ATT_I2C0_REG_ADDR:
                                    regAddr = RxDataBuffer[4];
                                    break;

                                case ATT_I2C0_RW_2BYTE:
                                    i2c_reg_write(regAddr, RxDataBuffer[5], RxDataBuffer[4]);
                                    break;

                                case ATT_TRIGGER_MODE:
                                    trigger_mode = RxDataBuffer[4];
                                    if(trigger_mode==TRIGGER_MODE_NORMAL) {
                                        transfer_data_cnt = TRANSFER_DATA_DISABLE;
                                        enable_timer2();
                                    } else if(trigger_mode==TRIGGER_MODE_CONTINUOUS) {
                                        transfer_data_cnt = TRANSFER_DATA_EMPTY;
                                        enable_timer2();
                                    } else {
                                        disable_timer2();
                                    }
                                    break;

                                case ATT_COLOR:
                                    set_color(RxDataBuffer[4]);
                                    break;

                                default:
                                    TxDataBuffer[2] = RC_FAIL; // Return error code
                                    break;

                            } // end of switch (id)
                            break;

                        default:
                            TxDataBuffer[0] = PROTOCOL_VERSION | 3; // packet length
                            TxDataBuffer[2] = RC_FAIL; // Return error code
                            break;

                    } // switch (opcode)

                    // Send response
                    memcpy(usb_get_in_buffer(1), TxDataBuffer, EP_1_IN_LEN);
                    usb_send_in_buffer(1, EP_1_IN_LEN);
                    
                } // end of if (pcktVer == PROTOCOL_VERSION && pcktLen > 1)
            }
            usb_arm_out_endpoint(1);
        }
        else if (usb_is_configured()) {

            // When usb_out_endpoint_has_data(1) is false, check send_interrupt
            if(trigger_mode==TRIGGER_MODE_CONTINUOUS && transfer_data_cnt==TRANSFER_DATA_FULL) {

                unsigned char *TxDataBuffer = usb_get_in_buffer(1);

                if (!usb_in_endpoint_halted(1)) {
                    /* Wait for EP 1 IN to become free then send. This of
                     * course only works using interrupts. */
                    while (usb_in_endpoint_busy(1)) {
                        ;
                    }
                        
                    TxDataBuffer[0] = 8*TRANSFER_DATA_BUF_SIZE + 2;
                    TxDataBuffer[1] = OP_INTERRUPT_TRANSFER;

                    GIE = 0;  // Disable interrupt
                    memcpy(&TxDataBuffer[2], transfer_data_buf, 8*TRANSFER_DATA_BUF_SIZE);
                    GIE = 1;  // Enable Interrupt
                
                    memcpy(usb_get_in_buffer(1), TxDataBuffer, EP_1_IN_LEN);
                    usb_send_in_buffer(1, EP_1_IN_LEN);
                    
                    transfer_data_cnt = TRANSFER_DATA_EMPTY;
                } else {
                    ;
                }
            }
        }

#ifndef USB_USE_INTERRUPTS
        usb_service();
#endif
    }

    return 0;
}

/* Callbacks. These function names are set in usb_config.h. */
void app_set_configuration_callback(uint8_t configuration) {
}

uint16_t app_get_device_status_callback() {
    return 0x0000;
}

void app_endpoint_halt_callback(uint8_t endpoint, bool halted) {
}

int8_t app_set_interface_callback(uint8_t interface, uint8_t alt_setting) {
    return 0;
}

int8_t app_get_interface_callback(uint8_t interface) {
    return 0;
}

void app_out_transaction_callback(uint8_t endpoint) {

}

void app_in_transaction_complete_callback(uint8_t endpoint) {

}

int8_t app_unknown_setup_request_callback(const struct setup_packet *setup) {
    /* To use the HID device class, have a handler for unknown setup
     * requests and call process_hid_setup_request() (as shown here),
     * which will check if the setup request is HID-related, and will
     * call the HID application callbacks defined in usb_hid.h. For
     * composite devices containing other device classes, make sure
     * MULTI_CLASS_DEVICE is defined in usb_config.h and call all
     * appropriate device class setup request functions here.
     */
    return process_hid_setup_request(setup);
}

int16_t app_unknown_get_descriptor_callback(const struct setup_packet *pkt, const void **descriptor) {
    return -1;
}

void app_start_of_frame_callback(void) {

}

void app_usb_reset_callback(void) {

}

/* HID Callbacks. See usb_hid.h for documentation. */

static uint8_t report_buf[3];

static int8_t get_report_callback(bool transfer_ok, void *context) {
    /* Nothing to do here really. It either succeeded or failed. If it
     * failed, the host will ask for it again. It's nice to be on the
     * device side in USB. */
    return 0;
}

int16_t app_get_report_callback(uint8_t interface, uint8_t report_type,
        uint8_t report_id, const void **report,
        usb_ep0_data_stage_callback *callback,
        void **context) {
    /* This isn't a composite device, so there's no need to check the
     * interface here. Also, we know that there's only one report for
     * this device, so there's no need to check report_type or report_id.
     *
     * Set report, callback, and context; and the USB stack will send
     * the report, calling our callback (get_report_callback()) when
     * it has finished.
     */
    *report = report_buf;
    *callback = get_report_callback;
    *context = NULL;
    return sizeof (report_buf);
}

int8_t app_set_report_callback(uint8_t interface, uint8_t report_type,
        uint8_t report_id) {
    /* To handle Set_Report, call usb_start_receive_ep0_data_stage()
     * here. See the documentation for HID_SET_REPORT_CALLBACK() in
     * usb_hid.h. For this device though, there are no output or
     * feature reports. */
    return -1;
}

uint8_t app_get_idle_callback(uint8_t interface, uint8_t report_id) {
    return 0;
}

int8_t app_set_idle_callback(uint8_t interface, uint8_t report_id,
        uint8_t idle_rate) {
    return -1;
}

int8_t app_get_protocol_callback(uint8_t interface) {
    return 1;
}

int8_t app_set_protocol_callback(uint8_t interface, uint8_t report_id) {
    return -1;
}


#ifdef _PIC14E

void interrupt isr() {    

    usb_service();
    
//    if(TMR1IF){
//        TMR1IF = 0;
    if(TMR2IF) {
        TMR2IF = 0;

        uint16_t cnt = 500;
        while(PORTAbits.RA5) {  // Wait for Alert trigger from INA228
            cnt--;
            if(cnt==0) {
                LED0_ON();
                break;
            }
        }
        
        check_diag_alert();
        latest_curr = get_current();
        latest_volt = get_voltage();
        if(transfer_data_cnt<TRANSFER_DATA_FULL) {
            transfer_data_buf[8*transfer_data_cnt] = 0x00;
            transfer_data_buf[8*transfer_data_cnt+4] = 0x00;
            memcpy(&transfer_data_buf[8*transfer_data_cnt+1], &latest_volt, 3);
            memcpy(&transfer_data_buf[8*transfer_data_cnt+5], &latest_curr, 3);
            transfer_data_cnt++;
        }

        // Check RID again because user might change shunt resistor.
        set_parameters_by_rid();         
    }

    /* Do NOT use these interrupts
     if(IOCIF){
        if(IOCAF) {
            if(IOCAF3) {  // Detect RA3 negative edge interrupt
            }
            if(IOCAF5) {  // Detect RA5 negative edge interrupt
            }
            IOCAF = 0; // Clear interrupt flag
        }
        IOCIF = 0; // Clear interrupt flag
    }
    */
    
}
#elif _PIC18

#ifdef __XC8

void interrupt high_priority isr() {
    usb_service();
}
#elif _PICC18
#error need to make ISR
#endif

#endif

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "py/runtime.h"
#include "py/obj.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "py/runtime.h"
#include "py/runtime0.h"
#include "py/stream.h"
#include "py/mphal.h"
#include "py/objstr.h"
#include "modmachine.h"

#include "extmod/machine_mem.h"
#include "extmod/machine_pinbase.h"
#include "extmod/machine_pulse.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/rmt.h"
#include "driver/periph_ctrl.h"
#include "soc/rmt_reg.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"

#if MICROPY_PLAT_DEV_MEM

#include <errno.h>

#include <fcntl.h>

#include <sys/mman.h>

#define MICROPY_PAGE_SIZE 4096

#define MICROPY_PAGE_MASK (MICROPY_PAGE_SIZE - 1)

#endif



static const char* NEC_TAG = "NEC";

#define RMT_RX_SELF_TEST   1
#if RMT_RX_SELF_TEST
#define RMT_RX_ACTIVE_LEVEL  1   /*!< Data bit is active high for self test mode */
#define RMT_TX_CARRIER_EN    0   /*!< Disable carrier for self test mode  */
#else
#define RMT_RX_ACTIVE_LEVEL  0   /*!< If we connect with a IR receiver, the data is active low */
#define RMT_TX_CARRIER_EN    1   /*!< Enable carrier for IR transmitter test with IR led */
#endif

#define RMT_TX_CHANNEL    1     /*!< RMT channel for transmitter */
#define RMT_TX_GPIO_NUM  16     /*!< GPIO number for transmitter signal */
#define RMT_RX_CHANNEL    0     /*!< RMT channel for receiver */
#define RMT_RX_GPIO_NUM  19     /*!< GPIO number for receiver */
#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */

#define NEC_HEADER_HIGH_US    9000                         /*!< NEC protocol header: positive 9ms */
#define NEC_HEADER_LOW_US     4500                         /*!< NEC protocol header: negative 4.5ms*/
#define NEC_BIT_ONE_HIGH_US    560                         /*!< NEC protocol data bit 1: positive 0.56ms */
#define NEC_BIT_ONE_LOW_US    (2250-NEC_BIT_ONE_HIGH_US)   /*!< NEC protocol data bit 1: negative 1.69ms */
#define NEC_BIT_ZERO_HIGH_US   560                         /*!< NEC protocol data bit 0: positive 0.56ms */
#define NEC_BIT_ZERO_LOW_US   (1120-NEC_BIT_ZERO_HIGH_US)  /*!< NEC protocol data bit 0: negative 0.56ms */
#define NEC_BIT_END            560                         /*!< NEC protocol end: positive 0.56ms */
#define NEC_BIT_MARGIN         20                          /*!< NEC parse margin time */

#define NEC_ITEM_DURATION(d)  ((d & 0x7fff)*10/RMT_TICK_10_US)  /*!< Parse duration time from memory register value */
#define NEC_DATA_ITEM_NUM   34  /*!< NEC code item number: header + 32bit data + end */
#define RMT_TX_DATA_NUM  2    /*!< NEC tx test data number */
#define rmt_item32_tIMEOUT_US  9500   /*!< RMT receiver timeout value(us) */

#define RESTOREDURATION(duration) (duration * 100 / 80)
#define FIRSTDURATION duration0
#define NEXTDURATION  duration1
#define DATA0MAX 660
#define DATA0MIN 460
#define DATA1MAX 1790
#define DATA1MIN 1590

typedef struct _mnecir_obj_t {
    mp_obj_base_t base;
    gpio_num_t gpio_id;
	mp_obj_t call_back;
} mnecir_obj_t;

mnecir_obj_t mnecir_obj[] = {
    {{&machine_necir_type}, GPIO_NUM_0,NULL},
    {{&machine_necir_type}, GPIO_NUM_1,NULL},
    {{&machine_necir_type}, GPIO_NUM_2,NULL},
    {{&machine_necir_type}, GPIO_NUM_3,NULL},
    {{&machine_necir_type}, GPIO_NUM_4,NULL},
    {{&machine_necir_type}, GPIO_NUM_5,NULL},
    {{&machine_necir_type}, GPIO_NUM_6,NULL},
    {{&machine_necir_type}, GPIO_NUM_7,NULL},
    {{&machine_necir_type}, GPIO_NUM_8,NULL},
    {{&machine_necir_type}, GPIO_NUM_9,NULL},
    {{&machine_necir_type}, GPIO_NUM_10,NULL},
    {{&machine_necir_type}, GPIO_NUM_11,NULL},
    {{&machine_necir_type}, GPIO_NUM_12,NULL},
    {{&machine_necir_type}, GPIO_NUM_13,NULL},
    {{&machine_necir_type}, GPIO_NUM_14,NULL},
    {{&machine_necir_type}, GPIO_NUM_15,NULL},
    {{&machine_necir_type}, GPIO_NUM_16,NULL},
    {{&machine_necir_type}, GPIO_NUM_17,NULL},
    {{&machine_necir_type}, GPIO_NUM_18,NULL},
    {{&machine_necir_type}, GPIO_NUM_19,NULL},
    {{NULL}, -1,NULL},
    {{&machine_necir_type}, GPIO_NUM_21,NULL},
    {{&machine_necir_type}, GPIO_NUM_22,NULL},
    {{&machine_necir_type}, GPIO_NUM_23,NULL},
    {{NULL}, -1,NULL},
    {{&machine_necir_type}, GPIO_NUM_25,NULL},
    {{&machine_necir_type}, GPIO_NUM_26,NULL},
    {{&machine_necir_type}, GPIO_NUM_27,NULL},
    {{NULL}, -1,NULL},
    {{NULL}, -1,NULL},
    {{NULL}, -1,NULL},
    {{NULL}, -1,NULL},
    {{&machine_necir_type}, GPIO_NUM_32,NULL},
    {{&machine_necir_type}, GPIO_NUM_33,NULL},
    {{&machine_necir_type}, GPIO_NUM_34,NULL},
    {{&machine_necir_type}, GPIO_NUM_35,NULL},
    {{&machine_necir_type}, GPIO_NUM_36,NULL},
    {{&machine_necir_type}, GPIO_NUM_37,NULL},
    {{&machine_necir_type}, GPIO_NUM_38,NULL},
    {{&machine_necir_type}, GPIO_NUM_39,NULL},
};
static int my_nec_parseItems(rmt_item32_t* item, int item_num, uint16_t* addr, uint16_t* data) {

	rmt_item32_t* tempItem = item;
	uint16_t tempAddr = 0, tempData = 0;
	int num = item_num;
	if(num < NEC_DATA_ITEM_NUM) {
        return -1;
    }
    if(RESTOREDURATION(item->duration0) < 8800 || RESTOREDURATION(item->duration0) > 9200 || \
       RESTOREDURATION(item->duration1) < 4300 || RESTOREDURATION(item->duration1) > 4700 ) {
    	return -1;
    }
    item ++;
	int i = 0;
	tempItem ++;
	for(i = 0;i < 16;i ++) {
		if(RESTOREDURATION(item->FIRSTDURATION) < DATA0MIN || RESTOREDURATION(item->FIRSTDURATION) > DATA0MAX) {
			return -2;
		}
		if(RESTOREDURATION(item->NEXTDURATION) > DATA0MIN && RESTOREDURATION(item->NEXTDURATION) < DATA0MAX) {
			tempAddr |= 0x00 << i;
		} else if(RESTOREDURATION(item->NEXTDURATION) > DATA1MIN && RESTOREDURATION(item->NEXTDURATION) < DATA1MAX) {
			tempAddr |= 0x01 << i;
		} else {
			return -3;
		}
		item ++;
	}
	for(i = 0;i < 16;i ++) {
		if(RESTOREDURATION(item->FIRSTDURATION) < DATA0MIN || RESTOREDURATION(item->FIRSTDURATION) > DATA0MAX) {
			return -4;
		}
		if(RESTOREDURATION(item->NEXTDURATION) > DATA0MIN && RESTOREDURATION(item->NEXTDURATION) < DATA0MAX) {
			tempData |= 0x00 << i;
		} else if(RESTOREDURATION(item->NEXTDURATION) > DATA1MIN && RESTOREDURATION(item->NEXTDURATION) < DATA1MAX) {
			tempData |= 0x01 << i;
		} else {
			return -5;
		}
		item ++;
	}
    *addr = tempAddr;
    *data = tempData;
    i = 32;
    return i;
}

static void nec_rx_init()
{
    rmt_config_t rmt_rx;
    rmt_rx.channel = (rmt_channel_t)RMT_RX_CHANNEL;
    rmt_rx.gpio_num = (gpio_num_t)RMT_RX_GPIO_NUM;
    rmt_rx.clk_div = RMT_CLK_DIV;
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);
}
STATIC mp_obj_t mnecir_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw,
        const mp_obj_t *args) {
	gpio_num_t pin_id = machine_pin_get_id(args[0]);
	const mnecir_obj_t *self = NULL;
    for (int i = 0; i < MP_ARRAY_SIZE(mnecir_obj); i++) {
        if (pin_id == mnecir_obj[i].gpio_id) { self = &mnecir_obj[i]; break; }
    }
	if (!self) nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "invalid Pin for NecIR"));
    rmt_config_t rmt_rx;
    rmt_rx.channel = (rmt_channel_t)RMT_RX_CHANNEL;
    rmt_rx.gpio_num = pin_id;
    rmt_rx.clk_div = RMT_CLK_DIV;
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);
return MP_OBJ_FROM_PTR(self);
}


mp_obj_t mycb;
mnecir_obj_t *myself = NULL;
mp_obj_t addr;
mp_obj_t cmd;
STATIC mp_obj_t mod_NecIR_test(mp_obj_t self_in)
{
  mp_call_function_2(mycb,addr,cmd);
return (mp_obj_get_type(mycb));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_NecIR_test_obj, mod_NecIR_test);

static void rmt_example_nec_rx_task()
{
	
	uint32_t ret=0;
	int channel = RMT_RX_CHANNEL;
    RingbufHandle_t rb = NULL;
    rmt_get_ringbuf_handler((rmt_channel_t)channel, &rb);
    rmt_rx_start((rmt_channel_t)channel, 1);
    while(rb) {
        size_t rx_size = 0;
        rmt_item32_t* item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 1000);
        if(item) {
            uint16_t rmt_addr;
            uint16_t rmt_cmd;
            int offset = 0;
            while(1) {
                int res = my_nec_parseItems(item + offset, rx_size / 4 - offset, &rmt_addr, &rmt_cmd);
                if(res > 0) {
                    offset += res + 1;
                    //ESP_LOGI(NEC_TAG, "RMT RCV --- addr: 0x%04x cmd: 0x%04x", rmt_addr, rmt_cmd);
					ret = (rmt_addr << 16) + rmt_cmd;
					addr = mp_obj_new_int((int)rmt_addr);
					cmd = mp_obj_new_int((int)rmt_cmd);
					mp_sched_schedule(MP_OBJ_FROM_PTR(&mod_NecIR_test_obj), MP_OBJ_NULL);
                }else{
					break;
				}
            }
            vRingbufferReturnItem(rb, (void*) item);
        }
    }
}



STATIC mp_obj_t mnecir_read(mp_obj_t self_in,mp_obj_t cb) {

	mnecir_obj_t *self = self_in;
	mycb = cb;
	myself=self;
	xTaskCreate(rmt_example_nec_rx_task, "rmt_nec_rx_task", 2048, NULL, 10, NULL);
return(mp_obj_get_type(cb));
}

MP_DEFINE_CONST_FUN_OBJ_2(mnecir_read_obj, mnecir_read);

STATIC const mp_rom_map_elem_t mnecir_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&mnecir_read_obj) },
};
STATIC MP_DEFINE_CONST_DICT(mnecir_locals_dict, mnecir_locals_dict_table);

const mp_obj_type_t machine_necir_type = {
    { &mp_type_type },
    .name = MP_QSTR_NecIR,
    .make_new = mnecir_make_new,
    .locals_dict = (mp_obj_t)&mnecir_locals_dict,
};

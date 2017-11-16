#include <string.h>

#include "esp_system.h"

#include "py/mpconfig.h"
#include "py/obj.h"
#include "py/objstr.h"
#include "py/runtime.h"
#include "py/mperrno.h"
#include "extmod/vfs.h"
#include "extmod/vfs_fat.h"
#include "genhdr/mpversion.h"

#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"

//#include "sdmmc_types.h"
//#include "lib/oofatfs/ff.h"
//#include "lib/oofatfs/diskio.h"
typedef struct _mytest{
	mp_obj_base_t base;
	sdmmc_card_t* sd;
}mytest;
const mp_obj_type_t mytest_obj;
static sdmmc_card_t* s_card=NULL;
extern void mp_hal_stdout_tx_str(const char * str);

STATIC mp_obj_t SD_sdcard_readblocks(mp_obj_t self_in,mp_obj_t block_num,mp_obj_t buf){
	uint32_t blocknum=mp_obj_get_int(block_num);	
	mytest* card=MP_OBJ_TO_PTR(self_in);
	size_t block_size = card->sd->csd.sector_size;
	mp_buffer_info_t bufinfo;
	mp_get_buffer(buf,&bufinfo,MP_BUFFER_RW);
	int nblocks = bufinfo.len/512;

	sdmmc_read_sectors(card->sd,bufinfo.buf,blocknum,nblocks);
	return MP_OBJ_FROM_PTR(card);
	/*
	sdmmc_command_t cmd={
		.flags = SCF_CMD_ADTC | SCF_CMD_READ | SCF_RSP_R1,
		.blklen = block_size,
		.data = bufinfo.buf,
		//.datalen = bufinfo.len
		.datalen = nblocks*block_size
	};

	if(nblocks==1){
		cmd.opcode=MMC_READ_BLOCK_SINGLE;
	}else{
		cmd.opcode=MMC_READ_BLOCK_MULTIPLE;
	}

	if (card->sd->ocr & SD_OCR_SDHC_CAP){
		cmd.arg = blocknum;
	}else{
		cmd.arg = (blocknum)*(card->sd->csd.sector_size);
	}

	char msg2[600];
	sprintf(msg2,"sending cmd slot=%d op=%d arg=%x flags=%x data=%p blklen=%d datelen=%d\r\n",card->sd->host.slot,cmd.opcode,cmd.arg,cmd.flags,cmd.data,cmd.blklen,cmd.datalen);
	mp_hal_stdout_tx_str(msg2);
	esp_err_t sendcmderr = card->sd->host.do_transaction(card->sd->host.slot,&cmd);
	
	if(sendcmderr==0){
		//mp_hal_stdout_tx_str("send cmd ok\r\n");
		;
	}else{
		mp_hal_stdout_tx_str("send cmd err\r\n");
		return mp_const_none;
	}

	uint32_t status=0;
	size_t count=0;
	while(!(status & MMC_R1_READY_FOR_DATA)){
		sdmmc_command_t cmd2 = {
			.opcode = MMC_SEND_STATUS,
			.arg = MMC_ARG_RCA(card->sd->rca),
			.flags = SCF_CMD_AC | SCF_RSP_R1
		};
		sendcmderr = card->sd->host.do_transaction(card->sd->host.slot,&cmd2);
		if (sendcmderr !=0){
			break;
		}
		if(++count % 10==0){
			//mp_hal_stdout_tx_str("wait for card to become ready\r\n");
			break;
		}
	}

	//return mp_obj_new_bytes(bufinfo.buf,bufinfo.len);
	return MP_OBJ_FROM_PTR(card);
	*/
}

STATIC MP_DEFINE_CONST_FUN_OBJ_3(SD_sdcard_readblocks_obj, SD_sdcard_readblocks);


STATIC mp_obj_t SD_sdcard_writeblocks(mp_obj_t self_in, mp_obj_t block_num, mp_obj_t buf){

	mytest* card=MP_OBJ_TO_PTR(self_in);

	mp_buffer_info_t bufinfo;
	mp_get_buffer(buf,&bufinfo,MP_BUFFER_RW);
	uint32_t nblocks = bufinfo.len/512;
	size_t block_size = card->sd->csd.sector_size;
	uint32_t blocknum=mp_obj_get_int(block_num);


	sdmmc_write_sectors(card->sd,bufinfo.buf,blocknum,nblocks);
	return MP_OBJ_FROM_PTR(card);


	/*
	sdmmc_command_t cmd={
		.flags = SCF_CMD_ADTC | SCF_RSP_R1,
		.blklen = block_size,
		.data = bufinfo.buf,
		//.datalen = bufinfo.len
		.datalen = nblocks*block_size
	};

	if (nblocks==1){
		cmd.opcode = MMC_WRITE_BLOCK_SINGLE;
	}else{
		cmd.opcode = MMC_WRITE_BLOCK_MULTIPLE;
	}

	if(card->sd->ocr & SD_OCR_SDHC_CAP){
		cmd.arg = blocknum;
	}else{
		cmd.arg = blocknum * block_size;
	}

	esp_err_t sendcmderr = card->sd->host.do_transaction(card->sd->host.slot, &cmd);
	if (sendcmderr == 0){
		//mp_hal_stdout_tx_str("send cmd ok...\r\n");
		;
	}else{
		mp_hal_stdout_tx_str("send cmd err...\r\n");
	}

	char msg2[600];
	sprintf(msg2,"sending cmd slot=%d op=%d arg=%x flags=%x data=%p blklen=%d datelen=%d\r\n",card->sd->host.slot,cmd.opcode,cmd.arg,cmd.flags,cmd.data,cmd.blklen,cmd.datalen);
	mp_hal_stdout_tx_str(msg2);
	


	uint32_t status=0;
	size_t count=0;
	while(!(status & MMC_R1_READY_FOR_DATA)){
		sdmmc_command_t cmd2 = {
			.opcode = MMC_SEND_STATUS,
			.arg = MMC_ARG_RCA(card->sd->rca),
			.flags = SCF_CMD_AC | SCF_RSP_R1
		};
		sendcmderr = card->sd->host.do_transaction(card->sd->host.slot, &cmd2);
		if (sendcmderr != 0){
			mp_hal_stdout_tx_str("write err\r\n");
			break;
		}
		int state = MMC_R1_CURRENT_STATE(cmd.response);
		status = MMC_R1(cmd.response);

		if(++count % 10==0){
			mp_hal_stdout_tx_str("wait for card to become ready\r\n");
			break;
		}
	}
	

	return MP_OBJ_FROM_PTR(card);
	*/
}

STATIC MP_DEFINE_CONST_FUN_OBJ_3(SD_sdcard_writeblocks_obj, SD_sdcard_writeblocks);



/*
STATIC mp_obj_t SD_sdcard_ioctl(mp_obj_t self_in){
	mp_hal_stdout_tx_str("sd ioctl\r\n");
	return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_1(SD_sdcard_ioctl_obj, SD_sdcard_ioctl);

STATIC mp_obj_t SD_sdcard_sync(){
	mp_hal_stdout_tx_str("sd sync\r\n");
	return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_0(SD_sdcard_sync_obj, SD_sdcard_sync);
*/
STATIC mp_obj_t SD_sdcard_count( mp_obj_t self_in){
	mp_hal_stdout_tx_str("sd count\r\n");
	//mp_arg_check_num(n_args,n_kw,1,1,false);
	mytest* card=MP_OBJ_TO_PTR(self_in);
	char msg[20];
	sprintf(msg,"count=%d\r\n",card->sd->csd.capacity);
	mp_hal_stdout_tx_str(msg);
	return MP_OBJ_FROM_PTR(card->sd->csd.capacity);
}

STATIC MP_DEFINE_CONST_FUN_OBJ_1(SD_sdcard_count_obj, SD_sdcard_count);

extern void ff_diskio_register_sdmmc(BYTE pdrv,sdmmc_card_t* card);
extern esp_err_t ff_diskio_get_drive(BYTE* out_pdrv);
extern esp_err_t esp_vfs_fat_register(const char* base_path,const char* fat_drive,size_t max_files,FATFS** out_fs);
STATIC mp_obj_t sdcard_make_new(const mp_obj_type_t *type,size_t n_args,size_t n_kw,const mp_obj_t *args){
	//mp_hal_stdout_tx_str("sd makenew\r\n");
	//mp_arg_check_num(n_args,n_kw,1,1,false);

	mytest* self_in1 = m_new_obj(mytest);

	//mp_hal_stdout_tx_str("Initializing SD card\r\n");
	sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
	sdmmc_host_t host=SDMMC_HOST_DEFAULT();
	sdmmc_host_deinit();
	esp_err_t init_err =  sdmmc_host_init();
	if (init_err==0){
		//mp_hal_stdout_tx_str("sd init ok.....\r\n");
	}else{
		printf("sd init err....=%d\r\n",init_err);
		return mp_const_none;
	}
	esp_err_t slot_err =  sdmmc_host_init_slot((&host)->slot,&slot_config);
	if (slot_err==0){
		//mp_hal_stdout_tx_str("slot ok\r\n");
		;
	}else{
		mp_hal_stdout_tx_str("slot err\r\n");
		return mp_const_none;
	}
	s_card = m_new_obj(sdmmc_card_t);
	
	esp_err_t initsd_err =  sdmmc_card_init(&host,s_card);
	if (initsd_err==0){
		//mp_hal_stdout_tx_str("initsd ok \r\n");
		;
	}else{
		mp_hal_stdout_tx_str("initsd err\r\n");
		return mp_const_none;
	}
	
	//disk_initialize(s_card);
	//ff_diskio_register_sdmmc(0xFF, s_card);



	//mp_hal_stdout_tx_str(s_card->cid.name);
	//mp_hal_stdout_tx_str((s_card->ocr & SD_OCR_SDHC_CAP)?"SDHC/SDXC\r\n":"SDSC\r\n");
	//mp_hal_stdout_tx_str((s_card->csd.tr_speed>25000000)?"high speed\r\n":"default speed\r\n");
	//char msg[5];
	//sprintf(msg,"%d\r\n",s_card->csd.sector_size);
	//mp_hal_stdout_tx_str(msg);

	self_in1->base.type = type;
	self_in1->sd=s_card;
    //sdmmc_host_deinit();
    //free(s_card);
	return MP_OBJ_FROM_PTR(self_in1);
}

STATIC const mp_rom_map_elem_t SD_sdcard_locals_dict_table[] = {
	{ MP_ROM_QSTR(MP_QSTR_readblocks), MP_ROM_PTR(&SD_sdcard_readblocks_obj) },
	{ MP_ROM_QSTR(MP_QSTR_writeblocks),MP_ROM_PTR(&SD_sdcard_writeblocks_obj) },
	{ MP_ROM_QSTR(MP_QSTR_count),      MP_ROM_PTR(&SD_sdcard_count_obj) },
};
STATIC MP_DEFINE_CONST_DICT(SD_sdcard_locals_dict,SD_sdcard_locals_dict_table);

const mp_obj_type_t SD_sdcard_type = {
	{ &mp_type_type },
	.name = MP_QSTR_sdcard,
	.make_new = sdcard_make_new,
	.locals_dict = (mp_obj_t)&SD_sdcard_locals_dict,
};


STATIC const mp_rom_map_elem_t SD_module_globals_table[] = {
	{ MP_ROM_QSTR(MP_QSTR_sdcard),MP_ROM_PTR(&SD_sdcard_type) },
};
STATIC MP_DEFINE_CONST_DICT(SD_module_globals,SD_module_globals_table);

const mp_obj_module_t mp_module_SD = {
	.base = { &mp_type_module },
	.globals = (mp_obj_dict_t *)&SD_module_globals,
};

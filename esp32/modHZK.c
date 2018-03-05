#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "py/runtime.h"
#include "py/obj.h"
#include "modmachine.h"
#include "mphalport.h"

extern void mp_hal_stdout_tx_str(const char *str);
extern void mp_machine_soft_spi_transfer(mp_obj_base_t *self_in, size_t len, const uint8_t *src, uint8_t *dest);

#define ASC0808D2HZ_ADDR ( 0x66c0  )     //7*8 ascii code
#define ASC0812M2ZF_ADDR ( 0x66d40 )      //6*12 ascii code
//#define GBEX0816ZF_ADDR   243648          //8*16 ascii code
#define GBEX0816ZF_ADDR  ( 0x27BFAA )
#define ZF1112B2ZF_ADDR  ( 0x3cf80 )      //12*12 12点字符 
#define HZ1112B2HZ_ADDR  ( 0x3cf80+376*24 ) //12*12 12点汉字 
#define CUTS1516ZF_ADDR  0x00         //16*16 16点字符 
//#define JFLS1516HZ_ADDR  27072        //16*16 16点汉字 
#define JFLS1516HZ_ADDR  ( 0x21E72C )        //16*16 16点汉字 

//---------------------------------------------
// 保留 
//--------------------------------------------
#define ASCII0507ZF_ADDR        245696
#define ARIAL_16B0_ADDR         246464
#define ARIAL_12B0_ADDR         422720
#define SPAC1616_ADDR           425264
#define GB2311ToUnicode_addr    (12032)
//#define UnicodeToGB2312_addr    (425328)
#define UnicodeToGB2312_addr    ( 0x267B06 )

//------------------------------------------------
// 要显示的汉字的类型 
//------------------------------------------------
#define   TYPE_8    0 //8  点字符 
#define   TYPE_12   1 //12 点汉字字符 
#define   TYPE_16   2 //16 点汉字字符 

#define UNICODE 0
#define GBK     1
#define ADDRESS 2
#define ADDR_CCP         (0x22242c)


typedef struct _machine_pin_obj_t {
    mp_obj_base_t base;
    int id;
} machine_pin_obj_t;

typedef struct _mp_machine_soft_spi_obj_t {
    mp_obj_base_t base;
    uint32_t delay_half; // microsecond delay for half SCK period
    uint8_t polarity;
    uint8_t phase;
    mp_hal_pin_obj_t sck;
    mp_hal_pin_obj_t mosi;
    mp_hal_pin_obj_t miso;
} mp_machine_soft_spi_obj_t;

mp_machine_soft_spi_obj_t  *hzk_spi=NULL;
machine_pin_obj_t  *hzk_pin=NULL;

uint8_t wbuf[37]={0x03};
uint8_t rbuf[37]={0x00};
uint8_t use_HZK=0;
uint16_t UnicodeToGB2312(uint16_t uni)
{
  uint32_t addr;
  uint8_t result=0;
  uint32_t h;
  uint32_t code=uni;
  uint16_t gb2312;

  if(code<0xa0) result=1;
  else if(code<=0xf7) h=code-160-1281;//-1441   
  else if(code<0x2c7) result=1;
  else if(code<=0x2c9) h=code-160-1281-463; //-1904
  else if(code<0x2010) result=1;
  else if(code<=0x2642) h=code-160-463-7494-333-948;//-9398
  else if(code<0x3000) result=1;
  else if(code<=0x3017) h=code-160-463-7494-333-2493-948;//-11891
  else if(code<0x3220) result=1;
  else if(code<=0x3229) h=code-160-463-7494-333-2493-520-428;//-11571
  else if(code<0x4e00) result=1;
  else if(code<=0x9fbf) h=code-160-463-7494-333-2493-520-7126;//-18589
  else if(code == 0xe76c) h=code-160-463-7494-333-2493-520-7126-295-316-18379-22699;//-60278
  else if(code<=0xe774) h=code-160-463-7494-333-2493-520-7126-295-316-18379;//-37579
  else if(code<0xff00) result=1;
  else if(code<=0xff5f) h=code-160-463-7494-333-2493-520-7126-295-316-18379-6027  +666;//-42940
  else if(code<0xffe0) result=1;
  else if(code<0xffe5) h=code-160-463-7494-333-2493-520-7126-295-316-18379-6027-128+666;//-43068
  else if(code==0xffe5) h=code-160-463-7494-333-2493-520-7126-295-316-18379-6027-128+812631;//+768897
  else result=1;
  if(result==0){
	
    addr = UnicodeToGB2312_addr  + (h<<1);
    wbuf[1] = addr>>16; wbuf[2] = addr>>8; wbuf[3] = addr;

    gpio_set_level(hzk_pin->id, 0);
    mp_machine_soft_spi_transfer(&hzk_spi->base, 6, wbuf, rbuf);    
    gpio_set_level(hzk_pin->id, 1);
    gb2312 = (rbuf[4]<<8)+rbuf[5];
	{
/*
      char buf[30];
      sprintf(buf,"gb2312=%x\r\n",gb2312);
      mp_hal_stdout_tx_str(buf);
*/
      //fflush(stdout);

    }
    return gb2312;
  }else{
    return 0xa1a1;
  }
  return 0;
}

uint32_t GB2312Addr(uint16_t gb2312,unsigned char type)
{
  uint32_t temp = 0;
  unsigned char ss[2];
  ss[0] = gb2312>>8;
  ss[1] = gb2312;

  if((ss[0] & 0x80) != 0x80) {
    if( ss[0] >= ' ' )
    temp = ss[0] - ' ';
    if( type == TYPE_8 )        temp = temp*8  + ASC0808D2HZ_ADDR;      //7*8 ascii code
    else if( type == TYPE_12 )  temp = temp*12 + ASC0812M2ZF_ADDR;      //6*12 ascii code
    else if( type == TYPE_16 )  temp = temp*16 + GBEX0816ZF_ADDR;     //8*16 ascii code
  } else {
    if(ss[0] >=0xA4 && ss[0] <= 0Xa8 && ss[1] >=0xA1)
			temp = JFLS1516HZ_ADDR;
    else if(ss[0] >=0xA1 && ss[0] <= 0Xa9 && ss[1] >=0xA1)
			temp =( (ss[0] - 0xA1) * 94 + (ss[1] - 0xA1))*32+ ADDR_CCP;
    else if(ss[0] >=0xB0 && ss[0] <= 0xF7 && ss[1] >=0xA1)
			temp = ((ss[0] - 0xB0) * 94 + (ss[1] - 0xA1)+ 846)*32+ JFLS1516HZ_ADDR;
  }
  return temp;
}
STATIC mp_obj_t mod_HZK_init(mp_obj_t _a, mp_obj_t _b) {
    mp_machine_soft_spi_obj_t *s = (mp_machine_soft_spi_obj_t*)_a;
    machine_pin_obj_t *p = (machine_pin_obj_t*)_b;
	
    if(s->base.type == &mp_machine_soft_spi_type){
       hzk_spi = s;
    }else{
       mp_hal_stdout_tx_str("is not SPI\r\n");
       //fflush(stdout);
    }

    if(p->base.type == &machine_pin_type){
       hzk_pin=p;
    }else{
       mp_hal_stdout_tx_str("is not Pin\r\n");
	   //fflush(stdout);
    }
    if(hzk_spi && hzk_pin){
        use_HZK = 1;
    }
    return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_2(mod_HZK_init_obj, mod_HZK_init);


void ASCII_search(uint8_t ascii, void *zimo)
{
    uint32_t addr;
    if ((ascii < 32) || (ascii > 127)){
        return;
    }
    //addr=(ascii-32)*16+0x3b7c0;
	addr=(ascii-32)*16+0x27BFAA;
    wbuf[1]=addr>>16; wbuf[2]=addr>>8; wbuf[3]=addr;
    gpio_set_level(hzk_pin->id, 0);
    mp_machine_soft_spi_transfer(&hzk_spi->base, 20, wbuf, rbuf);
    gpio_set_level(hzk_pin->id, 1);
    memcpy(zimo,rbuf+4,20);
    {
/*
      const char *p=zimo;
      char buf[36];
      sprintf(buf,"zimo=%x %x %x %x %x %x\r\n",p[0],p[1],p[2],p[3],p[4],p[5]);
      mp_hal_stdout_tx_str(buf);
	 */
    }  
}

void HZK_search(uint8_t type, uint32_t value, void * zimo)
{

    uint16_t gb2312=0;
    uint32_t addr=0;

    if(hzk_spi==NULL || hzk_pin==NULL){
       mp_hal_stdout_tx_str("plase call init function to initial SPI and Pin for HZK\r\n");
       return;
    }
    if(type == UNICODE){
        gb2312 = UnicodeToGB2312(value);
        addr = GB2312Addr(gb2312,TYPE_16);
    }else if(type == GBK){
        gb2312 = value;
        addr = GB2312Addr(gb2312,TYPE_16);
    }else if(type == ADDRESS){
        addr = value;
    }else{
	}
    wbuf[1]=addr>>16; wbuf[2]=addr>>8; wbuf[3]=addr;
    gpio_set_level(hzk_pin->id, 0);
    mp_machine_soft_spi_transfer(&hzk_spi->base, 36, wbuf, rbuf);
    gpio_set_level(hzk_pin->id, 1);
    memcpy(zimo,rbuf+4,32);
    {
/*
      char buf[36];
      sprintf(buf,"gb2312=%x addr=%x\r\n",gb2312,addr);
      mp_hal_stdout_tx_str(buf);
	  fflush(stdout);
*/
    }  

}

void printZimo(void *buf_,int len)
{

  uint8_t *buf = (uint8_t *)buf_;
  int i,j,k;
  char line[20]={0};

  if(len == 32){
    for(i = 0; i < 16; i++){
      memset(line,0,sizeof(line));

      for(j=0;j<2;j++){
        uint8_t b = buf[i*2+j];
        for(k=0;k<8;k++,b<<=1){
          if(b&0x80)
            strcat(line,"X");
          else
            strcat(line," ");
        }
      }

      strcat(line,"\r\n");
      mp_hal_stdout_tx_str(line);
      fflush(stdout);
    }
  }
}

STATIC mp_obj_t mod_HZK_searchUnicode(mp_obj_t uni_) {

    mp_int_t uni = mp_obj_get_int(uni_);
    char buf[32];
    HZK_search(UNICODE,uni, buf);
    //printZimo(buf,32);
    return mp_obj_new_bytearray(32,buf);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_HZK_searchUnicode_obj, mod_HZK_searchUnicode);

STATIC mp_obj_t mod_HZK_searchGBK(mp_obj_t gbk_) {
    //mp_int_t gbk = mp_obj_get_int(gbk_);
	int gbk = mp_obj_get_int(gbk_);
    char buf[32];
    HZK_search(GBK,(uint16_t)gbk, buf);
    //printZimo(buf,32);
    return mp_obj_new_bytearray(32,buf);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_HZK_searchGBK_obj, mod_HZK_searchGBK);

STATIC mp_obj_t mod_HZK_searchAddr(mp_obj_t addr_) {

    //mp_int_t addr = mp_obj_get_int(addr_);
	int addr = mp_obj_get_int(addr_);
    char buf[32];
    HZK_search(ADDRESS,(uint16_t)addr, buf);
    //printZimo(buf,32);
    return mp_obj_new_bytearray(32,buf);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_HZK_searchAddr_obj, mod_HZK_searchAddr);

STATIC mp_obj_t mod_HZK_Unicode2GBK(mp_obj_t uni_) {
    mp_int_t uni = mp_obj_get_int(uni_);
    uint16_t gbk = UnicodeToGB2312(uni);
    return mp_obj_new_int(gbk);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_HZK_Unicode2GBK_obj, mod_HZK_Unicode2GBK);

STATIC const mp_rom_map_elem_t HZK_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_HZK) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mod_HZK_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_Unicode2GBK), MP_ROM_PTR(&mod_HZK_Unicode2GBK_obj) },
    { MP_ROM_QSTR(MP_QSTR_searchUnicode), MP_ROM_PTR(&mod_HZK_searchUnicode_obj) },
    { MP_ROM_QSTR(MP_QSTR_searchGBK), MP_ROM_PTR(&mod_HZK_searchGBK_obj) },
    { MP_ROM_QSTR(MP_QSTR_searchAddr), MP_ROM_PTR(&mod_HZK_searchAddr_obj) },
};

STATIC MP_DEFINE_CONST_DICT(HZK_module_globals, HZK_module_globals_table);

const mp_obj_module_t mp_module_HZK = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&HZK_module_globals,
};


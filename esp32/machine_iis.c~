#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "py/runtime.h"
#include "py/obj.h"
#include "py/stream.h"
#include "py/objstr.h"
#include "modmachine.h"
#include "mphalport.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "rom/lldesc.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "sys/socket.h"

#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "iistest/camera.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include <fcntl.h>
#include "extmod/vfs.h"
#include "extmod/vfs_fat.h"
#include "unix/fdfile.h"
#include "py/test_thread.h"
#include "math.h"

#define I2S_MCLK                    22
#define WRITE_BIT                   I2C_MASTER_WRITE
#define READ_BIT                    I2C_MASTER_READ
#define ESP_SLAVE_ADDR              (0X34) 
#define ACK_CHECK_EN                0x1
#define ACK_CHECK_DIS               0x0
#define I2C_MASTER_NUM              I2C_NUM_1
#define I2C_MASTER_SCL_IO           GPIO_NUM_27
#define I2C_MASTER_SDA_IO           GPIO_NUM_26
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_FREQ_HZ          600

#define PLAY       1
#define BEGIN      1
#define PAUSE      2
#define STOP       3
#define SET        4

#define QQVGA     4
#define QQVGA2    5
#define QICF      6
#define HQVGA     7
#define QVGA      8
#define VGA       10
#define RGB555    0
#define GRAYSCALE 2

#define RECORDER  1
#define PLAYER    2
#define CAMERA    3

static xTaskHandle  xPlayWAV = NULL;
static xTaskHandle  xRecordSound = NULL;
static xTaskHandle  xPlayer = NULL;

char     filename[80];
char     pictureFilename[80];
char     outputFilename[80];
char     nextfile[80];
uint8_t  mark=STOP;
uint8_t  rmark=STOP;
uint8_t  Volume1=0;
uint8_t  Volume2=0;
uint8_t  loop=0;
uint8_t  Size=8;
uint8_t  Pixel=0;
uint8_t  set=0;
uint8_t  rChannels=0;
int      rFramerate=0;
uint8_t  rSampwidth=0;
uint8_t  time2=0;

float    time1=0.00;

mp_obj_t mycb;

typedef struct _miis_obj_t {
    mp_obj_base_t base;
	int mode;
} miis_obj_t;

typedef struct WAV_HEADER
{
    char                  riffType[4];
    unsigned int          riffSize;
    char                  waveType[4];
    char                  formatType[4];
    unsigned int          formatSize;
    uint16_t              compressionCode;
    i2s_channel_t         numChannels;
    uint32_t              sampleRate;
    unsigned int          bytesPerSecond;
    unsigned short        blockAlign;
    i2s_bits_per_sample_t bitsPerSample;
    char                  dataType1[1];
    char                  dataType2[3];
    unsigned int          dataSize;
    char                  test[800];
}WAV_HEADER;

struct WAV
{
    WAV_HEADER header;
    FILE *fp;
};

typedef struct WAV *HANDLE_WAV;

static void I2C_WriteNAU8822(int8_t addr, int16_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte( cmd ,(ESP_SLAVE_ADDR) | WRITE_BIT           ,ACK_CHECK_EN);
    i2c_master_write_byte( cmd ,((addr<<1) |(data>>8))| WRITE_BIT      ,ACK_CHECK_EN);
    i2c_master_write_byte( cmd ,((int8_t) (data & 0x00ff) )| WRITE_BIT ,ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100);
    i2c_cmd_link_delete(cmd);
}
static void I2C_Master_Init()
{

    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf={
		.mode             = I2C_MODE_MASTER,
    	.sda_io_num       = I2C_MASTER_SDA_IO,
    	.sda_pullup_en    = GPIO_PULLUP_ENABLE,
    	.scl_io_num       = I2C_MASTER_SCL_IO,
    	.scl_pullup_en    = GPIO_PULLUP_ENABLE,
    	.master.clk_speed = I2C_MASTER_FREQ_HZ ,
	};
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,I2C_MASTER_RX_BUF_DISABLE,I2C_MASTER_TX_BUF_DISABLE, 0);
}



void I2S_MCLK_Init(unsigned int SAMPLE_RATE)
{
    periph_module_enable(PERIPH_LEDC_MODULE);
    ledc_timer_bit_t bit_num = (ledc_timer_bit_t) 2;
    int duty                 = pow(2, (int) bit_num) / 2;
    ledc_timer_config_t timer_conf;
    timer_conf.bit_num       = bit_num;
    timer_conf.freq_hz       = SAMPLE_RATE*256; 
    timer_conf.speed_mode    = LEDC_HIGH_SPEED_MODE;
    timer_conf.timer_num     = LEDC_TIMER_0;
    ledc_timer_config(&timer_conf);
    ledc_channel_config_t ch_conf;
    ch_conf.channel          = LEDC_CHANNEL_1;
    ch_conf.timer_sel        = LEDC_TIMER_0;
    ch_conf.intr_type        = LEDC_INTR_DISABLE; 
    ch_conf.duty             = duty;
    ch_conf.speed_mode       = LEDC_HIGH_SPEED_MODE;
    ch_conf.gpio_num         = I2S_MCLK ;  
    ledc_channel_config(&ch_conf);
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1); 

}
void I2S_Master_Init(uint32_t SAMPLE_RATE,i2s_bits_per_sample_t BITS_PER_SAMPLE)
{
    i2s_config_t i2s_config  = {
        .mode                = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate         = SAMPLE_RATE,
        .bits_per_sample     = BITS_PER_SAMPLE,
        .channel_format      = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format= I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags    = ESP_INTR_FLAG_LEVEL2
    };
    i2s_config.dma_buf_count = 5;
    i2s_config.dma_buf_len   = 100;
    i2s_pin_config_t pin_config = {
        .bck_io_num   = 5,
        .ws_io_num    = 17,
        .data_out_num = 0,
        .data_in_num  = 39
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
}

void I2S_Slave_Init(uint32_t SAMPLE_RATE,i2s_bits_per_sample_t BITS_PER_SAMPLE)
{
    i2s_config_t i2s_config  = {
        .mode                = I2S_MODE_SLAVE | I2S_MODE_RX,
        .sample_rate         = SAMPLE_RATE, 
        .bits_per_sample     = BITS_PER_SAMPLE,
        .channel_format      = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format= I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags    = ESP_INTR_FLAG_LEVEL2
    };
//ESP_INTR_FLAG_LEVEL2 || ESP_INTR_FLAG_SHARED
    i2s_config.dma_buf_count = 5;
    i2s_config.dma_buf_len   = 100;
    i2s_pin_config_t pin_config = {
        .bck_io_num   = 5,
        .ws_io_num    = 17,
        .data_out_num = 0,
        .data_in_num  = 39
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
}




//char dir_name[20];
static char * get_dir_name(char *name){
	int i = 0;
	uint8_t len = strlen(name);
    char *dir_name = (char *)malloc(60);
	while(len){
		if(name[len] != '/'){
			//dir_name[i] = name[len];
		}else{
			break;
		}
		len--;
		i++;
	}
	//memset(dir_name,0,60);
	strncpy(dir_name,name,len);
	return dir_name;
}
  
static int mk_dir(char *dir)  
{  
    DIR *mydir = NULL;  
    if((mydir= opendir(dir))==NULL)//判断目录   
    {  
      int ret = mkdir(dir, fileMODE);//创建目录  
      if (ret != 0)  
      {  
          return -1;  
      }  
      //printf("%s created sucess!/n", dir);  
    }  
    else  
    {  
        //printf("%s exist!/n", dir);  
    }  
  
    return 0;  
}  
  
static int mk_all_dir(char *dir)  
{  
    bool flag = true;  
    char *pDir = dir;  
    while (flag)  
    {  
        char *pIndex = index(pDir, '/');  
        if (pIndex != NULL && pIndex != dir)  
        {  
            char buffer[512] = {0};  
            int msg_size = pIndex - dir;  
            memcpy(buffer, dir, msg_size);  
            int ret = mk_dir(buffer);  
            if (ret < 0)  
            {  
                //printf("%s created failed!/n", dir);  
            }  
        }  
        else if (pIndex == NULL && pDir == dir)  
        {  
            //printf("dir is not directory!/n");  
            return -1;  
        }  
        else if (pIndex == NULL && pDir != dir)  
        {  
            int ret = mk_dir(dir);  
            if (ret < 0)  
            {  
                //printf("%s created failed!/n", dir);  
            }  
  
            break;  
        }  
  
        pDir = pIndex+1;  
  
    }  
  
    return 0;  
}  






char nowname[80];
STATIC mp_obj_t mod_iis_callback(mp_obj_t self_in)
{
	mp_call_function_1(mycb,mp_obj_new_str((const char *)nowname,strlen(nowname),false));
	return (mp_obj_get_type(mycb));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mod_iis_callback_obj, mod_iis_callback);

void I2C_Setup_NAU8822_play()
{
    I2C_Master_Init();
    I2C_WriteNAU8822(0,  0x000);
    vTaskDelay(10);
    I2C_WriteNAU8822(1,  0x1FF);
    I2C_WriteNAU8822(2,  0x1BF);
    I2C_WriteNAU8822(3,  0x1FF);
    I2C_WriteNAU8822(4,  0x010);
    I2C_WriteNAU8822(5,  0x000);
    I2C_WriteNAU8822(6,  0x00C);
    I2C_WriteNAU8822(7,  0x000);
    I2C_WriteNAU8822(10, 0x008);
    I2C_WriteNAU8822(14, 0x108);
    I2C_WriteNAU8822(15, 0x0FF);
    I2C_WriteNAU8822(16, 0x1FF);
    I2C_WriteNAU8822(45, 0x0bf);
    I2C_WriteNAU8822(46, 0x1bf);
    I2C_WriteNAU8822(47, 0x175);
    I2C_WriteNAU8822(48, 0x175);
    I2C_WriteNAU8822(50, 0x001);
    I2C_WriteNAU8822(51, 0x001);
    if(Volume2 != 0){
        I2C_WriteNAU8822(52, Volume2);
        I2C_WriteNAU8822(53, Volume2+256);
    }else{
        I2C_WriteNAU8822(52, 0x040);
        I2C_WriteNAU8822(53, 0x040);
    }
    if(Volume1 != 0){
        I2C_WriteNAU8822(54, Volume1);
        I2C_WriteNAU8822(55, Volume1+256);
    }else{
        I2C_WriteNAU8822(54, 0x040);
        I2C_WriteNAU8822(55, 0x040);
    }
    i2c_driver_delete(I2C_MASTER_NUM);
}

void I2C_Setup_NAU8822_record()
{
    I2C_Master_Init();
    I2C_WriteNAU8822(0,  0x000);
    vTaskDelay(10);
    I2C_WriteNAU8822(1,  0x1FF);
    I2C_WriteNAU8822(2,  0x1BF);
    I2C_WriteNAU8822(3,  0x1FF);
    I2C_WriteNAU8822(4,  0x010);
    I2C_WriteNAU8822(5,  0x000);
    I2C_WriteNAU8822(6,  0x00D);
    I2C_WriteNAU8822(7,  0x007);
	I2C_WriteNAU8822(9 ,0x150);
    I2C_WriteNAU8822(10, 0x008);
    I2C_WriteNAU8822(14, 0x108);
    I2C_WriteNAU8822(15, 0x0FF);
    I2C_WriteNAU8822(16, 0x1FF);
//I2C_WriteNAU8822(32, 0x1b8);
	I2C_WriteNAU8822(44, 0x033);
    I2C_WriteNAU8822(45, 0x0bf);
    I2C_WriteNAU8822(46, 0x1bf);
    I2C_WriteNAU8822(47, 0x075);
    I2C_WriteNAU8822(48, 0x075);
    I2C_WriteNAU8822(50, 0x001);
    I2C_WriteNAU8822(51, 0x001);
    I2C_WriteNAU8822(52, 0x040);
    I2C_WriteNAU8822(53, 0x040);
    I2C_WriteNAU8822(54, 0x040);
    I2C_WriteNAU8822(55, 0x040);
    I2C_WriteNAU8822(74, 0x100);
    i2c_driver_delete(I2C_MASTER_NUM);
} 
bool thebitsPerSample = false; 
void playWAV(void *arg)
{
    while(1){
        while(mark==STOP){
            vTaskDelay(100);
        }
        I2C_Setup_NAU8822_play();
        HANDLE_WAV wav = (HANDLE_WAV)calloc(1, sizeof(struct WAV));
        if(wav == NULL){
			printf("playWAV(): Unable to allocate WAV struct.\n");
			break;
        }
        vTaskDelay(100);
        wav->fp = fopen(filename, "rb");
		strcpy(nowname,filename);
        if(wav->fp == NULL){
			printf("playWAV(): Unable to open wav file %s\n",filename);
			break;	
        }
        if(fread(&(wav->header.riffType), 1, 4, wav->fp) != 4){
			printf("playWAV(): couldn't read RIFF_ID\n");
			break;
        }
        if(strncmp("RIFF", wav->header.riffType, 4)){
			printf("playWAV(): RIFF descriptor not found.\n");
			break;
        }
        fread(&(wav->header.riffSize), 4, 1, wav->fp);
        if(fread(&wav->header.waveType, 1, 4, wav->fp) !=4){
			printf("playWAV(): couldn't read format.\n");
			break;
        }
        if(strncmp("WAVE", wav->header.waveType, 4)){
			printf("playWAV(): WAVE chunk ID not found.\n");
			break;
        }
        if(fread(&(wav->header.formatType), 1, 4, wav->fp) != 4){
			printf("playWAV(): couldn't read format_ID.\n");
			break;
        }
        if(strncmp("fmt", wav->header.formatType, 3)){
			printf("playWAV(): fmt chunk format not found.\n");
			break;
        }
        fread(&(wav->header.formatSize), 4, 1, wav->fp);
        fread(&(wav->header.compressionCode), 2, 1, wav->fp);
        fread(&(wav->header.numChannels), 2, 1, wav->fp);
        fread(&(wav->header.sampleRate), 4, 1, wav->fp);
        fread(&(wav->header.bytesPerSecond), 4, 1, wav->fp);
        fread(&(wav->header.blockAlign), 2, 1, wav->fp);
        fread(&(wav->header.bitsPerSample), 2, 1, wav->fp);
        while(1){
            if(fread(&wav->header.dataType1, 1, 1, wav->fp) != 1){
                printf("playWAV(): Unable to read data chunk ID.\n");
                free(wav);
                break;
            }
            if(strncmp("d", wav->header.dataType1, 1) == 0){
                fread(&wav->header.dataType2, 3, 1, wav->fp);
                if(strncmp("ata", wav->header.dataType2, 3) == 0){
                    fread(&(wav->header.dataSize),4,1,wav->fp);
                    break;
                }
            }
        }
		if(wav->header.bitsPerSample != 16 && wav->header.bitsPerSample != 24 && wav->header.bitsPerSample != 32 ){
			printf("%s sampling bit is %d. To ensure sound quality, open the file with 16 or 24 or 32-bit sampling bits\n",filename,wav->header.bitsPerSample);
			if(wav->header.bitsPerSample == 8){
				break;
			}
			thebitsPerSample =true;
			wav->header.bitsPerSample = 16;
		}else{
			thebitsPerSample = false;
		}
        I2S_MCLK_Init(wav->header.sampleRate);
        I2S_Master_Init(wav->header.sampleRate ,wav->header.bitsPerSample);
        i2s_set_sample_rates(I2S_NUM_0, wav->header.sampleRate);
        int i=floor(time1);
        float j;
        j = time1-i;
        j *= 100;
        i = i*60+j;
        fseek(wav->fp,wav->header.sampleRate * wav->header.bitsPerSample * wav->header.numChannels * i/8,1);
        while(fread(&wav->header.test, 1 , 400 , wav->fp)){
			int bytes_left=400,bytes_written = 0;
            char *buf=(char *)&wav->header.test;
			if(thebitsPerSample){
				char buf1[1600];
				int datalen = 400;
				int buf1len=0;
				while(datalen){
					buf1[buf1len]    = (buf[400-datalen] & 0xf0);
					buf1[buf1len +1] = buf1[buf1len] ;
					buf1[buf1len +2] = (buf[400-datalen] & 0x0f ) << 4;
					buf1[buf1len +3] = buf1[buf1len +2];
					datalen--;
					buf1len += 4;
				}
				buf = buf1;
				bytes_left=1600;
			}
            while(bytes_left > 0){
                bytes_written = i2s_write_bytes(I2S_NUM_0 , buf , bytes_left , 0);
                bytes_left   -= bytes_written;
                buf          += bytes_written;
                if(mark==PAUSE){
                    I2C_Master_Init();
                    I2C_WriteNAU8822(52, 0x040);
                    I2C_WriteNAU8822(53, 0x040);
                    I2C_WriteNAU8822(54, 0x040);
                    I2C_WriteNAU8822(55, 0x040);
                    i2c_driver_delete(I2C_MASTER_NUM);
                    while(mark==PAUSE){
                        vTaskDelay(10);
                    }
                    vTaskDelay(10);
                    I2C_Master_Init();
                    I2C_WriteNAU8822(52, Volume2);
                    I2C_WriteNAU8822(53, Volume2+256);
                    I2C_WriteNAU8822(54, Volume1);
                    I2C_WriteNAU8822(55, Volume1+256);
                    i2c_driver_delete(I2C_MASTER_NUM);
                }
                while(mark==SET){
                    vTaskDelay(10);
                }
            }
            if(mark==STOP){
                I2C_Master_Init();
                I2C_WriteNAU8822(52, 0x040);
                I2C_WriteNAU8822(53, 0x040);
                I2C_WriteNAU8822(54, 0x040);
                I2C_WriteNAU8822(55, 0x040);
                i2c_driver_delete(I2C_MASTER_NUM);
                break;
            }
        }
        i2s_stop(I2S_NUM_0);
        int err =i2s_driver_uninstall(I2S_NUM_0);
        fclose(wav->fp);
        free(wav);
		mp_sched_schedule(MP_OBJ_FROM_PTR(&mod_iis_callback_obj), MP_OBJ_NULL);
        if(loop){
             loop--;  
        }else{
             mark=STOP;
             if(nextfile[0] != 0){
				memset(filename,0,30);
				strcpy(filename,nextfile);
				memset(nextfile,0,30);
                time1=0;
                loop=0;
                mark=BEGIN; 
             }else{
				mark=STOP;
			 }
       } 
        vTaskDelay(100);
    }
	xPlayWAV = NULL;
	vTaskDelete(xPlayWAV);
}

void recordSound(void *arg)
{
    while(1){
        while(rmark==STOP){
            vTaskDelay(100);
        }
        I2C_Setup_NAU8822_record();
        HANDLE_WAV wav = (HANDLE_WAV)calloc(1, sizeof(struct WAV));
        if(wav == NULL){
			printf("recordSound(): Unable to allocate WAV struct.\n");
			break;
        }

		struct stat sb;
		char *dirname=get_dir_name(outputFilename);
		if(stat(dirname, &sb)){
			if(mk_all_dir(dirname) != 0){
				printf("recordSound(): unable to create file %s\n",outputFilename);
				break;
			}
		}
		free(dirname);
        wav->fp = fopen(outputFilename, "wb");
		//strcpy(outputFilename,filename);
		strcpy(nowname,outputFilename);
        if(wav->fp == NULL){
			printf("recordSound(): unable to create file %s\n",outputFilename);
			break;
        }
        strcpy(wav->header.riffType, "RIFF");
        wav->header.riffSize = 0;
        strcpy(wav->header.waveType, "WAVE");
        strcpy(wav->header.formatType, "fmt ");
        wav->header.formatSize      = 0x00000010;
        fwrite(&wav->header.riffType, 1, 20 , wav->fp);
        wav->header.compressionCode = 1;
        fwrite(&wav->header.compressionCode, 1 , 2 , wav->fp);
        if(rChannels == 1){
            wav->header.numChannels     = I2S_CHANNEL_MONO;
        }else{
            wav->header.numChannels     = I2S_CHANNEL_STEREO;
        }
        fwrite(&wav->header.numChannels, 1, 2, wav->fp);
        if(rFramerate){
            wav->header.sampleRate      = rFramerate;
        }else{
            wav->header.sampleRate      = 32000;
            rFramerate = 32000;
        }
        if(rSampwidth){
            wav->header.bitsPerSample   = rSampwidth;
        }else{
            wav->header.bitsPerSample   = I2S_BITS_PER_SAMPLE_16BIT;
        }
        fwrite(&wav->header.sampleRate, 1, 4 , wav->fp);
        wav->header.blockAlign      = (short)(wav->header.numChannels *(wav->header.bitsPerSample >> 3));
        wav->header.bytesPerSecond  = (rFramerate)*(wav->header.blockAlign);
        fwrite(&wav->header.bytesPerSecond, 1, 4 , wav->fp);
        fwrite(&wav->header.blockAlign, 1, 2 , wav->fp);
        fwrite(&wav->header.bitsPerSample, 1, 2 , wav->fp);
        strcpy(wav->header.dataType1, "d");
        strcpy(wav->header.dataType2, "ata");
        wav->header.dataSize        = 0;
        fwrite(&wav->header.dataType1 ,1,8, wav->fp);
        int bytes_written = 0;
        char *buf=(char *)&wav->header.test;
        I2S_MCLK_Init(rFramerate);
        I2S_Slave_Init(rFramerate,wav->header.bitsPerSample);
        int Sec = wav->header.bitsPerSample * rFramerate * time2 / 8;
        vTaskDelay(100);
/*
		while(rmark!=STOP){
			bytes_written = i2s_read_bytes(I2S_NUM_0 ,buf, 800 , 100);
            (wav->header.dataSize) += fwrite(buf, 1, bytes_written , wav->fp);
		}
*/
        while(rmark!=STOP && (Sec || !time2)){
            bytes_written = i2s_read_bytes(I2S_NUM_0 ,buf, 800 , 100);
            wav->header.dataSize+=fwrite(buf, 1, bytes_written , wav->fp);
            Sec -= bytes_written;
        }
		rmark = STOP;
		mp_sched_schedule(MP_OBJ_FROM_PTR(&mod_iis_callback_obj), MP_OBJ_NULL);
        time2 = 0;
        i2s_stop(I2S_NUM_0);
        int err = i2s_driver_uninstall(I2S_NUM_0);
        wav->header.riffSize =wav->header.dataSize+44;
        fseek(wav->fp,4,0);
        fwrite(&wav->header.riffSize, 1,4, wav->fp);
        fseek(wav->fp,40,0);
        fwrite(&wav->header.dataSize, 1,4, wav->fp);
        fclose(wav->fp);
        free(wav);
        vTaskDelay(100);
    }
	xRecordSound = NULL;
	vTaskDelete(xRecordSound);
}

STATIC mp_obj_t iis_play_init(mp_obj_t self_in){
	miis_obj_t *self = self_in;
	if(self->mode == CAMERA){
		//i2s_driver_uninstall(I2S_NUM_0);
		mark=STOP;//player
		rmark = STOP;//recorder
		vTaskDelay(20);
		stophttpserver();
		if(xPlayWAV != NULL){
			i2s_driver_uninstall(I2S_NUM_0);
			vTaskDelete(xPlayWAV);
			xPlayWAV = NULL;
		}
		if(xRecordSound != NULL){
			i2s_driver_uninstall(I2S_NUM_0);
			vTaskDelete(xRecordSound);
			xRecordSound = NULL;
		}
        I2C_Master_Init();
        I2C_WriteNAU8822(0,  0x000);
        I2C_WriteNAU8822(52, 0x040);
        I2C_WriteNAU8822(53, 0x040);
        I2C_WriteNAU8822(54, 0x040);
        I2C_WriteNAU8822(55, 0x040);
		i2c_driver_delete(I2C_MASTER_NUM);
	}else if(self->mode == PLAYER){
		//i2s_driver_uninstall(I2S_NUM_0);
		mark=STOP;//player
		rmark = STOP;//recorder
		vTaskDelay(20);
		if(xRecordSound != NULL){
			vTaskDelete(xRecordSound);
			xRecordSound = NULL;
		}
		if(xPlayWAV != NULL){
			i2s_driver_uninstall(I2S_NUM_0);
			vTaskDelete(xPlayWAV);
			xPlayWAV = NULL;
		}
		stophttpserver();
	}else if(self->mode == RECORDER){
		mark=STOP;//player
		rmark = STOP;
		vTaskDelay(20);
		if(xRecordSound != NULL){
			i2s_driver_uninstall(I2S_NUM_0);
			vTaskDelete(xRecordSound);
			xRecordSound = NULL;
		}

		if(xPlayWAV != NULL){
			i2s_driver_uninstall(I2S_NUM_0);
			vTaskDelete(xPlayWAV);
			xPlayWAV = NULL;
		}

		stophttpserver();
	}
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(iis_play_init_obj, iis_play_init);

STATIC mp_obj_t iis_play_load(mp_obj_t self_in, mp_obj_t name){
	if(xPlayWAV != NULL){
	}
	esp_vfs_fat_sdmmc_unmount();
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5
    };
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sd", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
			nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Failed to mount filesystem. If you want the card to be formatted, set format_if_mount_failed = true."));
        } else {
			nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Failed to initialize the card. Make sure SD card lines have pull-up resistors in place."));
        }
        return mp_const_false;
    }
    char *buf = (char *)mp_obj_str_get_str(name);
    //char SDfilename[87]="/sd";
    //strcat(SDfilename,buf);
	//strcpy(filename ,SDfilename);
	strcpy(filename ,buf);	
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(iis_play_load_obj, iis_play_load);
/*
STATIC mp_obj_t iis_play_play_helper(size_t n_args, const mp_obj_t *args){
	if(n_args == 2){
		printf("loops=%d\n",mp_obj_get_int(args[0]));
		printf("start=%f\n",mp_obj_get_float(args[1]));
	}
   mark=BEGIN;
   time1=mp_obj_get_float(args[1]);
   loop=mp_obj_get_int(args[0]);
   xTaskCreate(playWAV, "playWAV",4096, NULL, 5, NULL);
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(iis_play_play_obj, 0, 2, iis_play_play);
*/

STATIC mp_obj_t iis_play_play_helper(miis_obj_t *self,
        size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args){
	if(n_args == 2){
		time1=mp_obj_get_float(pos_args[1]);
		loop=mp_obj_get_int(pos_args[0]);
		//printf("loops=%d\n",mp_obj_get_int(pos_args[0]));
		//printf("start=%f\n",mp_obj_get_float(pos_args[1]));
	}else if(n_args == 1){
		loop=mp_obj_get_int(pos_args[0]);
	}
	mark= BEGIN;
    rmark = STOP;
	if(self->mode == PLAYER){
		if(xPlayWAV == NULL){
			xTaskCreate(&playWAV, "playWAV",4096, NULL, 5, &xPlayWAV);
		}
	}
	return mp_const_none;
}

STATIC mp_obj_t iis_play_play(size_t n_args,
        const mp_obj_t *args, mp_map_t *kw_args) {
    iis_play_play_helper(args[0], n_args - 1, args + 1, kw_args);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(iis_play_play_obj, 1, iis_play_play);



STATIC mp_obj_t iis_play_stop(mp_obj_t self_in){
	miis_obj_t *self = self_in;
	if(self->mode == PLAYER){
		mark=STOP;
		vTaskDelay(20);
		if(xPlayWAV != NULL){
			vTaskDelete(xPlayWAV);
			xPlayWAV = NULL;
		}
	}else if(self->mode == RECORDER){
		rmark = STOP;
		vTaskDelay(20);
		if(xRecordSound != NULL){
			vTaskDelete(xRecordSound);
			xRecordSound = NULL;
		}
		
	}
	return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(iis_play_stop_obj, iis_play_stop);

STATIC mp_obj_t iis_play_pause(mp_obj_t self_in){
    mark=PAUSE;
	return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(iis_play_pause_obj, iis_play_pause);

STATIC mp_obj_t iis_play_unpause(mp_obj_t self_in){
    mark=BEGIN;
	return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(iis_play_unpause_obj, iis_play_unpause);

STATIC mp_obj_t iis_play_fadeout(mp_obj_t self_in, mp_obj_t time){
	//printf("time=%d\n",mp_obj_get_int(time));
	return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(iis_play_fadeout_obj, iis_play_fadeout);

STATIC mp_obj_t iis_play_set_volume(mp_obj_t self_in,mp_obj_t volume){
    if(mp_obj_get_int(volume)>99){
        Volume2=99;
    }
    if(mp_obj_get_int(volume)<1){
        Volume2=0;
    }
    Volume2=((mp_obj_get_int(volume))*64/100);
	Volume1 = Volume2;
	if(mark == PLAY){
		mark=SET;
	}
	if(mark != PAUSE){
    	I2C_Master_Init();
    	I2C_WriteNAU8822(52, Volume2);
    	I2C_WriteNAU8822(53, Volume2+256);
    	I2C_WriteNAU8822(54, Volume1);
    	I2C_WriteNAU8822(55, Volume1+256);
    	i2c_driver_delete(I2C_MASTER_NUM);
	}
	if(mark == SET){
		mark=PLAY;
	}
    
	return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(iis_play_set_volume_obj, iis_play_set_volume);

STATIC mp_obj_t iis_play_get_volume(mp_obj_t self_in){
	return mp_obj_new_int(Volume1);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(iis_play_get_volume_obj, iis_play_get_volume);

STATIC mp_obj_t iis_play_get_busy(mp_obj_t self_in){
	if(mark==PLAY || rmark == PLAY)
        return mp_const_true;
    else
        return mp_const_false;   
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(iis_play_get_busy_obj, iis_play_get_busy);

STATIC mp_obj_t iis_play_set_endcallback(mp_obj_t self_in,mp_obj_t cb){
	mycb = cb;
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(iis_play_set_endcallback_obj, iis_play_set_endcallback);

STATIC mp_obj_t iis_play_queue(mp_obj_t self_in,mp_obj_t name){
    char *buf = (char *)mp_obj_str_get_str(name);
    //char SDfilename[87]="/sdcard";
    //strcat(SDfilename,buf);
	//strcpy(nextfile,SDfilename);
	strcpy(nextfile,buf);	
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(iis_play_queue_obj, iis_play_queue);

STATIC mp_obj_t iis_play_set_nchannels(mp_obj_t self_in,mp_obj_t channels){
	rChannels = mp_obj_get_int(channels);
    //printf("channels=%d\n",mp_obj_get_int(channels));
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(iis_play_set_nchannels_obj, iis_play_set_nchannels);

STATIC mp_obj_t iis_play_set_sampwidth(mp_obj_t self_in,mp_obj_t sampwidth){
    rSampwidth = mp_obj_get_int(sampwidth);
	if(rSampwidth != 16 && rSampwidth != 24 && rSampwidth != 32){
		nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Sampling bits only support 16 or 24 or 32 bits."));
	}
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(iis_play_set_sampwidth_obj, iis_play_set_sampwidth);

STATIC mp_obj_t iis_play_set_framerate(mp_obj_t self_in, mp_obj_t framerate){
    rFramerate = mp_obj_get_int(framerate);
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(iis_play_set_framerate_obj, iis_play_set_framerate);
/*
STATIC mp_obj_t iis_play_set_record(size_t n_args, const mp_obj_t *args){    
    //(mp_obj_t Filename,mp_obj_t time)
    char SDfilename[30]="/sdcard";
    strcat(SDfilename,Filename);
    strcpy(outputFilename,SDfilename);
    time2=time;
    rmark=BEGIN;
    mark =STOP;
    xTaskCreate(recordSound, "recordSound",2048, NULL, 5, NULL);
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(iis_play_set_record_obj, 1, 2,  iis_play_set_record);
*/

STATIC mp_obj_t iis_play_set_record_helper(miis_obj_t *self,
        size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args){    
    //(mp_obj_t Filename,mp_obj_t time)
    //char SDfilename[87]="/sdcard";
	char *Filename = (char *)mp_obj_str_get_str(pos_args[0]);
	if(n_args >= 2){
		time2 = mp_obj_get_int(pos_args[1]);
	}
    //strcat(SDfilename,Filename);
    //strcpy(outputFilename,SDfilename);
	strcpy(outputFilename,Filename);
    rmark=BEGIN;
    mark =STOP;

	esp_vfs_fat_sdmmc_unmount();
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5
    };
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sd", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
			nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Failed to mount filesystem. If you want the card to be formatted, set format_if_mount_failed = true."));
        } else {
			nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Failed to initialize the card. Make sure SD card lines have pull-up resistors in place."));
        }
        return mp_const_false;
    }
	if(self->mode == RECORDER){
		if(xRecordSound == NULL){
			xTaskCreate(&recordSound, "recordSound",4096, NULL, 5, &xRecordSound);
		}
	}
	return mp_const_none;
}

STATIC mp_obj_t iis_play_set_record(size_t n_args,
        const mp_obj_t *args, mp_map_t *kw_args) {
    iis_play_set_record_helper(args[0], n_args - 1, args + 1, kw_args);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_KW(iis_play_set_record_obj, 1, iis_play_set_record);

STATIC mp_obj_t iis_play_set_stop(mp_obj_t self_in){
    rmark = STOP;
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(iis_play_set_stop_obj, iis_play_set_stop);



STATIC mp_obj_t miis_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw,
        const mp_obj_t *args) {
	miis_obj_t* self = m_new_obj(miis_obj_t);
	self->base.type = type;
	self->mode = mp_obj_get_int(args[0]);
	return MP_OBJ_FROM_PTR(self);
}

/*photo*/
/*
STATIC mp_obj_t iis_connectNet(mp_obj_t self_in, mp_obj_t ssid, mp_obj_t password){
	connectnet(mp_obj_str_get_str(ssid),mp_obj_str_get_str(password));	
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(iis_connectNet_obj, iis_connectNet);
*/
STATIC mp_obj_t iis_setFreamsize(mp_obj_t self_in, mp_obj_t Freamsize){
	Size = mp_obj_get_int(Freamsize);
	if(set){
		cameramode(Size,Pixel);
		set = 0;
	}else{
		set = 1;
	}
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(iis_setFreamsize_obj, iis_setFreamsize);

STATIC mp_obj_t iis_setPixformat(mp_obj_t self_in, mp_obj_t Pixformat){
	Pixel = mp_obj_get_int(Pixformat);
	if(set){
		cameramode(Size,Pixel);
		set = 0;
	}else{
		set = 1;
	}
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(iis_setPixformat_obj, iis_setPixformat);

STATIC mp_obj_t iis_sendPhoto(mp_obj_t self_in){
	sendtonet();
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(iis_sendPhoto_obj, iis_sendPhoto);

STATIC mp_obj_t iis_httpServerStop(mp_obj_t self_in){
	stophttpserver();
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(iis_httpServerStop_obj, iis_httpServerStop);

STATIC mp_obj_t iis_snapshot(mp_obj_t self_in,mp_obj_t name){
	esp_vfs_fat_sdmmc_unmount();
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5
    };
    sdmmc_card_t* card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sd", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
			nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Failed to mount filesystem. If you want the card to be formatted, set format_if_mount_failed = true."));
        } else {
			nlr_raise(mp_obj_new_exception_msg(&mp_type_ValueError, "Failed to initialize the card. Make sure SD card lines have pull-up resistors in place."));
        }
        return mp_const_false;
    }
	//char SDfilename[87]="/sdcard";
	char *Filename = (char *)mp_obj_str_get_str(name);
    //strcat(SDfilename,Filename);
    //strcpy(pictureFilename,SDfilename);
	strcpy(pictureFilename,Filename);
    camera_run(pictureFilename);
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(iis_snapshot_obj, iis_snapshot);

STATIC const mp_rom_map_elem_t miis_locals_dict_table[] = {
	{ MP_ROM_QSTR(MP_QSTR_init), 			MP_ROM_PTR(&iis_play_init_obj) },
	{ MP_ROM_QSTR(MP_QSTR_load), 			MP_ROM_PTR(&iis_play_load_obj) },
	{ MP_ROM_QSTR(MP_QSTR_play),      		MP_ROM_PTR(&iis_play_play_obj) },
	{ MP_ROM_QSTR(MP_QSTR_stop),      		MP_ROM_PTR(&iis_play_stop_obj) },
	{ MP_ROM_QSTR(MP_QSTR_pause),      		MP_ROM_PTR(&iis_play_pause_obj) },
	{ MP_ROM_QSTR(MP_QSTR_unpause),      	MP_ROM_PTR(&iis_play_unpause_obj) },
	{ MP_ROM_QSTR(MP_QSTR_fadeout),      	MP_ROM_PTR(&iis_play_fadeout_obj) },
	{ MP_ROM_QSTR(MP_QSTR_set_volume),      MP_ROM_PTR(&iis_play_set_volume_obj) },
	{ MP_ROM_QSTR(MP_QSTR_get_volume),      MP_ROM_PTR(&iis_play_get_volume_obj) },
	{ MP_ROM_QSTR(MP_QSTR_get_busy),      	MP_ROM_PTR(&iis_play_get_busy_obj) },
	{ MP_ROM_QSTR(MP_QSTR_set_endcallback), MP_ROM_PTR(&iis_play_set_endcallback_obj) },
	{ MP_ROM_QSTR(MP_QSTR_queue),     		MP_ROM_PTR(&iis_play_queue_obj) },
	{ MP_ROM_QSTR(MP_QSTR_set_nchannels),   MP_ROM_PTR(&iis_play_set_nchannels_obj) },
	{ MP_ROM_QSTR(MP_QSTR_set_sampwidth),   MP_ROM_PTR(&iis_play_set_sampwidth_obj) },
	{ MP_ROM_QSTR(MP_QSTR_set_framerate),   MP_ROM_PTR(&iis_play_set_framerate_obj) },
	{ MP_ROM_QSTR(MP_QSTR_record),      	MP_ROM_PTR(&iis_play_set_record_obj) },
	{ MP_ROM_QSTR(MP_QSTR_set_stop),      	MP_ROM_PTR(&iis_play_set_stop_obj) },

	//{ MP_ROM_QSTR(MP_QSTR_connectNet),    MP_ROM_PTR(&iis_connectNet_obj) },
	{ MP_ROM_QSTR(MP_QSTR_setFramesize),    MP_ROM_PTR(&iis_setFreamsize_obj) },
	{ MP_ROM_QSTR(MP_QSTR_setPixformat),    MP_ROM_PTR(&iis_setPixformat_obj) },
	{ MP_ROM_QSTR(MP_QSTR_httpServerStart), MP_ROM_PTR(&iis_sendPhoto_obj) },
	{ MP_ROM_QSTR(MP_QSTR_httpServerStop),  MP_ROM_PTR(&iis_httpServerStop_obj) },
	{ MP_ROM_QSTR(MP_QSTR_snapshot),      	MP_ROM_PTR(&iis_snapshot_obj) },

	{ MP_OBJ_NEW_QSTR(MP_QSTR_QQVGA), 		MP_OBJ_NEW_SMALL_INT(QQVGA) },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_QICF), 		MP_OBJ_NEW_SMALL_INT(QICF) },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_HQVGA), 		MP_OBJ_NEW_SMALL_INT(HQVGA) },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_QVGA), 		MP_OBJ_NEW_SMALL_INT(QVGA) },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_RGB555), 		MP_OBJ_NEW_SMALL_INT(RGB555) },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_GRAYSCALE), 	MP_OBJ_NEW_SMALL_INT(GRAYSCALE) },

	{ MP_OBJ_NEW_QSTR(MP_QSTR_RECORDER), 	MP_OBJ_NEW_SMALL_INT(RECORDER) },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_PLAYER), 		MP_OBJ_NEW_SMALL_INT(PLAYER) },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_CAMERA), 		MP_OBJ_NEW_SMALL_INT(CAMERA) },
};

STATIC MP_DEFINE_CONST_DICT(miis_locals_dict, miis_locals_dict_table);

const mp_obj_type_t machine_iis_type = {
    { &mp_type_type },
    .name = MP_QSTR_IIS,
    .make_new = miis_make_new,
    .locals_dict = (mp_obj_t)&miis_locals_dict,
};



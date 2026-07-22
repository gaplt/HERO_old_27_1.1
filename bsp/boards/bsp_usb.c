#include "bsp_usb.h"
#include "cJSON.h"
#include "malloc.h"
#include "cmd_ctr.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

Usb_Type_t Usb_Ture;

uint8_t usb_buf[256];

uint8_t result = USBD_OK;

uint8_t usb_Receive(uint8_t* Buf,uint32_t Len,USBD_HandleTypeDef *pdev)
{
	uint32_t i;
	result = USBD_CDC_ReceivePacket(pdev);
	for(i=0;i<Len;i++)
	{
		Usb_Ture.RInit_data[i] = * Buf++;
	}
//	Usb_Ture.RInit_data[i] = '\n';
  Usb_Ture.flag=1;
	Usb_Ture.whileflag=1;
	memcpy(cJson_buffer,&Usb_Ture.RInit_data,i);//将数据复制到cjson处理的数组里
	parse_loop((const char*)cJson_buffer);	//处理cjson	
	memset(Usb_Ture.RInit_data , 0,1000);
	memset(cJson_buffer , 0,2048);
  return result;
}

void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);

    CDC_Transmit_FS(usb_buf, len);
}

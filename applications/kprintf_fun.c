#include "kprintf_func.h"

rt_device_t usart_device;

int kprintf_set_device()
{
    usart_device=rt_device_find("uart2");

    rt_device_open(usart_device,RT_DEVICE_FLAG_STREAM);

}
INIT_APP_EXPORT(kprintf_set_device);

int send(unsigned char * buf,rt_uint16_t size)
{
    rt_device_write(usart_device,0,buf,size);
}
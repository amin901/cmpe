
#include "ExtraCredit.hpp"
#include "io.hpp"
#include "queue.h"
#include "stdio.h"
#include "uart0.hpp"
#include "sys_config.h"
#include "rtc.h"


#define BIT_0 (1<<0)
#define BIT_1 (1<<1)

xQueueHandle lightValue = 0;
float light = 0;
Uart0& u0 = Uart0::getInstance();


taskOne::taskOne(uint8_t priority) : scheduler_task("one",1024,priority)
{
    lightValue = xQueueCreate(1,sizeof(LS.getPercentValue()));
}

bool taskOne::taskOne_init(void)
{
    return 1;
}


bool taskOne::run(void *p)
{


   // u0.printf("%d\n",sizeof(LS.getPercentValue()));



    for (int i = 0;i<100;i++)
    {
        light += LS.getPercentValue();
        vTaskDelay(1);

    }

    light = light/100;
   // u0.printf("light value sent = %f\n",light);
    xQueueSend(lightValue,&light,1000);

    return 1;
}

bool taskOne::taskEntry()
{
    setRunDuration(1000);
    return 1;
}

taskTwo::taskTwo(uint8_t priority) : scheduler_task("two",1024,priority)
{
    file = 0;

}

bool taskTwo::taskTwo_init(void)
{
    return 1;
}

bool taskTwo::run(void *p)
{

//    if(fd)
//    {
//        u0.printf("file is open..\n");
//        fclose(fd);
//    }

    char data[32];
    if(xQueueReceive(lightValue,&light,portMAX_DELAY))
    {

        file = fopen("1:sensor.txt","a");
        //printf(rtc_get_date_time_str());
        sprintf(data,"%f",light);
        //u0.printf("%s",data);

        fputs(rtc_get_date_time_str(),file);
        fputs(" ",file);
        fputs(data,file);

       // Storage::write("1:sensor.txt",data,sizeof(data),0);
       // printf("%f\n", light);
        fclose(file);

    }
    return 1;
}

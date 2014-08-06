/*
 * gps_task.cpp
 *
 *  Created on: Apr 10, 2014
 *      Author: Arpit
 */


#include <string.h>
#include <stdlib.h>
#include "utilities.h"
#include "gps_task.hpp"
#include "uart2.hpp"
#include "stdio.h"
#include "uart0_min.h"



gps_task::gps_task(uint8_t priority) :
                scheduler_task("remote", 1024*2, priority), gps(0)
{

}

bool gps_task::init(void)
{
    {
       Uart2 &u2 = Uart2::getInstance();
       u2.init(9600);
       gps = &u2;
    }

    //setRunDuration(10000);
    return 1;
}

bool gps_task::run(void *p)
{

    char message[20];
    //gps->flush();
    char buff[128] = {0};
    char gpsData[] = {0};
    char *temp;
    char final_latitude[16] ={0};
    char final_longitude[16] = {0};
    double latitude = 0.0, longitude = 0.0;
    int count = 0;

    while(gps->gets(buff,sizeof(buff)))
    {

        if(strstr(buff,"$GPGLL"))
                {
                    //printf("****************FOUND****************\n");
                    strncpy(gpsData,buff,sizeof(buff));
                    //printf("\n\n%s\n\n",gpsData);
                    //printf("%i\n",strlen(gpsData));
                    temp = strtok(gpsData,",");
                    count = 0;
                    while(temp !=NULL)
                    {
                        //printf("%s\n",temp);
                        temp = strtok(NULL,",");
                        //printf("%d\n",count);
                        count++;
                        if(count == 1)
                        {
                            latitude = atof(temp);
                             //printf("%s\n",temp);
                             //printf("%f\n lat",latitude);
                            // calculate(latitude);
                        }
                        else if(count == 2)
                        {
                            if (strstr(temp,"S"))
                            {
                                latitude = latitude * -1;
                                calculate(latitude, final_latitude);
                            }
                            else
                            {
                                calculate(latitude, final_latitude);
                            }
                        }

                        else if(count == 3)
                        {
                            longitude = atof(temp);
                             //printf("%f\n lon",longitude);
                             //calculate(longitude);
                        }
                        else if (count == 4)
                        {
                            if (strstr(temp,"W"))
                                 {
                                    longitude = longitude * -1;
                                    calculate(longitude,final_longitude);
                                 }
                             else
                                 {
                                 calculate(longitude,final_longitude);
                                 }

                        }
                    }
                }
        else
        {
            continue;
        }

        // this message is to be sent via text
        strcpy(message,final_latitude);
        strcat(message,",");
        strcat(message,final_longitude);


        printf("%s\n",message);
        delay_ms(1000);




    }
    return 1;
}

bool gps_task::taskEntry(void *p)
{

    //setRunDuration(10000);
    return 1;
}


// this function calculate gps coordinates from gnss data.
// and return the value latitude and than longitude
void gps_task::calculate(double cordinates, char * TxValue)
{
//    char TxValue[16];
    double value = cordinates;
    value = value/100;
    int wholeValue = value;
    value = value - wholeValue;
    value = (value/60)*100;
    value = value + wholeValue;
   // printf("%.7f\n",value);
    sprintf(TxValue,"%f",value);
    //printf("%s\n",TxValue);
//    return TxValue;
    //return value;
}


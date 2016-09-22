/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "system/common/sys_module.h"   // SYS function prototypes


// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

int ledPin = 13;

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    //TRISA = 0x0000;
    //TRISB = 0x0000;
    //TRISC = 0x0000;
    //TRISD = 0x0000;
    
    //LATA = 0x0008;//RA3 = ld4 = pin 13 Last byte = 0000 1000 (bit 3, 4th bit, is asserted)
    //LATB = 0x0000;
    //LATC = 0x0002; //RC1 (not RC0) = ld5 
    //LATD = 0x0000;
    
            //timer example from p2659
        DRV_TMR_INIT     init;
        SYS_MODULE_OBJ   object;
        SYS_STATUS       tmrStatus;
        // populate the TMR init configuration structure
        init.moduleInit.value = SYS_MODULE_POWER_RUN_FULL;
        init.tmrId= TMR_ID_2;
        init.clockSource = TMR_CLOCK_SOURCE_PERIPHERAL_CLOCK;
        init.prescale= TMR_PRESCALE_VALUE_256;
        init.interruptSource = INT_SOURCE_TIMER_2;
        init.mode = DRV_TMR_OPERATION_MODE_16_BIT; 
        init.asyncWriteEnable = false;
        
        //object = DRV_TMR_Initialize (DRV_TMR_INDEX_0, (SYS_MODULE_INIT *)&init);
    
    
}


/*Tasks*/
QueueHandle_t xQueue;
TimerHandle_t xTimers[ 1 ];

void taskTx(void* p)
{
    int myInt = 0;
    while(1)
    {
        myInt++;
        if(!xQueueSend(xQueue, &myInt, 500)) {
            //puts("Failed to send item to queue within 500ms");
        }
        vTaskDelay(1000);
    }
}
 

void taskRx(void* p){
    
    int myInt=0;
        while(1){
            if(xQueue != 0){
                if( xQueueReceive( xQueue, &myInt, portMAX_DELAY ) )
                {
                    if(!xQueueReceive(xQueue, &myInt, 1000)) {
                     //  puts("Failed to receive item within 1000 ms");
                    }
                    else {
                     //   printf("Received: %u\n", myInt);
                    }
                }
                
            }
        }
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
            
                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            break;
        }
        /* TODO: implement your application state machine.*/
        
        TRISA = 0x0000;
        TRISB = 0x0000;
        TRISC = 0x0000;
        TRISD = 0x0000;
        
        LATA = 0x0008;//RA3 = ld4 = pin 13 Last byte = 0000 1000 (bit 3, 4th bit, is asserted)
        LATB = 0x0000;
        LATC = 0x0002; //RC1 (not RC0) = ld5 
        LATD = 0x0000;
        
        
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */

// Modified by Juan Carlos Juárez
// Calculator with Floating Point Numbers. USB Connection.

/***************************
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
 ***************************/

// DOM-IGNORE-BEGIN
/***************************
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
 ***************************/
// DOM-IGNORE-END


// ***************************
// ***************************
// Section: Included Files 
// ***************************
// ***************************

#include "app.h"


// ***************************
// ***************************
// Section: Global Data Definitions
// ***************************
// ***************************

const uint8_t __attribute__((aligned(16))) switchPrompt[] = "\r\nPUSH BUTTON PRESSED";


uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];

// ***************************
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

#define TRANS_COUNT 9
#define EDO_COUNT 15

char chr;
int acum1 = 0;
int acum2 = 0;
float res = 0;
enum Oper{Suma,Resta,Mult,Div};
enum Oper oper;

int edo=0;
int edoAnt=0;
int trans=0;

float decimal1 = 0;
float decimal2 = 0;

int times1 = 1;
int times2 = 1;

float currDecimal1 = 0;
float currDecimal2 = 0;

float final1 = 0;
float final2 = 0;

int i = 0;

int chrTrans[TRANS_COUNT]=
					{ 0,'(',')','=',  8, 27, 6 , 7, '.'};
int mtzTrans[EDO_COUNT][TRANS_COUNT]={
					{ 0, 1 , 0 , 0 , 0 , 0 , 0 , 0,  0},

					{ 1, 1 , 1 , 1 , 99, 99, 2 , 1,  1},
					{ 2, 2 , 2 , 2 , 99, 99, 3 , 2,  4},
					{ 3, 2 , 2 , 2 , 99, 99, 2 , 2,  2},

					{ 4, 4 , 4 , 4 , 99, 99, 5 , 4,  4},
					{ 5, 5 , 5 , 5 , 99, 99, 6 , 7,  5},
					{ 6, 5 , 5 , 5 , 99, 99, 5 , 5,  5},

					{ 7, 7 , 7 , 7 , 99, 99, 8 , 7,  7},
					{ 8, 8 , 8 , 8 , 99, 99, 9 , 8,  10},
					{ 9, 8 , 8 , 8 , 99, 99, 8 , 8,  8},

					{ 10, 10 , 10 , 10 , 99, 99, 11 , 10,  10},
					{ 11, 11 , 13 , 11 , 99, 99, 12 , 11,  11},
					{ 12, 11 , 11 , 11 , 99, 99, 11 , 11,  11},

					{ 13, 13 , 13 , 14 , 99, 99, 13 , 13,  13},
					{ 14, 13 , 13 , 14 , 99, 99, 13 , 13,  13}};

// Functions

int calcTrans(char chr) {
	int trans=0;
	if ((chr>='0')&&(chr<='9')){	//Digito
		return(6);
	}else if (chr == '.') {
		//printf("co");
        return(8);
    }
	switch (chr) {
		case'+':
		case'-':
		case'*':
		case'/':
				return(7);
	}
	for (trans=5;trans>0;trans--)
		if (chr==chrTrans[trans])
			break;
	return(trans);
}

int sigEdo(int edo, int trans) {
	return(mtzTrans[edo][trans]);
}

int ejecutaEdo(int edo) {
	switch(edo) {
		case 0:
				break;
		case 1:
				acum1=0;
				//printf("%c",chr);
                appData.readBuffer[0] = chr;
                appData.numBytesRead = 1;
				break;
		case 2:
				//printf("%c",chr);
                appData.readBuffer[0] = chr;
                appData.numBytesRead = 1;
				acum1*=10;
				acum1+=(chr-'0');
				//printf("**2**");
				break;
		case 3:
				//printf("%c",chr);
                appData.readBuffer[0] = chr;
                appData.numBytesRead = 1;
				acum1*=10;
				acum1+=(chr-'0');
				//printf("**3**");
				return(2);
		case 4:
				//printf(".");
                appData.readBuffer[0] = '.';
                appData.numBytesRead = 1;
				times1 = 1;
				decimal1 = 0;
				//acum2=0;	//Preparar la entrada al estado 4
				break;
		case 5:
				//printf("%c",chr);
                appData.readBuffer[0] = chr;
                appData.numBytesRead = 1;
				currDecimal1 = chr - '0';
				
				for(i = 0; i < times1; i++){
					currDecimal1 *= 0.1;
				}
				decimal1 += currDecimal1;
				times1++;
				break;
		case 6:
				//printf("%c",chr);
                appData.readBuffer[0] = chr;
                appData.numBytesRead = 1;
				currDecimal1 = chr - '0';
				
				for(i = 0; i < times1; i++){
					currDecimal1 *= 0.1;
				}
				decimal1 += currDecimal1;
				times1++;
				return(5);
		case 7:
				//printf(" ----- %f ----- %d \n",decimal1, acum1);
				//printf("correct");
				acum2=0;
				switch (chr) {
					case'+':
							oper=Suma;
							break;
					case'-':
							oper=Resta;
							break;
					case'*':
							oper=Mult;
							break;
					case'/':
							oper=Div;
							break;
				}
				//printf("%c",chr);
                appData.readBuffer[0] = chr;
                appData.numBytesRead = 1;
				break;

		case 8:
				//printf("%c",chr);
                appData.readBuffer[0] = chr;
                appData.numBytesRead = 1;
				acum2*=10;
				acum2+=(chr-'0');
				//printf("**2**");
				break;

		case 9:
				//printf("%c",chr);
                appData.readBuffer[0] = chr;
                appData.numBytesRead = 1;
				acum2*=10;
				acum2+=(chr-'0');
				//printf("**2**");
				return (8);

		case 10:
				//printf(".");
                appData.readBuffer[0] = '.';
                appData.numBytesRead = 1;
				times2 = 1;
				decimal2 = 0;
				//acum2=0;	//Preparar la entrada al estado 4
				break;

		case 11:
				//printf("%c",chr);
                appData.readBuffer[0] = chr;
                appData.numBytesRead = 1;
				currDecimal2 = chr - '0';
				
				for(i = 0; i < times2; i++){
					currDecimal2 *= 0.1;
				}
				decimal2 += currDecimal2;
				times2++;
				break;

		case 12:
				//printf("%c",chr);
                appData.readBuffer[0] = chr;
                appData.numBytesRead = 1;
				currDecimal2 = chr - '0';
				
				for(i = 0; i < times2; i++){
					currDecimal2 *= 0.1;
				}
				decimal2 += currDecimal2;
				times2++;
				return (11);

		case 13:
				//printf(" ----- %f ----- %d \n",decimal1, acum1);
				//printf(" ----- %f ----- %d \n",decimal2, acum2);
				//printf("correct");
				//printf("%c",chr);
                appData.readBuffer[0] = chr;
                appData.numBytesRead = 1;
				break;


		case 14: // Pending
				//printf("%c",chr);
				final1 = (float)acum1;
				final2 = (float)acum2;
				final1 += decimal1;
				final2 += decimal2;
				switch(oper) {
					case Suma:
							res=final1+final2;
							break;
					case Resta:
							res=final1-final2;
							break;
					case Mult:
							res=final1*final2;
							break;
					case Div:
							if (acum2)
								res=final1/final2;
							else
								res=-1;
							break;
				}
				//printf("%f\n",res);
                //Here we convert pass it to the temporal buffer
                
                // Sum is the whole part of res
                int sum = (int)res;
                
                //Intermediate variable to take away the sum (Whole part treated as float)
                float intermediateFloat = (float)sum;
                
                //The final decimalPart
                float decimalSum = res - intermediateFloat;
                
                char decimalPart[10];
                char currData[10];
                int currSize = 0;
                int maxTwo = 2;
                int currDecimalSize = 0;
                
                //First we add the '='
                appData.readBuffer[0] = '=';
                int index = 1;
                int j;
                
                //Then the Whole Part
                
                while(sum){
                    int currNum = sum % 10;
                    currData[currSize] = currNum + '0';
                    currSize++;
                    sum /= 10;
                }
                for(j = currSize -1; j >= 0; j--){
                    appData.readBuffer[index] = currData[j];
                    index++;
                }
                
                //We add the dot
                appData.readBuffer[index] = '.';
                index++;
                
                //Now the Decimal Part
                
                while(decimalSum > 0 && maxTwo){
                    decimalSum *= 10;
                    int convertedInteger = (int)decimalSum;
                    int currInteger = convertedInteger % 10;
                    decimalPart[currDecimalSize] = currInteger + '0';
                    currDecimalSize++;
                    float newIntermediate = (float)currInteger;
                    decimalSum -= newIntermediate;
                    maxTwo--;
                }
                
                for(j = 0; j < currDecimalSize; j++){
                    appData.readBuffer[index] = decimalPart[j];
                    index++;
                }
                
                appData.readBuffer[index] = ' ';
                index++;
                appData.readBuffer[index] = '|';
                index++;
                appData.readBuffer[index] = ' ';
                index++;
             
                appData.numBytesRead = index;

				final1 = 0;
				final2 = 0;
				return(0);
		case 99:
				//printf("\n<<<Captura cancelada>>>\n");
				return(0);	//Estado aceptor, rompe la rutina y marca estado de salida
	}
	return(edo);	//Para estados no aceptores regresar el estado ejecutado
}



// ***************************
// ***************************
// Section: Application Callback Functions
// ***************************
// ***************************



/*******************
 * USB CDC Device Events - Application Event Handler
 *******************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
    USB_DEVICE_CDC_INDEX index ,
    USB_DEVICE_CDC_EVENT event ,
    void * pData,
    uintptr_t userData
)
{
    APP_DATA * appDataObject;
    appDataObject = (APP_DATA *)userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;
    USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE * eventDataRead; 

    switch ( event )
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *)pData)->breakDuration;
            
            /* Complete the control transfer by sending a ZLP  */
            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            eventDataRead = (USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE *)pData;
            appDataObject->isReadComplete = true;
            appDataObject->numBytesRead = eventDataRead->length; 
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}

/*****************
 * Application USB Device Layer Event Handler.
 *****************/
void APP_USBDeviceEventHandler ( USB_DEVICE_EVENT event, void * eventData, uintptr_t context )
{
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch ( event )
    {
        case USB_DEVICE_EVENT_SOF:

            /* This event is used for switch debounce. This flag is reset
             * by the switch process routine. */
            appData.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */
            BSP_LEDOn ( APP_USB_LED_1 );
            BSP_LEDOn ( APP_USB_LED_2 );
            BSP_LEDOff ( APP_USB_LED_3 );

            appData.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuratio. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData;
            if ( configuredEventData->configurationValue == 1)
            {
                /* Update LED to show configured state */
                BSP_LEDOff ( APP_USB_LED_1 );
                BSP_LEDOff ( APP_USB_LED_2 );
                BSP_LEDOn ( APP_USB_LED_3 );

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t)&appData);

                /* Mark that the device is now configured */
                appData.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
            BSP_LEDOff ( APP_USB_LED_1 );
            BSP_LEDOn ( APP_USB_LED_2 );
            BSP_LEDOn ( APP_USB_LED_3 );
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// ***************************
// ***************************
// Section: Application Local Functions
// ***************************
// ***************************

void APP_ProcessSwitchPress()
{
    /* This function checks if the switch is pressed and then
     * debounces the switch press*/
    if(BSP_SWITCH_STATE_PRESSED == (BSP_SwitchStateGet(APP_USB_SWITCH_1)))
    {
        if(appData.ignoreSwitchPress)
        {
            /* This measn the key press is in progress */
            if(appData.sofEventHasOccurred)
            {
                /* A timer event has occurred. Update the debounce timer */
                appData.switchDebounceTimer ++;
                appData.sofEventHasOccurred = false;
                if (USB_DEVICE_ActiveSpeedGet(appData.deviceHandle) == USB_SPEED_FULL)
                {
                    appData.debounceCount = APP_USB_SWITCH_DEBOUNCE_COUNT_FS;
                }
                else if (USB_DEVICE_ActiveSpeedGet(appData.deviceHandle) == USB_SPEED_HIGH)
                {
                    appData.debounceCount = APP_USB_SWITCH_DEBOUNCE_COUNT_HS;
                }
                if(appData.switchDebounceTimer == appData.debounceCount)
                {
                    /* Indicate that we have valid switch press. The switch is
                     * pressed flag will be cleared by the application tasks
                     * routine. We should be ready for the next key press.*/
                    appData.isSwitchPressed = true;
                    appData.switchDebounceTimer = 0;
                    appData.ignoreSwitchPress = false;
                }
            }
        }
        else
        {
            /* We have a fresh key press */
            appData.ignoreSwitchPress = true;
            appData.switchDebounceTimer = 0;
        }
    }
    else
    {
        /* No key press. Reset all the indicators. */
        appData.ignoreSwitchPress = false;
        appData.switchDebounceTimer = 0;
        appData.sofEventHasOccurred = false;
    }

}

/*******************
 * This function is called in every step of the
 * application state machine.
 *******************/

bool APP_StateReset(void)
{
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if(appData.isConfigured == false)
    {
        appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData.isReadComplete = true;
        appData.isWriteComplete = true;
        retVal = true;
    }
    else
    {
        retVal = false;
    }

    return(retVal);
}

// ***************************
// ***************************
// Section: Application Initialization and State Machine Functions
// ***************************
// ***************************

/***************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    /* Device Layer Handle  */
    appData.deviceHandle = USB_DEVICE_HANDLE_INVALID ;

    /* Device configured status */
    appData.isConfigured = false;

    /* Initial get line coding state */
    appData.getLineCodingData.dwDTERate = 9600;
    appData.getLineCodingData.bParityType =  0;
    appData.getLineCodingData.bParityType = 0;
    appData.getLineCodingData.bDataBits = 8;

    /* Read Transfer Handle */
    appData.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Write Transfer Handle */
    appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

    /* Intialize the read complete flag */
    appData.isReadComplete = true;

    /*Initialize the write complete flag*/
    appData.isWriteComplete = true;

    /* Initialize Ignore switch flag */
    appData.ignoreSwitchPress = false;

    /* Reset the switch debounce counter */
    appData.switchDebounceTimer = 0;

    /* Reset other flags */
    appData.sofEventHasOccurred = false;
    appData.isSwitchPressed = false;

    /* Set up the read buffer */
    appData.readBuffer = &readBuffer[0];

       
}


/**************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks (void )
{
    /* Update the application state machine based
     * on the current state */
    int i; 
    switch(appData.state)
    {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

            if(appData.deviceHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if(appData.isConfigured)
            {
                /* If the device is configured then lets start reading */
                appData.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if(APP_StateReset())
            {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if(appData.isReadComplete == true)
            {
                appData.isReadComplete = false;
                appData.readTransferHandle =  USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read (USB_DEVICE_CDC_INDEX_0,
                        &appData.readTransferHandle, appData.readBuffer,
                        APP_READ_BUFFER_SIZE);
                
                if(appData.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID)
                {
                    appData.state = APP_STATE_ERROR;
                    break;
                }
            }

            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:
        case APP_STATE_CHECK_SWITCH_PRESSED:

            if(APP_StateReset())
            {
                break;
            }

            APP_ProcessSwitchPress();

            /* Check if a character was received or a switch was pressed.
             * The isReadComplete flag gets updated in the CDC event handler. */

            if(appData.isReadComplete || appData.isSwitchPressed)
            {
                appData.state = APP_STATE_SCHEDULE_WRITE;
            }

            break;


        case APP_STATE_SCHEDULE_WRITE:
            
            

            if(APP_StateReset())
            {
                break;
            }

            /* Setup the write */

            appData.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData.isWriteComplete = false;
            appData.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;

            if(appData.isSwitchPressed)
            {
                /* If the switch was pressed, then send the switch prompt*/
                appData.isSwitchPressed = false;
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle, switchPrompt, 23,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
            }
            else
            {
                /* Else echo each received character by adding 1 */
                for(i=0; i<appData.numBytesRead; i++)
                {
                    if((appData.readBuffer[i] != 0x0A) && (appData.readBuffer[i] != 0x0D))
                    {
                        //appData.readBuffer[i] = appData.readBuffer[i] + 1;
                        chr = appData.readBuffer[i];
                        trans = calcTrans(chr);
                        if (trans) {
                            edoAnt=edo;
                            edo=sigEdo(edoAnt,trans);
                            if (edoAnt!=edo){
                                edo=ejecutaEdo(edo);
                                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                                    &appData.writeTransferHandle,
                                    appData.readBuffer, appData.numBytesRead,
                                    USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                            }
                        }
                        
                        //*********************************************************
                    }
                }
                
                /*
                USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                        &appData.writeTransferHandle,
                        appData.readBuffer, appData.numBytesRead,
                        USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                 */ 
                 
            }

            break;

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if(APP_StateReset())
            {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if(appData.isWriteComplete == true)
            {
                appData.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}

/***************************
 End of File
 */
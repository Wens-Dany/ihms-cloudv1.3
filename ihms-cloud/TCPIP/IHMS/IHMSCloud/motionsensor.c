#include "main.h"
#include <plib.h>

//#define GetSystemClock()	(40000000ul)                // 40MHz is set by Max Choi, but the system clock should be 80MHz according to MCU specification.
#define GetInstructionClock()	(GetSystemClock()/1)
#define GetPeripheralClock()	(GetInstructionClock()/1)	// Set your divider according to your Peripheral Bus Frequency configuration fuse setting
#define I2C_CLOCK_FREQ          200000

// EEPROM Constants
#define sensor_I2C_BUS              I2C5
#define sensor_ADDRESS              0x1C       // The MMA8451Q's standard slave address is a choice between the two sequential addresses 0011100 - 0x1C and 0011101 - 0X1D. The
                                                //selection is made by the high and low logic level of the SA0 (pin 7)



BYTE sensortmpdata[7];  //for reading MMA845X Q  Data Registers from 0x00 to 0x06

BOOL StartTransfer( BOOL restart );
BOOL TransmitOneByte( UINT8 data );
void StopTransfer( void );
void init_LED(void);

void LED(char x);
void delay(int time);
BOOL i2c_write_reg(UINT reg_address, UINT data );
BOOL i2c_read_reg(int f_read);

BOOL motionsensor_init()
{
    UINT32     actualClock;
    BOOL Success = TRUE;

    actualClock = I2CSetFrequency(sensor_I2C_BUS, GetPeripheralClock(), I2C_CLOCK_FREQ);
    if ( abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10 )
        {
//            TCPPutROMString(sktHTTP, "Clock freq have more than 10% error ! ");
            return FALSE;
        }
    TRISBbits.TRISB14 = 0;
    LATBbits.LATB14 = 0;

    // Enable the I2C bus
    I2CEnable(sensor_I2C_BUS, TRUE);
    //  set Normal Power Mode for 0x00, High resolution mode for 0x02 (the last two bit in control 2 register 0x2B
    if(!i2c_write_reg(0x2B,0x00)) Success = FALSE;

    // to set dynamic range to 8G for XYZ axis
    if(!i2c_write_reg(0x0E ,0x12)) Success = FALSE;

    // set sensor to be active mode, normal read mode, 0x11 ODR is 200Hz, 0x09 ODR is 400Hz
    // 0x01 ODR is 800Hz (Maximum ODR)
    if(!i2c_write_reg(0x2A ,0x01)) Success = FALSE;

    return Success;
}

int collect_sensor_data(int f_read)
{
    int i, j, data_size;
    BOOL  Success = TRUE;
    
    if(f_read>0) data_size = 4;
            else    data_size = 7;

    i = 0;
    LED2_ON();

    while(i<MAXSENSORDATASIZE)
    {
        Success = i2c_read_reg(f_read);
        if(Success)
        {
            for(j=0; j<data_size; j++)
            {
                /*
                if(j==0)
                {
                    if(sensortmpdata[j]!=0x0f) break;
                }
                 */
              motionsensor[i]=sensortmpdata[j];
              i++;
              
              /*
              if(i>= MAXSENSORDATASIZE)
              {
                    LED2_OFF();
                return i;
              }
              */
            }
        }
    }

    LED2_OFF();
 
    return i;

}

BOOL i2c_read_reg(int f_read)
{
    BOOL                Success = FALSE;
    int                 DataSz;
    UINT8               i2cData[10];
    I2C_7_BIT_ADDRESS   SlaveAddress;
    int                 Index;
    UINT8               i2cbyte=0x00;
    int                 delay_time = 100, i=0;

    UINT8        reg_address = 0x00;

    for (i=0; i<7; i++) sensortmpdata[i]=0;

        // Initialize the data buffer
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, sensor_ADDRESS, I2C_WRITE);
        i2cData[0] = SlaveAddress.byte;
        i2cData[1] = reg_address;              // EEPROM location to program (high address byte)
        DataSz = 2;
        // Start the transfer to write data to the EEPROM
     //   delay(delay_time);

        if( !StartTransfer(FALSE) )
        {
            return FALSE;
        }
        // Transmit all data
        Index = 0;

        while( Success && (Index < DataSz) )
        {
            // Transmit a byte

         //   delay(delay_time);

            if (TransmitOneByte(i2cData[Index]))
            {
                // Advance to the next byte
                Index++;
         //       delay(delay_time);
                // Verify that the byte was acknowledged
                if(!I2CByteWasAcknowledged(sensor_I2C_BUS))
                {
                    Success = FALSE;
                }
            }
            else
            {
                Success = FALSE;
            }
        }
        LED1_ON();
    // End the transfer (hang here if an error occured)
    //StopTransfer();
    StopTransfer();
        // Send a Repeated Started condition
    //    delay(delay_time);

    // now restart i2c transfer again, so we can use repeatstart to speed the i2c bus configuration

     if( !StartTransfer(TRUE) )
        {
            Success = FALSE;
        }


        // Transmit the address with the READ bit set
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, sensor_ADDRESS, I2C_READ);
     //   delay(delay_time);

        if (TransmitOneByte(SlaveAddress.byte))
        {
            // Verify that the byte was acknowledged
       //     delay(delay_time);
            if(!I2CByteWasAcknowledged(sensor_I2C_BUS))
            {
                Success = FALSE;
            }
        }
        else
        {
            Success = FALSE;
        }

    
    // Read the data from the desired address
    if(f_read>0){
        DataSz = 4;
    }
    else
        DataSz = 7;

    LED0_ON();

    Success = TRUE;
    i=0;
    while(Success&&i<DataSz)
    {
             if(I2CReceiverEnable(sensor_I2C_BUS, TRUE) == I2C_RECEIVE_OVERFLOW)
                 Success = FALSE;
             else
             {
                while(!I2CReceivedDataIsAvailable(sensor_I2C_BUS))  ;

                sensortmpdata[i] = I2CGetByte(sensor_I2C_BUS);
                if(i == 0 &&(sensortmpdata[i]==0x00||sensortmpdata[i]==0x09||sensortmpdata[i]==0x0b))
                {
                     I2CAcknowledgeByte(sensor_I2C_BUS,FALSE);
                     while(!I2CAcknowledgeHasCompleted(sensor_I2C_BUS));
                     StopTransfer();
                     return FALSE;
                }

        //     if(i==0 && sensortmpdata[i]!=0x0F )  Success = FALSE;

                if(i<(DataSz-1)) I2CAcknowledgeByte(sensor_I2C_BUS,TRUE);
                else
                    I2CAcknowledgeByte(sensor_I2C_BUS,FALSE);
                i++;

                while(!I2CAcknowledgeHasCompleted(sensor_I2C_BUS)); //  delay(delay_time);
             }
    }

    StopTransfer();

    if(!Success) LEDS_ON();

    return Success;

}

/*******************************************************************************
  Function:
    BOOL StartTransfer( BOOL restart )

  Summary:
    Starts (or restarts) a transfer to/from the EEPROM.

  Description:
    This routine starts (or restarts) a transfer to/from the EEPROM, waiting (in
    a blocking loop) until the start (or re-start) condition has completed.

  Precondition:
    The I2C module must have been initialized.

  Parameters:
    restart - If FALSE, send a "Start" condition
            - If TRUE, send a "Restart" condition

  Returns:
    TRUE    - If successful
    FALSE   - If a collision occured during Start signaling

  Example:
    <code>
    StartTransfer(FALSE);
    </code>

  Remarks:
    This is a blocking routine that waits for the bus to be idle and the Start
    (or Restart) signal to complete.
  *****************************************************************************/
BOOL StartTransfer( BOOL restart )
{
    I2C_STATUS  status;

    // Send the Start (or Restart) signal
    if(restart)
    {
        I2CRepeatStart(sensor_I2C_BUS);
    }
    else
    {
        // Wait for the bus to be idle, then start the transfer
        while( !I2CBusIsIdle(sensor_I2C_BUS) );

        if(I2CStart(sensor_I2C_BUS) != I2C_SUCCESS)
        {
        //    DBPRINTF("Error: Bus collision during transfer Start\n");
            return FALSE;
        }
    }

    // Wait for the signal to complete
    do
    {
        status = I2CGetStatus(sensor_I2C_BUS);

    } while ( !(status & I2C_START) );

    return TRUE;
}


/*******************************************************************************
  Function:
    BOOL TransmitOneByte( UINT8 data )

  Summary:
    This transmits one byte to the EEPROM.

  Description:
    This transmits one byte to the EEPROM, and reports errors for any bus
    collisions.

  Precondition:
    The transfer must have been previously started.

  Parameters:
    data    - Data byte to transmit

  Returns:
    TRUE    - Data was sent successfully
    FALSE   - A bus collision occured

  Example:
    <code>
    TransmitOneByte(0xAA);
    </code>

  Remarks:
    This is a blocking routine that waits for the transmission to complete.
  *****************************************************************************/

BOOL TransmitOneByte( UINT8 data )
{
    // Wait for the transmitter to be ready
    while(!I2CTransmitterIsReady(sensor_I2C_BUS));

    // Transmit the byte
    if(I2CSendByte(sensor_I2C_BUS, data) == I2C_MASTER_BUS_COLLISION)
    {
        DBPRINTF("Error: I2C Master Bus Collision\n");
        return FALSE;
    }

    // Wait for the transmission to finish
    while(!I2CTransmissionHasCompleted(sensor_I2C_BUS));

    return TRUE;
}


/*******************************************************************************
  Function:
    void StopTransfer( void )

  Summary:
    Stops a transfer to/from the EEPROM.

  Description:
    This routine Stops a transfer to/from the EEPROM, waiting (in a
    blocking loop) until the Stop condition has completed.

  Precondition:
    The I2C module must have been initialized & a transfer started.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    StopTransfer();
    </code>

  Remarks:
    This is a blocking routine that waits for the Stop signal to complete.
  *****************************************************************************/

void StopTransfer( void )
{
    I2C_STATUS  status;
    int i;
    // Send the Stop signal
    I2CStop(sensor_I2C_BUS);

    // Wait for the signal to complete
    do
    {   I2CStop(sensor_I2C_BUS);
    for(i = 0; i < 10; i++);
        status = I2CGetStatus(sensor_I2C_BUS);

    } while ( !(status & I2C_STOP) );
}


/*******************************************************************************
 init the hardware LED light

 ******************************************************************************/

 void init_LED(void){

    //LEDS_OFF();
    LED0_TRIS = 0;
    LED1_TRIS = 0;
    LED2_TRIS = 0;
 }
 void LED(char x){
    if((x%2)==1){
        LED0_ON();
    }
    else{
        LED0_OFF();
    }
    if(((x/2)%2)==1){
        LED1_ON();
    }
    else{
        LED1_OFF();
    }
    if(((x/4)%2)==1){
        LED2_ON();
    }
    else{
        LED2_OFF();
    }
    }
/*******************************************************************************
 delay time

 ******************************************************************************/
void delay(int time){
    int i = 0;
    BOOL a = TRUE;

    while(a){
        i++;

        if(i>10*time)
        {
            a = FALSE;
        }

    }
}

BOOL i2c_write_reg(UINT reg_address, UINT data )
{
    BOOL                Success = FALSE;
    int                 DataSz;
    UINT8               i2cData[10];
    I2C_7_BIT_ADDRESS   SlaveAddress;
    int                 Index;
    int                 delay_time = 100;

        Success = TRUE;
        // Initialize the data buffer
        I2C_FORMAT_7_BIT_ADDRESS(SlaveAddress, sensor_ADDRESS, I2C_WRITE);
        i2cData[0] = SlaveAddress.byte;
        i2cData[1] = reg_address;              // EEPROM location to program (high address byte)
        i2cData[2] = data;
        DataSz = 3;
        
        // Start the transfer to write data

        if( !StartTransfer(FALSE) )
        {
    //        TCPPutROMString(sktHTTP, "Cannot start i2c");
            return FALSE;
        }

        // Transmit all data
        Index = 0;

        while( Success && (Index < DataSz) )
        {
            // Transmit a byte

          //  delay(delay_time);

            if (TransmitOneByte(i2cData[Index]))
            {
                // Advance to the next byte
                Index++;
            //    delay(delay_time);
                // Verify that the byte was acknowledged
                if(!I2CByteWasAcknowledged(sensor_I2C_BUS))
                {
                    Success = FALSE;
                }
            }
            else
            {
                Success = FALSE;
            }
        }
    // End the transfer (hang here if an error occured)

    StopTransfer();
  
    return Success;
}



//int testing_counter = 0 ;
//			Get_i2c_data(0x01);     // get x_ data  high 8bit
//			Get_i2c_data(0x02);     // get x_ data  low 8bit

//			Get_i2c_data(0x03);     // get y_ data high 8bit
//			Get_i2c_data(0x04);     // get y_ data  low 8bit

//			Get_i2c_data(0x05);     // get z_ data


void Get_i2c_data(char num)
{

    UINT8               i2cbyte=0x00;
    BYTE data_send_out[] = "0000";
    INT16               output_data_in_number;


    LEDS_ON();                                              //status LED
    i2cbyte = i2c_read_reg(num);

    output_data_in_number = i2cbyte*4;

    if(output_data_in_number<512)           // convert counting to value
    {
        data_send_out[0]= '-';
    }
    else
    {
        output_data_in_number = 1024 - output_data_in_number ;
        data_send_out[0]= '+';
    }


    data_send_out[1]= output_data_in_number/100%10+0x30;    //convert value into ascii code
    data_send_out[2]= output_data_in_number/10%10+0x30;
    data_send_out[3]= output_data_in_number%10+0x30;

    LEDS_OFF();                                             //status LED
//    TCPPutROMString(sktHTTP, data_send_out);
    return;

}


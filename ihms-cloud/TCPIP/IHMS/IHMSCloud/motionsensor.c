#include "main.h"
#include <plib.h>
#include <math.h>

//#define GetSystemClock()	(40000000ul)                // 40MHz is set by Max Choi, but the system clock should be 80MHz according to MCU specification.
#define GetInstructionClock()	(GetSystemClock()/1)
#define GetPeripheralClock()	(GetInstructionClock()/1)	// Set your divider according to your Peripheral Bus Frequency configuration fuse setting
#define I2C_CLOCK_FREQ          200000

// EEPROM Constants
#define sensor_I2C_BUS              I2C5
#define sensor_ADDRESS              0x1C       // The MMA8451Q's standard slave address is a choice between the two sequential addresses 0011100 - 0x1C and 0011101 - 0X1D. The
                                                //selection is made by the high and low logic level of the SA0 (pin 7)

#define WINDOW_LENGTH 1000


BYTE sensortmpdata[7];  //for reading MMA845X Q  Data Registers from 0x00 to 0x06

BOOL FIRST_VALUE = TRUE;
BOOL FALL_DETECTED = FALSE;
int LastSitDownTime;

float MaxXmin = 0.2146, MaxXmax = 7.999, MeanXmin = -0.0171, MeanXmax = 0.0201, MaxYmin = 0.2285, MaxYmax = 7.9990, MaxZmin = 0.1934, MaxZmax = 7.9990, MeanYmin = -0.0470, MeanYmax = 0.0192, EnYmin = 16.1496, EnYmax = 1213.7, EnZmin = 1.7870, EnZmax = 797.5010, StdZmin = 0.0423, StdZmax = 0.8933, CovYZmin = -0.6177, CovYZmax = 0.7763, CovXZmax = 0.7429, CovXZmin = -0.5056;

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
    LastSitDownTime = 0;
    LastFallDetectedTime = 0;

    actualClock = I2CSetFrequency(sensor_I2C_BUS, GetPeripheralClock(), I2C_CLOCK_FREQ);
    if ( abs(actualClock-I2C_CLOCK_FREQ) > I2C_CLOCK_FREQ/10 )
        {
//            TCPPutROMString(sktHTTP, "Clock freq have more than 10% error ! ");
            return FALSE;
        }
    TRISBbits.TRISB14 = 0;
    LATBbits.LATB14 = 0;

    TRISBbits.TRISB1 = 0;
    LATBbits.LATB1 = 0;

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
    int signX, signY, signZ;  //the sign of x and y
    int i, j, m, data_size;
    volatile char* p;
    int c = 56000, x;
    unsigned short sx, sy, sz;
    float fx, fy, fz;
    float sumX=0, sumY=0, sumZ = 0, sumYZ = 0, sumXZ = 0, meanX=0, meanY=0, meanZ = 0, energySumY=0, energySumX = 0, maxX = -10000, maxY = -10000, maxZ = -10000, energySumZ = 0, stdZ = 0, covXZ = 0, covYZ = 0;
    float halfSumX=0, halfSumY=0, halfSumZ = 0, halfSumYZ = 0, halfSumXZ = 0, halfEnergySumX = 0, halfEnergySumY=0, halfEnergySumZ = 0, halfMaxX = -10000, halfMaxY = -10000, halfMaxZ = -10000;
    BOOL  Success = TRUE;
    FIRST_VALUE = TRUE;
    if(f_read>0) data_size = 4;
            else    data_size = 7;

    i = m = 0;
    LED2_ON();
    FIRST_VALUE = TRUE;
    if(TickGet() - LastFallDetectedTime > 2 * TICK_SECOND){
        FALL_DETECTED = FALSE;
        LED1_OFF();
    }

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
            if((FIRST_VALUE && sensortmpdata[0] == 0xFF) || sensortmpdata[0] == 0x0F){
                if(FIRST_VALUE)FIRST_VALUE = FALSE;
                signX = (sensortmpdata[1] & 0x80) ? -1 : 1;
                signY = (sensortmpdata[3] & 0x80) ? -1 : 1;
                signZ = (sensortmpdata[5] & 0x80) ? -1 : 1;;
                sx = ((sensortmpdata[1]<<6) + ((sensortmpdata[2]>>2)&0x3F)) & 0x1FFF;
                if(signX < 0){
                    sx = 0x2000 - sx;
                }
                sy = ((sensortmpdata[3]<<6) + ((sensortmpdata[4]>>2)&0x3F)) & 0x1FFF;
                if(signY < 0){
                    sy = 0x2000 - sy;
                }
                sz = ((sensortmpdata[5]<<6) + ((sensortmpdata[6]>>2)&0x3F)) & 0x1FFF;
                if(signZ < 0){
                    sz = 0x2000 - sz;
                }
                fx = signX * (float)sx / 1024.0;
                fy = signY * (float)sy / 1024.0;
                fz = signZ * (float)sz / 1024.0;
                if(m < WINDOW_LENGTH / 2)
                {
                    sumX += fx;
                    sumY += fy;
                    sumZ += fz;
                    sumYZ += fy * fz;
                    sumXZ += fx * fz;
                    if(fx > maxX)maxX = fx;
                    if(fy > maxY)maxY = fy;
                    if(fz > maxZ)maxZ = fz;
                    energySumX += fx * fx;
                    energySumY += fy * fy;
                    energySumZ += fz * fz;
                }else if(m <= WINDOW_LENGTH){
                    if(m % WINDOW_LENGTH == 0){
                        meanX = sumX / WINDOW_LENGTH;
                        meanY = sumY / WINDOW_LENGTH;
                        meanZ = sumZ / WINDOW_LENGTH;
                        stdZ = sqrt((energySumZ - WINDOW_LENGTH * meanZ * meanZ)/WINDOW_LENGTH);
                        covXZ = (WINDOW_LENGTH * sumXZ - sumX * sumZ) / (sqrt(WINDOW_LENGTH * energySumX - sumX * sumX) * sqrt(WINDOW_LENGTH * energySumZ - sumZ * sumZ));
                        covYZ = (WINDOW_LENGTH * sumYZ - sumY * sumZ) / (sqrt(WINDOW_LENGTH * energySumY - sumY * sumY) * sqrt(WINDOW_LENGTH * energySumZ - sumZ * sumZ));
                        covXZ = 2 * (covXZ - CovXZmin)/(CovXZmax - CovXZmin) - 1;
                        covYZ = 2 * (covYZ - CovYZmin)/(CovYZmax - CovYZmin) - 1;
                        energySumZ = 2 * (energySumZ - EnZmin)/(EnZmax - EnZmin) - 1;
                        stdZ = 2 * (stdZ - StdZmin)/(StdZmax - StdZmin) -1;
                        meanX = 2 * (meanX - MeanXmin)/(MeanXmax - MeanXmin) - 1;
                        meanY = 2 * (meanY - MeanYmin)/(MeanYmax - MeanYmin) - 1;
                        maxX = 2 * (maxX - MaxXmin)/(MaxXmax - MaxXmin) - 1;
                        maxY = 2 * (maxY - MaxYmin)/(MaxYmax - MaxYmin) - 1;
                        maxZ = 2 * (maxZ - MaxZmin)/(MaxZmax - MaxZmin) - 1;
                        energySumY = 2 * (energySumY - EnYmin)/(EnYmax - EnYmin) - 1;
                        //decision(maxX, maxY, meanX, meanY, energySumY);
                        //decision3(meanX, maxY, maxZ, covYZ, energySumZ);
                        //decision2(maxY, stdZ);
                        decision4(maxX, meanX, maxZ, covXZ, covYZ);
                        maxX = halfMaxX;
                        maxY = halfMaxY;
                        maxZ = halfMaxZ;
                        halfMaxX = -10000;
                        halfMaxY = -10000;
                        halfMaxZ = -10000;
                        m = WINDOW_LENGTH/2;//m = WINDOW_LENGTH * 0.3; //for 30% overlay window
                        sumX = halfSumX + fx;
                        sumY = halfSumY + fy;
                        sumZ = halfSumZ + fz;
                        sumYZ = halfSumYZ + fy * fz;
                        sumXZ = halfSumXZ + fx * fz;
                        energySumY = halfEnergySumY + fy * fy;
                        energySumX = halfEnergySumX + fx * fx;
                        energySumZ = halfEnergySumZ + fz * fz;
                        halfSumX = fx;
                        //halfSumX = 0;      //for 30% overlay window
                        halfSumY = fy;
                        //halfSumY = 0;
                        halfSumZ = fz;
                        //halfSumZ = 0;
                        halfSumXZ = fx * fz;
                        halfSumYZ = fy * fz;
                        //halfSumYZ = 0;
                        halfEnergySumX = fx * fx;
                        halfEnergySumY = fy * fy;
                        //halfEnergySumY = 0;
                        halfEnergySumZ = fz * fz;
                        //halfEnergySumZ = 0;
                        if(fx > maxX)maxX = fx;
                        if(fx > halfMaxX)halfMaxX = fx;
                        if(fy > maxY)maxY = fy;
                        if(fy > halfMaxY)halfMaxY = fy;
                        if(fz > maxZ)maxZ = fz;
                        if(fz > halfMaxZ)halfMaxZ = fz;
                    }else if(m >= WINDOW_LENGTH / 2){        //m >= WINDOW_LENGTH * 0.7, 30%overlay window
                        sumX += fx;
                        sumY += fy;
                        sumZ += fz;
                        sumYZ += fy * fz;
                        sumXZ += fx * fz;
                        energySumX += fx*fx;
                        energySumY += fy*fy;
                        energySumZ += fz*fz;
                        halfSumX += fx;
                        halfSumY += fy;
                        halfSumZ += fz;
                        halfSumXZ += fx * fz;
                        halfSumYZ += fy * fz;
                        halfEnergySumX += fx * fx;
                        halfEnergySumY += fy * fy;
                        halfEnergySumZ += fz * fz;
                        if(fx > maxX)maxX = fx;
                        if(fy > maxY)maxY = fy;
                        if(fz > maxZ)maxZ = fz;
                        if(fx > halfMaxX)halfMaxX = fx;
                        if(fy > halfMaxY)halfMaxY = fy;
                        if(fz > halfMaxZ)halfMaxZ = fz;
                    }/*else{
                        sumX += fx;
                        sumY += fy;
                        sumZ += fz;
                        sumYZ += fy *fz;
                        energySumY += fy*fy;
                        energySumZ += fz*fz;
                        if(fx > maxX)maxX = fx;
                        if(fy > maxY)maxY = fy;
                        if(fz > maxZ)maxZ = fz;
                    }*/
                }
                m++;
            }
        }
    }

    LED2_OFF();
 
    return i;

}


void decision4(float maxX, float meanX, float maxZ, float covXZ, float covYZ){
    if(FALL_DETECTED && (TickGet() - LastFallDetectedTime > 3 * TICK_SECOND)){
        FALL_DETECTED = FALSE;
    }
    if(TickGet() - LastFallDetectedTime > 2 * TICK_SECOND){
        //B_SOUND_OFF();
        //LED1_OFF();
        //printf("3");
        //FALL_DETECTED = FALSE;
    }

    if(maxZ <= -0.81684){
        //printf("1"); //walk
    }else{
        if(covXZ <= 0.498979){
            if(meanX <= -0.0984032){
                if(maxX <= 0.244762){
                    //printf("4");
                }else{
                    //printf("3");
                    if(TickGet() - LastFallDetectedTime > 2 * TICK_SECOND){
                        //B_SOUND_ON();
                        LED1_ON();
                        //printf("3");
                        FALL_DETECTED = TRUE;
                        LastFallDetectedTime = TickGet();
                    }
                }
            }else{
                if(covYZ <= 0.41822){
                    //printf("3");
                    if(TickGet() - LastFallDetectedTime > 2 * TICK_SECOND){
                        //B_SOUND_ON();
                        LED1_ON();
                        //printf("3");
                        FALL_DETECTED = TRUE;
                        LastFallDetectedTime = TickGet();
                    }
                }else{
                    //printf("4");
                }
            }
        }else{
            if(maxZ <= 0.130239){
                //printf("2");
            }else{
                //printf("3");
                if(TickGet() - LastFallDetectedTime > 2 * TICK_SECOND){
                    //B_SOUND_ON();
                    LED1_ON();
                    //printf("3");
                    FALL_DETECTED = TRUE;
                    LastFallDetectedTime = TickGet();
                }
            }
        }
    }
}

void decision3(float meanX, float maxY, float maxZ, float covYZ, float enZ){
    if(FALL_DETECTED && (TickGet() - LastFallDetectedTime > 2 * TICK_SECOND)){
        FALL_DETECTED = FALSE;
    }
    if(TickGet() - LastFallDetectedTime > 1 * TICK_SECOND){
        LED0_ON();
        B_SOUND_OFF();
        LED1_OFF();
        //printf("3");
        FALL_DETECTED = FALSE;
    }
    if(maxY > -0.38){
        if(TickGet() - LastFallDetectedTime > 2 * TICK_SECOND){
            LED0_OFF();
            B_SOUND_ON();
            LED1_OFF();
            //printf("3");
            FALL_DETECTED = TRUE;
            LastFallDetectedTime = TickGet();
        }
    }else if(enZ <= -0.981629){
        if(meanX <= -0.11545){
            if(!FALL_DETECTED && (TickGet() - LastSitDownTime > 2*TICK_SECOND)){
                LED1_ON();
                //printf("2");
                LastSitDownTime = TickGet();
            }
        }else if(maxY <= -0.974614){
            if(!FALL_DETECTED && (TickGet() - LastSitDownTime > 2*TICK_SECOND)){
                LED1_ON();
                //printf("2");
                LastSitDownTime = TickGet();
            }
        }else{
            //printf("4");
        }
    }else{
        if(covYZ > -0.481108){
            //printf("1");
        }else if(maxZ <= -0.846309){
            //printf("1");
        }else{
            //printf("4");
        }
    }
}

void decision2(float maxY, float stdZ){
    if(FALL_DETECTED && (TickGet() - LastFallDetectedTime > 5 * TICK_SECOND)){
        FALL_DETECTED = FALSE;
    }
    if(TickGet() - LastFallDetectedTime > 1 * TICK_SECOND){
        //B_SOUND_OFF();
        LED1_OFF();
        //printf("3");
        FALL_DETECTED = FALSE;
    }
    if(maxY > -0.768506){
        //printf("3");
        if(TickGet() - LastFallDetectedTime > 2 * TICK_SECOND){
            //B_SOUND_ON();
            LED1_ON();
            //printf("3");
            FALL_DETECTED = TRUE;
            LastFallDetectedTime = TickGet();
        }
    }else if(stdZ <= -0.910844){
        if(!FALL_DETECTED && (TickGet() - LastSitDownTime > 2*TICK_SECOND)){
            //LED1_ON();
            //printf("2");
            LastSitDownTime = TickGet();
        }
    }else{
        //printf("1");
    }
}

void decision(float maxX, float maxY, float meanX, float meanY, float enY){
    if(maxY <= -0.834681){
        if(maxX > -0.929199)
        {

            //printf("1");
        }else{
            if(maxY  <= -0.926206){

                //printf("1");
            }else{
                if(TickGet() - LastSitDownTime > 2*TICK_SECOND){
                    LED1_ON();
                    //printf("2");
                    LastSitDownTime = TickGet();
                }
            }
        }
    }else if(enY > -0.707265 && enY <= 0.610428){
        if(TickGet() - LastFallDetectedTime > 2 * TICK_SECOND){
            LED0_OFF();
            //printf("3");
            LastFallDetectedTime = TickGet();
        }
    }else if(enY > 0.610428){
        if(maxY <= 0.403129){
            if(TickGet() - LastSitDownTime > 2*TICK_SECOND){
                LED1_ON();
                //printf("2");
                LastSitDownTime = TickGet();
            }
        }else{
            if(TickGet() - LastFallDetectedTime > 2 * TICK_SECOND){
                LED0_OFF();
                //printf("3");
                LastFallDetectedTime = TickGet();
            }
        }
    }else if(enY > -0.890036 && enY < -0.707265){
        if(maxY > -0.255802){
            if(TickGet() - LastFallDetectedTime > 2 * TICK_SECOND){
                LED0_OFF();
                //printf("3");
                LastFallDetectedTime = TickGet();
            }
        }else{
            if(meanX <= 0.519166){
                if(TickGet() - LastSitDownTime > 2*TICK_SECOND){
                    LED1_ON();
                    //printf("2");
                    LastSitDownTime = TickGet();
                }
            }else{
                if(maxX <= 0.260974){
                    if(TickGet() - LastSitDownTime > 2*TICK_SECOND){
                        LED1_ON();
                        //printf("2");
                        LastSitDownTime = TickGet();
                    }
                }else{
                    if(TickGet() - LastFallDetectedTime > 2 * TICK_SECOND){
                        LED0_OFF();
                        //printf("3");
                        LastFallDetectedTime = TickGet();
                    }
                }
            }
        }
    }else{
        if(TickGet() - LastFallDetectedTime > 2 * TICK_SECOND){
            LED0_OFF();
            //printf("3");
            LastFallDetectedTime = TickGet();
        }
    }
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


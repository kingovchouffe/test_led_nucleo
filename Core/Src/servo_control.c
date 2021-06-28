/*

Code to control multiple Dynamixel AX-12 servo motors over USART
on an STM32F4 chip.

Kent deVillafranca
April 2013

*/

#include "main.h"


#define TEST_COMMANDS
__asm__(".global _printf_float");
__asm__(".global _scanf_float"); 



#ifndef true
#define true ((bool)1)
#endif

#ifndef false
#define false ((bool)0)
#endif

uint8_t servoErrorCode = 0;

ServoResponse response;

volatile uint8_t receiveBuffer[REC_BUFFER_LEN];
volatile uint8_t* volatile receiveBufferStart = receiveBuffer;
volatile uint8_t* volatile receiveBufferEnd = receiveBuffer;



#define RETURN_DELAY        0x05
#define BLINK_CONDITIONS    0x11
#define SHUTDOWN_CONDITIONS 0x12
#define TORQUE              0x22
#define MAX_SPEED           0x20
#define CURRENT_SPEED       0x26
#define GOAL_ANGLE          0x1e
#define CURRENT_ANGLE       0x24

void sendServoCommand (const uint8_t servoId,
                       const ServoCommand commandByte,
                       const uint8_t numParams,
                       const uint8_t *params)
{
    sendServoByte (0xff);
    sendServoByte (0xff);  // command header
    
    sendServoByte (servoId);  // servo ID
    uint8_t checksum = servoId;
    
    sendServoByte (numParams + 2);  // number of following bytes
    sendServoByte ((uint8_t)commandByte);  // command
    
    checksum += numParams + 2 + commandByte;
    
    for (uint8_t i = 0; i < numParams; i++)
    {
        sendServoByte (params[i]);  // parameters
        checksum += params[i];
    }
    
    sendServoByte (~checksum);  // checksum
}

bool getServoResponse (void)
{
    uint8_t retries = 0;
    
    clearServoReceiveBuffer();
    
    while (getServoBytesAvailable() < 4)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            #ifdef SERVO_DEBUG
            printf ("Too many retries at start\n\r");
            #endif
            return false;
        }
        
        
    }
    retries = 0;
    
    getServoByte();  // servo header (two 0xff bytes)
    getServoByte();
    
    response.id = getServoByte();
    response.length = getServoByte();
    
    if (response.length > SERVO_MAX_PARAMS)
    {
        #ifdef SERVO_DEBUG
        printf ("Response length too big: %d\n\r", (int)response.length);
        #endif
        return false;
    }
    
    while (getServoBytesAvailable() < response.length)
    {
        retries++;
        if (retries > REC_WAIT_MAX_RETRIES)
        {
            #ifdef SERVO_DEBUG
            printf ("Too many retries waiting for params, got %d of %d params\n\r", getServoBytesAvailable(), response.length);
            #endif
            return false;
        }
        
        
    }
    
    response.error = getServoByte();
    servoErrorCode = response.error;
    
    for (uint8_t i = 0; i < response.length - 2; i++)
        response.params[i] = getServoByte();
    
    
    uint8_t calcChecksum = response.id + response.length + response.error;
    for (uint8_t i = 0; i < response.length - 2; i++)
        calcChecksum += response.params[i];
    calcChecksum = ~calcChecksum;
    
    const uint8_t recChecksum = getServoByte();
    if (calcChecksum != recChecksum)
    {
        #ifdef SERVO_DEBUG
        printf ("Checksum mismatch: %x calculated, %x received\n\r", calcChecksum, recChecksum);
        #endif
        return false;
    }
    
    return true;
}

inline bool getAndCheckResponse (const uint8_t servoId)
{
    if (!getServoResponse())
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Servo %d did not respond correctly or at all\n\r", (int)servoId);
        #endif
        return false;
    }
    
    if (response.id != servoId)
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Response ID %d does not match command ID %d\n\r", (int)response.id);
        #endif
        return false;
    }
    
    if (response.error != 0)
    {
        #ifdef SERVO_DEBUG
        printf ("Servo error: Response error code was nonzero (%d)\n\r", (int)response.error);
        #endif
        return false;
    }
    
    return true;
}

// ping a servo, returns true if we get back the expected values
bool pingServo (const uint8_t servoId)
{
    sendServoCommand (servoId, PING, 0, 0);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool setServoReturnDelayMicros (const uint8_t servoId,
                                const uint16_t micros)
{
    if (micros > 510)
        return false;
    
    const uint8_t params[2] = {RETURN_DELAY,
                               (uint8_t)((micros / 2) & 0xff)};
    
    sendServoCommand (servoId, WRITE, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

// set the events that will cause the servo to blink its LED
bool setServoBlinkConditions (const uint8_t servoId,
                              const uint8_t flags)
{
    const uint8_t params[2] = {BLINK_CONDITIONS,
                               flags};
    
    sendServoCommand (servoId, WRITE, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

// set the events that will cause the servo to shut off torque
bool setServoShutdownConditions (const uint8_t servoId,
                                 const uint8_t flags)
{
    const uint8_t params[2] = {SHUTDOWN_CONDITIONS,
                               flags};
    
    sendServoCommand (servoId, WRITE, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}


// valid torque values are from 0 (free running) to 1023 (max)
bool setServoTorque (const uint8_t servoId,
                     const uint16_t torqueValue)
{
    const uint8_t highByte = (uint8_t)((torqueValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(torqueValue & 0xff);
    
    if (torqueValue > 1023)
        return false;
    
    const uint8_t params[3] = {TORQUE,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, 3, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool getServoTorque (const uint8_t servoId,
                     uint16_t *torqueValue)
{
    const uint8_t params[2] = {TORQUE,
                               2};  // read two bytes, starting at address TORQUE
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    *torqueValue = response.params[1];
    *torqueValue <<= 8;
    *torqueValue |= response.params[0];
    
    return true;
}

// speed values go from 1 (incredibly slow) to 1023 (114 RPM)
// a value of zero will disable velocity control
bool setServoMaxSpeed (const uint8_t servoId,
                       const uint16_t speedValue)
{
    const uint8_t highByte = (uint8_t)((speedValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(speedValue & 0xff);
    
    if (speedValue > 1023)
        return false;
    
    const uint8_t params[3] = {MAX_SPEED,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, 3, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool getServoMaxSpeed (const uint8_t servoId,
                       uint16_t *speedValue)
{
    const uint8_t params[2] = {MAX_SPEED,
                               2};  // read two bytes, starting at address MAX_SPEED
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    *speedValue = response.params[1];
    *speedValue <<= 8;
    *speedValue |= response.params[0];
    
    return true;
}

bool getServoCurrentVelocity (const uint8_t servoId,
                              int16_t *velocityValue)
{
    const uint8_t params[2] = {CURRENT_SPEED,
                               2};  // read two bytes, starting at address CURRENT_SPEED
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    *velocityValue = response.params[1];
    *velocityValue <<= 8;
    *velocityValue |= response.params[0];
    
    return true;
}

// make the servo move to an angle
// valid angles are between 0 and 300 degrees
bool setServoAngle (const uint8_t servoId,
                    const float angle)
{
    if (angle < 0 || angle > 300)
        return false;
    
    // angle values go from 0 to 0x3ff (1023)
    const uint16_t angleValue = (uint16_t)(angle * (1023.0 / 300.0));
    
    const uint8_t highByte = (uint8_t)((angleValue >> 8) & 0xff);
    const uint8_t lowByte = (uint8_t)(angleValue & 0xff);
    
    const uint8_t params[3] = {GOAL_ANGLE,
                               lowByte,
                               highByte};
    
    sendServoCommand (servoId, WRITE, 3, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    return true;
}

bool getServoAngle (const uint8_t servoId,
                    float *angle)
{
    const uint8_t params[2] = {CURRENT_ANGLE,
                               2};  // read two bytes, starting at address CURRENT_ANGLE
    
    sendServoCommand (servoId, READ, 2, params);
    
    if (!getAndCheckResponse (servoId))
        return false;
    
    uint16_t angleValue = response.params[1];
    angleValue <<= 8;
    angleValue |= response.params[0];
    
    *angle = (float)angleValue * 300.0 / 1023.0;
    
    return true;
}

void sendServoByte (uint8_t byte)
{
	  HAL_HalfDuplex_EnableTransmitter(&huart1);
	  HAL_UART_Transmit(&huart1,&byte,sizeof(byte),100);
      HAL_HalfDuplex_EnableReceiver(&huart1);
	  //Loop until the end of transmission
	  //while (USART_GetFlagStatus (USART1, USART_GetFlagStatus) == RESET);
}

void clearServoReceiveBuffer (void)
{
    receiveBufferStart = receiveBufferEnd;
}

size_t getServoBytesAvailable (void)
{
    volatile uint8_t *start = receiveBufferStart;
    volatile uint8_t *end = receiveBufferEnd;
    
    if (end >= start)
        return (size_t)(end - start);
    else
        return (size_t)(REC_BUFFER_LEN - (start - end));
}

uint8_t getServoByte (void)
{
    receiveBufferStart++;
    if (receiveBufferStart >= receiveBuffer + REC_BUFFER_LEN)
        receiveBufferStart = receiveBuffer;
    
    return *receiveBufferStart;
}

void USART1_IRQHandler (void)
{
	// check if the USART3 receive interrupt flag was set
	if (HAL_HalfDuplex_EnableReceiver(&huart1))
	{   
		uint8_t byte;
        HAL_UART_Receive(&huart1,&byte,sizeof(byte),150); // grab the byte from the data register
        
        receiveBufferEnd++;
        if (receiveBufferEnd >= receiveBuffer + REC_BUFFER_LEN)
            receiveBufferEnd = receiveBuffer;
        
        *receiveBufferEnd = byte;
	}
}




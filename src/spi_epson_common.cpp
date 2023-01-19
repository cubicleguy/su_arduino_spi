/**************************************************************************/
/*!
    @file     spi_epson_common.cpp

    Epson IMU/Accel Common Driver for Arduino SPI

    @section  HISTORY

    v1.0 - First release
    v1.1 - Refactoring
    v1.2 - Added constructor initialization list, remove debug code, cleanup

    @section LICENSE

    Software License Agreement (BSD License, see license.txt)

    Copyright (c) 2019-2023 Seiko Epson Corporation.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
    THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
    PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
    OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
    OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
    ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#include <math.h>
#include <string.h>
#include "spi_epson_common.h"

/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
//Hardware SPI constructor
SPI_EPSON_COM::SPI_EPSON_COM(int8_t cs, int8_t drdy, int8_t nrst):
  // initializer list
  _burstCnt_calculated(0),
  _initialised(false),
  _clk(-1),  //using hardware SPI
  _mosi(-1),
  _miso(-1) {
    _cs = cs;
    _drdy = drdy;                 //store ChipSelect and DataReady pins
    _nrst = nrst;                 //store nRESET pin
}

//Software SPI constructor
SPI_EPSON_COM::SPI_EPSON_COM(int8_t clk, int8_t miso, int8_t mosi, int8_t cs, int8_t drdy, int8_t nrst):
  // initializer list
  _burstCnt_calculated(0),
  _initialised(false) {
  //using software SPI so assign GPIO pin numbers for pins
  _cs = cs;
  _clk = clk;
  _mosi = mosi;
  _miso = miso;
  _drdy = drdy;
  _nrst = nrst;
}


/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Initializes SPI and reads the Product ID register
            for validation (call this function before doing
            anything else)

    @returns   False if DataReady timeout or Product ID is unknown,
               otherwise True
*/
/**************************************************************************/
boolean SPI_EPSON_COM::begin(void){
    // Configure DataReady Input pin
    pinMode(_drdy, INPUT);
    SerialConsole.print("DRDY on "); SerialConsole.println(_drdy, DEC);
    // Configure nRESET pin for output
    pinMode(_nrst, OUTPUT);
    SerialConsole.print("nRST on "); SerialConsole.println(_nrst, DEC);
    digitalWrite(_nrst, LOW);
    SerialConsole.print("Platform: ");
    if (_clk == -1) {  // For hardware SPI
#ifdef __SAM3X8E__
        SerialConsole.println("Arduino DUE");
        // When Arduino DUE detected use SPI Extensions for DUE
        SPI.begin(_cs);
        SPI.setClockDivider (_cs,84);  // 84/84 = 1 MHz
        SPI.setBitOrder(_cs,MSBFIRST);
        SPI.setDataMode(_cs,SPI_MODE3);
#elif defined(__MK66FX1M0__)    // Teensy 3.6
        SerialConsole.println("Teensy 3.6");
        // Use regular SPI
        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH);
        SPI.begin();
        SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
#else
        SerialConsole.println("Normal Arduino");
        // For standard Arduino UNO etc. use regular SPI
        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH);
        SPI.begin();
        SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
#endif
    }
    else {
        //setup pins for software emulated SPI
        SerialConsole.println("Software Emulated SPI");
        pinMode(_cs, OUTPUT);
        digitalWrite(_cs, HIGH);
        pinMode(_clk, OUTPUT);
        digitalWrite(_clk, HIGH);
        pinMode(_mosi, OUTPUT);
        pinMode(_miso, INPUT);
    }

    // Set nRESET pin HIGH
    digitalWrite(_nrst, HIGH);
    EpsonPowerOnDelay();

    // Sanity Check for DRDY = LOW  (This assumes DRDY is configured active HIGH)
    // If the SU is configured for DRDY active LOW, then this validation check will fail
    SerialConsole.println("Check DRDY");
    boolean ok = waitDataReady(false, EPSON_DRDYCHECK);
    if (ok == false) {
        SerialConsole.println("Error: Timeout waiting for DRDY = LOW, DataReady pin appears stuck HIGH");
        SerialConsole.println("Attempting hardware reset...");
        sensorHWReset();

        // Sanity Check for DRDY = LOW  (This assumes DRDY is configured active HIGH)
        // If the SU is configured for DRDY active LOW, then this validation check will fail
        ok = waitDataReady(false, EPSON_DRDYCHECK);

        if (ok == false) {
            SerialConsole.println("Error: Could not reset sensor, DataReady pin appears stuck HIGH");
        }
    }
    if (ok == true) {

        char modelNameReturned[9];
        getProdID(modelNameReturned);

        int result = strcmp(modelNameReturned, EPSON_MODEL_STR);
        if (result == 0) {
            SerialConsole.println(EPSON_MODEL_STR " Detected");
            _initialised = true;
        }
        else {
            char unexpstr[128];
            sprintf(unexpstr, EPSON_UNIT_TYPE " Device Error - Expected: " EPSON_MODEL_STR ", Detected : %s", modelNameReturned);
            SerialConsole.println(unexpstr);
        }
    }
    //return the current initialized state
    return _initialised;
}


/**************************************************************************/
/*!
    @brief  Writes an 8-bit value at the specific register address

    @param [in] winid
                The 8-bit window ID. Ignored for V340
    @param [in] addr
                The 8-bit register address
    @param [in] value
                The 8-bit value to write at address
    @param [in] verbose
                boolean to enabled debug output of register access
*/
/**************************************************************************/
void SPI_EPSON_COM::regWrite8(uint8_t winid, uint8_t addr, uint8_t value, boolean verbose) {

  // Set ChipSelect
  digitalWrite(_cs, LOW);

#ifndef EPSON_V340
  // Send the window command & win ID
  SPItransfer(ADDR_WIN_CTRL | 0x80);
  SPItransfer(winid & 0x01); // mask off unused bits

  // Delay between 16 bit transfers
  EpsonStall();
#endif

  // Send the address (command) to be written
  SPItransfer(addr | 0x80); // msb is set 1b for register write
  // Write data value
  SPItransfer(value);

  // Release ChipSelect
  digitalWrite(_cs, HIGH);

  // Delay between 16 bit transfers
  EpsonStall();

  // If debug output selected, print information about the transfer
  if (verbose) {
    SerialConsole.print("REG[0x");
    SerialConsole.print((addr&0x7F), HEX);
#ifndef EPSON_V340
    SerialConsole.print(" W(");
    SerialConsole.print(winid, DEC);
    SerialConsole.print(")");
#endif
    SerialConsole.print("] < 0x");
    SerialConsole.println(value, HEX);
  }
}


/**************************************************************************/
/*!
    @brief  Reads an 16 bit value from the specified register address

    @param [in] winid
                The 8-bit window ID. Ignored for V340.
    @param [in] addr
                The 8-bit register address (must be even, 16-bit aligned)
    @param [in] verbose
                boolean to enabled debug output of register access
    @returns    The 16-bit value retrieved from register
*/
/**************************************************************************/
uint16_t SPI_EPSON_COM::regRead16(uint8_t winid, uint8_t addr, boolean verbose) {

  //set ChipSelect
  digitalWrite(_cs, LOW);

#ifndef EPSON_V340
  // Send the window command & win ID
  SPItransfer(ADDR_WIN_CTRL | 0x80);
  SPItransfer(winid & 0x01); // mask off unused bits

  // Delay between 16 bit transfers
  EpsonStall();
#endif

  // Send the address
  SPItransfer(addr & 0x7F); // msb is set 0b for register read
  // Send dummy byte
  SPItransfer(0x00);

  // Delay between 16 bit transfers
  EpsonStall();

  // Initiate 16-bit dummy cycle to return data
  uint16_t readData = SPItransfer(0x00)<<8 | SPItransfer(0x00);

  // Release ChipSelect
  digitalWrite(_cs, HIGH);

  // Delay between 16 bit transfers
  EpsonStall();

  // If debug output selected, print information about the transfer
  if (verbose) {
    SerialConsole.print("REG[0x");
    SerialConsole.print((addr&0x7F), HEX);
#ifndef EPSON_V340
    SerialConsole.print(" W(");
    SerialConsole.print(winid, DEC);
    SerialConsole.print(")");
#endif
    SerialConsole.print("] > 0x");
    SerialConsole.println(readData, HEX);
  }
  // Return the data
  return readData;
}





/**************************************************************************/
/*!
    @brief  Goes to SAMPLING Mode

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean SPI_EPSON_COM::sensorStart(void) {

    boolean result = false;

    gotoSamplingMode();
    delayMicroseconds(1000);

    // Check that MODE_STAT bit returns 0, cycles out after 10,000 tries
    for (int32_t i=0; i<10000; i++) {
        uint16_t valRead = regRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO) & VAL_CONFIG_MASK;
        if (valRead == VAL_SAMPLING_MODE){
            result = true;
            break;
        }
    }
  return result;
}


/**************************************************************************/
/*!
    @brief  Places the sensor into CONFIG Mode

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean SPI_EPSON_COM::sensorStop(void) {

    boolean result = false;

    gotoConfigMode();
    delayMicroseconds(1000);

    // Check that MODE_STAT bit returns 0, cycles out after 10,000 tries
    for (int32_t i=0; i<10000; i++) {
        uint16_t valRead = regRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO) & VAL_CONFIG_MASK;
        if (valRead == VAL_CONFIG_MODE){
            result = true;
            break;
        }
    }
  return result;
}


/**************************************************************************/
/*!
    @brief  Enters Config Mode to initiate a selftest and returns the DIAG_STAT
            register value

    @returns    DIAG_STAT return value, 0 on success, nonzero for any errors
                and 0xFFFF if SELF_TEST bit is stuck 1b.
*/
/**************************************************************************/
uint16_t SPI_EPSON_COM::sensorSelfTest(void) {

    uint16_t valRead;

    if (!sensorStop()) {
        SerialConsole.println("Warning: Not entering Config Mode");
    }

    // Send the self test command
    regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_SELFTEST);

    // Wait for self test to process
    EpsonSelfTestDelay();

    // Check that SELF_TEST bit returns 0
    valRead = regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO) & VAL_SELF_TEST_BIT;
    if ( valRead ){
        valRead = 0xFFFF;   // SELF_TEST bit is stuck 1b
    }
    else {
        // Read the results in DIAG_STAT
        valRead = regRead16(CMD_WINDOW0, ADDR_DIAG_STAT) & VAL_DIAG_STAT_MASK;
    }
    return valRead;
}


/**************************************************************************/
/*!
    @brief  Issues a hardware reset and waits for the required delay time.
            Then checks NOT_READY bit. This function can be called at anytime.

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean SPI_EPSON_COM::sensorHWReset(void) {

    boolean result = false;
    // Asserts the nRESET pin LOW
    digitalWrite(_nrst, LOW);
    EpsonResetAssertDelay();

    // Asserts the nRESET pin HIGH
    digitalWrite(_nrst, HIGH);

    // Wait for the sensor re-initialization
    EpsonPowerOnDelay();

    // Check NOT_READY bit = 0
    uint16_t valRead = regRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO) & VAL_NOT_READY;
    if (valRead == 0){
        result = true;
    }

    _burstCnt_calculated = 0;

    return result;
}

/**************************************************************************/
/*!
    @brief  Issues a software reset and waits for the required delay time.
            Then checks NOT_READY bit. This function can be called at anytime.

    @returns    True on success, False on fail
*/
/**************************************************************************/
boolean SPI_EPSON_COM::sensorReset(void) {

    boolean result = false;

    // Set the RESET bit
    regWrite8(CMD_WINDOW1, ADDR_GLOB_CMD_LO, CMD_SOFTRESET);

    // Wait for the sensor software reset to process
    EpsonSwResetDelay();

    // Check NOT_READY bit = 0
    uint16_t valRead = regRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO) & VAL_NOT_READY;
    if (valRead == 0){
        result = true;
    }

    _burstCnt_calculated = 0;

    return result;
}


/**************************************************************************/
/*!
    @brief  Enters Config Mode to initiate a Flash test and returns the
            DIAG_STAT bit 2 result
            NOTE: V340 does not contain internal flash, therefore flash test is
                  not supported for V340

    @returns    FLASH Test return value, true on success, false for any errors
*/
/**************************************************************************/
boolean SPI_EPSON_COM::sensorFlashTest(void) {

    uint16_t valRead;

    if (!sensorStop()) {
        SerialConsole.println("Warning: Not entering Config Mode");
    }

    // Send the self test command
    regWrite8(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_FLASHTEST);

    // Wait for self test to process
    EpsonFlashTestDelay();

    // Check that Flash_TEST bit returns 0
    valRead = regRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO) & VAL_FLASH_STATUS_BIT;
    if (valRead & VAL_FLASH_STATUS_BIT){
        valRead = 0xFFFF;   // SELF_TEST bit is stuck 1b
    }
    else {
        // Read the results in DIAG_STAT
        valRead = regRead16(CMD_WINDOW0, ADDR_DIAG_STAT) & VAL_DIAG_FLASH_ERR;
    }
    return (valRead == VAL_DIAG_FLASH_ERR) ? false : true;
}


/**************************************************************************/
/*!
    @brief  Assumes SU is in Sampling mode
            Checks DRDY is active, then for
            V340: Issues consecutive SPI reads with incrementing address
            All other: Issues a burst read command and generates N x 16-bit
            read cycles to read back one sensor sample set
            If private variable _burstCnt_calculated is nonzero,
            then ignores the len parameter

    @param [out] arr pointer to 16-bit array (stores sensor values)

    @param [in]  len N length of array

    @returns    true if successful or false if there is timeout
                waiting for DRDY assertion

*/
/**************************************************************************/
boolean SPI_EPSON_COM::sensorReadBurst(uint16_t* arr, uint8_t len) {

    if (_burstCnt_calculated)
        len = _burstCnt_calculated>>1;

    if (!waitDataReady(true, 100000)) return false;
#ifdef EPSON_V340
    // Normal sequential 16-bit reads
    readN(arr, CMD_BURST, len);
#else
    // SPI Burst 16-bit reads
    readNB(arr, CMD_BURST, len);
#endif

   return true;
}



/*========================================================================*/
/*                           PRIVATE FUNCTIONS                             */
/*========================================================================*/


/**************************************************************************/
/*!
    @brief  Poll waiting for DataReady pin to go to specified state.
            Will continue to retry until the specified retry count is reached.
            There is a 500usec delay between polling reads.

    @param [in]  polarity
                 false for LOW, true for HIGH
    @param [in]  retryMaxCount
                 Maximum retries checking DRDY before a timeout

    @returns    True on success, False on timeout

*/
/**************************************************************************/
boolean SPI_EPSON_COM::waitDataReady(boolean polarity, uint32_t retryMaxCount) {
  uint32_t retryCount = 0;

  // Loop continuously to check the status of DataReady until Low or timeout
  do {
    retryCount++;
    delayMicroseconds(10);     // 10 usec
  } while ((digitalRead(_drdy)!=polarity) & (retryCount < retryMaxCount));

  // return true on success, or fail for a timeout
  if (retryCount < retryMaxCount)
    return true; // success
  else
    return false; // fail
}



/**************************************************************************/
/*!
    @brief  Returns the Config Mode Status.

    @returns  true on Config Mode, false on Sampling Mode

*/
/**************************************************************************/
boolean SPI_EPSON_COM::isConfigMode(void) {
  // get current mode and mask the bits that we want
  uint16_t modeStatus = regRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO) & VAL_CONFIG_MASK;
  return (modeStatus == VAL_CONFIG_MODE) ? true : false;
}


/**************************************************************************/
/*!
    @brief  Places the sensor into Sampling Mode.
            The sensor must be correctly configured before calling this function.
            This should be only called from Config Mode.
*/
/**************************************************************************/
void SPI_EPSON_COM::gotoSamplingMode(void) {
  regWrite8(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_BEGIN_SAMPLING);
}


/**************************************************************************/
/*!
    @brief  Places the sensor into Configuration Mode.
            This should be only called from Sampling Mode.
*/
/**************************************************************************/
void SPI_EPSON_COM::gotoConfigMode(void) {
  regWrite8(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_END_SAMPLING);
}


/**************************************************************************/
/*!
    @brief  Reads a specified number of Burst 16-bit registers
            starting with specified addr, followed by 0x0000 on subsequent
            cycles

    @param [out]  arrayOut (max 64 elements)
                  Array of return 16-bit values

    @param [in]  addr
                 When DRDY is used, this is the burst command sent to start
                 the transfer
    @param [in]  readLength
                 Specify the length of the burst read transfer in 16-bit words
                 (excluding the header & delimiter bytes)
*/
/**************************************************************************/
void SPI_EPSON_COM::readNB (uint16_t* arrayOut, uint8_t addr, uint8_t readLength) {

  // set ChipSelect
  digitalWrite(_cs, LOW);

  // send the starting address to be read
  SPItransfer(addr); // address
  SPItransfer(00);   // dummy byte

  // delay after 1st command
  EpsonBurstStall1();

  // read the required number of words
  for (int8_t i = 0; i < (readLength); i++) {
    arrayOut[i] = SPItransfer(00)<<8 | SPItransfer(00);
    EpsonBurstStall2();
  }

#ifdef DEBUG
  for (int i = 0; i < readLength; i++) {
    SerialConsole.println(arrayOut[i], HEX);
  }
#endif
  //release ChipSelect
  digitalWrite(_cs, HIGH);
}

/**************************************************************************/
/*!
    @brief  Reads a specified number of sequential 16-bit registers
            starting at the specified address.
            This is for V340 because SPI burst mode is not supported.

    @param [out]  arrayOut
                  Pointer to 16-bit array to store burst data
    @param [in]  addr
                 When DRDY is used, this is the burst command sent to start
                 the transfer
    @param [in]  readLength
                 Specify the length of the burst read transfer in 16-bit words
                 (excluding the header & delimiter bytes)
*/
/**************************************************************************/
void SPI_EPSON_COM::readN (uint16_t* arrayOut, uint8_t addr, uint8_t readLength) {

  // set ChipSelect
  digitalWrite(_cs, LOW);

  // send the starting address to be read
  SPItransfer(addr); // address
  SPItransfer(00);   // dummy byte

  // delay after 1st command
  EpsonStall();

  //Increment the address by 2
  addr = addr + 2;

  // read the required number of words
  for (int8_t i = 0; i < (readLength); i++) {
    arrayOut[i] = SPItransfer(addr)<<8 | SPItransfer(00);
    EpsonStall();
    addr = addr + 2;
  }

#ifdef DEBUG
  for (int i = 0; i < readLength; i++) {
    SerialConsole.println(arrayOut[i], HEX);
  }
#endif
  //release ChipSelect
  digitalWrite(_cs, HIGH);
}


/**************************************************************************/
/*!
    @brief  Assumes Config Mode and Reads the Product ID from the Sensor.

    @param [out]  prodID
                  Pointer to string of 8 ASCII bytes of PROD_ID
                  i.e.
                  Expected ASCII values: "G","3","6","4","P","D","C","0"
                  Return Value: 0x3347, 0x3436, 0x4450, 0x3043
*/
/**************************************************************************/
void SPI_EPSON_COM::getProdID(char* prodID) {

  uint8_t i;

  //read model name from registers, stored as ascii values
  for (i = 0; i < 8; i = i + 2) {
    uint16_t retVal = regRead16(CMD_WINDOW1, ADDR_PROD_ID1 + i);
    prodID[i] = (char) (retVal & 0xFF);
    prodID[i + 1] = (char) (retVal>>8);
#ifdef DEBUG
    // Print Return Values 16-bit
    SerialConsole.print(i, HEX);
    SerialConsole.print("\t");
    SerialConsole.println(retVal, HEX);
#endif
  }
  prodID[i] = 0x00;  // add NULL terminator to make it a string
#ifdef DEBUG
  // Print Return Values 8-bit
  for(int8_t j = 0; j < 9; j++) {
    SerialConsole.println(prodID[j], HEX);
  }
#endif
}

/**************************************************************************/
/*!
    @brief  Assumes Config Mode and Reads the Firmware Version from the Sensor.

    @returns  16-bit Firmware Number
*/
/**************************************************************************/
uint16_t SPI_EPSON_COM::getVersion(void) {

  return regRead16(CMD_WINDOW1, ADDR_VERSION);
}


/**************************************************************************/
/*!
    @brief  Assumes Config Mode and Reads the Serial Number from the Sensor.

    @param [out]  serialNumber
                  Pointer to string of 8 ASCII bytes of Serial Number
                  Each byte is an ASCII number.
*/
/**************************************************************************/
void SPI_EPSON_COM::getSerialNumber(char* serialNumber) {

  uint8_t i;

  //read serial number from registers, stored as ascii values
  for (i = 0; i < 8; i = i + 2) {
    uint16_t retVal = regRead16(CMD_WINDOW1, ADDR_SERIAL_NUM1 + i);
    serialNumber[i] = (char) (retVal & 0xFF);
    serialNumber[i + 1] = (char) (retVal>>8);
  }
  serialNumber[i] = 0x00;  // add NULL terminator to make it a string

}


/**************************************************************************/
/*!
    @brief  Wrapper for SPI transfers. Uses either Hardware SPI or Software SPI
            depending on Constructor call.
    @param [in]  x
                 byte shifted out on MOSI
    @return      byte shifted in on MISO
*/
/**************************************************************************/
uint8_t SPI_EPSON_COM::SPItransfer(uint8_t x) {

  if (_clk == -1) {  //Hardware SPI
#ifdef __SAM3X8E__
    // For Hardware SPI on DUE
    return SPI.transfer(_cs, x);
#else
    // For Hardware SPI on non-DUE
    return SPI.transfer(x);
#endif
  }
  else {  //For Software Emulated SPI
    uint8_t reply = 0;

    for (int i=7; i>=0; i--) {
      reply <<= 1;
      digitalWrite(_clk, LOW);
      digitalWrite(_mosi, x & (1<<i));
      digitalWrite(_clk, HIGH);
      if (digitalRead(_miso))
        reply |= 1;
    }
    return reply;
  }
}

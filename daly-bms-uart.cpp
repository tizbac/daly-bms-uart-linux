#include "daly-bms-uart.h"
#include <stdlib.h>
#include <math.h>
#include <memory.h>
// Uncomment the below define to enable debug printing
//#define DEBUG_SERIAL Serial

//----------------------------------------------------------------------
// Public Functions
//----------------------------------------------------------------------

inline uint8_t bitRead(uint32_t x, uint32_t bit){
    return (x >> bit) & 1;
}

Daly_BMS_UART::Daly_BMS_UART(const std::string& serialDev)
{
    this->my_serialIntf = open(serialDev.c_str(), O_RDWR);
    if (this->my_serialIntf < 0) {
        printf("Error %i from open: %s\n", errno, serialDev.c_str());
    }
    
    
}

bool Daly_BMS_UART::readyRead(bool delayed)
{
    FD_ZERO(&readfd);
    FD_SET(this->my_serialIntf, &readfd);
    timeval tv;
    tv.tv_sec = delayed ? 1 : 0;
    tv.tv_usec = 0;
    int ret = select(this->my_serialIntf+1, &readfd, NULL, NULL, &tv);
    return FD_ISSET(this->my_serialIntf, &readfd);
}


bool Daly_BMS_UART::Init()
{
    // Null check the serial interface
    if (this->my_serialIntf < 0)
    {
        return false;
    }

    // Initialize the serial link to 9600 baud with 8 data bits and no parity bits, per the Daly BMS spec
    struct termios tty;
    tcgetattr(this->my_serialIntf, &tty);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and 
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tcsetattr(this->my_serialIntf, TCSANOW, &tty);
    
    
    // Set up the output buffer with some values that won't be changing
    this->my_txBuffer[0] = 0xA5; // Start byte
    this->my_txBuffer[1] = 0x40; // Host address
    this->my_txBuffer[3] = 0x08; // Length

    // Fill bytes 5-11 with 0s
    for (uint8_t i = 4; i < 12; i++)
    {
        this->my_txBuffer[i] = 0x00;
    }

    return true;
}

bool Daly_BMS_UART::update()
{
    // Call all get___() functions to populate all members of the "get" struct
    if (!getPackMeasurements())
        return false; // 0x90
    if (!getMinMaxCellVoltage())
        return false; // 0x91
    if (!getPackTemp())
        return false; // 0x92
    if (!getDischargeChargeMosStatus())
        return false; // 0x93
    if (!getStatusInfo())
        return false; // 0x94
    if (!getCellVoltages())
        return false; // 0x95
    if (!getCellTemperature())
        return false; // 0x96
    if (!getCellBalanceState())
        return false; // 0x97
    if (!getFailureCodes())
        return false; // 0x98

    return true;
}

bool Daly_BMS_UART::getPackMeasurements() // 0x90
{
    this->sendCommand(COMMAND::VOUT_IOUT_SOC);

    if (!this->receiveBytes())
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<DALY-BMS DEBUG> Receive failed, V, I, & SOC values won't be modified!\n");
#endif
        return false;
    }

    // Pull the relevent values out of the buffer
    get.packVoltage = ((float)((this->my_rxBuffer[4] << 8) | this->my_rxBuffer[5]) / 10.0f);
    // The current measurement is given with a 30000 unit offset (see /docs/)
    get.packCurrent = ((float)(((this->my_rxBuffer[8] << 8) | this->my_rxBuffer[9]) - 30000) / 10.0f);
    get.packSOC = ((float)((this->my_rxBuffer[10] << 8) | this->my_rxBuffer[11]) / 10.0f);
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.println("<DALY-BMS DEBUG> " + (String)get.packVoltage + "V, " + (String)get.packCurrent + "A, " + (String)get.packSOC + "SOC");
#endif

    return true;
}

bool Daly_BMS_UART::getMinMaxCellVoltage() // 0x91
{
    this->sendCommand(COMMAND::MIN_MAX_CELL_VOLTAGE);

    if (!this->receiveBytes())
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<DALY-BMS DEBUG> Receive failed, min/max cell values won't be modified!\n");
#endif
        return false;
    }

    get.maxCellmV = (float)((this->my_rxBuffer[4] << 8) | this->my_rxBuffer[5]);
    get.maxCellVNum = this->my_rxBuffer[6];
    get.minCellmV = (float)((this->my_rxBuffer[7] << 8) | this->my_rxBuffer[8]);
    get.minCellVNum = this->my_rxBuffer[9];
    get.cellDiff = (get.maxCellmV - get.minCellmV);

    return true;
}

bool Daly_BMS_UART::getPackTemp() // 0x92
{
    this->sendCommand(COMMAND::MIN_MAX_TEMPERATURE);

    if (!this->receiveBytes())
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<DALY-BMS DEBUG> Receive failed, Temp values won't be modified!\n");
#endif
        return false;
    }

    // An offset of 40 is added by the BMS to avoid having to deal with negative numbers, see protocol in /docs/
    get.tempMax = (this->my_rxBuffer[4] - 40);
    get.tempMin = (this->my_rxBuffer[6] - 40);
    get.tempAverage = (get.tempMax + get.tempMin) / 2;

    return true;
}

bool Daly_BMS_UART::getDischargeChargeMosStatus() // 0x93
{
    this->sendCommand(COMMAND::DISCHARGE_CHARGE_MOS_STATUS);

    if (!this->receiveBytes())
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<DALY-BMS DEBUG> Receive failed, Charge / discharge mos Status won't be modified!\n");
#endif
        return false;
    }

    switch (this->my_rxBuffer[4])
    {
    case 0:
        get.chargeDischargeStatus = "Stationary";
        break;
    case 1:
        get.chargeDischargeStatus = "Charge";
        break;
    case 2:
        get.chargeDischargeStatus = "Discharge";
        break;
    }

    get.chargeFetState = this->my_rxBuffer[5];
    get.disChargeFetState = this->my_rxBuffer[6];
    get.bmsHeartBeat = this->my_rxBuffer[7];
    get.resCapacitymAh = ((uint32_t)my_rxBuffer[8] << 0x18) | ((uint32_t)my_rxBuffer[9] << 0x10) | ((uint32_t)my_rxBuffer[10] << 0x08) | (uint32_t)my_rxBuffer[11];

    return true;
}

bool Daly_BMS_UART::getStatusInfo() // 0x94
{
    this->sendCommand(COMMAND::STATUS_INFO);

    if (!this->receiveBytes())
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<DALY-BMS DEBUG> Receive failed, Status info won't be modified!\n");
#endif
        return false;
    }

    get.numberOfCells = this->my_rxBuffer[4];
    get.numOfTempSensors = this->my_rxBuffer[5];
    get.chargeState = this->my_rxBuffer[6];
    get.loadState = this->my_rxBuffer[7];

    // Parse the 8 bits into 8 booleans that represent the states of the Digital IO
    for (size_t i = 0; i < 8; i++)
    {
        get.dIO[i] = bitRead(this->my_rxBuffer[8], i);
    }

    get.bmsCycles = ((uint16_t)this->my_rxBuffer[9] << 0x08) | (uint16_t)this->my_rxBuffer[10];

    return true;
}

bool Daly_BMS_UART::getCellVoltages() // 0x95
{
    int cellNo = 0;

    // Check to make sure we have a valid number of cells
    if (get.numberOfCells < MIN_NUMBER_CELLS && get.numberOfCells >= MAX_NUMBER_CELLS)
    {
        return false;
    }

    this->sendCommand(COMMAND::CELL_VOLTAGES);

    for (size_t i = 0; i <= ceil(get.numberOfCells / 3); i++)
    {
        if (!this->receiveBytes())
        {
#ifdef DEBUG_SERIAL
            DEBUG_SERIAL.print("<DALY-BMS DEBUG> Receive failed, Cell Voltages won't be modified!\n");
#endif
            return false;
        }

        for (size_t i = 0; i < 3; i++)
        {

#ifdef DEBUG_SERIAL
            DEBUG_SERIAL.print("<DALY-BMS DEBUG> Frame No.: " + (String)this->my_rxBuffer[4]);
            DEBUG_SERIAL.println(" Cell No: " + (String)(cellNo + 1) + ". " + (String)((this->my_rxBuffer[5 + i + i] << 8) | this->my_rxBuffer[6 + i + i]) + "mV");
#endif

            get.cellVmV[cellNo] = (this->my_rxBuffer[5 + i + i] << 8) | this->my_rxBuffer[6 + i + i];
            cellNo++;
            if (cellNo >= get.numberOfCells)
                break;
        }
    }

    return true;
}

bool Daly_BMS_UART::getCellTemperature() // 0x96
{
    int sensorNo = 0;

    // Check to make sure we have a valid number of temp sensors
    if ((get.numOfTempSensors < MIN_NUMBER_TEMP_SENSORS) && (get.numOfTempSensors >= MAX_NUMBER_TEMP_SENSORS))
    {
        return false;
    }

    this->sendCommand(COMMAND::CELL_TEMPERATURE);

    for (size_t i = 0; i <= ceil(get.numOfTempSensors / 7); i++)
    {

        if (!this->receiveBytes())
        {
#ifdef DEBUG_SERIAL
            DEBUG_SERIAL.print("<DALY-BMS DEBUG> Receive failed, Cell Temperatures won't be modified!\n");
#endif
            return false;
        }

        for (size_t i = 0; i < 7; i++)
        {

#ifdef DEBUG_SERIAL
            DEBUG_SERIAL.print("<DALY-BMS DEBUG> Frame No.: " + (String)this->my_rxBuffer[4]);
            DEBUG_SERIAL.println(" Sensor No: " + (String)(sensorNo + 1) + ". " + String(this->my_rxBuffer[5 + i] - 40) + "°C");
#endif

            get.cellTemperature[sensorNo] = (this->my_rxBuffer[5 + i] - 40);
            sensorNo++;
            if (sensorNo + 1 >= get.numOfTempSensors)
                break;
        }
    }
    return true;
}

bool Daly_BMS_UART::getCellBalanceState() // 0x97
{
    int cellBalance = 0;
    int cellBit = 0;

    // Check to make sure we have a valid number of cells
    if (get.numberOfCells < MIN_NUMBER_CELLS && get.numberOfCells >= MAX_NUMBER_CELLS)
    {
        return false;
    }

    this->sendCommand(COMMAND::CELL_BALANCE_STATE);

    if (!this->receiveBytes())
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("<DALY-BMS DEBUG> Receive failed, Cell Balance State won't be modified!\n");
#endif
        return false;
    }

    // We expect 6 bytes response for this command
    for (size_t i = 0; i < 6; i++)
    {
        // For each bit in the byte, pull out the cell balance state boolean
        for (size_t j = 0; j < 8; j++)
        {
            get.cellBalanceState[cellBit] = bitRead(this->my_rxBuffer[i + 4], j);
            cellBit++;
            if (bitRead(this->my_rxBuffer[i + 4], j))
            {
                cellBalance++;
            }
            if (cellBit >= 47)
            {
                break;
            }
        }
    }

#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("<DALY-BMS DEBUG> Cell Balance State: ");
    for (int i = 0; i < get.numberOfCells; i++)
    {
        DEBUG_SERIAL.print(get.cellBalanceState[i]);
    }
    DEBUG_SERIAL.println();
#endif

    if (cellBalance > 0)
    {
        get.cellBalanceActive = true;
    }
    else
    {
        get.cellBalanceActive = false;
    }

    return true;
}

bool Daly_BMS_UART::getFailureCodes() // 0x98
{
    this->sendCommand(COMMAND::FAILURE_CODES);

    if (!this->receiveBytes())
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<DALY-BMS DEBUG> Receive failed, Failure Flags won't be modified!\n");
#endif
        return false;
    }

    /* 0x00 */
    alarm.levelOneCellVoltageTooHigh = bitRead(this->my_rxBuffer[4], 0);
    alarm.levelTwoCellVoltageTooHigh = bitRead(this->my_rxBuffer[4], 1);
    alarm.levelOneCellVoltageTooLow = bitRead(this->my_rxBuffer[4], 2);
    alarm.levelTwoCellVoltageTooLow = bitRead(this->my_rxBuffer[4], 3);
    alarm.levelOnePackVoltageTooHigh = bitRead(this->my_rxBuffer[4], 4);
    alarm.levelTwoPackVoltageTooHigh = bitRead(this->my_rxBuffer[4], 5);
    alarm.levelOnePackVoltageTooLow = bitRead(this->my_rxBuffer[4], 6);
    alarm.levelTwoPackVoltageTooLow = bitRead(this->my_rxBuffer[4], 7);

    /* 0x01 */
    alarm.levelOneChargeTempTooHigh = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelTwoChargeTempTooHigh = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelOneChargeTempTooLow = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelTwoChargeTempTooLow = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelOneDischargeTempTooHigh = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelTwoDischargeTempTooHigh = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelOneDischargeTempTooLow = bitRead(this->my_rxBuffer[5], 1);
    alarm.levelTwoDischargeTempTooLow = bitRead(this->my_rxBuffer[5], 1);

    /* 0x02 */
    alarm.levelOneChargeCurrentTooHigh = bitRead(this->my_rxBuffer[6], 0);
    alarm.levelTwoChargeCurrentTooHigh = bitRead(this->my_rxBuffer[6], 1);
    alarm.levelOneDischargeCurrentTooHigh = bitRead(this->my_rxBuffer[6], 2);
    alarm.levelTwoDischargeCurrentTooHigh = bitRead(this->my_rxBuffer[6], 3);
    alarm.levelOneStateOfChargeTooHigh = bitRead(this->my_rxBuffer[6], 4);
    alarm.levelTwoStateOfChargeTooHigh = bitRead(this->my_rxBuffer[6], 5);
    alarm.levelOneStateOfChargeTooLow = bitRead(this->my_rxBuffer[6], 6);
    alarm.levelTwoStateOfChargeTooLow = bitRead(this->my_rxBuffer[6], 7);

    /* 0x03 */
    alarm.levelOneCellVoltageDifferenceTooHigh = bitRead(this->my_rxBuffer[7], 0);
    alarm.levelTwoCellVoltageDifferenceTooHigh = bitRead(this->my_rxBuffer[7], 1);
    alarm.levelOneTempSensorDifferenceTooHigh = bitRead(this->my_rxBuffer[7], 2);
    alarm.levelTwoTempSensorDifferenceTooHigh = bitRead(this->my_rxBuffer[7], 3);

    /* 0x04 */
    alarm.chargeFETTemperatureTooHigh = bitRead(this->my_rxBuffer[8], 0);
    alarm.dischargeFETTemperatureTooHigh = bitRead(this->my_rxBuffer[8], 1);
    alarm.failureOfChargeFETTemperatureSensor = bitRead(this->my_rxBuffer[8], 2);
    alarm.failureOfDischargeFETTemperatureSensor = bitRead(this->my_rxBuffer[8], 3);
    alarm.failureOfChargeFETAdhesion = bitRead(this->my_rxBuffer[8], 4);
    alarm.failureOfDischargeFETAdhesion = bitRead(this->my_rxBuffer[8], 5);
    alarm.failureOfChargeFETTBreaker = bitRead(this->my_rxBuffer[8], 6);
    alarm.failureOfDischargeFETBreaker = bitRead(this->my_rxBuffer[8], 7);

    /* 0x05 */
    alarm.failureOfAFEAcquisitionModule = bitRead(this->my_rxBuffer[9], 0);
    alarm.failureOfVoltageSensorModule = bitRead(this->my_rxBuffer[9], 1);
    alarm.failureOfTemperatureSensorModule = bitRead(this->my_rxBuffer[9], 2);
    alarm.failureOfEEPROMStorageModule = bitRead(this->my_rxBuffer[9], 3);
    alarm.failureOfRealtimeClockModule = bitRead(this->my_rxBuffer[9], 4);
    alarm.failureOfPrechargeModule = bitRead(this->my_rxBuffer[9], 5);
    alarm.failureOfVehicleCommunicationModule = bitRead(this->my_rxBuffer[9], 6);
    alarm.failureOfIntranetCommunicationModule = bitRead(this->my_rxBuffer[9], 7);

    /* 0x06 */
    alarm.failureOfCurrentSensorModule = bitRead(this->my_rxBuffer[10], 0);
    alarm.failureOfMainVoltageSensorModule = bitRead(this->my_rxBuffer[10], 1);
    alarm.failureOfShortCircuitProtection = bitRead(this->my_rxBuffer[10], 2);
    alarm.failureOfLowVoltageNoCharging = bitRead(this->my_rxBuffer[10], 3);

    return true;
}

bool Daly_BMS_UART::setDischargeMOS(bool sw) // 0xD9 0x80 First Byte 0x01=ON 0x00=OFF
{
    if (sw)
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("Attempting to switch discharge MOSFETs on");
#endif
        // Set the first byte of the data payload to 1, indicating that we want to switch on the MOSFET
        this->my_txBuffer[4] = 0x01;
        this->sendCommand(COMMAND::DISCHRG_FET);
        // Clear the buffer for further use
        this->my_txBuffer[4] = 0x00;
    }
    else
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("Attempting to switch discharge MOSFETs off");
#endif
        this->sendCommand(COMMAND::DISCHRG_FET);
    }
    if (!this->receiveBytes())
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<DALY-BMS DEBUG> No response from BMS! Can't verify MOSFETs switched.\n");
#endif
        return false;
    }

    return true;
}

bool Daly_BMS_UART::setChargeMOS(bool sw) // 0xDA 0x80 First Byte 0x01=ON 0x00=OFF
{
    if (sw == true)
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("Attempting to switch charge MOSFETs on");
#endif
        // Set the first byte of the data payload to 1, indicating that we want to switch on the MOSFET
        this->my_txBuffer[4] = 0x01;
        this->sendCommand(COMMAND::CHRG_FET);
        // Clear the buffer for further use
        this->my_txBuffer[4] = 0x00;
    }
    else
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("Attempting to switch charge MOSFETs off");
#endif
        this->sendCommand(COMMAND::CHRG_FET);
    }

    if (!this->receiveBytes())
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<DALY-BMS DEBUG> No response from BMS! Can't verify MOSFETs switched.\n");
#endif
        return false;
    }

    return true;
}

bool Daly_BMS_UART::setBmsReset() // 0x00 Reset the BMS
{
    this->sendCommand(COMMAND::BMS_RESET);

    if (!this->receiveBytes())
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<DALY-BMS DEBUG> Send failed, can't verify BMS was reset!\n");
#endif
        return false;
    }

    return true;
}

//----------------------------------------------------------------------
// Private Functions
//----------------------------------------------------------------------

void Daly_BMS_UART::sendCommand(COMMAND cmdID)
{
    while ( readyRead() )
    {
        char discard;
        read(this->my_serialIntf, &discard, 1);
    };

    uint8_t checksum = 0;
    this->my_txBuffer[2] = cmdID;
    // Calculate the checksum
    for (uint8_t i = 0; i <= 11; i++)
    {
        checksum += this->my_txBuffer[i];
    }
    // put it on the frame
    this->my_txBuffer[12] = checksum;

#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("\n<DALY-BMS DEBUG> Send command: 0x");
    DEBUG_SERIAL.print(cmdID, HEX);
    DEBUG_SERIAL.print(" Checksum = 0x");
    DEBUG_SERIAL.println(checksum, HEX);
#endif
    write(this->my_serialIntf, this->my_txBuffer, XFER_BUFFER_LENGTH);
}

bool Daly_BMS_UART::receiveBytes(void)
{
    // Clear out the input buffer
    memset(this->my_rxBuffer, 0, XFER_BUFFER_LENGTH);
    
    while ( !readyRead(true) )
    {
    }
    
    // Read bytes from the specified serial interface
    uint8_t rxByteNum = read(this->my_serialIntf,  this->my_rxBuffer, XFER_BUFFER_LENGTH);

    // Make sure we got the correct number of bytes
    if (rxByteNum != XFER_BUFFER_LENGTH)
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.print("<DALY-BMS DEBUG> Error: Received the wrong number of bytes! Expected 13, got ");
        DEBUG_SERIAL.println(rxByteNum, DEC);
        this->barfRXBuffer();
#endif
        return false;
    }

    if (!validateChecksum())
    {
#ifdef DEBUG_SERIAL
        DEBUG_SERIAL.println("<DALY-BMS DEBUG> Error: Checksum failed!");
        this->barfRXBuffer();
#endif
        return false;
    }
    return true;
}

bool Daly_BMS_UART::validateChecksum()
{
    uint8_t checksum = 0x00;

    for (int i = 0; i < XFER_BUFFER_LENGTH - 1; i++)
    {
        checksum += this->my_rxBuffer[i];
    }

#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("<DALY-BMS DEBUG> Calculated checksum: " + (String)checksum + ", Received: " + (String)this->my_rxBuffer[XFER_BUFFER_LENGTH - 1] + "\n");
#endif

    // Compare the calculated checksum to the real checksum (the last received byte)
    return (checksum == this->my_rxBuffer[XFER_BUFFER_LENGTH - 1]);
}

void Daly_BMS_UART::barfRXBuffer(void)
{
#ifdef DEBUG_SERIAL
    DEBUG_SERIAL.print("<DALY-BMS DEBUG> RX Buffer: [");
    for (int i = 0; i < XFER_BUFFER_LENGTH; i++)
    {
        DEBUG_SERIAL.print(",0x" + (String)this->my_rxBuffer[i]);
    }
    DEBUG_SERIAL.print("]\n");
#endif
}

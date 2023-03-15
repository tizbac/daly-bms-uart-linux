#include <iostream>
#include "daly-bms-uart.h"
int main(int argc, char **argv) {
    Daly_BMS_UART bms(argv[1]);
    bms.Init();
    bms.update();
    std::cout << "basic BMS Data:             " << bms.get.packVoltage << "V" << bms.get.packCurrent << "I" << bms.get.packSOC << "\% " << std::endl;
    std::cout << "Package Temperature:        " << bms.get.tempAverage << std::endl;
    std::cout << "Highest Cell Voltage Nr:    " << bms.get.maxCellVNum << " with voltage" << (bms.get.maxCellmV / 1000) << std::endl;
    std::cout << "Lowest Cell Voltage Nr:     " << bms.get.minCellVNum << " with voltage" << (bms.get.minCellmV / 1000) << std::endl;
    std::cout << "Number of Cells:            " << bms.get.numberOfCells << std::endl;
    std::cout << "Number of Temp Sensors:     " << bms.get.numOfTempSensors << std::endl;
    std::cout << "BMS Chrg / Dischrg Cycles:  " << bms.get.bmsCycles << std::endl;
    std::cout << "BMS Heartbeat:              " << bms.get.bmsHeartBeat << std::endl; // cycle 0-255
    std::cout << "Discharge MOSFet Status:    " << bms.get.disChargeFetState << std::endl;
    std::cout << "Charge MOSFet Status:       " << bms.get.chargeFetState << std::endl;
    std::cout << "Remaining Capacity mAh:     " << bms.get.resCapacitymAh << std::endl;
    return 0;
}

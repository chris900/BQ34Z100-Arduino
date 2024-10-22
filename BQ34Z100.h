/*
   USCRPL
   BQ34Z100-G1 Sensor Driver
 
   Reads the current state of the main battery, can determine charge remaining inside
   as well as read instantaneous voltage, current, or temperature.
 
   Datasheet: http://www.ti.com/lit/ds/symlink/bq34z100-g1.pdf
   Existing arduino code to reference from unknown author: https://github.com/Ralim/BQ34Z100
   Partial library from TI that may be useful: http://www.ti.com/lit/an/slua801/slua801.pdf

   Adapted for Arduino by Chris Shepherd
 */
 
#ifndef BQ34Z100_H
#define BQ34Z100_H
 
//Definitions
#define GAUGE_ADDRESS 0x55 // 0xAA
 
//Battery configuration settings
//Stored in flash: run flashSettings() then reset sensor to save
#define DESIGNCAP 1400 //mAh, per cell (Page 48, offset 11)
#define DESIGNENERGY 5180 //mWh, per cell (Page 48, offset 14)
#define CELLCOUNT 0x03 //number of series cells (Page 65, offset 7)
#define LEDCONFIG 0x4b //5-LED Expander with I2C host comm (Page 64, offset 4)
#define VOLTSEL true //Switches to an external battery voltage divider
#define ZEROCHARGEVOLT 3375 //mV, charge cut-off voltage/Cell terminate voltages
#define FLASH_UPDATE_OK_VOLT 2800 // mV, below this voltage per cell flash writes will not go through
 
#define QMAX0 1400 //mAh, datasheet says to use c-rate current
 
//The voltage divider works by this formula: Gain = (TOP LEG R/BOTTOM LEG R)*1000
//Top leg: 294Kohm and bottom leg: 16.5Kohm
//This only works if you enable the external voltage divider (VOLTSEL) option for the sensor
//Note: requires calibration after setting in flash
#define VOLTAGEGAIN 17818
#define LOADSELECT 0x01 // "Load Select defines the type of power or current model to be used to compute load-compensated capacity in the Impedance Track algorithm"
#define LOADMODE 0x00 // "Load Mode is used to select either the constant current or constant power model for the Impedance Track algorithm"
#define RESETVOLTAGE 22200 //mV, voltage to reset to after unsuccessful voltage calibration
 
//Sense resistor value
#define SENSE_RES 5.0f //mOhms, value of guage sense resistor
 
#define USE_EXTERNAL_THERMISTOR 0 // If 1, use an external thermistor connected to the IT pin.  If 0, use the internal temp sensor.

#include "Arduino.h"

class BQ34Z100
{
    // Top-level commands - the first byte that you can send on an I2C transaction
    enum class Command : uint8_t
    {
        Control = 0x0,
        StateOfCharge = 0x2,
        MaxError = 0x3,
        RemainingCapacity = 0x4,
        FullChargeCapacity = 0x6,
        Voltage = 0x8,
        AverageCurrent = 0xA,
        Temperature = 0xC,
        Flags = 0xE,
        Current = 0x10,
        FlagsB = 0x12,
        // ...
        SerialNumber = 0x28,
        InternalTemperature = 0x2A,
        CycleCount = 0x2C,
        StateOfHealth = 0x2E,
        // ...
        DataFlashClass = 0x3E,
        DataFlashBlock = 0x3F,
        BlockData = 0x40,
        // ...
        BlockDataCheckSum = 0x60,
        BlockDataControl = 0x61
    };
 
    // subcommands for Control command
    enum class Control : uint16_t
    {
        CONTROL_STATUS = 0x0,
        DEVICE_TYPE = 0x1,
        FW_VERSION = 0x02,
        HW_VERSION = 0x03,
        RESET_DATA = 0x5,
        PREV_MACWRITE = 0x7,
        CHEM_ID = 0x8,
        BOARD_OFFSET = 0x9,
        CC_OFFSET = 0xA,
        CC_OFFSET_SAVE = 0xB,
        DF_VERSION = 0xC,
        SET_FULLSLEEP = 0x10,
        STATIC_CHEM_CHECKSUM = 0x17,
        SEALED = 0x20,
        IT_ENABLE = 0x21,
        CAL_ENABLE = 0x2D,
        RESET = 0x41,
        EXIT_CAL = 0x80,
        ENTER_CAL = 0x81,
        OFFSET_CAL = 0x82,
 
        // unseal key, from datasheet p.21
        UNSEAL_KEY1 = 0x0414,
        UNSEAL_KEY2 = 0x3672
    };
 
public:
        /** Create an GQ34Z100-G1 object connected to the specified I2C pins
         *
         * @param sda The I2C data pin.
         * @param scl The I2C clock pin.
         * @param hz The I2C bus frequency (400kHz acc. to datasheet).
         */
        BQ34Z100(int sda, int scl);
 
        //returs status of key features
        uint16_t getStatus();
 
        //Allows use of the entry and exit functions to the CALIBRATION mode
        //Use before enterCal or exitCal()
        void enableCal();
 
        //Enables CALIBRATION mode
        void enterCal();
 
        //Exits CALIBRATION mode
        void exitCal();
 
        //Allows the sensor to begin the Impednance Track algorithm (learns about
        //the battery and begins to find the SOC)
        void ITEnable();
 
        //Returns the predicted remaining battery capacity expressed as a percentage
        //From 0% to 100%
        uint8_t getSOC();
 
        //Returns the expected margin of error in the SOC calculation ranging from
        //1% to 100%. The value is updated continuously internally in increments
        //of 0.05%
        uint16_t getError();
 
        //Estimated remaining battery capacity (1 mAH per bit)
        uint16_t getRemaining();
 
        //Returns the measured battery voltage in mV (0 to 65535 mV)
        uint16_t getVoltage();
 
        //Returns measured current flow through the sense resistor (mA)
        int16_t getCurrent();
 
        //Returns internal sensor temperature in units of celsius
        double getTemperature();
 
        //Returns the current ChemID configured inside the chip.
        // Returns as a hex number, e.g. programming ID 2109 would cause
        // this function to return 8457 = 0x2109
        uint16_t getChemID();
 
        //Returns a percent from 0 to 100 that is the ratio of the predicted FCC over the design capacity
        //FCC = the full charge capacity
        //For example, an old battery will not be able to charge fully to its design capacity
        //This sensor can tell us if we should throw the battery away because its worn out! Cool!
        uint16_t getStateOfHealth();
 
        //Returns the pack serial number programmed in the data flash
        int getSerial();
 
        //Instructs the fuel gauge to perform a full reset. This command is only
        //available when the fuel gauge is UNSEALED
        void reset();
 
        //Unseal some registers, allowing writing (recommended before writing to flash)
        void unseal();
 
        //Reverse unseal command, do not recommend ever using this
        void seal();
 
        //Data flash functions
 
        //Changes to a different register in BlockData()
        void changePage(char subclass, uint16_t offset);
 
        //Reads the checksum and updates it using the flashbytes array
        //Required by the sensor to save changes to the flash
        void updateChecksum();
 
        //Reads the blockData in the current page and saves it to
        //flashbytes array
        void readFlash();
 
        //Writes a specific index in the page to a given value
        //given the length of the property
        void writeFlash(uint8_t index, uint32_t value, int len);
 
        //Returns array (pointer) to flashbytes internal array
        //which stores the last updated page in flash
        uint8_t* getFlashBytes();
 
        //The following functions change different pages
        //The number is the subClass ID that is being changed
        //Each page has different properties that are in those registers
        void changePage48();
        void changePage64();
        void changePage80();
        void changePage82();
 
        //Calibrates the Voltage Divider register (run at least 3 times sequentially)
        // NOTE: Voltage divider changes seem to require a chip reset to take effect.
        // So you must reset the chip between each calibration.
        uint16_t calibrateVoltage(uint16_t currentVoltage);
 
        //If a problem arises with the voltage divider calibration,
        //reset the register with this function
        void setVoltageDivider(uint16_t newVoltage = RESETVOLTAGE);
 
        //Calibrate CCGain and CCOffset registers, which control current shunt
        //and therefore affect the current and voltage readouts
        void calibrateShunt(int16_t calCurrent);
 
        //Set the Chem ID in the flash
        void setChemID();
 
        //Set sense resistor - Calibrate shunt also works, but that requires and external
        //current measurement.
        void setSenseResistor();
 
        // Read the device type.  Should be 0x100 for the BQ34Z100.
        uint16_t readDeviceType();
 
        // reads the contents of the SW and HW version registers.
        uint16_t readFWVersion();
        uint16_t readHWVersion();
 
        // Convert float value to Xemics floating point used on the BQZ.
        // Documented in this app note: http://www.ti.com/lit/pdf/slva148
        static uint32_t floatToXemics(float value);
        static float xemicsToFloat(uint32_t xemics);
 
        // Read the update status register from flash.
        // This gives the status of the Impedance Track algorithm and learning process.
        uint8_t getUpdateStatus();
 
        /**
         * Use the Flags and FlagsB commands to get the contents of the Gas Gauge Status Register.
         * @return
         */
        //std::pair<uint16_t, uint16_t> getFlags();
 
private:
        uint8_t currFlashPage = 0; // current flash page that we have read
        uint8_t currFlashBlockIndex = 0; // 32 bit block index into current flash page
        uint8_t flashbytes[32]; //Stores page in flash memory on Hamster
 
        //Internal commands for writing/reading i2c
 
        // Send a control command and return the result
        void sendControlCommand(Control control);
 
        uint16_t readControlCommand(Control control);
 
        void write(Command command, const uint8_t cmd); //1 byte only
        void write(Command command, const uint8_t cmd1, const uint8_t cmd2); //2 bytes
        uint32_t read(Command command, const uint8_t length);
 
        // Calculate the checksum of the current flashbytes
        uint8_t calcChecksum();
};
#endif

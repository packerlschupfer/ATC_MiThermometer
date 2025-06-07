///////////////////////////////////////////////////////////////////////////////////////////////////
// ATC_MiThermometer.h
//
// Bluetooth low energy thermometer/hygrometer sensor client for MCUs supported by NimBLE-Arduino.
// For sensors running ATC_MiThermometer firmware (see https://github.com/pvvx/ATC_MiThermometer)
//
// https://github.com/matthias-bs/ATC_MiThermometer
//
// Based on:
// ---------
// NimBLE-Arduino by h2zero (https://github.com/h2zero/NimBLE-Arduino)
// LYWSD03MMC.py by JsBergbau (https://github.com/JsBergbau/MiTemperature2)
//
// created: 11/2022
//
//
// MIT License
//
// Copyright (c) 2022 Matthias Prinke
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// History:
//
// 20221123 Created
// 20240403 Added reedSwitchState, gpioTrgOutput, controlParameters,
//          tempTriggerEvent &humiTriggerEvent
// 20240425 Added device name
// 20240426 Added parameter activeScan to begin()
// 20250106 Added whitelist filtering support
// 20250107 Updated for NimBLE v2.x compatibility
// 20250126 Added flexible address type configuration
// 20250127 Added hardware timestamp support for accurate beacon timing
//
// ToDo: 
// -
//
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ATC_MiThermometer_h
#define ATC_MiThermometer_h

#ifdef ATC_MITHERMOMETER_DEBUG
#define LOG_ATC(fmt, ...) log_d("[ATC_MiThermometer] " fmt, ##__VA_ARGS__)
#else
#define LOG_ATC(fmt, ...) ((void)0) // No-op if debugging is disabled
#endif

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>


// MiThermometer data struct / type
struct MiThData_S {
        bool        valid;          //!< data valid
        std::string name;           //!< BT device name
        int16_t     temperature;    //!< temperature x 100°C
        uint16_t    humidity;       //!< humidity x 100%
        uint16_t    batt_voltage;   //!< battery voltage [mv]
        uint8_t     batt_level;     //!< battery level   [%]
        int16_t     rssi;           //!< RSSI [dBm]
        uint8_t     count;        	//!< measurement count
        bool 	    reedSwitchState;
        bool        gpioTrgOutput;
        bool 	    controlParameters;
        bool 	    tempTriggerEvent;
        bool 	    humiTriggerEvent;
        
        // Timing fields for accurate beacon interval tracking
        uint32_t    timestamp_ms;    //!< Software timestamp in milliseconds
        uint64_t    hw_timestamp_us; //!< Hardware timestamp in microseconds (if available)
};

typedef struct MiThData_S MiThData_t; //!< Shortcut for struct MiThData_S

/*!
  \brief BLE address type configuration
*/
enum class AddressType {
    AUTO_DETECT = 0,  //!< Try both types (default, safe option)
    RANDOM_ONLY = 1,  //!< Only add as random address
    PUBLIC_ONLY = 2,  //!< Only add as public address
    BOTH_TYPES = 3    //!< Force add both types
};

/*!
  \class ATC_MiThermometer

  \brief BLE ATC_MiThermometer thermometer/hygrometer sensor client
*/
class ATC_MiThermometer {
    // Forward declaration for friend class
    friend class ScanCallbacks;
    
    public:
        /*!
        \brief Constructor.
        
        \param known_sensors    Vector of BLE MAC addresses of known sensors, e.g. {"11:22:33:44:55:66", "AA:BB:CC:DD:EE:FF"}
        */
        ATC_MiThermometer(std::vector<std::string> known_sensors) {
            _known_sensors = known_sensors;
            data.resize(known_sensors.size());
        };

        /*!
         * \brief Initialization.
         *
         * \param activeScan Set to true for active scan, which uses more power, 
         *                   but get results faster. As a side effect, the device name
         *                   is received (most of the times).
         */
        void begin(bool activeScan = true);
        
        /*!
         * \brief Initialization with whitelist filtering.
         *
         * \param activeScan    Set to true for active scan, which uses more power, 
         *                      but get results faster. As a side effect, the device name
         *                      is received (most of the times).
         * \param useWhitelist  Set to true to enable hardware MAC address filtering
         *                      using the known_sensors list as whitelist
         */
        void beginFiltered(bool activeScan = true, bool useWhitelist = false);
        
        /*!
         * \brief Initialization with whitelist filtering and address type configuration.
         *
         * \param activeScan    Set to true for active scan, which uses more power, 
         *                      but get results faster. As a side effect, the device name
         *                      is received (most of the times).
         * \param useWhitelist  Set to true to enable hardware MAC address filtering
         *                      using the known_sensors list as whitelist
         * \param addrType      Address type configuration (AUTO_DETECT, RANDOM_ONLY, PUBLIC_ONLY, BOTH_TYPES)
         */
        void beginFiltered(bool activeScan, bool useWhitelist, AddressType addrType);
        
        /*!
        \brief Delete results from BLEScan buffer to release memory.
        */        
        void clearScanResults(void) {
            _pBLEScan->clearResults();
        };
        
        /*!
        \brief Get data from sensors by running a BLE scan.
        
        \param duration     Scan duration in milliseconds (changed from seconds in v2.x)
        */                
        unsigned getData(uint32_t duration);
        
        /*!
        \brief Set sensor data invalid.
        */                        
        void resetData(void);
        
        /*!
        \brief Sensor data.
        */
        std::vector<MiThData_t>  data;
        
        /*!
        \brief Check if whitelist filtering is enabled.
        
        \return true if whitelist filtering is active
        */
        bool isWhitelistEnabled(void) const { return _useWhitelist; };
        
        /*!
        \brief Set the address type for sensors
        
        \param type Address type to use (AUTO_DETECT, RANDOM_ONLY, PUBLIC_ONLY, BOTH_TYPES)
        */
        void setAddressType(AddressType type) { _addressType = type; };
        
        /*!
        \brief Get the current address type configuration
        
        \return Current address type setting
        */
        AddressType getAddressType(void) const { return _addressType; };
        
        /*!
        \brief Detect address type for the first configured sensor (debug function)
        
        \note This function performs test scans to determine the actual address type
        */
        void detectAddressType(void);

        /*!
        * \brief Get the precise discovery timestamp for a device
        * 
        * \param index The index of the device in the known_sensors list
        * \return The timestamp in milliseconds when the advertisement was discovered, or 0 if not found
        */
        uint32_t getDeviceDiscoveryTime(size_t index);
        
        /*!
        \brief Get the hardware timestamp for when a device packet was received.
        
        \param index    Index in the known_sensors array
        \return         Timestamp in microseconds, or 0 if not found
        */
        uint64_t getDeviceHardwareTimestamp(size_t index);
        
        /*!
        \brief Check if hardware timestamps are enabled.
        
        \return         true if CONFIG_NIMBLE_CPP_ATT_VALUE_HRTIMESTAMP_ENABLED is set
        */
        static bool isHardwareTimestampEnabled() {
            #if CONFIG_NIMBLE_CPP_ATT_VALUE_HRTIMESTAMP_ENABLED
                return true;
            #else
                return false;
            #endif
        }

    protected:
        std::vector<std::string> _known_sensors;
        NimBLEScan*              _pBLEScan;
        bool                     _useWhitelist = false;
        AddressType              _addressType = AddressType::AUTO_DETECT;
        
        /*!
        \brief Configure BLE whitelist with known sensor addresses.
        
        \note This is now a no-op as filtering is handled in beginFiltered()
        */
        void configureWhitelist(void);
};
#endif
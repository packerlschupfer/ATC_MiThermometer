///////////////////////////////////////////////////////////////////////////////////////////////////
// ATC_MiThermometer.cpp
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
// 20221223 Added support for ATC1441 format
// 20240403 Added reedSwitchState, gpioTrgOutput, controlParameters,
//          tempTriggerEvent &humiTriggerEvent
// 20240425 Added device name
// 20250125 Updated for NimBLE-Arduino v2.x
// 20250126 Added AddressType support
// 20250127 Added precise timestamp tracking with hardware timestamp support
//
// ToDo:
// -
//
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <ATC_MiThermometer.h>

/*!
 * \class ScanCallbacks
 *
 * \brief Enhanced ScanCallbacks to capture precise timestamps with hardware support
 */
class ScanCallbacks : public NimBLEScanCallbacks
{
private:
    ATC_MiThermometer* _parent = nullptr;
    
    // Store precise timestamps for each device
    struct DeviceTimestamp {
        std::string address;
        uint32_t discoveryTime;      // Software timestamp (millis)
        uint64_t hwTimestamp;        // Hardware timestamp (microseconds)
        uint32_t resultTime;         // When result callback was called
        int16_t rssi;
        uint8_t count;               // Beacon count for duplicate detection
        bool hasData;
    };
    std::vector<DeviceTimestamp> _timestamps;
    
public:
    ScanCallbacks(ATC_MiThermometer* parent) : _parent(parent) {}
    
    void clearTimestamps() {
        _timestamps.clear();
    }
    
    uint32_t getDiscoveryTime(const std::string& address) {
        for (const auto& dt : _timestamps) {
            if (dt.address == address) {
                return dt.discoveryTime;
            }
        }
        return 0;
    }
    
    uint64_t getHardwareTimestamp(const std::string& address) {
        for (const auto& dt : _timestamps) {
            if (dt.address == address) {
                return dt.hwTimestamp;
            }
        }
        return 0;
    }
    
    void onDiscovered(const NimBLEAdvertisedDevice* advertisedDevice) override
    {
        // Get timestamps immediately
        uint32_t swTimestamp = millis();
        uint64_t hwTimestamp = 0;
        
        #if CONFIG_NIMBLE_CPP_ATT_VALUE_HRTIMESTAMP_ENABLED
            // Get hardware timestamp from the advertised device
            hwTimestamp = advertisedDevice->getHrTimestamp();
            if (hwTimestamp == 0) {
                // If no hardware timestamp in the device, use current time
                hwTimestamp = esp_timer_get_time();
            }
            LOG_ATC("DISCOVERED %s at SW: %u ms, HW: %llu us (%.2f ms)", 
                    advertisedDevice->getAddress().toString().c_str(),
                    swTimestamp, hwTimestamp, hwTimestamp / 1000.0);
        #else
            // Fallback to software timestamp
            hwTimestamp = swTimestamp * 1000ULL;  // Convert to microseconds
            LOG_ATC("DISCOVERED %s at %u ms (HW timestamps disabled)", 
                    advertisedDevice->getAddress().toString().c_str(), swTimestamp);
        #endif
        
        std::string addr = advertisedDevice->getAddress().toString();
        
        // Check if whitelisted (if whitelist enabled)
        if (_parent && _parent->isWhitelistEnabled()) {
            bool found = false;
            for (const auto& knownAddr : _parent->_known_sensors) {
                if (addr == knownAddr) {
                    found = true;
                    break;
                }
            }
            if (!found) return;
        }
        
        // Store or update the timestamp
        bool found = false;
        for (auto& dt : _timestamps) {
            if (dt.address == addr) {
                // Check if this is a new beacon based on hardware timestamp
                if (hwTimestamp > 0 && dt.hwTimestamp > 0) {
                    uint64_t interval = hwTimestamp - dt.hwTimestamp;
                    if (interval > 1000000) {  // More than 1 second = new beacon
                        LOG_ATC("New beacon detected - interval: %.2f ms", interval / 1000.0);
                        dt.discoveryTime = swTimestamp;
                        dt.hwTimestamp = hwTimestamp;
                        dt.rssi = advertisedDevice->getRSSI();
                        dt.hasData = false;
                    } else {
                        LOG_ATC("Duplicate beacon - interval: %.2f ms", interval / 1000.0);
                    }
                } else {
                    dt.discoveryTime = swTimestamp;
                    dt.hwTimestamp = hwTimestamp;
                    dt.rssi = advertisedDevice->getRSSI();
                    dt.hasData = false;
                }
                found = true;
                break;
            }
        }
        
        if (!found) {
            _timestamps.push_back({
                addr, 
                swTimestamp, 
                hwTimestamp,
                0,
                advertisedDevice->getRSSI(),
                0,
                false
            });
        }
        
        LOG_ATC("Discovered Advertised Device: %s", advertisedDevice->toString().c_str());
    }

    void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override
    {
        // This is called later when full data is available
        uint32_t resultTime = millis();
        std::string addr = advertisedDevice->getAddress().toString();
        
        // Update result time
        for (auto& dt : _timestamps) {
            if (dt.address == addr) {
                dt.resultTime = resultTime;
                dt.hasData = true;
                
                // Extract beacon count if available
                if (advertisedDevice->haveServiceData()) {
                    std::string serviceData = advertisedDevice->getServiceData();
                    if (serviceData.length() >= 14) {
                        dt.count = serviceData[13];  // Count byte
                    }
                }
                
                #ifdef ATC_MITHERMOMETER_DEBUG
                LOG_ATC("RESULT for %s - Processing delay: %u ms, Count: %u, RSSI: %d dBm", 
                                        addr.c_str(), resultTime - dt.discoveryTime, dt.count, dt.rssi);
                #endif
                break;
            }
        }
        
        log_d("Advertised Device: %s", advertisedDevice->toString().c_str());
        
        // Add to whitelist if it has our service data
        if (advertisedDevice->haveServiceData())
        {
            /* If this is a device with data we want to capture, add it to the whitelist */
            if (advertisedDevice->getServiceData(NimBLEUUID("181A")) != "")
            {
                log_d("Adding %s to whitelist", addr.c_str());
                NimBLEDevice::whiteListAdd(advertisedDevice->getAddress());
            }
        }
    }

    void onScanEnd(const NimBLEScanResults &results, int reason) override
    {
        #ifdef ATC_MITHERMOMETER_DEBUG
        LOG_ATC("Scan ended at %u ms; reason = %d", millis(), reason);
        #endif
        
        // Log timing summary
        #if CONFIG_NIMBLE_CPP_ATT_VALUE_HRTIMESTAMP_ENABLED
            LOG_ATC("=== Hardware Timestamp Analysis ===");
            analyzeTimestamps();
        #else
            LOG_ATC("=== Software Timestamp Summary ===");
            for (const auto& dt : _timestamps) {
                if (dt.hasData) {
                    LOG_ATC("Device %s: discovered at %u ms, result at %u ms (delay: %u ms), RSSI %d dBm, Count %u",
                            dt.address.c_str(), 
                            dt.discoveryTime, 
                            dt.resultTime,
                            dt.resultTime - dt.discoveryTime,
                            dt.rssi,
                            dt.count);
                } else {
                    LOG_ATC("Device %s: discovered at %u ms but no result data",
                            dt.address.c_str(), dt.discoveryTime);
                }
            }
        #endif
    }
    
    void analyzeTimestamps() {
        // Group timestamps by device
        std::map<std::string, std::vector<uint64_t>> deviceTimestamps;
        
        for (const auto& dt : _timestamps) {
            if (dt.hwTimestamp > 0) {
                deviceTimestamps[dt.address].push_back(dt.hwTimestamp);
            }
        }
        
        // Analyze intervals for each device
        for (const auto& [addr, timestamps] : deviceTimestamps) {
            if (timestamps.size() < 2) continue;
            
            LOG_ATC("Device %s - %zu detections:", addr.c_str(), timestamps.size());
            
            std::vector<uint64_t> sortedTimes = timestamps;
            std::sort(sortedTimes.begin(), sortedTimes.end());
            
            uint64_t sumIntervals = 0;
            uint64_t minInterval = UINT64_MAX;
            uint64_t maxInterval = 0;
            int intervalCount = 0;
            
            for (size_t i = 1; i < sortedTimes.size(); i++) {
                uint64_t interval = sortedTimes[i] - sortedTimes[i-1];
                
                // Only count intervals > 1 second (new beacons)
                if (interval > 1000000) {
                    #ifdef ATC_MITHERMOMETER_DEBUG
                    LOG_ATC("  Interval %zu: %.2f ms", i, interval / 1000.0);
                    #endif
                    
                    sumIntervals += interval;
                    minInterval = std::min(minInterval, interval);
                    maxInterval = std::max(maxInterval, interval);
                    intervalCount++;
                }
            }
            
            if (intervalCount > 0) {
                LOG_ATC("  Average interval: %.2f ms (expected: 2500 ms)", (sumIntervals / intervalCount) / 1000.0);
                LOG_ATC("  Min: %.2f ms, Max: %.2f ms", minInterval / 1000.0, maxInterval / 1000.0);
                LOG_ATC("  Jitter: %.2f ms", (maxInterval - minInterval) / 1000.0);
                LOG_ATC("  Accuracy: %.1f%%", ((sumIntervals / intervalCount) / 1000.0 / 2500.0) * 100.0);
            }
        }
    }
    
    // Friend access for parent class
    friend class ATC_MiThermometer;
};

// Static instance - declare outside the class
static ScanCallbacks* scanCallbacksInstance = nullptr;

// Original begin method - maintains backward compatibility
void ATC_MiThermometer::begin(bool activeScan)
{
    beginFiltered(activeScan, false);  // Call new method with whitelist disabled
}

// beginFiltered method with whitelist support (overload without AddressType)
void ATC_MiThermometer::beginFiltered(bool activeScan, bool useWhitelist)
{
    // Call the full method with default address type
    beginFiltered(activeScan, useWhitelist, AddressType::AUTO_DETECT);
}

// Full beginFiltered method with whitelist and address type support
void ATC_MiThermometer::beginFiltered(bool activeScan, bool useWhitelist, AddressType addrType) {
    if (!NimBLEDevice::isInitialized()) {
        NimBLEDevice::init("ble-scan");
    }
    _pBLEScan = NimBLEDevice::getScan();  // create new scan
    
    // Create callbacks instance with reference to this object
    if (scanCallbacksInstance != nullptr) {
        delete scanCallbacksInstance;
    }
    scanCallbacksInstance = new ScanCallbacks(this);
    
    _pBLEScan->setScanCallbacks(scanCallbacksInstance);
    _pBLEScan->setActiveScan(activeScan); //active scan uses more power, but get results faster
    
    _useWhitelist = useWhitelist;
    _addressType = addrType;
    
    // Log timestamp configuration
    #if CONFIG_NIMBLE_CPP_ATT_VALUE_HRTIMESTAMP_ENABLED
        log_i("Hardware timestamps ENABLED - beacon timing will be accurate");
    #else
        log_w("Hardware timestamps DISABLED - enable CONFIG_NIMBLE_CPP_ATT_VALUE_HRTIMESTAMP_ENABLED for accurate timing");
    #endif
    
    if (_useWhitelist && !_known_sensors.empty()) {
        // Clear existing whitelist by removing all entries
        size_t currentCount = NimBLEDevice::getWhiteListCount();
        if (currentCount > 0) {
            log_i("Clearing %d existing whitelist entries", currentCount);
            // Remove each address from whitelist
            while (NimBLEDevice::getWhiteListCount() > 0) {
                NimBLEAddress addr = NimBLEDevice::getWhiteListAddress(0);
                NimBLEDevice::whiteListRemove(addr);
            }
        }
        
        // Add all known sensors to hardware whitelist
        size_t addedCount = 0;
        size_t devicesAdded = 0;
        
        for (const auto& sensorAddr : _known_sensors) {
            bool addedRandom = false;
            bool addedPublic = false;
            
            switch (_addressType) {
                case AddressType::AUTO_DETECT:
                case AddressType::BOTH_TYPES:
                    // Try both address types
                    {
                        NimBLEAddress randomAddr(sensorAddr.c_str(), BLE_ADDR_RANDOM);
                        if (NimBLEDevice::whiteListAdd(randomAddr)) {
                            addedRandom = true;
                            addedCount++;
                        }
                        
                        NimBLEAddress publicAddr(sensorAddr.c_str(), BLE_ADDR_PUBLIC);
                        if (NimBLEDevice::whiteListAdd(publicAddr)) {
                            addedPublic = true;
                            addedCount++;
                        }
                    }
                    break;
                    
                case AddressType::RANDOM_ONLY:
                    // Only add as random address
                    {
                        NimBLEAddress randomAddr(sensorAddr.c_str(), BLE_ADDR_RANDOM);
                        if (NimBLEDevice::whiteListAdd(randomAddr)) {
                            addedRandom = true;
                            addedCount++;
                        }
                    }
                    break;
                    
                case AddressType::PUBLIC_ONLY:
                    // Only add as public address
                    {
                        NimBLEAddress publicAddr(sensorAddr.c_str(), BLE_ADDR_PUBLIC);
                        if (NimBLEDevice::whiteListAdd(publicAddr)) {
                            addedPublic = true;
                            addedCount++;
                        }
                    }
                    break;
            }

            // Log results based on address type configuration
            if (addedRandom || addedPublic) {
                devicesAdded++;
                #ifdef ATC_MITHERMOMETER_DEBUG
                if (addedRandom && addedPublic) {
                    log_i("Added %s to whitelist (both types)", sensorAddr.c_str());
                } else if (addedRandom) {
                    log_i("Added %s to whitelist (random type detected)", sensorAddr.c_str());
                } else {
                    log_i("Added %s to whitelist (public type detected)", sensorAddr.c_str());
                }
                #else
                log_i("Added %s to whitelist", sensorAddr.c_str());
                #endif
            } else {
                #ifdef ATC_MITHERMOMETER_DEBUG
                switch (_addressType) {
                    case AddressType::AUTO_DETECT:
                    case AddressType::BOTH_TYPES:
                        log_e("Failed to add %s to whitelist with neither type", sensorAddr.c_str());
                        break;
                    case AddressType::RANDOM_ONLY:
                        log_e("Failed to add %s to whitelist with random type", sensorAddr.c_str());
                        break;
                    case AddressType::PUBLIC_ONLY:
                        log_e("Failed to add %s to whitelist with public type", sensorAddr.c_str());
                        break;
                }
                #else
                log_e("Failed to add %s to whitelist", sensorAddr.c_str());
                #endif
            }
        }
            
        if (devicesAdded > 0) {
            log_i("BLE scan initialized with hardware whitelist filtering");
            log_i("Whitelist contains %d entries for %d devices", 
                  NimBLEDevice::getWhiteListCount(), devicesAdded);
            log_i("Only whitelisted devices will be processed, protecting against BLE flooding");

            // Log efficiency info
            log_d("Hardware filtering active - non-whitelisted devices filtered at controller level");
            log_d("This prevents scan buffer overflow from high-traffic BLE environments");
        } else {
            log_w("No devices could be added to whitelist, falling back to normal scanning");
            _useWhitelist = false;
        }
    } else {
        if (_useWhitelist && _known_sensors.empty()) {
            log_w("Whitelist requested but no sensors configured");
            _useWhitelist = false;
        }
        log_i("BLE scan initialized without filtering");
    }

    // Configure scan parameters based on whether whitelist is active
    if (_useWhitelist) {
        // Use optimized parameters from ProjectConfig.h when whitelist filtering is active
        #ifdef BLE_FILTERED_SCAN_INTERVAL
            _pBLEScan->setInterval(BLE_FILTERED_SCAN_INTERVAL);
            _pBLEScan->setWindow(BLE_FILTERED_SCAN_WINDOW);
            
            #ifdef ATC_MITHERMOMETER_DEBUG
            LOG_ATC("Using filtered scan parameters: interval=%u, window=%u (%.0f%% duty cycle)",
                    BLE_FILTERED_SCAN_INTERVAL, BLE_FILTERED_SCAN_WINDOW,
                    (float)BLE_FILTERED_SCAN_WINDOW / BLE_FILTERED_SCAN_INTERVAL * 100.0f);
            #endif
        #else
            // Fallback to continuous scanning for reliability
            _pBLEScan->setInterval(80);   // 50ms interval
            _pBLEScan->setWindow(80);     // 50ms window (100% duty cycle)
            
            #ifdef ATC_MITHERMOMETER_DEBUG
            LOG_ATC("Using default continuous scan (100%% duty cycle)");
            #endif
        #endif
    } else {
        // Without whitelist, use less aggressive scanning to avoid flooding
        #ifdef BLE_SCAN_INTERVAL_MS
            // Convert milliseconds to BLE units (0.625ms per unit)
            uint16_t interval = BLE_SCAN_INTERVAL_MS * 1000 / 625;
            uint16_t window = BLE_SCAN_DURATION_MS * 1000 / 625;
            
            // Ensure window doesn't exceed interval
            if (window > interval) {
                window = interval;
            }
            
            _pBLEScan->setInterval(interval);
            _pBLEScan->setWindow(window);
            
            #ifdef ATC_MITHERMOMETER_DEBUG
            LOG_ATC("Using non-filtered scan parameters: interval=%u, window=%u (%.0f%% duty cycle)",
                    interval, window, (float)window / interval * 100.0f);
            #endif
        #else
            // Conservative defaults for non-filtered scanning
            _pBLEScan->setInterval(160);  // 100ms interval
            _pBLEScan->setWindow(80);     // 50ms window (50% duty cycle)
            
            #ifdef ATC_MITHERMOMETER_DEBUG
            LOG_ATC("Using default non-filtered scan (50%% duty cycle)");
            #endif
        #endif
    }
    
    // Always need duplicates for PVVX timing tracking
    _pBLEScan->setFilterPolicy(BLE_HCI_SCAN_FILT_USE_WL);
    _pBLEScan->setDuplicateFilter(0);
    _pBLEScan->setMaxResults(0xFF);  // Store up to 255 results
}

// Add method to get precise discovery timestamp
uint32_t ATC_MiThermometer::getDeviceDiscoveryTime(size_t index) {
    if (scanCallbacksInstance && index < _known_sensors.size()) {
        return scanCallbacksInstance->getDiscoveryTime(_known_sensors[index]);
    }
    return 0;
}

// Get hardware timestamp in microseconds
uint64_t ATC_MiThermometer::getDeviceHardwareTimestamp(size_t index) {
    if (scanCallbacksInstance && index < _known_sensors.size()) {
        return scanCallbacksInstance->getHardwareTimestamp(_known_sensors[index]);
    }
    return 0;
}

// Configure whitelist is now a no-op as filtering is done in callbacks
void ATC_MiThermometer::configureWhitelist(void)
{
    // In NimBLE v2.x, whitelist filtering is handled manually in the scan callbacks
    // This function is kept for API compatibility but doesn't need to do anything
    if (_useWhitelist) {
        log_i("Whitelist filtering enabled for %d addresses", _known_sensors.size());
    }
}

// Detect address type for debug purposes
void ATC_MiThermometer::detectAddressType(void)
{
    if (_known_sensors.empty()) {
        LOG_ATC("No sensors configured to detect address type");
        return;
    }
    
    LOG_ATC("Detecting address type for sensor: %s", _known_sensors[0].c_str());

    // Test with random address
    {
        LOG_ATC("Testing RANDOM address type...");
        beginFiltered(false, true, AddressType::RANDOM_ONLY);
        getData(5000);  // <-- Just call without storing the result
        if (data[0].valid) {
            LOG_ATC("✓ Sensor responds to RANDOM address type");
        } else {
            LOG_ATC("✗ Sensor does NOT respond to RANDOM address type");
        }
        resetData();
    }
    
    delay(1000);
    
    // Test with public address
    {
        LOG_ATC("Testing PUBLIC address type...");
        beginFiltered(false, true, AddressType::PUBLIC_ONLY);
        getData(5000);
        if (data[0].valid) {
            LOG_ATC("✓ Sensor responds to PUBLIC address type");
        } else {
            LOG_ATC("✗ Sensor does NOT respond to PUBLIC address type");
        }
        resetData();
    }
   
    LOG_ATC("Detection complete. Use the address type that worked.");
}

// Get sensor data by running BLE device scan
unsigned ATC_MiThermometer::getData(uint32_t scanTime)
{
    // Clear previous timestamps
    if (scanCallbacksInstance) {
        scanCallbacksInstance->clearTimestamps();
    }
    

    // Configure scan parameters before each scan
    if (_useWhitelist && !_known_sensors.empty()) {
        // When using hardware whitelist, set the filter policy
        _pBLEScan->setFilterPolicy(BLE_HCI_SCAN_FILT_USE_WL);
        LOG_ATC("Hardware whitelist filtering active");
        
        // Log whitelist status
        #ifdef ATC_MITHERMOMETER_DEBUG
        size_t wlCount = NimBLEDevice::getWhiteListCount();
        LOG_ATC("Whitelist contains %d entries", wlCount);
        
        // Log whitelist contents (for debugging)
        for (size_t i = 0; i < wlCount && i < 5; i++) {  // Limit to 5 to avoid spam
            LOG_ATC("  Whitelist[%d]: %s", i, NimBLEDevice::getWhiteListAddress(i).toString().c_str());
        }
        #else
        LOG_ATC("Whitelist contains %d entries", NimBLEDevice::getWhiteListCount());
        #endif
    } else {
        // No filtering
        _pBLEScan->setFilterPolicy(BLE_HCI_SCAN_FILT_NO_WL);
    }
    
    // Start scanning
    // NimBLE v2.x uses getResults with time in milliseconds
    NimBLEScanResults foundDevices = _pBLEScan->getResults(scanTime, false);

    LOG_ATC("Found %d devices", foundDevices.getCount());
    if (_useWhitelist) {
        LOG_ATC("Whitelist filtering enabled for %d addresses", _known_sensors.size());
        
        // If we found 0 devices with whitelist, there might be an issue
        if (foundDevices.getCount() == 0) {
            LOG_ATC("WARNING: No devices found with whitelist filtering!");
            LOG_ATC("Target sensor: %s", _known_sensors.empty() ? "NONE" : _known_sensors[0].c_str());
            
            // Check if sensor is on whitelist
            if (!_known_sensors.empty()) {
                // Check based on configured address type
                bool onWhitelist = false;
                switch (_addressType) {
                    case AddressType::AUTO_DETECT:
                    case AddressType::BOTH_TYPES:
                        {
                            NimBLEAddress randomAddr(_known_sensors[0], BLE_ADDR_RANDOM);
                            NimBLEAddress publicAddr(_known_sensors[0], BLE_ADDR_PUBLIC);
                            onWhitelist = NimBLEDevice::onWhiteList(randomAddr) || 
                                         NimBLEDevice::onWhiteList(publicAddr);
                        }
                        break;
                    case AddressType::RANDOM_ONLY:
                        {
                            NimBLEAddress randomAddr(_known_sensors[0], BLE_ADDR_RANDOM);
                            onWhitelist = NimBLEDevice::onWhiteList(randomAddr);
                        }
                        break;
                    case AddressType::PUBLIC_ONLY:
                        {
                            NimBLEAddress publicAddr(_known_sensors[0], BLE_ADDR_PUBLIC);
                            onWhitelist = NimBLEDevice::onWhiteList(publicAddr);
                        }
                        break;
                }
                
                if (onWhitelist) {
                    LOG_ATC("Target IS on whitelist");
                } else {
                    LOG_ATC("ERROR: Target is NOT on whitelist!");
                }
            }
        }
    }
  
    LOG_ATC("Assigning scan results...");
    for (unsigned i=0; i<foundDevices.getCount(); i++) {
        const NimBLEAdvertisedDevice* device = foundDevices.getDevice(i);
        if (!device) continue;  // Safety check
        
        LOG_ATC("haveName(): %d", device->haveName());
        LOG_ATC("getName(): %s", device->getName().c_str());
        
        // Match all devices found against list of known sensors
        for (unsigned n = 0; n < _known_sensors.size(); n++)
        {
            LOG_ATC("Found: %s  comparing to: %s",
                  device->getAddress().toString().c_str(),
                  _known_sensors[n].c_str());
            
            if (device->getAddress().toString() == _known_sensors[n])
            {
                LOG_ATC(" -> Match! Index: %d", n);
                data[n].valid = true;
                
                // Check if device has service data
                if (!device->haveServiceData()) {
                    LOG_ATC("No service data available");
                    continue;
                }
                
                // In NimBLE v2.x, getServiceData returns std::string
                std::string serviceDataStr = device->getServiceData();
                int len = serviceDataStr.length();
                LOG_ATC("Length of ServiceData: %d", len);
                
                // Convert to unsigned char array for easier access
                const unsigned char* serviceData = reinterpret_cast<const unsigned char*>(serviceDataStr.c_str());
 
                data[n].name = device->getName();
                if (len == 15)
                {
                    LOG_ATC("Custom format");
                    // Temperature
                    int temp_msb = serviceData[7];
                    int temp_lsb = serviceData[6];
                    data[n].temperature = (temp_msb << 8) | temp_lsb;

                    // Humidity
                    int hum_msb = serviceData[9];
                    int hum_lsb = serviceData[8];
                    data[n].humidity = (hum_msb << 8) | hum_lsb;

                    // Battery voltage
                    int volt_msb = serviceData[11];
                    int volt_lsb = serviceData[10];
                    data[n].batt_voltage = (volt_msb << 8) | volt_lsb;

                    // Battery state [%]
                    data[n].batt_level = serviceData[12];

                    // Count
                    data[n].count = serviceData[13];

                    // Flags
                    uint8_t flagsByte = serviceData[14];
                    data[n].reedSwitchState = flagsByte & 0x01;          // Extract bit 0 (Reed Switch)
                    data[n].gpioTrgOutput = (flagsByte >> 1) & 0x01;     // Extract bit 1 (GPIO_TRG pin output)
                    data[n].controlParameters = (flagsByte >> 2) & 0x01; // Extract bit 2 (Control parameters)
                    data[n].tempTriggerEvent = (flagsByte >> 3) & 0x01;  // Extract bit 3 (Temperature trigger event)
                    data[n].humiTriggerEvent = (flagsByte >> 4) & 0x01;  // Extract bit 4 (Humidity trigger event)
                }
                else if (len == 13)
                {
                    LOG_ATC("ATC1441 format");

                    // Temperature
                    int temp_lsb = serviceData[7];
                    int temp_msb = serviceData[6];
                    data[n].temperature = (temp_msb << 8) | temp_lsb;
                    data[n].temperature *= 10;

                    // Humidity
                    data[n].humidity = serviceData[8];
                    data[n].humidity *= 100;

                    // Battery voltage
                    int volt_lsb = serviceData[11];
                    int volt_msb = serviceData[10];
                    data[n].batt_voltage = (volt_msb << 8) | volt_lsb;

                    // Battery state [%]
                    data[n].batt_level = serviceData[9];
                }
                else
                {
                    LOG_ATC("Unknown ServiceData format");
                }

                // Received Signal Strength Indicator [dBm]
                data[n].rssi = device->getRSSI();
                
                // Get timestamps
                data[n].timestamp_ms = getDeviceDiscoveryTime(n);
                data[n].hw_timestamp_us = getDeviceHardwareTimestamp(n);
                
                LOG_ATC("Device %d timestamps: sw=%u ms, hw=%llu us", 
                       n, data[n].timestamp_ms, data[n].hw_timestamp_us);
                
                // Additional debug: check if timestamps are reasonable
                if (data[n].hw_timestamp_us > 0) {
                    LOG_ATC("Hardware timestamp in seconds: %.3f", data[n].hw_timestamp_us / 1000000.0);
                } else {
                    LOG_ATC("WARNING: Hardware timestamp is 0!");
                }
                
                // Debug: Also get the device's own timestamp if available
                #if CONFIG_NIMBLE_CPP_ATT_VALUE_TIMESTAMP_ENABLED
                    time_t deviceTime = device->getTimestamp();
                    LOG_ATC("Device timestamp (time_t): %ld", deviceTime);
                #endif

                #if CONFIG_NIMBLE_CPP_ATT_VALUE_HRTIMESTAMP_ENABLED
                    LOG_ATC("Device HR timestamp: %llu us (%.3f ms)", 
                            device->getHrTimestamp(), device->getHrTimestamp() / 1000.0);
                #endif
            }
            else
            {
                LOG_ATC();
            }
        }
    }
    return foundDevices.getCount();
}

// Set all array members invalid
void ATC_MiThermometer::resetData(void)
{
    for (int i = 0; i < _known_sensors.size(); i++)
    {
        data[i].valid = false;
    }
}
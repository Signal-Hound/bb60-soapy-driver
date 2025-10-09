#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <mutex>

#include "bb_api.h"

#define BB60_CLOCK 40e6

/***********************************************************************
 * Device interface
 **********************************************************************/
class SignalHoundBB60 : public SoapySDR::Device {
private:
    // Variables used
    mutable std::mutex devMutex;
    bool streamActive, serialSpecified;
    int deviceId, serial, type, decimation, numDevices; 
    int serials[BB_MAX_DEVICES], types[BB_MAX_DEVICES];
    unsigned int port1, port2;
    double sampleRate, centerFrequency, bandwidth, rfGain, attenLevel;
    bbStatus status;
    std::string dataFormat;
    // Decimation to max bandwidth with filters
    const std::map<int, double> bb60Decimation = {{8192, 4e3},
                                                 {4096, 8e3},
                                                 {2048, 15e3},
                                                 {1024, 30e3},
                                                 {512, 65e3},
                                                 {256, 140e3},
                                                 {128, 250e3},
                                                 {64, 500e3},
                                                 {32, 1e6},
                                                 {16, 2e6},
                                                 {8, 3.75e6},
                                                 {4, 8e6},
                                                 {2, 17.8e6},
                                                 {1, 27e6}};

    // Port 1 configs
    const std::map<std::string, unsigned int> port1_config = {{"DEFAULT", 0},
                                                              {"INT_REF_OUT_AC", BB_PORT1_INT_REF_OUT|BB_PORT1_AC_COUPLED},
                                                              {"INT_REF_OUT_DC", BB_PORT1_INT_REF_OUT|BB_PORT1_DC_COUPLED},
                                                              {"EXT_REF_IN_AC", BB_PORT1_EXT_REF_IN|BB_PORT1_AC_COUPLED},
                                                              {"EXT_REF_IN_DC", BB_PORT1_EXT_REF_IN|BB_PORT1_DC_COUPLED},
                                                              {"EXT_REF_IN_DC", BB_PORT1_EXT_REF_IN|BB_PORT1_DC_COUPLED},
                                                              {"OUT_LOGIC_LOW_AC", BB_PORT1_OUT_LOGIC_LOW|BB_PORT1_AC_COUPLED},
                                                              {"OUT_LOGIC_LOW_DC", BB_PORT1_OUT_LOGIC_LOW|BB_PORT1_DC_COUPLED},
                                                              {"OUT_LOGIC_HIGH_AC", BB_PORT1_OUT_LOGIC_HIGH|BB_PORT1_AC_COUPLED},
                                                              {"OUT_LOGIC_HIGH_DC", BB_PORT1_OUT_LOGIC_HIGH|BB_PORT1_DC_COUPLED}};

    // Port 2 configs
    const std::map<std::string, unsigned int> port2_config = {{"DEFAULT", 0},
                                                              {"OUT_LOGIC_LOW", BB_PORT2_OUT_LOGIC_LOW},
                                                              {"OUT_LOGIC_HIGH", BB_PORT2_OUT_LOGIC_HIGH},
                                                              {"IN_TRIGGER_RISING_EDGE", BB_PORT2_IN_TRIGGER_RISING_EDGE},
                                                              {"IN_TRIGGER_FALLING_EDGE", BB_PORT2_IN_TRIGGER_FALLING_EDGE},
                                                              {"UART", BB60D_PORT2_OUT_UART}};

public:
    
    /*******************************************************************
     * Constructor and Destructor
     ******************************************************************/

    SignalHoundBB60(const SoapySDR::Kwargs &args) 
    {
        // Defaults
        streamActive = false;
        serialSpecified = false;
        deviceId = -1;
        type = 0;
        decimation = 1;
        numDevices = -1;
        port1 = 0;
        port2 = 0;
        sampleRate = 0;
        centerFrequency = 1.0e9;
        bandwidth = BB60_CLOCK/decimation;
        rfGain = 0.0;
        attenLevel = 0.0;

        if(args.count("serial") != 0) {
            try {
                serial = std::stoull(args.at("serial"), nullptr, 10);
            } catch (const std::invalid_argument &) {
                throw std::runtime_error("serial is not a number");
            } catch (const std::out_of_range &) {
                throw std::runtime_error("serial value of out range");
            }
            serialSpecified = true;
        } else if(args.count("device_id") != 0) {
            try {
                deviceId = std::stoi(args.at("device_id"));
            } catch (const std::invalid_argument &) {
                throw std::runtime_error("device_id is not a number");
            } catch (const std::out_of_range &) {
                throw std::runtime_error("device_id of out range");
            }
        }

        status = bbGetSerialNumberList2(serials, types, &numDevices);
        if(status != bbNoError) {
            throw std::runtime_error("Failed to retrieve list of BB60 devices");
        }

        if(numDevices < 1) {
            throw std::runtime_error("No BB60 devices found");
        }

        if(serialSpecified) {
            // Find serial
            for(int i = 0; i < numDevices; i++) {
                if(serials[i] == serial) {
                    deviceId = i;
                    break;
                }
            }
            if(deviceId < 0) {
                throw std::runtime_error("BB60 device with S/N " 
                                            + std::to_string(serial) 
                                            + " not found");
            }
        } else {
            if(deviceId < 0) {
                deviceId = 0; // Default
            } else if(deviceId >= numDevices) {
                throw std::runtime_error("BB60 device_id out of range [0 .. " 
                                            + std::to_string(numDevices-1) 
                                            + "].");
            }
            serial = serials[deviceId];
        }

        // Open device 
        bbStatus status;
        if((status = bbOpenDeviceBySerialNumber(&deviceId, serial)) != bbNoError) {
            throw std::runtime_error("Unable to open BB60 device " 
                                            + std::to_string(deviceId) 
                                            + " with S/N " 
                                            + std::to_string(serial));
        }
        type = types[deviceId];
    }

    ~SignalHoundBB60(void) 
    {
        bbAbort(deviceId);
        bbCloseDevice(deviceId);
    }

    /*******************************************************************
     * Identification API
     ******************************************************************/

    std::string getDriverKey(void) const 
    {
        return "Signal Hound BB Series";
    }

    std::string getHardwareKey(void) const
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        if(type == BB_DEVICE_BB60A) {
            return "Signal Hound BB60A";
        } else if(type == BB_DEVICE_BB60C) {
            return "Signal Hound BB60C";
        } else if(type == BB_DEVICE_BB60D) {
            return "Signal Hound BB60D";
        } else {
            return "Signal Hound BB60";    
        }
    }

    SoapySDR::Kwargs getHardwareInfo(void) const 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        int firmware = 0;
        bbGetFirmwareVersion(deviceId, &firmware);

        float temp, volt, curr;
        bbGetDeviceDiagnostics(deviceId, &temp, &volt, &curr);

        SoapySDR::Kwargs args;

        args["device_id"] = std::to_string(deviceId);
        args["serial"] = std::to_string(serial);
        args["api_version"] = bbGetAPIVersion();
        args["firmware"] = std::to_string(firmware);
        args["temperature"] = std::to_string(temp);
        args["voltage"] = std::to_string(volt);
        args["current"] = std::to_string(curr);

        return args;
    }

    /*******************************************************************
     * Channels API
     ******************************************************************/
    size_t getNumChannels(const int direction) const 
    {
        return (direction == SOAPY_SDR_RX) ? 1 : 0;
    }

    SoapySDR::Kwargs getChannelInfo(const int direction, const size_t channel) const 
    {
        SoapySDR::Kwargs args;
        if(direction == SOAPY_SDR_RX && channel == 1) {
            args["channel"] = std::to_string(channel);
            args["dBm_MAX"] = "+20 dBm";
            args["impedance"] = "50 ohm";
            args["rf_range"] = "9kHz to 6GHz";
            args["max_sensitivty"] = "-30 dBm";
            args["instantaneous_bandwidth"] = "27 MHz";
        } else {
            SoapySDR::Kwargs args;
            args["channel"] = std::to_string(channel);
            args["dBm_MAX"] = "None";
            args["impedance"] = "None";
            args["rf_range"] = "None";
            args["max_sensitivty"] = "None";
            args["instantaneous_bandwidth"] = "None";
        }
        return args;
    }

    /*******************************************************************
     * Stream API
     ******************************************************************/

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const 
    {
        std::vector<std::string> formats;

        formats.push_back(SOAPY_SDR_CF32);
        formats.push_back(SOAPY_SDR_CS16);

        return formats;
    }

    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const 
    {
        fullScale = 1.0;

        return SOAPY_SDR_CF32;
    }

    SoapySDR::Stream* setupStream(const int direction,
                                  const std::string &format,
                                  const std::vector<size_t> &channels = std::vector<size_t>(),
                                  const SoapySDR::Kwargs &args = SoapySDR::Kwargs()) 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        // Check channel config
        if(channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0)) {
            throw std::runtime_error("setupStream invalid channel selection");
        }

        dataFormat = format;
        configStream();

        return (SoapySDR::Stream*) this;
    }

    void closeStream(SoapySDR::Stream *stream) 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        bbAbort(deviceId);
        streamActive = false;
    }

    int activateStream(SoapySDR::Stream* stream,
                       const int flags = 0,
                       const long long timeNs = 0,
                       const size_t numElems = 0) 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        if (flags != 0) {
            return SOAPY_SDR_NOT_SUPPORTED;
        }
        // Check if stream already active
        if (streamActive) {
            return 0;
        }

        // Initiate stream
        initStream();
        return 0;
    }

    int deactivateStream(SoapySDR::Stream *stream, 
                         const int flags = 0, 
                         const long long timeNs = 0) 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        if(flags != 0 || timeNs != 0) {
            return SOAPY_SDR_NOT_SUPPORTED;
        }
        bbAbort(deviceId);
        streamActive = false;
        return 0;
    }

    int readStream(SoapySDR::Stream *stream,
                   void * const *buffs,
                   const size_t numElems,
                   int &flags,
                   long long &timeNs,
                   const long timeoutUs = 100000) 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        bbIQPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.iqData = buffs[0];
        pkt.iqCount = numElems;
        bbStatus status = bbGetIQ(deviceId, &pkt);

        if (status != bbNoError) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "GetIQ: %s", bbGetErrorString(status));
            return 0;
        }

        if (pkt.sampleLoss == BB_TRUE) {
            SoapySDR_logf(SOAPY_SDR_WARNING, "Sample Overrun");
        }

        return pkt.iqCount;
    }

    /*******************************************************************
     * Antenna API
     ******************************************************************/

    std::vector<std::string> listAntennas(const int direction, const size_t channel) const 
    {
        std::vector<std::string> antennas;
        antennas.push_back("RX");
        return antennas;
    }

    void setAntenna(const int direction, const size_t channel, const std::string &name) 
    {
        return;
    }

    std::string getAntenna(const int direction, const size_t channel) const 
    {
        return "RX";
    }

    /*******************************************************************
     * Gain API
     ******************************************************************/

    std::vector<std::string> listGains(const int direction, const size_t channel) const 
    {
        std::vector<std::string> results;

        results.push_back("RF");
        results.push_back("ATT");

        return results;
    }

    bool hasGainMode(const int direction, const size_t channel) const {
        return false;
    }

    void setGain(const int direction,
		         const size_t channel,
		         const double value )
    {
        // Do nothing
    }

    void setGain(const int direction, 
                 const size_t channel, 
                 const std::string &name, 
                 const double value) 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        if(name == "RF") {
            rfGain = value;
        } else if(name == "ATT") {
            attenLevel = value;
        } else {
            throw std::runtime_error(std::string("Unknown GAIN ")+name);
        }
        reconfigure();
    }

    double getGain(const int direction, const size_t channel, const std::string &name) const 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        if(name=="RF") {
            return rfGain;
        } else if(name=="ATT") {
            return attenLevel;
        } 

        throw std::runtime_error(std::string("Unsupported GAIN ")+name);

        return 0.0;
    }

    double getGain(const int direction,
		         const size_t channel)
    {
        return rfGain + attenLevel;
    }

    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const 
    {
        
        if(type == BB_DEVICE_BB60C) {
            if(name == "ATT") {
                return SoapySDR::Range(-30, 0);
            } else if(name == "RF") {
                return SoapySDR::Range(0, 35);
            }
        } else if(type == BB_DEVICE_BB60D) {
            if(name == "ATT") {
                return SoapySDR::Range(-30, 0);
            } else if(name == "RF") {
                return SoapySDR::Range(0, 20);
            }
        }

        throw std::runtime_error(std::string("Unsupported gain: ") + name);

        return SoapySDR::Range(0,0);
    }

    /*******************************************************************
     * Frequency API
     ******************************************************************/

    void setFrequency(const int direction, 
                      const size_t channel, 
                      const double frequency, 
                      const SoapySDR::Kwargs &args) 
    {
        setFrequency(direction, channel, "RF", frequency, args);
    }

    void setFrequency(const int direction, 
                     const size_t channel, 
                     const std::string &name, 
                     const double frequency, 
                     const SoapySDR::Kwargs &args) 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        if(name != "RF") {
            SoapySDR_logf(SOAPY_SDR_ERROR, "setFrequency: invalid name");
            return;
        }
        centerFrequency = (double)frequency;
        reconfigure();
        return;
    }

    double getFrequency(const int direction, const size_t channel) const 
    {
        return getFrequency(direction, channel, "RF");
    }

    double getFrequency(const int direction, const size_t channel, const std::string &name) const 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        if(name == "RF") {
            return (double)centerFrequency;
        }

        return (double)SOAPY_SDR_NOT_SUPPORTED;
    }

    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const 
    {
        std::vector<std::string> names;

        names.push_back("RF");

        return names;
    }

    SoapySDR::RangeList getFrequencyRange(
            const int direction,
            const size_t channel) const 
    {
        return getFrequencyRange(direction, channel, "RF");
    }

    SoapySDR::RangeList getFrequencyRange(
            const int direction,
            const size_t channel,
            const std::string &name) const 
    {
        SoapySDR::RangeList results;

        if(name == "RF") {
            results.push_back(SoapySDR::Range(BB60_MIN_FREQ, BB60_MAX_FREQ));
        }

        return results;
    }

    /*******************************************************************
     * Sample Rate API
     ******************************************************************/


    void setSampleRate(const int direction, const size_t channel, const double rate) 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        if(sampleRate != rate) {
            auto revii = bb60Decimation.rbegin();
            int dec = revii->first;
            double bw = bb60Decimation.at(dec);

            while(revii != bb60Decimation.rend()) {
                if((BB60_CLOCK/revii->first) >= rate) {
                    decimation = revii->first;
                    bw = revii->second;
                    break;
                }
                revii++;
            }

            sampleRate = rate;
            std::stringstream sstream;
            sstream << "BB60 set decimation " << decimation << " BW " << bw/1e6 << "MHz SR: " << BB60_CLOCK/decimation/1e6 << "MHz";
            SoapySDR_log(SOAPY_SDR_INFO, sstream.str().c_str());

            reconfigure();
        }
    }

    double getSampleRate(const int direction, const size_t channel) const 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        return BB60_CLOCK/decimation;
    }

    SoapySDR::RangeList getSampleRateRange(const int direction, const size_t channel) const 
    {
        SoapySDR::RangeList results;

        for(auto &decimator: bb60Decimation) {
            results.push_back(SoapySDR::Range(BB60_CLOCK/decimator.first,BB60_CLOCK/decimator.first));
        }

        return results;
    }

    // Deprecated dont use
    std::vector<double> listSampleRates(const int direction, const size_t channel) const 
    {
        SoapySDR_log(SOAPY_SDR_WARNING, "listSampleRates: This function is deprecrated.");
        std::vector<double> results;
        for(auto &ii: bb60Decimation) {
            results.insert(results.begin(),BB60_CLOCK/ii.first);
        }
        return results;
    }

    /*******************************************************************
     * Bandwidth API
     ******************************************************************/

    void setBandwidth(const int direction, const size_t channel, const double bw) 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        bandwidth = std::min(bb60Decimation.at(decimation), bw);
        reconfigure();
    }

    double getBandwidth(const int direction, const size_t channel) const 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        return bandwidth;
    }

    SoapySDR::RangeList getBandwidthRange(const int direction, const size_t channel) const 
    {
        SoapySDR::RangeList results;

        for(auto &decimator: bb60Decimation) {
            results.push_back(SoapySDR::Range(decimator.second,decimator.second));
        }

        return results;
    }

    // Deprecated dont use
    std::vector<double> listBandwidths(const int direction, const size_t channel) const 
    {
        SoapySDR_log(SOAPY_SDR_WARNING, "listBandwidths: This function is deprecrated.");
        std::vector<double> results;
        for(auto &ii: bb60Decimation) {
            results.insert(results.begin(), ii.second);
        }
        return results;
    }

    /*******************************************************************
     * Settings API
     ******************************************************************/

    SoapySDR::ArgInfoList getSettingInfo(void) const 
    {
        SoapySDR::ArgInfoList setArgs;

        SoapySDR::ArgInfo arg;

        arg.key = "port1";
        arg.value = "DEFAULT";
        arg.name = "PORT 1";
        arg.description = "BNC connector 1";
        arg.type = SoapySDR::ArgInfo::STRING;
        arg.options = {"DEFAULT", "INT_REF_OUT_AC", "INT_REF_OUT_DC",
                       "EXT_REF_IN_AC", "EXT_REF_IN_DC",
                       "OUT_LOGIC_LOW_AC", "OUT_LOGIC_LOW_DC",
                       "OUT_LOGIC_HIGH_AC", "OUT_LOGIC_HIGH_DC"};

        setArgs.push_back(arg);

        arg.key = "port2";
        arg.value = "DEFAULT";
        arg.name = "PORT 2";
        arg.description = "BNC connector 2";
        arg.type = SoapySDR::ArgInfo::STRING;
        arg.options = {"DEFAULT", "OUT_LOGIC_LOW_DC", "OUT_LOGIC_HIGH_DC",
                       "IN_TRIGGER_RISING_EDGE", "IN_TRIGGER_FALLING_EDGE"};

        setArgs.push_back(arg);

        return setArgs;
    }

    SoapySDR::ArgInfo getSettingInfo(const std::string &key) const 
    {
        if (key == "port1") {
            SoapySDR::ArgInfo arg;

            arg.key = "port1";
            arg.value = "DEFAULT";
            arg.name = "PORT 1";
            arg.description = "BNC connector 1";
            arg.type = SoapySDR::ArgInfo::STRING;
            arg.options = {"DEFAULT", "INT_REF_OUT_AC", "INT_REF_OUT_DC",
                           "EXT_REF_IN_AC", "EXT_REF_IN_DC",
                           "OUT_LOGIC_LOW_AC", "OUT_LOGIC_LOW_DC",
                           "OUT_LOGIC_HIGH_AC", "OUT_LOGIC_HIGH_DC"};
            return arg;
        } else if (key == "port2") {
            SoapySDR::ArgInfo arg;

            arg.key = "port2";
            arg.value = "DEFAULT";
            arg.name = "PORT 2";
            arg.description = "BNC connector 2";
            arg.type = SoapySDR::ArgInfo::STRING;
            arg.options = {"DEFAULT", "OUT_LOGIC_LOW_DC", "OUT_LOGIC_HIGH_DC",
                           "IN_TRIGGER_RISING_EDGE", "IN_TRIGGER_FALLING_EDGE", "UART"};
            return arg;
        } else {
            SoapySDR::ArgInfo arg;
            return arg;
        }
    }

    void writeSetting(const std::string &key, const std::string &value) 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        if(key == "port1" && port1_config.count(value) > 0) {
            port1 = port1_config.at(value);
            reconfigure();
            return;
        }

        if(key == "port2" && port2_config.count(value) > 0) {
            port2 = port2_config.at(value);
            reconfigure();
            return;
        }

        SoapySDR_logf(SOAPY_SDR_WARNING, "Invalid setting '%s'=='%s'", key.c_str(),value.c_str());
    }

    std::string readSetting(const std::string &key) const 
    {
        const std::lock_guard<std::mutex> lock(devMutex);
        if(key == "port1") {
            std::string ret = "UNKNOWN";
            for(auto &ii: port1_config) {
                if(ii.second == port1) {
                    ret = ii.first;
                    break;
                }
            }
            return ret;
        }

        if(key == "port2") {
            std::string ret = "UNKNOWN";
            for(auto &ii: port2_config) {
                if(ii.second == port2) {
                    ret = ii.first;
                    break;
                }
            }
            return ret;
        }

        SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());

        return "";    
    }

    /*******************************************************************
     * Native Access API
     ******************************************************************/

    void* getNativeDeviceHandle(void) const 
    {
        return (void*) &deviceId;
    }

    /*******************************************************************
     * Util API
     ******************************************************************/

    void configStream()
    {
        bbStatus status = bbNoError;
        streamActive = false;
        status = bbAbort(deviceId);
            

        // Check format
        if(dataFormat == SOAPY_SDR_CF32) {
            SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32");
            bbConfigureIQDataType(deviceId, bbDataType32fc);
        } else if(dataFormat == SOAPY_SDR_CS16) {
            SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16");
            bbConfigureIQDataType(deviceId, bbDataType16sc);
        } else {
            throw std::runtime_error("setupStream: Invalid format '" + dataFormat
                            + "' -- Only CF32 and CS16 are supported by SoapyBB60C module.");
        }

        // BB60 Gain and Atten are kind of funky an you don't have 1db resolution so the below logic converts
        // or gain and atten from db values to the values used by the signalhound api. See bbConfigureGainAtten 
        // documentation to see the relationship.
        // https://signalhound.com/sigdownloads/SDK/online_docs/bb_api/bb__api_8h.html
        int actualGain = 0;
        if(type == BB_DEVICE_BB60C) {
            if(rfGain < 2.5) {
                actualGain = 0;
            } else if(rfGain < 17.5) {
                actualGain = 1;
            } else if(rfGain < 32.5) {
                actualGain = 2;
            } else {
                actualGain = 3;
            } 
        }  else if(type == BB_DEVICE_BB60D) {
            if(rfGain < 2.5) {
                actualGain = 0;
            } else if(rfGain  < 10) {
                actualGain = 1;
            } else if(rfGain < 17.5) {
                actualGain = 2;
            } else {
                actualGain = 3;
            } 
        }

        int actualAtten = (int)attenLevel / -10;

        status = bbConfigureGainAtten(deviceId, actualGain, actualAtten);
        if(status != bbNoError) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "ConfigureGainAtten: %s", bbGetErrorString(status));
        }
        // Set a sample rate of 40MS/s and a bandwidth of 27MHz
        status = bbConfigureIQCenter(deviceId, centerFrequency);
        SoapySDR_logf(SOAPY_SDR_ERROR, "ConfigureIQCenter: %lf", centerFrequency);
        if (status != bbNoError) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "ConfigureIQCenter: %s", bbGetErrorString(status));
        }

        status = bbConfigureIO(deviceId, port1, port2);
        if(status != bbNoError) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "ConfigureIO: %s", bbGetErrorString(status));
        } else {
            SoapySDR_logf(SOAPY_SDR_INFO, "ConfigureIO: %d %d", port1, port2);
        }

        // Choose the smaller - bandwidth or sample rate
        double actualBW = std::min(bb60Decimation.at(decimation), bandwidth);
        status = bbConfigureIQ(deviceId, decimation, actualBW);
        if (status != bbNoError) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "ConfigureIQ: %s", bbGetErrorString(status));
        }
    }

    void initStream()
    {
        // Initiate the device, once this function returns the device
        // will be streaming I/Q.
        status = bbInitiate(deviceId, BB_STREAMING, BB_STREAM_IQ);
        if(status != bbNoError) {
            SoapySDR_logf(SOAPY_SDR_ERROR, "Initiate: %s", bbGetErrorString(status));
        }
        streamActive = true;
    }

    void reconfigure()
    {
        configStream();
        initStream();
    }
};

/***********************************************************************
 * Find available devices
 **********************************************************************/
SoapySDR::KwargsList findSignalHoundBB60(const SoapySDR::Kwargs &args) 
{
    int serials[BB_MAX_DEVICES], types[BB_MAX_DEVICES];
    int count = -1;
    bbStatus status = bbGetSerialNumberList2(serials, types, &count);
    if(status != bbNoError) {
      SoapySDR_logf(SOAPY_SDR_ERROR, "Error: %s\n", bbGetErrorString(status));
    }

    SoapySDR::KwargsList devices;

    for(int i = 0; i < count; i++) {
        SoapySDR::Kwargs deviceInfo;

        deviceInfo["device_id"] = std::to_string(i);
        if (types[i] == BB_DEVICE_BB60A) {
            deviceInfo["label"] = "BB60A [" + std::to_string(serials[i]) + "]";
        } else if (types[i] == BB_DEVICE_BB60C) {
            deviceInfo["label"] = "BB60C [" + std::to_string(serials[i]) + "]";
        } else if (types[i] == BB_DEVICE_BB60D) {
            deviceInfo["label"] = "BB60D [" + std::to_string(serials[i]) + "]";
        } else {
            deviceInfo["label"] = "BB60 [" + std::to_string(serials[i]) + "]";    
        }
        deviceInfo["serial"] = std::to_string(serials[i]);

        devices.push_back(deviceInfo);
    }

    return devices;
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
SoapySDR::Device* makeSignalHoundBB60(const SoapySDR::Kwargs &args) 
{
    return new SignalHoundBB60(args);
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry registerSignalHoundBB60("SignalHoundBB60", &findSignalHoundBB60, &makeSignalHoundBB60, SOAPY_SDR_ABI_VERSION);
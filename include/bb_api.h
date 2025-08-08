// Copyright (c).2014-2022, Signal Hound
// For licensing information, please see the API license in the software_licenses folder

/*!
 * \file bb_api.h
 * \brief API functions for the BB60 spectrum analyzers.
 *
 * This is the main file for user accessible functions for controlling the
 * BB60 spectrum analyzers.
 *
 */

#ifndef BB_API_H
#define BB_API_H

#if defined(_WIN32)
    #ifdef BB_EXPORTS
        #define BB_API __declspec(dllexport)
    #else
        #define BB_API __declspec(dllimport)
    #endif

    // bare minimum stdint typedef support
    #if _MSC_VER < 1700 // For VS2010 or earlier
        typedef signed char        int8_t;
        typedef short              int16_t;
        typedef int                int32_t;
        typedef long long          int64_t;
        typedef unsigned char      uint8_t;
        typedef unsigned short     uint16_t;
        typedef unsigned int       uint32_t;
        typedef unsigned long long uint64_t;
    #else
        #include <stdint.h>
    #endif

    #define BB_DEPRECATED(comment) __declspec(deprecated(comment))
#else // Linux
    #define BB_API __attribute__((visibility("default")))

    #include <stdint.h>

    #if defined(__GNUC__)
        #define BB_DEPRECATED(comment) __attribute__((deprecated))
    #else
        #define BB_DEPRECATED(comment) comment
    #endif
#endif

/** Used for boolean true when integer parameters are being used. */
#define BB_TRUE (1)
/** Used for boolean false when integer parameters are being used. */
#define BB_FALSE (0)

/** Device type: No Device. See #bbGetDeviceType. */
#define BB_DEVICE_NONE (0)
/** Device type: BB60A. See #bbGetDeviceType. */
#define BB_DEVICE_BB60A (1)
/** Device type: BB60C. See #bbGetDeviceType. */
#define BB_DEVICE_BB60C (2)
/** Device type: BB60D. See #bbGetDeviceType. */
#define BB_DEVICE_BB60D (3)

/**
 * Maximum number of devices that can be interfaced in the API. See
 * #bbGetSerialNumberList, #bbGetSerialNumberList2.
 */
#define BB_MAX_DEVICES (8)

/**
 * Minimum frequency (Hz) for sweeps, and minimum center frequency for I/Q
 * measurements. See #bbConfigureCenterSpan, #bbConfigureIQCenter.
 */
#define BB_MIN_FREQ (9.0e3)
/**
 * Maximum frequency (Hz) for sweeps, and maximum center frequency for I/Q
 * measurements. See #bbConfigureCenterSpan, #bbConfigureIQCenter.
 */
#define BB_MAX_FREQ (6.4e9)

/** Minimum span (Hz) for sweeps. See #bbConfigureCenterSpan. */
#define BB_MIN_SPAN (20.0)
/** Maximum span (Hz) for sweeps. See #bbConfigureCenterSpan. */
#define BB_MAX_SPAN (BB_MAX_FREQ - BB_MIN_FREQ)

/** Minimum RBW (Hz) for sweeps. See #bbConfigureSweepCoupling. */
#define BB_MIN_RBW (0.602006912)
/** Maximum RBW (Hz) for sweeps. See #bbConfigureSweepCoupling. */
#define BB_MAX_RBW (10100000.0)

/** Minimum sweep time in seconds. See #bbConfigureSweepCoupling. */
#define BB_MIN_SWEEP_TIME (0.00001) // 10us
/** Maximum sweep time in seconds. See #bbConfigureSweepCoupling. */
#define BB_MAX_SWEEP_TIME (1.0) // 1s

/** Minimum RBW (Hz) for device configured in real-time measurement mode. See #bbConfigureSweepCoupling. */
#define BB_MIN_RT_RBW (2465.820313)
/** Maximum RBW (Hz) for device configured in real-time measurement mode. See #bbConfigureSweepCoupling. */
#define BB_MAX_RT_RBW (631250.0)
/** Minimum span (Hz) for device configured in real-time measurement mode. See #bbConfigureCenterSpan. */
#define BB_MIN_RT_SPAN (200.0e3)
/** Maximum span (Hz) for BB60A device configured in real-time measurement mode. See #bbConfigureCenterSpan. */
#define BB60A_MAX_RT_SPAN (20.0e6)
/** Maximum span (Hz) for BB60C/D device configured in real-time measurement mode. See #bbConfigureCenterSpan. */
#define BB60C_MAX_RT_SPAN (27.0e6)

/** Minimum USB voltage. See #bbGetDeviceDiagnostics. */
#define BB_MIN_USB_VOLTAGE (4.4)

/** Maximum reference level in dBm. See #bbConfigureRefLevel. */
#define BB_MAX_REFERENCE (20.0)

/** Automatically choose attenuation based on reference level. See #bbConfigureGainAtten. */
#define BB_AUTO_ATTEN (-1)
/** Maximum attentuation. Valid values [0,3] or -1 for auto. See #bbConfigureGainAtten. */
#define BB_MAX_ATTEN (3)
/** Automatically choose gain based on reference level. See #bbConfigureGainAtten. */
#define BB_AUTO_GAIN (-1)
/** Maximum gain. Valid values [0,3] or -1 for auto. See #bbConfigureGainAtten. */
#define BB_MAX_GAIN (3)

/** No decimation. See #bbConfigureIQ. */
#define BB_MIN_DECIMATION (1)
/** Maimumx decimation for I/Q streaming. See #bbConfigureIQ. */
#define BB_MAX_DECIMATION (8192)

/** Measurement mode: Idle, no measurement. See #bbInitiate. */
#define BB_IDLE (-1)
/** Measurement mode: Swept spectrum analysis. See #bbInitiate. */
#define BB_SWEEPING (0)
/** Measurement mode: Real-time spectrum analysis. See #bbInitiate. */
#define BB_REAL_TIME (1)
/** Measurement mode: I/Q streaming. See #bbInitiate. */
#define BB_STREAMING (4)
/** Measurement mode: Audio demod. See #bbInitiate. */
#define BB_AUDIO_DEMOD (7)
/** Measurement mode: Tracking generator sweeps for scalar network analysis. See #bbInitiate. */
#define BB_TG_SWEEPING (8)

/** Turn off spur rejection. See #bbConfigureSweepCoupling. */
#define BB_NO_SPUR_REJECT (0)
/** Turn on spur rejection. See #bbConfigureSweepCoupling. */
#define BB_SPUR_REJECT (1)

/** Specifies dBm units of sweep and real-time spectrum analysis measurements. See #bbConfigureAcquisition. */
#define BB_LOG_SCALE (0)
/** Specifies mV units of sweep and real-time spectrum analysis measurements. See #bbConfigureAcquisition. */
#define BB_LIN_SCALE (1)
/** Specifies dBm units, with no corrections, of sweep and real-time spectrum analysis measurements. See #bbConfigureAcquisition. */
#define BB_LOG_FULL_SCALE (2)
/** Specifies mV units, with no corrections, of sweep and real-time spectrum analysis measurements. See #bbConfigureAcquisition. */
#define BB_LIN_FULL_SCALE (3)

/** Specifies the Nuttall window used for sweep and real-time analysis. See #bbConfigureSweepCoupling. */
#define BB_RBW_SHAPE_NUTTALL (0)
/** Specifies the Stanford flattop window used for sweep and real-time analysis. See #bbConfigureSweepCoupling. */
#define BB_RBW_SHAPE_FLATTOP (1)
/** Specifies a Gaussian window with 6dB cutoff used for sweep and real-time analysis. See #bbConfigureSweepCoupling. */
#define BB_RBW_SHAPE_CISPR (2)

/** Use min/max detector for sweep and real-time spectrum analysis. See #bbConfigureAcquisition. */
#define BB_MIN_AND_MAX (0)
/** Use average detector for sweep and real-time spectrum analysis. See #bbConfigureAcquisition. */
#define BB_AVERAGE (1)

/** VBW processing occurs in dBm. See #bbConfigureProcUnits. */
#define BB_LOG (0)
/** VBW processing occurs in linear voltage units (mV). See #bbConfigureProcUnits. */
#define BB_VOLTAGE (1)
/** VBW processing occurs in linear power units (mW). See #bbConfigureProcUnits. */
#define BB_POWER (2)
/** No VBW processing. See #bbConfigureProcUnits. */
#define BB_SAMPLE (3)

/** Audio demodulation type: AM. See #bbConfigureDemod. */
#define BB_DEMOD_AM (0)
/** Audio demodulation type: FM. See #bbConfigureDemod. */
#define BB_DEMOD_FM (1)
/** Audio demodulation type: Upper side band. See #bbConfigureDemod. */
#define BB_DEMOD_USB (2)
/** Audio demodulation type: Lower side band. See #bbConfigureDemod. */
#define BB_DEMOD_LSB (3)
/** Audio demodulation type: CW. See #bbConfigureDemod. */
#define BB_DEMOD_CW (4)

/** Default for #BB_SWEEPING measurement mode. See #bbInitiate. */
#define BB_STREAM_IQ (0x0)
/** For BB60C/D devices. See #bbInitiate. */
#define BB_DIRECT_RF (0x2)
/** Time stamp data using an external GPS receiver. See @ref gpsAndTimestamps and #bbInitiate. */
#define BB_TIME_STAMP (0x10)

/** Configure BB60A/C port 1 as AC coupled. This is the default. See #bbConfigureIO. */
#define BB60C_PORT1_AC_COUPLED (0x00)
/** Configure BB60A/C port 1 as DC coupled. See #bbConfigureIO. */
#define BB60C_PORT1_DC_COUPLED (0x04)
/** Use BB60A/C internal 10MHz reference. The internal reference is also output
 * on port 1, but it is considered unused. See #bbConfigureIO. */
#define BB60C_PORT1_10MHZ_USE_INT (0x00)
/** Output BB60A/C internal 10 MHz reference on port 1. Use this setting when
 * external equipment such as signal generators are using the 10MHz from the
 * BB60. See #bbConfigureIO. */
#define BB60C_PORT1_10MHZ_REF_OUT (0x100)
/** Use an external 10MHz provided on BB60A/C port 1. Best phase noise is
 * achieved by using a low jitter 3.3V CMOS input. See #bbConfigureIO. */
#define BB60C_PORT1_10MHZ_REF_IN (0x8)
/** Output logic low on BB60A/C port 1. See #bbConfigureIO. */
#define BB60C_PORT1_OUT_LOGIC_LOW (0x14)
/** Output logic high on BB60A/C port 1. See #bbConfigureIO. */
#define BB60C_PORT1_OUT_LOGIC_HIGH (0x1C)
/** Output logic low on BB60A/C port 2. See #bbConfigureIO. */
#define BB60C_PORT2_OUT_LOGIC_LOW (0x00)
/** Output logic high on BB60A/C port 2. See #bbConfigureIO. */
#define BB60C_PORT2_OUT_LOGIC_HIGH (0x20)
/** Detect and report external triggers rising edge on BB60A/C port 2 when I/Q
 * streaming. See #bbConfigureIO. */
#define BB60C_PORT2_IN_TRIG_RISING_EDGE (0x40)
/** Detect and report external triggers falling edge on BB60A/C port 2 when I/Q
 * streaming. See #bbConfigureIO. */
#define BB60C_PORT2_IN_TRIG_FALLING_EDGE (0x60)

/** Disable BB60D port 1. See #bbConfigureIO. */
#define BB60D_PORT1_DISABLED (0)
/** Discipline BB60D to an externally generated 10 MHz reference on port 1. See #bbConfigureIO. */
#define BB60D_PORT1_10MHZ_REF_IN (1)
/** Disable BB60D port 2. See #bbConfigureIO. */
#define BB60D_PORT2_DISABLED (0)
/** Output B660D internal 10MHz reference on port 2. See #bbConfigureIO. */
#define BB60D_PORT2_10MHZ_REF_OUT (1)
/** Detect and report external triggers rising edge on BB60D port 2 when I/Q
 * streaming. See #bbConfigureIO. */
#define BB60D_PORT2_IN_TRIG_RISING_EDGE (2)
/** Detect and report external triggers falling edge on BB60D port 2 when I/Q
 * streaming. See #bbConfigureIO. */
#define BB60D_PORT2_IN_TRIG_FALLING_EDGE (3)
/** Output logic low on BB60D port 2. See #bbConfigureIO. */
#define BB60D_PORT2_OUT_LOGIC_LOW (4)
/** Output logic high on BB60D port 2. See #bbConfigureIO. */
#define BB60D_PORT2_OUT_LOGIC_HIGH (5)
/** Use when any BB60D UART output functionality is desired. See #bbConfigureIO. */
#define BB60D_PORT2_OUT_UART (6)

/** Set 4800 baud rate for BB60D UART transmissions. See bbSetUARTRate. */
#define BB60D_UART_BAUD_4_8K (0)
/** Set 9600 baud rate for BB60D UART transmissions. See bbSetUARTRate. */
#define BB60D_UART_BAUD_9_6K (1)
/** Set 19200 baud rate for BB60D UART transmissions. See bbSetUARTRate. */
#define BB60D_UART_BAUD_19_2K (2)
/** Set 38400 baud rate for BB60D UART transmissions. See bbSetUARTRate. */
#define BB60D_UART_BAUD_38_4K (3)
/** Set 14400 baud rate for BB60D UART transmissions. See bbSetUARTRate. */
#define BB60D_UART_BAUD_14_4K (4)
/** Set 28800 baud rate for BB60D UART transmissions. See bbSetUARTRate. */
#define BB60D_UART_BAUD_28_8K (5)
/** Set 57600 baud rate for BB60D UART transmissions. See bbSetUARTRate. */
#define BB60D_UART_BAUD_57_6K (6)
/** Set 115200 baud rate for BB60D UART transmissions. See bbSetUARTRate. */
#define BB60D_UART_BAUD_115_2K (7)
/** Set 125000 baud rate for BB60D UART transmissions. See bbSetUARTRate. */
#define BB60D_UART_BAUD_125K (8)
/** Set 250000 baud rate for BB60D UART transmissions. See bbSetUARTRate. */
#define BB60D_UART_BAUD_250K (9)
/** Set 500000 baud rate for BB60D UART transmissions. See bbSetUARTRate. */
#define BB60D_UART_BAUD_500K (10)
/** Set 1000000 baud rate for BB60D UART transmissions. See bbSetUARTRate. */
#define BB60D_UART_BAUD_1000K (11)

/** Minimum number of frequency/data pairs given in #bbEnableUARTSweeping. See @ref UARTAntennaSwitching. */
#define BB60D_MIN_UART_STATES (2)
/** Maximum number of frequency/data pairs given in #bbEnableUARTSweeping. See @ref UARTAntennaSwitching. */
#define BB60D_MAX_UART_STATES (8)

/** In scalar network analysis, use the next trace as a thru. See #bbStoreTgThru. */
#define TG_THRU_0DB (0x1)
/** In scalar network analysis, improve accuracy with a second thru step. See #bbStoreTgThru. */
#define TG_THRU_20DB (0x2)

/** Additional corrections are applied to tracking generator timebase. See #bbSetTgReference. */
#define TG_REF_UNUSED (0)
/** Use tracking generator timebase as frequency standard for system, and do not apply additional corrections. See #bbSetTgReference. */
#define TG_REF_INTERNAL_OUT (1)
/** Use an external reference for TG124A, and do not apply additional corrections to timebase. See #bbSetTgReference. */
#define TG_REF_EXTERNAL_IN (2)

/**
 * Used to encapsulate I/Q data and metadata. See #bbGetIQ.
 */
typedef struct bbIQPacket {
    /**
     * Pointer to an array of 32-bit complex floating-point values. Complex
     * values are interleaved real-imaginary pairs. This must point to a
     * contiguous block of iqCount complex pairs.
     */
    void *iqData;
    /** Number of I/Q data pairs to return. */
    int iqCount;
    /**
     * pointer to an array of integers. If the external trigger input is
     * active, and a trigger occurs during the acquisition time, triggers will
     * be populated with values which are relative indices into the iqData
     * array where external triggers occurred. Any unused trigger array values
     * will be set to zero.
     */
    int *triggers;
    /** Size of the triggers array. */
    int triggerCount;
    /**
     * Specifies whether to discard any samples acquired by the API since the
     * last time and bbGetIQ function was called. Set to #BB_TRUE if you wish
     * to discard all previously acquired data, and #BB_FALSE if you wish to
     * retrieve the contiguous I/Q values from a previous call to this
     * function.
     */
    int purge;
    /**  How many I/Q samples are still left buffered in the API. Set by API. */
    int dataRemaining;
    /**
     * Returns #BB_TRUE or #BB_FALSE. Will return #BB_TRUE when the API is
     * required to drop data due to internal buffers wrapping. This can be
     * caused by I/Q samples not being polled fast enough, or in instances
     * where the processing is not able to keep up (underpowered systems, or
     * other programs utilizing the CPU) Will return #BB_TRUE on the capture in
     * which the sample break occurs. Does not indicate which sample the break
     * occurs on. Will always return false if purge is true. Set by API.
     */
    int sampleLoss;
    /**
     * Seconds since epoch representing the timestamp of the first sample in the
     * returned array. Set by API.
     */
    int sec;
    /**
     * Nanoseconds representing the timestamp of the first sample in the returned
     * array. Set by API.
     */
    int nano;
} bbIQPacket;

/**
 * Status code returned from all BB API functions.
 * Errors are negative and suffixed with 'Err'.
 * Errors stop the flow of execution, warnings do not.
 */
typedef enum bbStatus {
    /** Invalid mode */
    bbInvalidModeErr             = -112,
    /** Reference level cannot exceed 20dBm */
    bbReferenceLevelErr          = -111,
    /** Invalid video processing units specified */
    bbInvalidVideoUnitsErr       = -110,
    /** Invalid window */
    bbInvalidWindowErr           = -109,
    /** Invalid bandwidth type */
    bbInvalidBandwidthTypeErr    = -108,
    /** Invalid sweep time */
    bbInvalidSweepTimeErr        = -107,
    /** Invalid bandwidth */
    bbBandwidthErr               = -106,
    /** Invalid gain */
    bbInvalidGainErr             = -105,
    /** Invalid attenuation */
    bbAttenuationErr             = -104,
    /** Frequency range out of bounds */
    bbFrequencyRangeErr          = -103,
    /** Invalid span */
    bbInvalidSpanErr             = -102,
    /** Invalid scale parameter */
    bbInvalidScaleErr            = -101,
    /** Invalid detector type */
    bbInvalidDetectorErr         = -100,

    /** Invalid file size */
    bbInvalidFileSizeErr         = -19,
    /** Unable to initialize libusb */
    bbLibusbError                = -18,
    /** Attempting to perform an operation on a device that does not support it. */
    bbNotSupportedErr            = -17,
    /** Tracking generator not found */
    bbTrackingGeneratorNotFound  = -16,

    /** USB timeout error */
    bbUSBTimeoutErr              = -15,
    /** Device connection issues detected */
    bbDeviceConnectionErr        = -14,
    /** Device packet framing issues */
    bbPacketFramingErr           = -13,
    /** GPS receiver not found or not configured properly */
    bbGPSErr                     = -12,
    /** Gain cannot be auto */
    bbGainNotSetErr              = -11,
    /**
     * Function could not complete because the instrument is actively
     * configured for or making a measurement. Call #bbAbort and try again.
     */
    bbDeviceNotIdleErr           = -10,
    /** Invalid device */
    bbDeviceInvalidErr           = -9,
    /** Buffer provided too small */
    bbBufferTooSmallErr          = -8,
    /** Returned when one or more required pointer parameter is null. */
    bbNullPtrErr                 = -7,
    /** Allocation limit reached */
    bbAllocationLimitErr         = -6,
    /** Device is already streaming */
    bbDeviceAlreadyStreamingErr  = -5,
    /**
     * Returned when one or more parameters provided does not match the range
     * of possible values.
     */
    bbInvalidParameterErr        = -4,
    /**
     * Returned if the device is not properly configured for the desired
     * action. Often occurs when the device needs to be configured for a
     * specific measurement mode before taking an action.
     */
    bbDeviceNotConfiguredErr     = -3,
    /** Device not streaming */
    bbDeviceNotStreamingErr      = -2,
    /** Returned when the device handle provided does not match an open or known device. */
    bbDeviceNotOpenErr           = -1,

    /** Function returned successfully. No warnings or errors. */
    bbNoError                    = 0,

    // Warnings/Messages

    /** One or more parameters was clamped to a minimum or maximum limit. */
    bbAdjustedParameter          = 1,
    /** ADC overflow */
    bbADCOverflow                = 2,
    /** No trigger found */
    bbNoTriggerFound             = 3,
    /** One or more parameters was clamped to a maximum upper limit. */
    bbClampedToUpperLimit        = 4,
    /** One or more parameters was clamped to a minimum lower limit. */
    bbClampedToLowerLimit        = 5,
    /** Device is uncalibrated */
    bbUncalibratedDevice         = 6,
    /** Break in data */
    bbDataBreak                  = 7,
    /** Sweep uncalibrated, data invalid or incomplete. */
    bbUncalSweep                 = 8,
    /** Invalid cal data, potentially corrupted */
    bbInvalidCalData             = 9
} bbStatus;

/**
 * Specifies a data type for data returned from the API.
 */
typedef enum bbDataType {
    /** 32-bit complex floats */
    bbDataType32fc = 0,
    /** 16-bit complex shorts */
    bbDataType16sc = 1
} bbDataType;

/**
 * Specifies device power state. See @ref powerState for more information.
 */
typedef enum bbPowerState {
    /** On */
    bbPowerStateOn = 0,
    /** Standby */
    bbPowerStateStandby = 1
} bbPowerState;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * This function returns the serial numbers for all unopened devices.
 *
 * The array provided is populated starting at index 0 up to #BB_MAX_DEVICES.
 * It is undefined behavior if the array provided contains fewer elements than
 * the number of unopened devices connected. For this reason, it is recommended
 * the array is #BB_MAX_DEVICES elements in length. The integer _deviceCount_
 * will contain the number of devices detected. Elements in the array beyond
 * deviceCount are not modified.
 *
 * Note: BB60A devices will have 0 returned as the serial number.
 *
 * @param[out] serialNumbers A pointer to an array of integers. Can be NULL.
 *
 * @param[out] deviceCount A pointer to an integer. Will be set to the number
 * of devices found on the system.
 *
 * @return
 */
BB_API bbStatus bbGetSerialNumberList(int serialNumbers[BB_MAX_DEVICES], int *deviceCount);

/**
 * This function returns the serial numbers and device types for all unopened
 * devices.
 *
 * The arrays provided are populated starting at index 0 up to #BB_MAX_DEVICES.
 * It is undefined behavior if the arrays provided contain fewer elements than
 * the number of unopened devices connected. For this reason, it is recommended
 * the arrays are #BB_MAX_DEVICES elements in length. The integer _deviceCount_
 * will contain the number of devices detected. Elements in the arrays beyond
 * deviceCount are not modified.
 *
 * Note: BB60A devices will have 0 returned as the serial number.
 *
 * @param[out] serialNumbers A pointer to an array of integers. Can be NULL.
 *
 * @param[out] deviceTypes A pointer to an array of integers. The possible
 * device types returned are #BB_DEVICE_BB60A, #BB_DEVICE_BB60C, and
 * #BB_DEVICE_BB60D. Can be NULL.
 *
 * @param[out] deviceCount A pointer to an integer. Will be set to the number
 * of devices found on the system.
 *
 * @return
 */
BB_API bbStatus bbGetSerialNumberList2(int serialNumbers[BB_MAX_DEVICES],
                                       int deviceTypes[BB_MAX_DEVICES],
                                       int *deviceCount);

/**
 * This function attempts to open the first unopened BB60 it detects. If a
 * device is opened successfully, a handle to the device will be returned
 * through the device pointer which can be used to target that device in
 * subsequent API function calls.
 *
 * When successful, this function takes about 3 seconds to return. During that
 * time, the calling thread is blocked.
 *
 *If you wish to target multiple devices or wish to target devices across
 *processes, see @ref multipleDevicesAndProcesses.
 *
 * @param[out] device Pointer to an integer. If successful, a device handle is
 * returned. This handle is used for all successive API function calls.
 *
 * @return
 */
BB_API bbStatus bbOpenDevice(int *device);

/**
 * The function attempts to open the device with the provided serial number. If
 * no devices are detected with that serial, the function returns an error. If
 * a device is opened successfully, a handle to the device will be returned
 * through the device pointer which can be used to target that device in
 * subsequent API function calls.
 *
 * Only BB60C/D devices can be opened by specifying the serial number. If the
 * serial number specified is 0, the first BB60A found will be opened.
 *
 * When successful, this function takes about 3 seconds to return. During that
 * time, the calling thread is blocked.
 *
 *If you wish to target multiple devices or wish to target devices across
 *processes, see @ref multipleDevicesAndProcesses.
 *
 * @param[out] device Pointer to an integer. If successful, the integer pointed
 * to by device will contain a valid device handle which can be used to
 * identify a device for successive API function calls.
 *
 * @param[in] serialNumber User-provided serial number.
 *
 * @return
 */
BB_API bbStatus bbOpenDeviceBySerialNumber(int *device, int serialNumber);

/**
 * This function closes a device, freeing internal allocated memory and USB 3.0
 * resources. The device closed will become available to be opened again.
 *
 * This function will abort any active measurements.
 *
 * @param[in] device Device handle.
 *
 * @return
 */
BB_API bbStatus bbCloseDevice(int device);

/**
 * This function is used to set the power state of a BB60D device. This
 * function will return an error for any non BB60D device. The device should
 * not be performing any measurements when calling this function. The device
 * can be configured for sweeps as long as it is not actively performing one.
 *
 * The device should not perform any measurements while in the standby power
 * state. Ideally no API functions calls should be performed between setting
 * the device into standby power state and returning to the on power state.
 *
 * See the @ref powerState section for more details as well as the C++
 * programming example.
 *
 * @param[in] device Device handle.
 *
 * @param[in] powerState Specify the on or standby power state.
 *
 * @return
 */
BB_API bbStatus bbSetPowerState(int device, bbPowerState powerState);

/**
 * This function returns the current device power state.
 *
 * See the @ref powerState section for more details as well as the C++
 * programming example.
 *
 * @param[in] device Device handle.
 *
 * @param[out] powerState Pointer to power state variable.
 *
 * @return
 */
BB_API bbStatus bbGetPowerState(int device, bbPowerState *powerState);

/**
 * This function instructs the device to perform a power cycle. This might be
 * useful if the device has entered an unresponsive state. The device must
 * still be able to receive USB commands for this function to work. If the
 * device receives the power cycle command, it will take roughly 3 seconds to
 * fully power cycle.
 *
 * An example of using this function is provided in the C++ example folder.
 *
 * @param[in] device Device handle.
 *
 * @return
 */
BB_API bbStatus bbPreset(int device);

/**
 * This function will fully preset, close, and reopen the device pointed to by
 * the device parameter. This function will only work if the device can still
 * receive USB commands.
 *
 * @param[in] device Pointer to a valid device handle.
 *
 * @return
 */
BB_API bbStatus bbPresetFull(int *device);

/**
 * This function is for the BB60A only.
 *
 * This function causes the device to recalibrate itself to adjust for internal
 * device temperature changes, generating an amplitude correction array as a
 * function of IF frequency. This function will explicitly call #bbAbort to
 * suspend all device operations before performing the calibration and will
 * return the device in an idle state and configured as if it was just opened.
 * The state of the device should not be assumed and should be fully
 * reconfigured after a self-calibration.
 *
 * Temperature changes of 2 degrees Celsius or more have been shown to
 * measurably alter the shape/amplitude of the IF. We suggest using
 * bbGetDeviceDiagnostics to monitor the device’s temperature and perform
 * self-calibrations when needed. Amplitude measurements are not guaranteed to
 * be accurate otherwise, and large temperature changes (10°C or more) may
 * result in adding a dB or more of error.
 *
 * Because this is a streaming device, we have decided to leave the programmer
 * in full control of when the device in calibrated. The device is calibrated
 * once upon opening the device through bbOpenDevice and is the responsibility
 * of the programmer after that.
 *
 * Note: After calling this function, the device returns to the default state.
 * Currently the API does not retain state prior to the calling of #bbSelfCal.
 * Fully reconfiguring the device will be necessary.
 *
 * @param[in] device Device handle.
 *
 * @return
 */
BB_API bbStatus bbSelfCal(int device);

/**
 * If this function returns successfully, the variable pointed to by
 * serialNumber will contain the serial number of the specified device.
 *
 * @param[in] device Device handle.
 *
 * @param[out] serialNumber Returns device serial number as unsigned integer.
 *
 * @return
 */
BB_API bbStatus bbGetSerialNumber(int device, uint32_t *serialNumber);

/**
 * If the function returns successfully, the variable pointed to by deviceType
 * will be one of several device type macro values, #BB_DEVICE_NONE,
 * #BB_DEVICE_BB60A, #BB_DEVICE_BB60C, or #BB_DEVICE_BB60D. These macros are
 * defined in the API header file.
 *
 * @param[in] device Device handle.
 *
 * @param[out] deviceType Returns device type. Can be #BB_DEVICE_NONE,
 * #BB_DEVICE_BB60A, #BB_DEVICE_BB60C, #BB_DEVICE_BB60D.
 *
 * @return
 */
BB_API bbStatus bbGetDeviceType(int device, int *deviceType);

/**
 * If the function returns successfully, the variable pointed to by version
 * will be the firmware version of the specified device.
 *
 * BB60 firmware version information is available on the BB60 downloads page on
 * the Signal Hound website.
 *
 * @param[in] device Device handle.
 *
 * @param[out] version Returns firmware version number as integer.
 *
 * @return
 */
BB_API bbStatus bbGetFirmwareVersion(int device, int *version);

/**
 * The device temperature is updated in the API after each sweep is retrieved,
 * and periodically while I/Q streaming. The temperature is returned in Celsius
 * and has a resolution of 1/8th of a degree. A USB voltage of below 4.4V may
 * cause readings to be out of spec. Check your cable for damage and USB
 * connectors for damage or oxidation.
 *
 * @param[in] device Device handle.
 *
 * @param[out] temperature Returns device temperature as float.
 *
 * @param[out] usbVoltage Returns USB voltage as float.
 *
 * @param[out] usbCurrent Returns usb current as float.
 *
 * @return
 */
BB_API bbStatus bbGetDeviceDiagnostics(int device, float *temperature, float *usbVoltage, float *usbCurrent);

/**
 * The device must be idle when calling this function.
 *
 * This function configures the two I/O ports on the BB60 rear panel. The
 * values passed to the port parameters will change depending on which device
 * is connected. It is up to the developer to determine which device is
 * connected and send the appropriate parameters.
 *
 * For a full list of all possible parameters see below. For examples on basic
 * configuration of the ports, see the C++ programming examples in the SDK.
 *
 * ### BB60A/C
 *
 * #### Port 1
 *
 * Port 1 can be configured for 10 MHz ref in/out or as a logic output and
 * accepts the following values:
 *
 * - #BB60C_PORT1_10MHZ_USE_INT Use internal 10 MHz reference. The internal
 * reference is also output on port 1, but it is considered unused.
 *
 * - #BB60C_PORT1_10MHZ_REF_OUT Output internal 10 MHz reference. Use this
 * setting when external equipment such as signal generators are using the
 * 10 MHz from the BB60.
 *
 * - #BB60C_PORT1_10MHZ_REF_IN Use an external 10 MHz. Best phase noise is
 * achieved by using a low jitter 3.3V CMOS input.
 *
 * - #BB60C_PORT1_OUT_LOGIC_LOW Output logic low.
 *
 * - #BB60C_PORT1_OUT_LOGIC_HIGH Output logic high.
 *
 * Only one of the values above can be selected for port 1. In addition to one
 * of the above values, port 1 can be configured as AC or DC coupled. If AC
 * coupled is desired, simply use one of the macros defined above. If DC
 * coupled is desired, bitwise or “|” one of the above values with
 * #BB60C_PORT1_DC_COUPLED.
 *
 * #### Port 2
 *
 * Port 2 can be configured as trigger input port or logic output port. Port 2
 * is always DC coupled and accepts the following values:
 *
 * - #BB60C_PORT2_OUT_LOGIC_LOW Output logic low.
 *
 * - #BB60C_PORT2_OUT_LOGIC_HIGH Output logic high.
 *
 * - #BB60C_PORT2_IN_TRIG_RISING_EDGE Detect and report external triggers
 * rising edge when I/Q streaming.
 *
 * - #BB60C_PORT2_IN_TRIG_FALLING_EDGE Detect and report external triggers
 * falling edge when I/Q streaming.
 *
 * ### BB60D
 *
 * #### Port 1
 *
 * Port 1 can be configured for reference in. Port 1 is always AC coupled. The
 * following values are accepted:
 *
 * - #BB60D_PORT1_DISABLED Disable port.
 *
 * - #BB60D_PORT1_10MHZ_REF_IN Discipline to an externally generated 10 MHz
 * reference.
 *
 * #### Port 2
 *
 * Port 2 is used for 10 MHz out, trigger input, logic outputs, and UART
 * outputs. Port 2 is always DC coupled. The following values are accepted:
 *
 * - #BB60D_PORT2_DISABLED Disable port.
 *
 * - #BB60D_PORT2_10MHZ_REF_OUT Output internal 10 MHz reference.
 *
 * - #BB60D_PORT2_IN_TRIG_RISING_EDGE Detect and report external triggers
 * rising edge when I/Q streaming.
 *
 * - #BB60D_PORT2_IN_TRIG_FALLING_EDGE Detect and report external triggers
 * falling edge when I/Q streaming.
 *
 * - #BB60D_PORT2_OUT_LOGIC_LOW Output logic low.
 *
 * - #BB60D_PORT2_OUT_LOGIC_HIGH Output logic high.
 *
 * - #BB60D_PORT2_OUT_UART Use when any UART output functionality is desired.
 *
 * @param[in] device Device handle.
 *
 * @param[in] port1 See description.
 *
 * @param[in] port2 See description.
 *
 * @return
 */
BB_API bbStatus bbConfigureIO(int device, uint32_t port1, uint32_t port2);

/**
 * This function is currently not supported on the Linux operating system.
 *
 * The connection to the COM port is only established for the duration of this
 * function. It is closed when the function returns. Call this function once
 * before using a GPS PPS signal to time-stamp RF data. The synchronization
 * will remain valid until the CPU clock drifts more than ¼ second, typically
 * several hours, and will re-synchronize continually while streaming data
 * using a PPS trigger input.
 *
 * This function calculates the offset between your CPU clock time and the GPS
 * clock time to within a few milliseconds and stores this value for
 * time-stamping RF data using the GPS PPS trigger. This function ignores time
 * zone, limiting the calculated offset to +/- 30 minutes. It was tested using
 * an FTS 500 from Connor Winfield at 38.4k baud. It uses the RMC sentence, so
 * you must set up your GPS to output this.
 *
 * @param[in] comPort COM port number for the NMEA data output from the GPS
 * receiver.
 *
 * @param[in] baudRate Baud Rate of the COM port.
 *
 * @return
 */
BB_API bbStatus bbSyncCPUtoGPS(int comPort, int baudRate);

/**
 * _BB60D only._
 *
 * Sets the baud rate on all UART transmissions. Must be configured when the
 * device is idle. Only the baud rates defined as macros are supported.
 *
 * See the section on @ref UARTAntennaSwitching for more information
 *
 * @param[in] device Device handle.
 *
 * @param[in] rate One of any of the baud rate macros defined in the API header
 * file. Macro names start with BB60D_UART_BAUD_.
 *
 * @return
 */
BB_API bbStatus bbSetUARTRate(int device, int rate);

/**
 * _BB60D only._
 *
 * Must be called when the device is idle prior to initiating the sweep.
 *
 * Configures the receiver to transmit UART bytes at certain frequency
 * thresholds. The BB60D steps the frequency spectrum in 20MHz steps, and thus
 * resolution is +/- 20MHz on any given UART transmission.
 *
 * Frequencies are sorted if they are not provided in increasing order.
 * Regardless of the first frequency provided, the UART byte in position 0 is
 * transmitted at the beginning of the sweep. We recommend simply setting the
 * first frequency to 0Hz.
 *
 * Port 2 must be configured as an UART output, see #bbConfigureIO.
 *
 * See the section on UART antenna switching for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[in] freqs Array of frequency thresholds at which to transmit a new
 * UART byte. Size of array must be equal to the number of states.
 *
 * @param[in] data Array of UART bytes to transmit at the given frequency
 * threshold. Size of array must be equal to the number of states.
 *
 * @param[in] states Number of freq/data pairs. Alternatively, the size of the
 * provided arrays. Must be between [2,8].
 *
 * @return
 */
BB_API bbStatus bbEnableUARTSweeping(int device, const double *freqs, const uint8_t *data, int states);

/**
 * _BB60D only._
 *
 * Disables UART sweep antenna switching and clears any states set in
 * #bbEnableUARTSweeping. Device must be idle when calling this function.
 *
 * See the section on @ref UARTAntennaSwitching for more information.
 *
 * @param[in] device Device handle.
 *
 * @return
 */
BB_API bbStatus bbDisableUARTSweeping(int device);

/**
 * _BB60D only._
 *
 * Configures the pseudo-doppler antenna switching for I/Q streaming. Must be
 * configured when the device is idle.
 *
 * When I/Q streaming, the device will cycle through the provided data/count
 * pairs, transmitting the data at that state, then waiting for the specified
 * number of 40MHz clocks. The specified wait count starts as soon as
 * transmission of the start bit occurs.
 *
 * An external trigger is inserted when the cycle restarts. This allows the
 * ability to determine which I/Q samples correspond to which UART output
 * states. There is a limitation on how quickly external triggers can be
 * generated. The minimum spacing on triggers is ~250us or 4kHz. If the UART
 * streaming configuration produces triggers at a rate faster than this, you
 * will not receive every triggers. In this situation, you will need to
 * extrapolate missing triggers.
 *
 * An example:
 *
 * If two states are provided, with data = { 1, 8 } and counts { 10000, 10000
 * }, with a baud rate of 1M. While I/Q streaming, the UART byte of ‘1’ starts
 * being transmitted, an external trigger is inserted when the start bit begins
 * transmitting. The UART byte of 1 will take 10us to transmit, (10 bits,
 * 8(data) + 1 (start bit) + 1(stop bit)) and the device will stream for 250us
 * (10k / 40MHz) from when the start bit is transmitted. Once 250us have
 * passed, the start bit for the transmission of ‘8’ is output. Again, it will
 * stream for 250us with a time of 1us for transmitting the 10 bits. At that
 * point it goes back to the first state with a ‘1’ being transmitted.
 *
 * See the section on @ref UARTAntennaSwitching for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[in] data Array of UART bytes to transmit.
 *
 * @param[in] counts Array of 40MHz clock counts to remain at this state.
 * Values cannot exceed 2^24. Values should also be longer than the time it
 * takes to transmit 10 bits at the selected baud rate.
 *
 * @param states Number of data/counts pairs. Alternatively, the size of the
 * provided arrays. Must be between [2,8].
 *
 * @return
 */
BB_API bbStatus bbEnableUARTStreaming(int device, const uint8_t *data, const uint32_t *counts, int states);

/**
 * _BB60D only._
 *
 * Disables UART pseudo-doppler switching for I/Q streaming and clears any
 * states set in #bbEnableUARTStreaming. Device must be idle when calling this
 * function. See the section on @ref UARTAntennaSwitching for more
 * information.
 *
 * @param[in] device Device handle.
 *
 * @return
 */
BB_API bbStatus bbDisableUARTStreaming(int device);

/**
 * _BB60D only._
 *
 * Device must be idle when calling this function. Outputs provided byte
 * immediately and returns when complete. Port 2 must be configured for UART
 * output. See #bbConfigureIO.
 *
 * @param[in] device Device handle.
 *
 * @param[in] data Byte to transmit.
 *
 * @return
 */
BB_API bbStatus bbWriteUARTImm(int device, uint8_t data);

/**
 * The reference level is used to control the sensitivity of the receiver.
 *
 * It is recommended to set the reference level ~5dB higher than the maximum
 * expected input level.
 *
 * The reference level is only used if the gain and attenuation are set to auto
 * (default). It is recommended to leave gain/atten as auto.
 *
 * @param[in] device Device handle.
 *
 * @param[in] refLevel Reference level in dBm.
 *
 * @return
 */
BB_API bbStatus bbConfigureRefLevel(int device, double refLevel);

/**
 * This function is used to manually control the gain and attenuation of the
 * receiver. Both gain and attenuation must be set to non-auto values to
 * override the reference level.
 *
 * It is recommended to leave them as auto. When left as auto, gain and
 * attenuation can be optimized for each band independently. If manually
 * chosen, a flat gain/attenuation is used across all bands.
 *
 * Below is the gain and attenuation used with each setting.
 *
 * |     Setting      |     BB60C               |     BB60D               |
 * |------------------|-------------------------|-------------------------|
 * |     gain = 0     |     0dB gain            |     0dB gain            |
 * |     gain = 1     |     5dB gain            |     5dB gain            |
 * |     gain = 2     |     30dB gain           |     15dB gain           |
 * |     gain = 3     |     35dB gain           |     20dB gain           |
 * |     atten = 0    |     0dB attenuation     |     0 dB attenuation    |
 * |     atten = 1    |     10dB attenuation    |     10dB attenuation    |
 * |     atten = 2    |     20dB attenuation    |     20dB attenuation    |
 * |     atten = 3    |     30dB attenuation    |     30dB attenuation    |
 *
 * @param[in] device Device handle.
 *
 * @param[in] gain Gain should be between [-1,3] where -1 represents auto.
 *
 * @param[in] atten Atten should be between [-1,3] where -1 represents auto.
 *
 * @return
 */
BB_API bbStatus bbConfigureGainAtten(int device, int gain, int atten);

/**
 * _See #bbConfigureIQCenter for configuring I/Q streaming center frequency._
 *
 * This function configures the sweep frequency range. Start and stop
 * frequencies can be determined from the center and span.
 * - start = center – (span / 2)
 * - stop = center + (span / 2)
 *
 * During initialization a more precise start frequency and span is determined
 * and returned in the #bbQueryTraceInfo function.
 *
 * The start/stop frequencies cannot exceed [9kHz, 6.4GHz].
 *
 * There is an absolute minimum operating span of 20 Hz, but 200kHz is a
 * suggested minimum.
 *
 * Certain modes of operation have specific frequency range limits. Those mode
 * dependent limits are tested against during #bbInitiate.
 *
 * @param[in] device Device handle.
 *
 * @param[in] center Center frequency in Hz.
 *
 * @param[in] span Span in Hz.
 *
 * @return
 */
BB_API bbStatus bbConfigureCenterSpan(int device, double center, double span);

/**
 * For standard bandwidths, the API uses the 3 dB points to define the RBW. For
 * the CISPR RBW shape, 6dB bandwidths are used.
 *
 * The video bandwidth is implemented as an IIR filter applied bin-by-bin to a
 * sequence of overlapping FFTs. Larger RBW/VBW ratios require more FFTs.
 *
 * The API uses the Stanford flattop window when FLATTOP is selected. The
 * Nuttall window shape trades increased measurement speed for reduces
 * measurement accuracy by adding up to 0.8dB scalloping losses. The CISPR
 * window uses a Gaussian window with 6dB cutoff.
 *
 * All windows use zero-padding to achieve arbitrary RBWs. Only powers of 2 FFT
 * sizes are used in the API.
 *
 * sweepTime applies to standard swept analysis and is ignored for other
 * operating modes. If in sweep mode, sweepTime is the amount of time the
 * device will spend collecting data before processing. Increasing this value
 * is useful for capturing signals of interest or viewing a more consistent
 * view of the spectrum. Increasing sweepTime can have a large impact on the
 * resources used by the API due to the increase of data needing to be stored
 * and the amount of signal processing performed.
 *
 * Rejection can be used to optimize certain aspects of the signal. The default
 * is #BB_NO_SPUR_REJECT and should be used for most measurements. If you have
 * a steady CW or slowly changing signal and need to minimize image and
 * spurious responses from the device, use #BB_SPUR_REJECT. Rejection is
 * ignored outside of standard swept analysis.
 *
 * @param[in] device Device handle.
 *
 * @param[in] rbw Resolution bandwidth in Hz. RBWs can be set to arbitrary
 * values but may be limited by mode of operation and span.
 *
 * @param[in] vbw Video bandwidth in Hz. VBW must be less than or equal to RBW.
 * VBW can be arbitrary. For best performance use RBW as the VBW. When VBW is
 * set equal to RBW, no VBW filtering is performed.
 *
 * @param[in] sweepTime Suggest a sweep time in seconds. In sweep mode, this
 * value specifies how long the BB60 should sample spectrum for the configured
 * sweep. Larger sweep times may increase the odds of capturing spectral events
 * at the cost of slower sweep rates. The range of possible sweepTime values
 * run from 1ms -> 100ms or [0.001 – 0.1].
 *
 * @param[in] rbwShape The possible values for rbwShape are
 * #BB_RBW_SHAPE_NUTTALL, #BB_RBW_SHAPE_FLATTOP, and #BB_RBW_SHAPE_CISPR. This
 * choice determines the window function used and the bandwidth cutoff of the
 * RBW filter. #BB_RBW_SHAPE_NUTTALL is default and unchangeable for real-time
 * operation.
 *
 * @param[in] rejection The possible values for rejection are
 * #BB_NO_SPUR_REJECT and #BB_SPUR_REJECT.
 *
 * @return
 */
BB_API bbStatus bbConfigureSweepCoupling(int device, double rbw, double vbw, double sweepTime,
                                         uint32_t rbwShape, uint32_t rejection);

/**
 * The _detector_ parameter specifies how to produce the results of the signal
 * processing for the final sweep. Depending on settings, potentially many
 * overlapping FFTs will be performed on the input time domain data to retrieve
 * a more consistent and accurate result. When the results overlap detector
 * chooses whether to average the results together or maintain the minimum and
 * maximum values. If averaging is chosen, the min and max trace arrays
 * returned from #bbFetchTrace will contain the same averaged data.
 *
 * The _scale_ parameter will change the units of returned sweeps. If
 * #BB_LOG_SCALE is provided, sweeps will be returned as dBm values, If
 * #BB_LIN_SCALE is return, the returned units will be in milli-volts. If the
 * full-scale units are specified, no corrections are applied to the data and
 * amplitudes are taken directly from the full scale input.
 *
 * @param[in] device Device handle.
 *
 * @param[in] detector Specifies the video detector. The two possible values
 * for detector type are #BB_AVERAGE and #BB_MIN_AND_MAX.
 *
 * @param[in] scale Specifies the scale in which sweep results are returned
 * int. The four possible values for scale are #BB_LOG_SCALE, #BB_LIN_SCALE,
 * #BB_LOG_FULL_SCALE, and #BB_LIN_FULL_SCALE.
 *
 * @return
 */
BB_API bbStatus bbConfigureAcquisition(int device, uint32_t detector, uint32_t scale);

/**
 * The _units_ provided determines scale video processing occurs in. The chart
 * below shows which unit types are used for each units selection.
 *
 * For “average power” measurements, #BB_POWER should be selected. For cleaning
 * up an amplitude modulated signal, #BB_VOLTAGE would be a good choice. To
 * emulate a traditional spectrum analyzer, select #BB_LOG. To minimize
 * processing power, select #BB_SAMPLE.
 *
 * | Macro      | Unit                |
 * |------------|---------------------|
 * | BB_LOG     | dBm                 |
 * | BB_VOLTAGE | mV                  |
 * | BB_POWER   | mW                  |
 * | BB_SAMPLE  | No video processing |
 *
 * @param[in] device Device handle.
 *
 * @param[in] units The possible values are #BB_LOG, #BB_VOLTAGE, #BB_POWER,
 * #BB_SAMPLE.
 *
 * @return
 */
BB_API bbStatus bbConfigureProcUnits(int device, uint32_t units);

/**
 * The function allows you to configure additional parameters of the real-time
 * frames returned from the API. If this function is not called a scale of
 * 100dB is used and a frame rate of 30fps is used. For more information
 * regarding real-time mode see @ref realTimeAnalysis.
 *
 * @param[in] device Device handle.
 *
 * @param[in] frameScale Specifies the height in dB of the real-time frame. The
 * value is ignored if the scale is linear. Possible values range from [10 –
 * 200].
 *
 * @param[in] frameRate Specifies the rate at which frames are generated in
 * real-time mode, in frames per second. Possible values range from [4 – 30],
 * where four means a frame is generated every 250ms and 30 means a frame is
 * generated every ~33 ms.
 *
 * @return
 */
BB_API bbStatus bbConfigureRealTime(int device, double frameScale, int frameRate);

/**
 * By setting the advance rate users can control the overlap rate of the FFT
 * processing in real-time spectrum analysis. The _advanceRate_ parameter
 * specifies how far the FFT window slides through the data for each FFT as a
 * function of FFT size. An _advanceRate_ of 0.5 specifies that the FFT window
 * will advance 50% the FFT length for each FFT for a 50% overlap rate.
 * Specifying a value of 1.0 would mean the FFT window advances the full FFT
 * length meaning there is no overlap in real-time processing. The default
 * value is 0.5 and the range of acceptable values are between [0.5, 10].
 * Increasing the advance rate reduces processing considerably but also
 * increases the 100% probability of intercept of the device.
 *
 * @param[in] device Device handle.
 *
 * @param[in] advanceRate FFT advance rate.
 *
 * @return
 */
BB_API bbStatus bbConfigureRealTimeOverlap(int device, double advanceRate);

/**
 * Configure the center frequency for I/Q streaming.
 *
 * When switching back to sweep acquisition from I/Q acquisition, you will need
 * to call the #bbConfigureCenterSpan function again.
 *
 * The center frequency must be between [#BB_MIN_FREQ, #BB_MAX_FREQ]
 *
 * @param[in] device Device handle.
 *
 * @param[in] centerFreq Center frequency in Hz.
 *
 * @return
 */
BB_API bbStatus bbConfigureIQCenter(int device, double centerFreq);

/**
 * Configure the sample rate and bandwidth of the I/Q data stream.
 *
 * The decimation rate divides the I/Q sample rate. The final sample rate is
 * calculated with the equation `40MHz / downsampleFactor`.
 * _downsampleFactor_ must be a power of 2 between [1,8192].
 *
 * _bandwidth_ specifies the 3dB bandwidth of the I/Q data stream. _bandwidth_
 * controls the filter cutoff of a FIR filter applied to the data stream prior
 * to decimation.
 *
 * For each given decimation rate, a maximum bandwidth value is specified to
 * account for sufficient filter roll off. A table of maximum bandwidths can be
 * found in @ref IQFilteringAndBandwidthLimitations.
 *
 * @param[in] device Device handle.
 *
 * @param[in] downsampleFactor Specify a decimation rate for the 40MS/s I/Q
 * stream.
 *
 * @param[in] bandwidth Specify a bandpass filter width for the I/Q stream.
 *
 * @return
 */
BB_API bbStatus bbConfigureIQ(int device, int downsampleFactor, double bandwidth);

/**
 * See @ref IQDataTypes for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[in] dataType Data type can be specified either as 32-bit complex
 * floats or 16-bit complex shorts.
 *
 * @return
 */
BB_API bbStatus bbConfigureIQDataType(int device, bbDataType dataType);

/**
 * See the @ref IQStreaming for more information on triggering and
 * how the trigger sentinel value is used.
 *
 * @param[in] sentinel Value used to fill the remainder of the trigger buffer
 * when the trigger buffer provided is larger than the number of triggers
 * returned. The default sentinel value is zero.
 *
 * @return
 */
BB_API bbStatus bbConfigureIQTriggerSentinel(int sentinel);

/**
 * Below is the overall flow of data through our audio processing algorithm:
 *
 * ![](img/audio_demod_data_flow.jpg)
 *
 * This function can be called while the device is active.
 *
 * @param[in] device Device handle.
 *
 * @param[in] modulationType Specifies the demodulation scheme, possible values
 * are #BB_DEMOD_AM, #BB_DEMOD_FM, #BB_DEMOD_USB (upper sideband),
 * #BB_DEMOD_LSB (lower sideband), #BB_DEMOD_CW.
 *
 * @param[in] freq Center frequency. For best results, re-initiate the device
 * if the center frequency changes +/- 8MHz from the initial value.
 *
 * @param[in] IFBW Intermediate frequency bandwidth centered on freq. Filter
 * takes place before demodulation. Specified in Hz. Should be between 500 Hz
 * and 500 kHz.
 *
 * @param[in] audioLowPassFreq Post demodulation filter in Hz. Should be
 * between 1kHz and 12kHz Hz.
 *
 * @param[in] audioHighPassFreq Post demodulation filter in Hz. Should be
 * between 20 and 1000Hz.
 *
 * @param[in] FMDeemphasis Specified in micro-seconds. Should be between 1 and
 * 100.
 *
 * @return
 */
BB_API bbStatus bbConfigureDemod(int device, int modulationType, double freq, float IFBW,
                                 float audioLowPassFreq, float audioHighPassFreq, float FMDeemphasis);

/**
 * This function configures the device into a state determined by the _mode_
 * parameter. For more information regarding operating states, refer to the
 * @ref theoryOfOperation and @ref measurementTypes sections. This function
 * calls #bbAbort before attempting to reconfigure. It should be noted, if an
 * error is returned, any past operating state will no longer be active.
 *
 * @param[in] device Device handle.
 *
 * @param[in] mode The possible values for mode are #BB_SWEEPING,
 * #BB_REAL_TIME, #BB_AUDIO_DEMOD, #BB_STREAMING, #BB_TG_SWEEPING.
 *
 * @param[in] flag The default value should be zero. If the mode is equal to
 * #BB_STREAMING, the flag should be set to #BB_STREAM_IQ (0). flag can be used
 * to inform the API to time stamp data using an external GPS receiver. Mask
 * the bandwidth flag (‘|’ in C) with #BB_TIME_STAMP to achieve this. See @ref
 * gpsAndTimestamps for information on how to set this up.
 *
 * @return
 */
BB_API bbStatus bbInitiate(int device, uint32_t mode, uint32_t flag);

/**
 * Stops the device operation and places the device into an idle state.
 *
 * @param[in] device Device handle.
 *
 * @return
 */
BB_API bbStatus bbAbort(int device);

/**
 * This function should be called to determine sweep characteristics after a
 * device has been configured and initiated for sweep mode.
 *
 * @param[in] device Device handle.
 *
 * @param[out] traceLen Pointer to uint32_t to contain the size of the sweeps
 * returned from #bbFetchTrace.
 *
 * @param[out] binSize Pointer to double, to contain the frequency delta
 * between two sequential bins in the returned sweep.
 *
 * @param[out] start Pointer to double to contains the frequency of the first
 * bin in the sweep.
 *
 * @return
 */
BB_API bbStatus bbQueryTraceInfo(int device, uint32_t *traceLen, double *binSize, double *start);

/**
 * This function should be called after initializing the device for real-time
 * mode.
 *
 * @param[in] device Device handle.
 *
 * @param[out] frameWidth Pointer to uint32_t to contain the width of the
 * real-time frame in bins.
 *
 * @param[out] frameHeight Pointer to uint32_t to contain the height of the
 * real-time frame in bins.
 *
 * @return
 */
BB_API bbStatus bbQueryRealTimeInfo(int device, int *frameWidth, int *frameHeight);

/**
 * The device must be configured for real-time spectrum analysis to call this
 * function.
 *
 * @param[in] device Device handle.
 *
 * @param[out] poi Pointer to double to contain the 100% probability of
 * intercept duration in seconds.
 *
 * @return
 */
BB_API bbStatus bbQueryRealTimePoi(int device, double *poi);

/**
 * The device must be configured for I/Q streaming to call this function.
 *
 * @param[in] device Device handle.
 *
 * @param[out] sampleRate Pointer to double to contain the I/Q streaming sample
 * rate in Hz.
 *
 * @param[out] bandwidth Pointer to double to contain the I/Q streaming
 * bandwidth.
 *
 * @return
 */
BB_API bbStatus bbQueryIQParameters(int device, double *sampleRate, double *bandwidth);

/**
 * Retrieve the I/Q correction factor for I/Q streaming with 16-bit complex
 * shorts. The device must be configured for I/Q streaming to call this
 * function.
 *
 * @param[in] device Device handle.
 *
 * @param[out] correction Pointer to float to contain the scalar value used to
 * convert full scale I/Q data to amplitude corrected I/Q. The formulas for
 * these conversions are in @ref IQDataTypes. Cannot be null.
 *
 * @return
 */
BB_API bbStatus bbGetIQCorrection(int device, float *correction);

/**
 * Get one sweep in sweep mode. If the detector type is set to average, the
 * _traceMin_ array returned will equal max, in which case NULL can be used for
 * the min parameter.
 *
 * The first element returned in the sweep corresponds to the _startFreq_
 * returned from #bbQueryTraceInfo.
 *
 * @param[in] device Device handle.
 *
 * @param[in] arraySize This parameter is deprecated and ignored.
 *
 * @param[out] traceMin Pointer to a float array whose length should be equal
 * to or greater than _traceSize_ returned from #bbQueryTraceInfo. Set to NULL
 * to ignore this parameter.
 *
 * @param[out] traceMax Pointer to a float array whose length should be equal
 * to or greater than _traceSize_ returned from #bbQueryTraceInfo. Set to NULL
 * to ignore this parameter.
 *
 * @return
 */
BB_API bbStatus bbFetchTrace_32f(int device, int arraySize, float *traceMin, float *traceMax);

/**
 * Get one sweep in sweep mode. If the detector type is set to average, the
 * _traceMin_ array returned will equal max, in which case NULL can be used for
 * the min parameter.
 *
 * The first element returned in the sweep corresponds to the _startFreq_
 * returned from #bbQueryTraceInfo.
 *
 * @param[in] device Device handle.
 *
 * @param[in] arraySize This parameter is deprecated and ignored.
 *
 * @param[out] traceMin Pointer to a double array whose length should be equal
 * to or greater than _traceSize_ returned from #bbQueryTraceInfo. Set to NULL
 * to ignore this parameter.
 *
 * @param[out] traceMax Pointer to a double array whose length should be equal
 * to or greater than _traceSize_ returned from #bbQueryTraceInfo. Set to NULL
 * to ignore this parameter.
 *
 * @return
 */
BB_API bbStatus bbFetchTrace(int device, int arraySize, double *traceMin, double *traceMax);

/**
 * This function is used to retrieve the real-time sweeps, frame, and alpha
 * frame. This function should be used instead of #bbFetchTrace for real-time
 * mode. The sweep arrays should be equal to the trace length returned from the
 * #bbQueryTraceInfo function. The _frame_ and _alphaFrame_ should be WxH
 * values long, where W and H are the values returned from
 * #bbQueryRealTimeInfo. For more information see @ref realTimeAnalysis.
 *
 * @param[in] device Device handle.
 *
 * @param[out] traceMin If this pointer is non-null, the min held sweep will be
 * returned to the user. If the detector is set to average, this array will be
 * identical to the _traceMax_ array.
 *
 * @param[out] traceMax If this pointer is non-null, the max held sweep will be
 * returned to the user. If the detector is set to average, this array contains
 * the averaged results over the measurement interval.
 *
 * @param[out] frame Pointer to a float array. If the function returns
 * successfully, the contents of the array will contain a single real-time
 * frame.
 *
 * @param[out] alphaFrame Pointer to a float array. If the function returns
 * successfully, the contents of the array will contain the alphaFrame
 * corresponding to the frame. Can be NULL.
 *
 * @return
 */
BB_API bbStatus bbFetchRealTimeFrame(int device, float *traceMin, float *traceMax, float *frame, float *alphaFrame);

/**
 * This function retrieves one block of I/Q data as specified by the
 * #bbIQPacket struct. The members of the #bbIQPacket struct and how they
 * affect the acquisition are described in the parameters section.
 *
 * The timestamps returned will either be synchronized to the GPS if it was
 * properly configured or the PC system clock if not. For timestamps generated
 * by the system clock, one should only use the first timestamp collected and
 * use the index and sample rate to determine the time of an individual sample.
 *
 * The BB60 will report ~5k triggers per second. Use an adequate size trigger
 * buffer if you wish to receive all potential triggers. If the API has more
 * triggers to report than the size of the buffer provided, any excess triggers
 * will be discarded.
 *
 * @param[in] device Device handle.
 *
 * @param[out] pkt Pointer to a #bbIQPacket structure.
 *
 * @return
 */
BB_API bbStatus bbGetIQ(int device, bbIQPacket *pkt);

/**
 * This function provides a method for retrieving I/Q data without needing the
 * #bbIQPacket struct. Each parameter in #bbGetIQUnpacked has a one-to-one
 * mapping to variables found in the #bbIQPacket struct. This function serves
 * as a convenience for creating bindings in various programming languages and
 * environments such as Python, C#, LabVIEW, MATLAB, etc. This function is
 * implemented by taking the parameters provided into the #bbIQPacket struct
 * and calling #bbGetIQ.
 *
 * @param[in] device Device handle.
 *
 * @param[out] iqData Pointer to an array of 32-bit complex floating-point
 * values. Complex values are interleaved real-imaginary pairs. This must point
 * to a contiguous block of _iqCount_ complex pairs.
 *
 * @param[in] iqCount Number of I/Q data pairs to return.
 *
 * @param[out] triggers Pointer to an array of integers. If the external
 * trigger input is active, and a trigger occurs during the acquisition time,
 * triggers will be populated with values which are relative indices into the
 * _iqData_ array where external triggers occurred. Any unused trigger array
 * values will be set to zero.
 *
 * @param[in] triggerCount Size of the triggers array.
 *
 * @param[in] purge Specifies whether to discard any samples acquired by the
 * API since the last time a #bbGetIQ function was called. Set to #BB_TRUE if
 * you wish to discard all previously acquired data, and #BB_FALSE if you wish
 * to retrieve the contiguous I/Q values from a previous call to this function.
 *
 * @param[out] dataRemaining How many I/Q samples are still left buffered in
 * the API.
 *
 * @param[out] sampleLoss Returns #BB_TRUE or #BB_FALSE. Will return #BB_TRUE
 * when the API is required to drop data due to internal buffers wrapping. This
 * can be caused by I/Q samples not being polled fast enough, or in instances
 * where the processing is not able to keep up (underpowered systems, or other
 * programs utilizing the CPU) Will return #BB_TRUE on the capture in which the
 * sample break occurs. Does not indicate which sample the break occurs on.
 * Will always return false if _purge_ is true.
 *
 * @param[out] sec Seconds since epoch representing the timestamp of the first
 * sample in the returned array.
 *
 * @param[out] nano Nanoseconds representing the timestamp of the first sample
 * in the returned array.
 *
 * @return
 */
BB_API bbStatus bbGetIQUnpacked(int device, void *iqData, int iqCount, int *triggers,
                                int triggerCount, int purge, int *dataRemaining,
                                int *sampleLoss, int *sec, int *nano);

/**
 * If the device is initiated and running in the audio demodulation mode, the
 * function is a blocking call which returns the next 4096 audio samples. The
 * approximate blocking time for this function is 128 ms if called again
 * immediately after returning. There is no internal buffering of audio,
 * meaning the audio will be overwritten if this function is not called in a
 * timely fashion. The audio values are typically -1.0 to 1.0, representing
 * full-scale audio. In FM mode, the audio values will scale with a change in
 * IF bandwidth.
 *
 * @param[in] device Device handle.
 *
 * @param[out] audio Pointer to an array of 4096 32-bit floating point values.
 *
 * @return
 */
BB_API bbStatus bbFetchAudio(int device, float *audio);

/**
 * This function connects a TG device and associates it with the specified BB60
 * device. Once the TG is opened, it cannot be opened in any other application
 * until the BB60 is closed via #bbCloseDevice.
 *
 * @param[in] device Device handle.
 *
 * @return
 */
BB_API bbStatus bbAttachTg(int device);

/**
 * This is a helper function to determine if a Signal Hound tracking generator
 * has been previously paired with the specified device.
 *
 * @param[in] device Device handle.
 *
 * @param[out] attached Pointer to a boolean variable. If this function returns
 * successfully, the variable _attached_ points to will contain a true/false
 * value as to whether a tracking generator is paired with the spectrum
 * analyzer.
 *
 * @return
 */
BB_API bbStatus bbIsTgAttached(int device, bool *attached);

/**
 * This function configures the tracking generator sweeps. Through this
 * function you can request a sweep size. The sweep size is the number of
 * discrete points returned in the sweep over the configured span. The final
 * value chosen by the API can be different than the requested size by a factor
 * of 2 at most. The dynamic range of the sweep is determined by the choice of
 * _highDynamicRange_ and _passiveDevice_. A value of true for both provides
 * the highest dynamic range sweeps. Choosing false for _passiveDevice_
 * suggests to the API that the device under test is an active device
 * (amplification).
 *
 * @param[in] device Device handle.
 *
 * @param[in] sweepSize Suggested sweep size.
 *
 * @param[in] highDynamicRange Request the ability to perform two store
 * throughs for an increased dynamic range sweep.
 *
 * @param[in] passiveDevice Specify whether the device under test is a passive
 * device (no gain).
 *
 * @return
 */
BB_API bbStatus bbConfigTgSweep(int device, int sweepSize, bool highDynamicRange, bool passiveDevice);

/**
 * This function, with flag set to #TG_THRU_0DB, notifies the API to use the
 * next trace as a thru (your 0 dB reference). Connect your tracking generator
 * RF output to your spectrum analyzer RF input. This can be accomplished using
 * the included SMA to SMA adapter, or anything else you want the software to
 * establish as the 0 dB reference (e.g. the 0 dB setting on a step attenuator,
 * or a 20 dB attenuator you will be including in your amplifier test setup).
 *
 * After you have established your 0 dB reference, a second step may be
 * performed to improve the accuracy below -40 dB. With approximately 20-30 dB
 * of insertion loss between the spectrum analyzer and tracking generator, call
 * #bbStoreTgThru with flag set to #TG_THRU_20DB. This corrects for slight
 * variations between the high gain and low gain sweeps.
 *
 * @param[in] device Device handle.
 *
 * @param[in] flag Specify the type of store thru. Possible values are
 * #TG_THRU_0DB, #TG_THRU_20DB.
 *
 * @return
 */
BB_API bbStatus bbStoreTgThru(int device, int flag);

/**
 * This function sets the output frequency and amplitude of the tracking
 * generator. This can only be performed if a tracking generator is paired with
 * a spectrum analyzer and is currently not configured and initiated for TG
 * sweeps.
 *
 * @param[in] device Device handle.
 *
 * @param[in] frequency Set the frequency, in Hz, of the TG output.
 *
 * @param[in] amplitude Set the amplitude, in dBm, of the TG output.
 *
 * @return
 */
BB_API bbStatus bbSetTg(int device, double frequency, double amplitude);

/**
 * Retrieve the last set TG output parameters the user set through the #bbSetTg
 * function. The #bbSetTg function must have been called for this function to
 * return valid values. If the TG was used to perform scalar network analysis
 * at any point, this function will not return valid values until the #bbSetTg
 * function is called again. If a previously set parameter was clamped in the
 * #bbSetTg function, this function will return the final clamped value. If any
 * pointer parameter is null, that value is ignored and not returned.
 *
 * @param[in] device Device handle.
 *
 * @param[out] frequency Pointer to a double that will contain the last set
 * frequency of the TG output in Hz.
 *
 * @param[out] amplitude Pointer to a double that will contain the last set
 * amplitude of the TG output in dBm.
 *
 * @return
 */
BB_API bbStatus bbGetTgFreqAmpl(int device, double *frequency, double *amplitude);

/**
 * Configure the time base for the tracking generator attached to the device
 * specified. When #TG_REF_UNUSED is specified additional frequency corrections
 * are applied. If using an external reference or you are using the TG time
 * base frequency as the frequency standard for your system, you will want to
 * specify #TG_REF_INTERNAL_OUT or #TG_REF_EXTERNAL_IN so the additional
 * corrections are not applied.
 *
 * @param[in] device Device handle.
 *
 * @param[in] reference A valid time base setting value. Possible values are
 * #TG_REF_UNUSED, #TG_REF_INTERNAL_OUT, #TG_REF_EXTERNAL_IN.
 *
 * @return
 */
BB_API bbStatus bbSetTgReference(int device, int reference);

/**
 * Get API version.
 *
 * @return The returned string is of the form _major.minor.revision_. Ascii
 * periods (“.”) separate positive integers. Major/Minor/Revision are not
 * guaranteed to be a single decimal digit. The string is null terminated. An
 * example string: [ ‘3’ | ‘.’ | ‘0’ | ‘.’ | ‘1’ | ‘1’ | ‘\0’ ] = “3.0.11”
 *
 */
BB_API const char* bbGetAPIVersion();

/**
 * Get product ID.
 *
 * @return The returned string is of the form ####-####.
 */
BB_API const char* bbGetProductID();

/**
 * Get an ascii string description of a provided status code.
 *
 * @param[in] status A #bbStatus value returned from an API call.
 *
 * @return A pointer to a non-modifiable null terminated string. The memory
 * should not be freed/deallocated.
 */
BB_API const char* bbGetErrorString(bbStatus status);

// Deprecated functions, use suggested alternatives

// Use bbConfigureRefLevel instead
BB_API bbStatus bbConfigureLevel(int device, double ref, double atten);
// Use bbConfigureGainAtten instead
BB_API bbStatus bbConfigureGain(int device, int gain);
// Use bbQueryIQParameters instead
BB_API bbStatus bbQueryStreamInfo(int device, int *return_len, double *bandwidth, int *samples_per_sec);

#ifdef __cplusplus
} // extern "C"
#endif

// Deprecated macros, use alternatives where available
#define BB60_MIN_FREQ (BB_MIN_FREQ)
#define BB60_MAX_FREQ (BB_MAX_FREQ)
#define BB60_MAX_SPAN (BB_MAX_SPAN)
#define BB_MIN_BW (BB_MIN_RBW)
#define BB_MAX_BW (BB_MAX_RBW)
#define BB_MAX_ATTENUATION (30.0) // For deprecated bbConfigureLevel function
#define BB60C_MAX_GAIN (BB_MAX_GAIN)
#define BB_PORT1_INT_REF_OUT (0x00)
#define BB_PORT1_EXT_REF_IN (BB60C_PORT1_10MHZ_REF_IN)
#define BB_RAW_PIPE (BB_STREAMING)
#define BB_STREAM_IF (0x1) // No longer supported
// Use new device specific port 1 macros
#define BB_PORT1_AC_COUPLED (BB60C_PORT1_AC_COUPLED)
#define BB_PORT1_DC_COUPLED (BB60C_PORT1_DC_COUPLED)
#define BB_PORT1_10MHZ_USE_INT (BB60C_PORT1_10MHZ_USE_INT)
#define BB_PORT1_10MHZ_REF_OUT (BB60C_PORT1_10MHZ_REF_OUT)
#define BB_PORT1_10MHZ_REF_IN (BB60C_PORT1_10MHZ_REF_IN)
#define BB_PORT1_OUT_LOGIC_LOW (BB60C_PORT1_OUT_LOGIC_LOW)
#define BB_PORT1_OUT_LOGIC_HIGH (BB60C_PORT1_OUT_LOGIC_HIGH)
// Use new device specific port 2 macros
#define BB_PORT2_OUT_LOGIC_LOW (BB60C_PORT2_OUT_LOGIC_LOW)
#define BB_PORT2_OUT_LOGIC_HIGH (BB60C_PORT2_OUT_LOGIC_HIGH)
#define BB_PORT2_IN_TRIGGER_RISING_EDGE (BB60C_PORT2_IN_TRIG_RISING_EDGE)
#define BB_PORT2_IN_TRIGGER_FALLING_EDGE (BB60C_PORT2_IN_TRIG_FALLING_EDGE)

#endif // BB_API_H

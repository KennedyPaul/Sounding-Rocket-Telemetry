// RH_E32.h
//
// Definitions for E32-TTL-1W family serial radios:
//  http://www.cdebyte.com/en/product-view-news.aspx?id=108
//
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2017 Mike McCauley
// $Id: RH_E32.h,v 1.4 2018/02/11 23:57:18 mikem Exp $
//

#ifndef RH_E32_h
#define RH_E32_h

#include <RHGenericDriver.h>

// The buffer in the E32 is 512 bytes, but we arbitrarily limit messages to a maximum of 58 octets
// We use some for headers, keeping fewer for RadioHead messages
// The E32 sends messages longer than 58 octets in several packets of 58 octets each, and since we dont have any
// framing or message level checksums there is a risk that long messages from 2 different sources
// could be incorrectly interpreted as a single message
#define RH_E32_MAX_PAYLOAD_LEN 58

// The length of the headers we add:
//  message length (including these headers)
//  to
//  from
//  id
//  flags
// The headers are inside the E32 payload
#define RH_E32_HEADER_LEN 5

// This is the maximum RadioHead user message length that can be supported by this module. Limited by
#define RH_E32_MAX_MESSAGE_LEN (RH_E32_MAX_PAYLOAD_LEN-RH_E32_HEADER_LEN)//53

// Commands to alter module behaviour
#define RH_E32_COMMAND_WRITE_PARAMS_SAVE         0xC0
#define RH_E32_COMMAND_READ_PARAMS               0xC1
#define RH_E32_COMMAND_WRITE_PARAMS_NOSAVE       0xC2
#define RH_E32_COMMAND_READ_VERSION              0xC3
#define RH_E32_COMMAND_RESET                     0xC4

// Various flags and masks for param bytes
#define RH_E32_PARAM_SPED_UART_MODE_MASK         0xC0
#define RH_E32_PARAM_SPED_UART_MODE_8N1          0x00
#define RH_E32_PARAM_SPED_UART_MODE_8O1          0x40
#define RH_E32_PARAM_SPED_UART_MODE_8E1          0x80

#define RH_E32_PARAM_SPED_UART_BAUD_MASK         0x38
#define RH_E32_PARAM_SPED_UART_BAUD_1200         0x00
#define RH_E32_PARAM_SPED_UART_BAUD_2400         0x08
#define RH_E32_PARAM_SPED_UART_BAUD_4800         0x10
#define RH_E32_PARAM_SPED_UART_BAUD_9600         0x18
#define RH_E32_PARAM_SPED_UART_BAUD_19200        0x20
#define RH_E32_PARAM_SPED_UART_BAUD_38400        0x28
#define RH_E32_PARAM_SPED_UART_BAUD_57600        0x30
#define RH_E32_PARAM_SPED_UART_BAUD_115200       0x38

#define RH_E32_PARAM_SPED_DATARATE_MASK          0x07
#define RH_E32_PARAM_SPED_DATARATE_1KBPS         0x00
#define RH_E32_PARAM_SPED_DATARATE_2KBPS         0x01
#define RH_E32_PARAM_SPED_DATARATE_5KBPS         0x02
#define RH_E32_PARAM_SPED_DATARATE_8KBPS         0x03
#define RH_E32_PARAM_SPED_DATARATE_10KBPS        0x04
#define RH_E32_PARAM_SPED_DATARATE_15KBPS        0x05
#define RH_E32_PARAM_SPED_DATARATE_20KBPS        0x06
#define RH_E32_PARAM_SPED_DATARATE_25KBPS        0x07

#define RH_E32_PARAM_OPTION_FIXED_MASK           0x80

#define RH_E32_PARAM_OPTION_IODRIVE_MASK         0x40

#define RH_E32_PARAM_OPTION_WAKEUP_TIME_MASK     0x38
#define RH_E32_PARAM_OPTION_WAKEUP_TIME_250MS    0x00
#define RH_E32_PARAM_OPTION_WAKEUP_TIME_500MS    0x08
#define RH_E32_PARAM_OPTION_WAKEUP_TIME_750MS    0x10
#define RH_E32_PARAM_OPTION_WAKEUP_TIME_1000MS   0x18
#define RH_E32_PARAM_OPTION_WAKEUP_TIME_1250MS   0x20
#define RH_E32_PARAM_OPTION_WAKEUP_TIME_1500MS   0x28
#define RH_E32_PARAM_OPTION_WAKEUP_TIME_1750MS   0x30
#define RH_E32_PARAM_OPTION_WAKEUP_TIME_2000MS   0x38

#define RH_E32_PARAM_OPTION_FEC_MASK             0x04

#define RH_E32_PARAM_OPTION_POWER_MASK           0x03
#define RH_E32_PARAM_OPTION_POWER_30DBM          0x00
#define RH_E32_PARAM_OPTION_POWER_27DBM          0x01
#define RH_E32_PARAM_OPTION_POWER_24DBM          0x02
#define RH_E32_PARAM_OPTION_POWER_21DBM          0x03


class Stream;
class RH_E32 : public RHGenericDriver
{
 public:

    /// \brief Values to be passed to setDataRate() to control the on-air data rate
    ///
    /// This is NOT to be used to control the baud rate of the serial connection to the radio
    typedef enum
    {
      DataRate1kbps  = RH_E32_PARAM_SPED_DATARATE_1KBPS,
      DataRate2kbps  = RH_E32_PARAM_SPED_DATARATE_2KBPS,
      DataRate5kbps  = RH_E32_PARAM_SPED_DATARATE_5KBPS,
      DataRate8kbps  = RH_E32_PARAM_SPED_DATARATE_8KBPS,
      DataRate10kbps = RH_E32_PARAM_SPED_DATARATE_10KBPS,
      DataRate15kbps = RH_E32_PARAM_SPED_DATARATE_15KBPS,
      DataRate20kbps = RH_E32_PARAM_SPED_DATARATE_20KBPS,
      DataRate25kbps = RH_E32_PARAM_SPED_DATARATE_25KBPS
    } DataRate;

    /// \brief Values to be passed to setPower() to control the transmitter power
    ///
    typedef enum
    {
      Power30dBm = RH_E32_PARAM_OPTION_POWER_30DBM,
      Power27dBm = RH_E32_PARAM_OPTION_POWER_27DBM,
      Power24dBm = RH_E32_PARAM_OPTION_POWER_24DBM,
      Power21dBm = RH_E32_PARAM_OPTION_POWER_21DBM,
    } PowerLevel;

    /// \brief Values to be passed to setBaudRate() to control the radio serial connection baud rate
    ///
    /// This is NOT to be used to control the on-air data rate the radio transmits and receives at
    typedef enum
    {
      BaudRate1200   = RH_E32_PARAM_SPED_UART_BAUD_1200,
      BaudRate2400   = RH_E32_PARAM_SPED_UART_BAUD_2400,
      BaudRate4800   = RH_E32_PARAM_SPED_UART_BAUD_4800,
      BaudRate9600   = RH_E32_PARAM_SPED_UART_BAUD_9600,
      BaudRate19200  = RH_E32_PARAM_SPED_UART_BAUD_19200,
      BaudRate38400  = RH_E32_PARAM_SPED_UART_BAUD_38400,
      BaudRate57600  = RH_E32_PARAM_SPED_UART_BAUD_57600,
      BaudRate115200 = RH_E32_PARAM_SPED_UART_BAUD_115200,
    } BaudRate;

    /// \brief Values to be passed to setBaudRate() to control the parity of the serial connection to the radio
    typedef enum
    {
      Parity8N1 = RH_E32_PARAM_SPED_UART_MODE_8N1,
      Parity8O1 = RH_E32_PARAM_SPED_UART_MODE_8O1,
      Parity8E1 = RH_E32_PARAM_SPED_UART_MODE_8E1,
    } Parity;

    /// Contructor. You can have multiple instances, but each instance must have its own
    /// serial connection, M0 M1 and AUX connections. Initialises the mode of the referenced pins
    /// Does NOT set the baud rate of the serial connection to the radio.
    /// \param[in] s Reference to the SoftwareSerial or HardwareSerial port used to connect to the radio
    /// \param[in] m0_pin Pin number of the Arduino pin that connects to the radio M0 input
    /// \param[in] m1_pin Pin number of the Arduino pin that connects to the radio M1 input
    /// \param[in] aux_pin Pin number of the Arduino pin that connects to the radio AUX output
    RH_E32(Stream *s=&Serial, uint8_t m0_pin = 4, uint8_t m1_pin = 5, uint8_t aux_pin = 8);

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly, including setting the serial port baud rate and parity to that
    /// configured in the radio (typically 9600 baud, 8N1) before calling init().
    /// Sets the module to 443MHz, 21dBm power and 5kbps data rate (you can change these after initialisation with
    /// the various set* functions).
    /// This function may not return if the AUX pin is not connected.
    /// Initialisation failure can be caused by:
    /// Electrical connections to the radio incorrect or incomplete
    /// Radio configured to use a different baud rate to the one configured to the Ardiono serial port
    /// Incorrect radio module connected tot he serial port.
    /// Other serial communicaitons problems between the Arduino and the radio
    /// \return true if initialisation succeeded.
    bool init(uint8_t mode = 1);

    /// Tests whether a new message is available
    /// from the Driver.
    /// This can and should be called multiple times in a timeout loop. You should call this as frequently as possible
    /// whenever a message might be received
    /// \return true if a new, complete, error-free uncollected message is available to be retreived by recv().
    bool available();

    /// If there is a valid message available, copy it to buf and return true
    /// else return false.
    /// If a message is copied, *len is set to the length (Caution, 0 length messages are permitted).
    /// You should be sure to call this function frequently enough to not miss any messages
    /// It is recommended that you call it in your main loop.
    /// \param[in] buf Location to copy the received message
    /// \param[in,out] len Pointer to available space in buf. Set to the actual number of octets copied.
    /// \return true if a valid message was copied to buf
    bool recv(uint8_t* buf, uint8_t* len);

    /// Waits until any previous transmit packet is finished being transmitted with waitPacketSent().
    /// Then loads a message into the transmitter and starts the transmitter. Note that a message length
    /// of 0 is permitted.
    /// \param[in] data Array of data to be sent
    /// \param[in] len Number of bytes of data to send
    /// \return true if the message length was valid and it was correctly queued for transmit. Return false
    /// if CAD was requested and the CAD timeout timed out before clear channel was detected.
    bool send(const char * data, uint8_t len);

    /// Returns the maximum message length
    /// available in this Driver.
    /// \return The maximum legal message length
    uint8_t maxMessageLength();

    /// Waits for any currently transmitting packet to be completely sent
    /// Returns true if successful
    bool waitPacketSent();

    /// Sets the on-air data rate to be used by the transmitter and receiver
    /// \param[in] rate A valid data rate from the DataRate enum
    /// \return true if successful
    bool setDataRate(DataRate rate);

    /// Sets the transmitter power output
    /// \param[in] level A valid power setting from the Power enum
    /// \return true if successful
    bool setPower(PowerLevel level);

    /// Sets the radio serial port baud rate and parity (not the on-air data rate)
    /// Does not set the Aruino rate or parity: you wil nned to do this afterwards
    /// \param[in] rate A valid baud rate from the BaudRate enum
    /// \param[in] parity A valid parity from the PArity enum
    /// \return true if successful
    bool setBaudRate(BaudRate rate = BaudRate9600, Parity parity = Parity8N1);

    /// Sets the tarnsmitter and receiver frequency.
    /// \param[in] frequency Desired frequency in MHx from 410 to 441 MHz inclusive
    /// \return true if successful
    bool setFrequency(uint16_t frequency);

    /// Waits for the AUX pin to go high
    /// For internal use only
    void waitAuxHigh();

    /// Waits for the AUX pin to go low
    /// For internal use only
    void waitAuxLow();

protected:

    /// \brief Defines values to be passed to setOperatinMode
    ///
    /// For internal driver user only
    typedef enum
    {
      ModeNormal = 0,    ///< Normal mode for sending and receiving messages
      ModeWakeUp,        ///< Adds a long preamble to transmission to allow destination receivers to wake up
      ModePowerSaving,   ///< Receiver sleeps until a message is received
      ModeSleep          ///< Use during parameter setting
    } OperatingMode;

    /// \brief Structure for reading and writing radio control parameters
    ///
    /// For internal driver user only
    typedef struct
    {
      uint8_t head;      ///< 0xc2 (no save) or 0xc0 (save)
      uint8_t addh;      ///< High address byte (not used by this driver)
      uint8_t addl;      ///< Low address byte (not used by this driver)
      uint8_t sped;      ///< Data and baud rate parameters
      uint8_t chan;      ///< Radio channel
      uint8_t option;    ///< Various control options

    } Parameters;

    /// Sets the operating mode of the radio.
    /// For internal use only
    void setOperatingMode(OperatingMode mode);

    /// Retrieves the version number for the radio and checks that it is valid
    /// \return true if the version could be retrieved and is radio model number is correct
    bool getVersion();

    /// Issues a reset command to the radio
    /// WARNING: this seems to break reception. Why?
    /// \return true if successful
    bool reset();

    /// Read the radio configuration parameters into
    /// local memory
    /// \param[in] params Reference to a Parameter structure which will be filled if successful
    /// \return true if successful
    bool readParameters(Parameters& params);

    /// Write radio configuration parameters from local memory
    /// to the radio. You can choose whether the parameter will be saved across power down or not
    /// \param[in] params Reference to a Parameter structure containing the radio configuration parameters
    /// to be written to the radio.
    /// \param[in] save If true, the parameters will be saved across power down in the radio
    /// \return true if successful
    bool writeParameters(Parameters& params, bool save = false);

    /// Examine the receive buffer to determine whether the message is for this node
    /// For internal use only
    void validateRxBuf();

    /// Clear our local receive buffer
    /// For internal use only
    void clearRxBuf();

private:
    /// Serial stream (hardware or software serial)
    Stream*     _s;

    /// Pin number connected to M0
    uint8_t     _m0_pin;

    /// Pin number connected to M1
    uint8_t     _m1_pin;

    /// Pin number connected to AUX
    uint8_t     _aux_pin;

    /// Number of octets in the buffer
    uint8_t             _bufLen;

    /// The receiver/transmitter buffer
    uint8_t             _buf[RH_E32_MAX_PAYLOAD_LEN];

    /// True when there is a valid message in the buffer
    bool                _rxBufValid;

};

/// @example e32_client.pde
/// @example e32_server.pde

#endif

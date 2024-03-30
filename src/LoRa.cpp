// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <LoRa.h>

// The SX1276 offers a number of configuration and status registers which can be accessed via the SPI interface.
// The following are the register addresses of the SX1276 that are used in this library.
// Registers name and address. Check the datasheet for specific bitfields description.

#define REG_FIFO                 0x00 // FIFO address pointer in TX and RX mode. It is cleared and not accessible in Sleep mode.
#define REG_OP_MODE              0x01 //Lora vs FSK/OOK mode selection and access to shared registers
#define REG_FRF_MSB              0x06 //MSB of RF carrier frequency
#define REG_FRF_MID              0x07 //Middle byte of RF carrier frequency
#define REG_FRF_LSB              0x08 //LSB of RF carrier frequency
#define REG_PA_CONFIG            0x09 //Power amplifier config
#define REG_OCP                  0x0B //Over current protection control
#define REG_LNA                  0x0C //Low noise amplifier settings
#define REG_FIFO_ADDR_PTR        0x0D //Location pointer in FIFO data buffer for TX/RX operations. Before any TX/RX operation, it must be set to the corresponding base address.
#define REG_FIFO_TX_BASE_ADDR    0x0E //Write base address in FIFO data buffer for TX modulator
#define REG_FIFO_RX_BASE_ADDR    0x0F //Read base address in FIFO data buffer for RX demodulator
#define REG_FIFO_RX_CURRENT_ADDR 0x10 //Indicates the location of the last packet received in the FIFO
#define REG_IRQ_FLAGS            0x12 //Interrupt flags for TX and RX events, which can be polled or interrupt driven via DIO0 pin.
#define REG_RX_NB_BYTES          0x13 //Number of payload bytes of latest packet received for explicit header mode.
#define REG_PKT_SNR_VALUE        0x19 //Average SNR of last packet received (dB)
#define REG_PKT_RSSI_VALUE       0x1A //Average RSSI of last packet received
#define REG_RSSI_VALUE           0x1B //Instantaneous RSSI value of the last packet received
#define REG_MODEM_CONFIG_1       0x1D //Bandwidth, coding rate, and header mode settings
#define REG_MODEM_CONFIG_2       0x1E //Spreading factor, TX continuous mode, CRC, and RX timeout MSB
#define REG_PREAMBLE_MSB         0x20 //MSB of set preamble length
#define REG_PREAMBLE_LSB         0x21 //LSB of set preamble length
#define REG_PAYLOAD_LENGTH       0x22 //Indicates how many bytes are queued for transmission in the FIFO
#define REG_MODEM_CONFIG_3       0x26 //Low data rate optimization setting and AGC auto on/off control
#define REG_FREQ_ERROR_MSB       0x28 //Estimated frequency error MSB
#define REG_FREQ_ERROR_MID       0x29 //Estimated frequency error middle byte
#define REG_FREQ_ERROR_LSB       0x2A //Estimated frequency error LSB
#define REG_RSSI_WIDEBAND        0x2C //Can be used to generate a random number between 0 and 255 using the RSSI noise floor as a source of entropy
#define REG_DETECTION_OPTIMIZE   0x31 //Detection optimization when using Spreading Factor 6
#define REG_INVERTIQ             0x33 //Invert LoRa I and Q signals
#define REG_DETECTION_THRESHOLD  0x37 //RSSI detection threshold for receiver
#define REG_SYNC_WORD            0x39 //Register that stores the network ID. Both transmitter and receiver must have the same network ID.
#define REG_INVERTIQ2            0x3B
#define REG_DIO_MAPPING_1        0x40 //Mapping of pins DIO0 to DIO3, with DIO0 occupying bits [7-6]
#define REG_VERSION              0x42 //Semtech ID relating the silicon revision
#define REG_PA_DAC               0x4D //Higher power settings of the PA

// Operation modes on RegOpMode(0x01)
#define MODE_LONG_RANGE_MODE     BIT(7) // 0 = FSK/OOK mode, 1 = LoRa mode. This can only be set in SLEEP mode.

//Bitfield [2-0] from RegOpMode(0x01)
#define MODE_CAD                 0b111 // Modem is checking for preamble symbols to detected activity while using less power than RX modes.
#define MODE_RX_SINGLE           0b110 // Reception blocks are powered up until a valid packet is received. Then returns to STANDBY mode.
#define MODE_RX_CONTINUOUS       0b101 // Reception blocks are powered, processing all received data until user requests change of mode.
#define MODE_FSRX				 0b100 // Frequency synthesis RX mode. PLL is locked and active at receive frequency. RF part is off.
#define MODE_TX                  0b011 // Transmission blocks are powered, the PA is being ramnped, the packet is transmitted and returns to STANDBY mode.
#define MODE_FSTX				 0b010 // Frequency synthesis TX mode. PLL is locked and active at transmit frequency. RF part is off.
#define MODE_STDBY               0b001 // Both crystal oscillator and Lora baseband blocks are enabled. RF and PLLs are disabled.
#define MODE_SLEEP               0b000 // Low power mode with only SPI and config registers acessible. Lora FIFO is not accessible.

// Power amplifier config for RegPaConfig(0x09) to set 20dBm output power on
#define PA_BOOST 	BIT(7)

// IRQ positions for reading RegIrqFLags(0x12) register, which contains multiple bits that are set when certain modem events occur.
// Polling can be done by reading this register and checking if any of these bits are high
// However, using interrupts on DIO0 pin is a better way to do it as it does not waste CPU cycles
// If you want to disable any of these interrupts, the same positions can be used to set the bits
// in RegIrqFlagsMask(0x11) register. This is useful when you want to receive only certain interrupts
// and ignore others, sparing some CPU cycles from an if/else statement.

#define IRQ_RX_TIMEOUT_MASK       		BIT(7)
#define IRQ_RX_DONE_MASK           		BIT(6)
#define IRQ_PAYLOAD_CRC_ERROR_MASK 		BIT(5)
#define IRQ_VALID_HEADER_MASK      		BIT(4)
#define IRQ_TX_DONE_MASK           		BIT(3)
#define IRQ_CAD_DONE_MASK          		BIT(2)
#define IRQ_FHSS_CHANGE_CHANNEL_MASK 	BIT(1)
#define IRQ_CAD_DETECTED_MASK      		BIT(0)

// DIO0 function is mapped on bits [7-6] of RegDioMapping1(0x40) register.
// It is used to map the interrupt signal to DIO0 pin,, which can be used to trigger an interrupt on the microcontroller.

#define DIO_RX_DONE (0b00 << 6)
#define DIO_TX_DONE (0b01 << 6)
#define DIO_CAD_DONE (0b10 << 6)

#define RF_MID_BAND_THRESHOLD    5256
#define RSSI_OFFSET_HF_PORT      157
#define RSSI_OFFSET_LF_PORT      164

#define MAX_PKT_LENGTH           255 //FIFO max size

#if (ESP8266 || ESP32)
    #define ISR_PREFIX ICACHE_RAM_ATTR
#else
    #define ISR_PREFIX
#endif

LoRaClass::LoRaClass() :
    _spiSettings(LORA_DEFAULT_SPI_FREQUENCY, MSBFIRST, SPI_MODE0),
    _spi(&LORA_DEFAULT_SPI),
    _ss(LORA_DEFAULT_SS_PIN), _reset(LORA_DEFAULT_RESET_PIN), _dio0(LORA_DEFAULT_DIO0_PIN),
    _frequency(0),
    _packetIndex(0),
    _implicitHeaderMode(0),
    _onReceive(NULL),
    _onCadDone(NULL),
    _onTxDone(NULL)
{
    // overide Stream timeout value
    setTimeout(0);
}

int LoRaClass::begin(long frequency)
{
    #if defined(ARDUINO_SAMD_MKRWAN1300) || defined(ARDUINO_SAMD_MKRWAN1310)
    pinMode(LORA_IRQ_DUMB, OUTPUT);
    digitalWrite(LORA_IRQ_DUMB, LOW);

    // Hardware reset
    pinMode(LORA_BOOT0, OUTPUT);
    digitalWrite(LORA_BOOT0, LOW);

    pinMode(LORA_RESET, OUTPUT);
    digitalWrite(LORA_RESET, HIGH);
    delay(200);
    digitalWrite(LORA_RESET, LOW);
    delay(200);
    digitalWrite(LORA_RESET, HIGH);
    delay(50);
    #endif

    // setup pins
    pinMode(_ss, OUTPUT);
    // set SS high
    digitalWrite(_ss, HIGH);

    if (_reset != -1) {
        pinMode(_reset, OUTPUT);

        // perform reset
        digitalWrite(_reset, LOW);
        delay(10);
        digitalWrite(_reset, HIGH);
        delay(10);
    }

    // start SPI
    _spi->begin();

    // check version
    uint8_t version = readRegister(REG_VERSION);
    if (version != 0x12) {
        return 0;
    }

    // put in sleep mode
    sleep();

    // set frequency
    setFrequency(frequency);

    // set base addresses
    writeRegister(REG_FIFO_TX_BASE_ADDR, 0);
    writeRegister(REG_FIFO_RX_BASE_ADDR, 0);

    // set LNA boost
    writeRegister(REG_LNA, readRegister(REG_LNA) | 0x03);

    // set auto AGC
    writeRegister(REG_MODEM_CONFIG_3, 0x04);

    // set output power to 17 dBm
    setTxPower(17);

    // put in standby mode
    standby();

    return 1;
}

void LoRaClass::end()
{
    // put in sleep mode
    sleep();

    // stop SPI
    _spi->end();
}

int LoRaClass::beginPacket(int implicitHeader)
{
    if (isTransmitting()) {
        return 0;
    }

    // put in standby mode
    standby();

    if (implicitHeader) {
        implicitHeaderMode();
    } else {
        explicitHeaderMode();
    }

    // reset FIFO address and paload length
    writeRegister(REG_FIFO_ADDR_PTR, 0);
    writeRegister(REG_PAYLOAD_LENGTH, 0);

    return 1;
}

int LoRaClass::endPacket(bool async)
{

    //DIO0 function is mapped on bits [7-6] of RegDioMapping1(0x40) register.
    //0000 0000 => DIO0 => RXDONE
    //0100 0000 => DIO0 => TXDONE
    //1000 0000 => DIO0 => CADDONE
  
    if ((async) && (_onTxDone))
        writeRegister(REG_DIO_MAPPING_1, 0x40); // DIO0 => TXDONE

    // put in TX mode
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    if (!async) {
        // wait for TX done
        while ((readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0) {
        yield();
        }
        // clear IRQ's
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return 1;
}

bool LoRaClass::isTransmitting()
{
    if ((readRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
        return true;
    }

    if (readRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
        // clear IRQ's
        writeRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    }

    return false;
}

int LoRaClass::parsePacket(int size)
{
    int packetLength = 0;
    int interruptRequestFlags = readRegister(REG_IRQ_FLAGS);

    if (size > 0) {
        implicitHeaderMode();

        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        explicitHeaderMode();
    }

    // clear IRQ's
    writeRegister(REG_IRQ_FLAGS, interruptRequestFlags);

    if ((interruptRequestFlags & IRQ_RX_DONE_MASK) && (interruptRequestFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) {
        // received a packet
        _packetIndex = 0;

        // read packet length
        if (_implicitHeaderMode) {
        packetLength = readRegister(REG_PAYLOAD_LENGTH);
        } else {
        packetLength = readRegister(REG_RX_NB_BYTES);
        }

        // set FIFO address to current RX address
        writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

        // put in standby mode
        standby();
    } else if (readRegister(REG_OP_MODE) != (MODE_LONG_RANGE_MODE | MODE_RX_SINGLE)) {
        // not currently in RX mode

        // reset FIFO address
        writeRegister(REG_FIFO_ADDR_PTR, 0);

        // put in single RX mode
        writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
    }

    return packetLength;
}

int LoRaClass::packetRssi()
{
     return (readRegister(REG_PKT_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

float LoRaClass::packetSnr()
{
    return ((int8_t)readRegister(REG_PKT_SNR_VALUE)) * 0.25;
}

long LoRaClass::packetFrequencyError()
{
    int32_t freqError = 0;
    freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB) & 0b111);
    freqError <<= 8L;
    freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID));
    freqError <<= 8L;
    freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB));

    if (readRegister(REG_FREQ_ERROR_MSB) & 0b1000) { // Sign bit is on
        freqError -= 524288; // 0b1000'0000'0000'0000'0000
    }

    const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
    const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f); // p. 37

    return static_cast<long>(fError);
}

int LoRaClass::rssi()
{
    return (readRegister(REG_RSSI_VALUE) - (_frequency < RF_MID_BAND_THRESHOLD ? RSSI_OFFSET_LF_PORT : RSSI_OFFSET_HF_PORT));
}

size_t LoRaClass::write(uint8_t byte)
{
    return write(&byte, sizeof(byte));
}

size_t LoRaClass::write(const uint8_t *buffer, size_t size)
{
    int currentLength = readRegister(REG_PAYLOAD_LENGTH);

    // check size
    if ((currentLength + size) > MAX_PKT_LENGTH) {
        size = MAX_PKT_LENGTH - currentLength;
    }

    // write data
    for (size_t i = 0; i < size; i++) {
        writeRegister(REG_FIFO, buffer[i]);
    }

    // update length
    writeRegister(REG_PAYLOAD_LENGTH, currentLength + size);

    return size;
}

int LoRaClass::available()
{
     return (readRegister(REG_RX_NB_BYTES) - _packetIndex);
}

int LoRaClass::read()
{
    if (!available()) {
        return -1;
    }

    _packetIndex++;

    return readRegister(REG_FIFO);
}

int LoRaClass::peek()
{
    if (!available()) {
        return -1;
    }

    // store current FIFO address
    int currentAddress = readRegister(REG_FIFO_ADDR_PTR);

    // read
    uint8_t b = readRegister(REG_FIFO);

    // restore FIFO address
    writeRegister(REG_FIFO_ADDR_PTR, currentAddress);

    return b;
}

void LoRaClass::flush()
{
}

#ifndef ARDUINO_SAMD_MKRWAN1300
void LoRaClass::onReceive(void(*callback)(int))
{
    _onReceive = callback;

    if (callback) {
        pinMode(_dio0, INPUT);
    #ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
    #endif
        attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
    } else {
        detachInterrupt(digitalPinToInterrupt(_dio0));
    #ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
    #endif
    }
}

void LoRaClass::onCadDone(void(*callback)(boolean))
{
    _onCadDone = callback;

    if (callback) {
        pinMode(_dio0, INPUT);
    #ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
    #endif
        attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
    } else {
        detachInterrupt(digitalPinToInterrupt(_dio0));
    #ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
    #endif
    }
}

void LoRaClass::onTxDone(void(*callback)())
{
    _onTxDone = callback;

    if (callback) {
        pinMode(_dio0, INPUT);
    #ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI.usingInterrupt(digitalPinToInterrupt(_dio0));
    #endif
        attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
    } else {
        detachInterrupt(digitalPinToInterrupt(_dio0));
    #ifdef SPI_HAS_NOTUSINGINTERRUPT
        SPI.notUsingInterrupt(digitalPinToInterrupt(_dio0));
    #endif
    }
}

void LoRaClass::receive(int size)
{

    writeRegister(REG_DIO_MAPPING_1, 0x00); // DIO0 => RXDONE

    if (size > 0) {
        implicitHeaderMode();

        writeRegister(REG_PAYLOAD_LENGTH, size & 0xff);
    } else {
        explicitHeaderMode();
    }

    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

/// @brief Listen to preamble symbol to detect activity on a channel using less power before fully commiting to receive a packet
/// @param  
void LoRaClass::channelActivityDetection(void)
{
    writeRegister(REG_DIO_MAPPING_1, 0x80);// DIO0 => CADDONE
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);
}
#endif

/// @brief Puts the module in standby mode with the FIFO and registers retaining their values.
/// Writing to the configuration registers must be done in either SLEEP/STANDBY mode
void LoRaClass::standby()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/// @brief Puts the module in sleep mode, clearing FIFO buffer, which can't then be accessed in this mode.
/// Configuration registers can still be read and writing to configuration registers should only be done in SLEEP or STANDBY mode.
/// This state can be used to save battery power for example.
void LoRaClass::sleep()
{
    writeRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void LoRaClass::setTxPower(int level_dbm, int outputPin)
{
    if (PA_OUTPUT_RFO_PIN == outputPin) {
          
          // RFO pin is limited to +14 dBm
          level_dbm < 0 ? level_dbm = 0 : level_dbm > 14 ? level_dbm = 14 : level_dbm = level_dbm;
          writeRegister(REG_PA_CONFIG, 0x70 | level_dbm);
          return;
    } 

    // PA_OUTPUT_PA_BOOST_PIN is being used and requires some extra settings
    if (level_dbm > 17) {
        if (level_dbm > 20) {
            level_dbm = 20;
        }

        // subtract 3 from level_dbm, so 18 - 20 maps to 15 - 17
        level_dbm -= 3;

        // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
        writeRegister(REG_PA_DAC, 0x87);
        setOCP(140); //Enable over current protection in mA units
    } else {
        if (level_dbm < 2) {
            level_dbm = 2;
        }
        //Default value PA_HF/LF or +17dBm
        writeRegister(REG_PA_DAC, 0x84);
        setOCP(100); //Enable over current protection in mA units
    }

    writeRegister(REG_PA_CONFIG, PA_BOOST | (level_dbm - 2));
    
}

void LoRaClass::setFrequency(long frequency)
{

    _frequency = frequency;
    
    //Step frequency of the synthesizer is 61.035 Hz, which is taken by dividing the crystal clock frequency by 2^19,
    //which is the resolution of the phase accumulator used in the synthesizer.
    //The formula below then calculates the multiplier value that needs to be written to the registers, such that its product
    //with the step frequency gives the desired input frequency.

    uint64_t frf = ((uint64_t)frequency << 19) / 32000000; // Function is taken from Semtech SX1276/77/78/79 datasheet, p. 85

    writeRegister(REG_FRF_MSB, (uint8_t)(frf >> 16)); //Take the MSB byte and write it to the register
    writeRegister(REG_FRF_MID, (uint8_t)(frf >> 8)); //Take the middle byte and write it to the register
    writeRegister(REG_FRF_LSB, (uint8_t)(frf >> 0)); //Take the LSB byte and write it to the register
}

int LoRaClass::getSpreadingFactor()
{
  return readRegister(REG_MODEM_CONFIG_2) >> 4;
}

/// @brief The spreading factor (SF) is a number that determines how wide the signal is spread in the frequency domain.
/// Higher values increase the signal sensitivity, but decreases the data rate and increases the time on air.
/// @param spreadingFactor 
void LoRaClass::setSpreadingFactor(int spreadingFactor)
{
    if (spreadingFactor < 6) {
        spreadingFactor = 6;
    } else if (spreadingFactor > 12) {
        spreadingFactor = 12;
    }

    if (spreadingFactor == 6) {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc5);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0c);
    } else {
        writeRegister(REG_DETECTION_OPTIMIZE, 0xc3);
        writeRegister(REG_DETECTION_THRESHOLD, 0x0a);
    }

    writeRegister(REG_MODEM_CONFIG_2, (readRegister(REG_MODEM_CONFIG_2) & 0x0f) | ((spreadingFactor << 4) & 0xf0));
    setLdoFlag();
}

long LoRaClass::getSignalBandwidth()
{
    byte bandwidth = (readRegister(REG_MODEM_CONFIG_1) >> 4);

    switch (bandwidth) {
            case 0: return 7.8E3;
            case 1: return 10.4E3;
            case 2: return 15.6E3;
            case 3: return 20.8E3;
            case 4: return 31.25E3;
            case 5: return 41.7E3;
            case 6: return 62.5E3;
            case 7: return 125E3;
            case 8: return 250E3;
            case 9: return 500E3;
    }

    return -1;
}

void LoRaClass::setSignalBandwidth(long signalBandwidth)
{
    int bandwidth;

    if (signalBandwidth <= 7.8E3) {
        bandwidth = 0;
    } else if (signalBandwidth <= 10.4E3) {
        bandwidth = 1;
    } else if (signalBandwidth <= 15.6E3) {
        bandwidth = 2;
    } else if (signalBandwidth <= 20.8E3) {
        bandwidth = 3;
    } else if (signalBandwidth <= 31.25E3) {
        bandwidth = 4;
    } else if (signalBandwidth <= 41.7E3) {
        bandwidth = 5;
    } else if (signalBandwidth <= 62.5E3) {
        bandwidth = 6;
    } else if (signalBandwidth <= 125E3) {
        bandwidth = 7;
    } else if (signalBandwidth <= 250E3) {
        bandwidth = 8;
    } else /*if (signalBandwidth <= 250E3)*/ {
        bandwidth = 9;
    }

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bandwidth << 4));
    setLdoFlag();
}

void LoRaClass::setLdoFlag()
{
    // Section 4.1.1.5
    long symbolDuration = 1000 / ( getSignalBandwidth() / (1L << getSpreadingFactor()) ) ;

    // Section 4.1.1.6
    boolean lowDataRateOptimizationOn = symbolDuration > 16;

    uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
    bitWrite(config3, 3, lowDataRateOptimizationOn);
    writeRegister(REG_MODEM_CONFIG_3, config3);
}

void LoRaClass::setLdoFlagForced(const boolean lowDataRateOptimizationOn)
{
    uint8_t config3 = readRegister(REG_MODEM_CONFIG_3);
    bitWrite(config3, 3, lowDataRateOptimizationOn);
    writeRegister(REG_MODEM_CONFIG_3, config3);
}

/// @brief Sets the amount of redundant bits to be used for forward error correction, but slows down the data rate.
/// @param denominator 
void LoRaClass::setCodingRate4(int denominator)
{
    if (denominator < 5) {
        denominator = 5;
    } else if (denominator > 8) {
        denominator = 8;
    }

    int codingRate = denominator - 4;

    writeRegister(REG_MODEM_CONFIG_1, (readRegister(REG_MODEM_CONFIG_1) & 0xf1) | (codingRate << 1));
}

/// @brief Sets the number of preamble symbols to be transmitted before the payload.
/// The preamble is used for receiver synchronization and AGC calibration.
/// @param length 
void LoRaClass::setPreambleLength(long length)
{
    writeRegister(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
    writeRegister(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/// @brief Network ID used to filter out packets with dissimilar network IDs.
/// Transmitter and receiver must share the same network ID.
/// @param syncWord 
void LoRaClass::setSyncWord(int syncWord)
{
    writeRegister(REG_SYNC_WORD, syncWord); 
}

void LoRaClass::enableCrc()
{
    writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) | 0x04);
}

void LoRaClass::disableCrc()
{
    writeRegister(REG_MODEM_CONFIG_2, readRegister(REG_MODEM_CONFIG_2) & 0xfb);
}

void LoRaClass::enableInvertIQ()
{
    writeRegister(REG_INVERTIQ,  0x66);
    writeRegister(REG_INVERTIQ2, 0x19);
}

void LoRaClass::disableInvertIQ()
{
    writeRegister(REG_INVERTIQ,  0x27);
    writeRegister(REG_INVERTIQ2, 0x1d);
}


void LoRaClass::enableLowDataRateOptimize()
{
    setLdoFlagForced(true);
}

void LoRaClass::disableLowDataRateOptimize()
{
    setLdoFlagForced(false);
}

/// @brief Set the maximum current supply to the power amplifier in mA units.
void LoRaClass::setOCP(uint8_t mA)
{
    uint8_t ocpTrim = 27;

    if (mA <= 120) {
        ocpTrim = (mA - 45) / 5;
    } else if (mA <=240) {
        ocpTrim = (mA + 30) / 10;
    }

    writeRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

void LoRaClass::setGain(uint8_t gain)
{
    // check allowed range
    if (gain > 6) {
        gain = 6;
    }
    
    // set to standby
    standby();
    
    // set gain
    if (gain == 0) {
        // if gain = 0, enable AGC
        writeRegister(REG_MODEM_CONFIG_3, 0x04);
    } else {
        // disable AGC
        writeRegister(REG_MODEM_CONFIG_3, 0x00);
        
        // clear Gain and set LNA boost
        writeRegister(REG_LNA, 0x03);
        
        // set gain
        writeRegister(REG_LNA, readRegister(REG_LNA) | (gain << 5));
    }
}

/// @brief Obtain a random byte value using the RSSI noise floor as a source of entropy.
/// @return 
byte LoRaClass::random()
{
    return readRegister(REG_RSSI_WIDEBAND);
}

/// @brief 
/// @param slaveSelect Line must be pulled low to initiate SPI transaction and pulled high to end it.
/// @param reset Pulling this line low resets the SX127x chip in case of a malfunction or hangup
/// @param dio0 GPIO used to listen for interrupt signal generated by the SX127x chip
void LoRaClass::setPins(int slaveSelect, int reset, int dio0)
{
    _ss = slaveSelect;
    _reset = reset;
    _dio0 = dio0;
}

/// @brief Select an instance of a SPIClass object to be used to communicate with the Lora modem
/// @param spi 
void LoRaClass::setSPI(SPIClass& spi)
{
    _spi = &spi;
}

void LoRaClass::setSPIFrequency(uint32_t frequency)
{
    _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

/// @brief Dump the values of all registers to any object that implements the Stream interface, such
// as a HardwareSerial or even the LoRaClass instance itself.
/// @param out 
void LoRaClass::dumpRegisters(Stream& out)
{
    for (int i = 0; i < 128; i++) {
        out.print("0x");
        out.print(i, HEX);
        out.print(": 0x");
        out.println(readRegister(i), HEX);
    }
}

/// @brief This mode must be used when the number of bytes to be received is not known in advance.
/// This mode is used when the receiver does not know the length of the packet in advance.
void LoRaClass::explicitHeaderMode()
{
    _implicitHeaderMode = 0;

    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) & 0xfe);
}

/// @brief This mode can be used when the number of bytes to be received is known in advance and remains fixed.
/// It can be used to reduce the time on air of the packet by removing the header overhead.
/// This leads to a more efficient transmission of packets when size is known in advance.
void LoRaClass::implicitHeaderMode()
{
    _implicitHeaderMode = 1;

    writeRegister(REG_MODEM_CONFIG_1, readRegister(REG_MODEM_CONFIG_1) | 0x01);
}

void LoRaClass::handleDio0Rise()
{

    //There is a dedicated register on the SX127x modem that stores the interrupt flags,
    //which contains multiple bits that are set when certain events occur.
    //When that happens, the modem generates a digital signal on its DIO0 pin, which is connected via a track to another pin
    //on the microcontroller, on which an interrupt can be set up. This is a more efficient way of handling the modem's events.
    //When the microcontroller detects a rising edge on the DIO0 pin, it reads the interrupt flags register
    //to determine which event occurred. It then clears the interrupt flags register and invokes the appropriate callback.
    //The register can also be polled for, but that would waste CPU cycles in synchronous code.

    int interruptRequestFlags = readRegister(REG_IRQ_FLAGS); //MCU reads the interrupt flags from the modem and stores it in the stack
    writeRegister(REG_IRQ_FLAGS, interruptRequestFlags);     //MCU clears the interrupt flags register on the modem

    if ((interruptRequestFlags & IRQ_CAD_DONE_MASK) != 0) {

        //Channel activity detection cycle completed, invoke registered callback with the result
        if (_onCadDone) {
            _onCadDone((interruptRequestFlags & IRQ_CAD_DETECTED_MASK) != 0);
        }
    
    } else if ((interruptRequestFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == 0) { //Check RX and TX done only if no CRC error is present

        if ((interruptRequestFlags & IRQ_RX_DONE_MASK) != 0) {

            // A packet has been received
            _packetIndex = 0;

            // Read packet length according to header mode
            int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH) : readRegister(REG_RX_NB_BYTES);

            // Set FIFO address to current RX address
            writeRegister(REG_FIFO_ADDR_PTR, readRegister(REG_FIFO_RX_CURRENT_ADDR));

            //Invoke RX callback with the packet length if registered
            if (_onReceive) {
                _onReceive(packetLength);
            }

        } else if ((interruptRequestFlags & IRQ_TX_DONE_MASK) != 0) {

            // A packet has been sent
            //Invoke TX callback if registered
            if (_onTxDone) {
                _onTxDone();
            }
        }
    }
}

/// @brief Reads a register and returns its value.
/// @param address 
/// @return 
uint8_t LoRaClass::readRegister(uint8_t address)
{
    // Most significant bit (7) of address byte should be 0 for reading the register
    // 0x7f = 0b01111111
    return singleTransfer(address & 0x7f, 0x00);
}

/// @brief Writes a value to a register.
/// If writing to a register the value it currently holds, it will clear the register.
/// @param address 
/// @param value 
void LoRaClass::writeRegister(uint8_t address, uint8_t value)
{
    // Most significant bit (7) of address byte should be 1 for writing to the register
    // 0x80 = 0b10000000
    singleTransfer(address | 0x80, value);
}

uint8_t LoRaClass::singleTransfer(uint8_t address, uint8_t value)
{
    uint8_t response;

    _spi->beginTransaction(_spiSettings);
    digitalWrite(_ss, LOW);
    _spi->transfer(address);
    response = _spi->transfer(value);
    digitalWrite(_ss, HIGH);
    _spi->endTransaction();

    return response;
}


ISR_PREFIX void LoRaClass::onDio0Rise()
{
     LoRa.handleDio0Rise();
}

LoRaClass LoRa; // Instance of the LoRaClass to be used by the application

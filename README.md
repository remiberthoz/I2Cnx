# I2Cnx

**An AVR Library to communicate over multiple I2C buses with devices having the
same addresses.**

I2CNX is an implementation of the I2C protocol, intended for communication with
multiple devices which share the same address. Communication with these devices
can occur in parallel by implementing I2C with multiple SDA lines.

This specific git repository currently supports AVR XMEGA only, but
pull-requests providing support for other AVR or even other architectures are
welcome.

## Usage

### Pinout

To communicate between one host (`H`) and three devices (`D1`, `D2`, `D3`) that
share the same I2C address:

- the four SCL pins must be connected together (`H-SCL`, `D1-SCL`, `D2-SCL`,
  `D3-SCL`)
- the SDA pin of each device (`D1-SDA`, `D2-SDA`, `D3-SDA`) must connect to
  separate pins on the host (`H-SDA1`, `H-SDA2`, `H-SDA3`)

```
      AVR XMEGA HOST          I2C DEVICES
    AVR PIN  FUNCTION         PIN
        PD5     H-SCL ---+--- D1-SCL
                         +--- D2-SCL
                         +--- D3-SCL
        PB2    H-SDA1 ------- D1-SDA
        PB3    H-SDA2 ------- D2-SDA
        PB4*   H-SDA3 ------- D3-SDA

    *SDA pins on the device must be consecutive pins on the same port
    (PortB Pins 2, 3, 4)
```

Configure pinout details in software by creating a `I2CNX` object:

```c++
uint8_t N_BUSES = 3;        // Number of SDA lines
Port_t* SCL_PORT = &PORTD;  // Port of SCL pin  (PD5 in the example)
uint8_t SCL_PIN = 5;        // Pin number of SCL pin on port (PD5 in the example)
Port_t* SDA_PORT = &PORTB;  // Port of SDA pins (PB2-4 in the example)
uint8_t SDA_FIRST_PIN = 2;  // Pin number of first SDA pin on port (PB2-4 in the example)

I2CNX my_i2cnx{N_BUSES, SCL_PORT, SCL_PIN, SDA_PORT, SDA_FIRST_PIN, 0};  // The last constructor parameter is currently ignored.
```

### Writing a byte to a register on each device

```c++
// Initialize the port status, and force the bus to IDLE
my_i2cnx.init();

// Start a transmit (host to device) transaction, with devices at I2C
// address 0xD0 and set the devices register address to 0x01. Do not
// terminate the transaction.
uint8_t i2c_addr = 0xD0;
uint8_t reg_addr = 0x01;
bool ack = my_i2cnx.start_transmit(i2c_addr, reg_addr, false);

// Send a different data byte to each device. Do not terminate transaction.
uint8_t data_1[3] = {42, 43, 44};
ack = my_i2cnx.transmit(data_1, false);

// Send more bytes to the devices (now at register address 0x02).
// And terminate the transaction.
uint8_t data_2[3] = {2, 4, 8};
ack = my_i2cnx.transmit(data_2, true);

// I2C buses are now in IDLE state.
```

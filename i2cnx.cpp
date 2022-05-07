#include "i2cnx.h"
#include <util/delay.h>
#include "../../Verbose/verbose.h"

#define eight(x) {x, x, x, x, x, x, x, x}


enum I2C_ADDR_BIT : uint8_t {
    HOST_WRITE = 0,
    HOST_READ = 1,
};

constexpr bool ZEROS[M_BUS] = eight(0);
constexpr bool ONES[M_BUS] = eight(1);


void I2CNX::sda_to_bools(const uint8_t sda, bool bools[M_BUS]) {
    for (uint8_t bus = 0; bus < N_BUS; bus++) {
        bools[bus] = static_cast<bool> ( sda & (0b1 << (bus + SDA_FIRST_PIN)) );
    }
}


void I2CNX::bools_to_sda(uint8_t& sda, const bool bools[M_BUS]) {
    sda = 0;
    for (uint8_t bus = 0; bus < N_BUS; bus++) {
        sda |= static_cast<uint8_t> ( bools[bus] << (bus + SDA_FIRST_PIN) );
    }
}


void I2CNX::delay() {
    _delay_us(5);
}


/**
 * Set the SCL line to the passive pull-up level.
 */
void I2CNX::pulldwn_scl() {

    printf_debugv("I2CNX: Pulldwn SCL\n");

    SCL_PORT->DIRSET = SCL_MASK;
    SCL_PORT->OUTCLR = SCL_MASK;
    delay();

}


/**
 * Set the SCL line to the passive pull-up level.
 */
void I2CNX::release_scl() {

    printf_debugv("I2CNX: Release SCL\n");

    SCL_PORT->DIRCLR = SCL_MASK;
    // SCL_PORT->OUTSET = SCL_MASK;
    delay();
}


/**
 * Set the SDA level on each bus to low or pull-up, based on the
 * {bits}. True values in {bits} will set the corresponding SDA
 * to passive pull-up; and false will set the SDA to low.
 *
 * The bit in {bits[0]} is mapped to the SDA {sda_first_pin}.
 */
void I2CNX::set_sda(const bool bits[M_BUS]) {

    printf_debugv("I2CNX: Set SDA buses: ");
    for (int bus = N_BUS - 1; bus >= 0; bus--) {
        printf_debugv_c("%u", bits[bus]);
    }
    printf_debugv_c("\n");

    uint8_t sda;
    bools_to_sda(sda, bits);

    // Release all
    SDA_PORT->DIRCLR = static_cast<uint8_t> ( SDA_MASK );
    // Pull down zeros
    SDA_PORT->OUTCLR = static_cast<uint8_t> ( (~sda) & SDA_MASK );
    SDA_PORT->DIRSET = static_cast<uint8_t> ( (~sda) & SDA_MASK );

    delay();
}


/**
 * Set the SDA line on each bus to the passive pull-up level.
 */
void I2CNX::release_sda() {

    printf_debugv("I2CNX: Release SDA buses\n");

    SDA_PORT->DIRCLR = static_cast<uint8_t> ( SDA_MASK );
    delay();
}


/**
 * Measure the SDA level on each line.
 *
 * Will only work if issued on buses where SDA are released. SDA can
 * be released with release_sda().
 */
void I2CNX::get_sda(bool bits[M_BUS]) {

    printf_debugv("I2CNX: Get SDA buses: ");

    delay();
    uint8_t sda = SDA_PORT->IN & SDA_MASK;
    sda_to_bools(sda, bits);

    for (int bus = N_BUS - 1; bus >= 0; bus--) {
        printf_debugv_c("%u", bits[bus]);
    }
    printf_debugv_c("\n");
}


/**
 * Send a START on each buses.
 *
 * Will only work if issued on IDLE buses. Buses can be forced to
 * IDLE state with init().
 */
void I2CNX::start() {
    set_sda(ZEROS);
}


void I2CNX::terminate(const I2C_TERMINATION termination) {

    switch(termination) {

        case I2C_TERMINATION::STOP:
            printf_debug("I2CNX: Stop\n");
            pulldwn_scl();
            release_scl();
            set_sda(ZEROS);
            release_sda();
            break;
        case I2C_TERMINATION::NO_STOP:
            printf_debug("I2CNX: No stop\n");
            break;
        case I2C_TERMINATION::ACK:
            printf_debug("I2CNX: HOST ACK\n");
            pulldwn_scl();
            set_sda(ZEROS);
            release_scl();
            break;
        case I2C_TERMINATION::NACK_STOP:
            printf_debug("I2CNX: HOST NACK+STOP\n");
            pulldwn_scl();
            set_sda(ONES);
            release_scl();
            terminate(I2C_TERMINATION::STOP);
            break;
    }

}


/**
 * Write one byte of data to each bus, anc check the device
 * ACKNOWLEDGE bits.
 *
 * Returns true if each device ACKNOWLEDGES, false otherwise.
 */
bool I2CNX::write_byte(const uint8_t bytes[M_BUS]) {

    printf_debugv("I2CNX: Sending bytes on buses: ");
    for (int bus = N_BUS - 1; bus >= 0; bus--) {
        printf_debugv_c("%u ", bytes[bus]);
    }
    printf_debugv_c("\n");

    for (int bit = 8 - 1; bit >= 0; bit--) {

        bool sda[M_BUS];
        for (uint8_t bus = 0; bus < N_BUS; bus++) {
            sda[bus] = bytes[bus] & (0b1 << bit);
        }

        pulldwn_scl();
        set_sda(sda);
        release_scl();
    }

    printf_debugv("I2CNX: Checking ACKNOWLEDGES\n");
    bool ack[M_BUS];
    pulldwn_scl();
    release_sda();
    release_scl();
    get_sda(ack);

    bool any_nack = false;
    for (uint8_t bus = 0; bus < N_BUS; bus++) {
        if (ack[bus] == 1) {
            if (any_nack == false) {
                any_nack = true;
                printf_warn("I2CNX: NACK on bus ");
            }
            printf_warn_c("%d ", bus);
        }
    }
    if (any_nack) {
        printf_warn_c("\n");
        return false;
    }

    return true;
}


/**
 * Read one byte of data from each bus.
 */
void I2CNX::read_byte(uint8_t bytes[M_BUS]) {

    printf_debugv("I2CNX: Receiving bytes...\n");

    for (uint8_t bus = 0; bus < N_BUS; bus++) {
        bytes[bus] = 0;
    }

    for (int bit = 8 - 1; bit >= 0; bit--) {

        bool sda[M_BUS];
        pulldwn_scl();
        release_sda();
        release_scl();
        get_sda(sda);

        for (uint8_t bus = 0; bus < N_BUS; bus++) {
            bytes[bus] |= static_cast<uint8_t> ( sda[bus] << bit );
        }
    }

    printf_debugv("I2CNX:  Receiving bytes... Received: ");
    for (int bus = N_BUS - 1; bus >= 0; bus--) {
        printf_debugv_c("%u ", bytes[bus]);
    }
    printf_debugv_c("\n");
}


I2CNX::I2CNX(const uint8_t n_bus, PORT_t*const scl_port, const uint8_t scl_pin, PORT_t*const sda_port, const uint8_t sda_first_pin, const uint8_t delay_us) :
        N_BUS{n_bus},
        SCL_PORT{scl_port},
        SCL_PIN{scl_pin},
        SDA_PORT{sda_port},
        SDA_FIRST_PIN{sda_first_pin},
        DELAY_US{delay_us},
        SCL_MASK{static_cast<uint8_t> ( 0b1 << SCL_PIN )},
        SDA_MASK{static_cast<uint8_t> ( (0xFF >> (M_BUS - N_BUS)) << SDA_FIRST_PIN )}
    {};


void I2CNX::init() {

    printf_debug("I2CNX: Initialising buses\n");

    pulldwn_scl();
    release_scl();
    set_sda(ZEROS);
    set_sda(ONES);
}


bool I2CNX::start_transmit(const uint8_t device_addr, const uint8_t register_addr, const bool continue_dont_stop) {

    printf_debug("I2CNX: Starting HOST WRITE transaction: device %u, register %u\n", device_addr, register_addr);

    bool ack;
    start();

    // Send device address
    uint8_t _device_addr[M_BUS] = eight(static_cast<uint8_t> ( device_addr | I2C_ADDR_BIT::HOST_WRITE ));
    ack = write_byte(_device_addr);
    if (!ack) {
        terminate(I2C_TERMINATION::STOP);
        return false;
    }

    // Send register address
    uint8_t _register_addr[M_BUS] = eight(register_addr);
    ack = write_byte(_register_addr);
    if (!ack) {
        terminate(I2C_TERMINATION::STOP);
        return false;
    }

    if (continue_dont_stop)
        terminate(I2C_TERMINATION::NO_STOP);
    else
        terminate(I2C_TERMINATION::STOP);

    return true;
}


bool I2CNX::start_receive(const uint8_t device_addr, const bool continue_dont_stop) {

    printf_debug("I2CNX: Starting HOST READ transaction: device %u\n", device_addr);

    start();

    // Send device address
    uint8_t _device_addr[M_BUS] = eight(static_cast<uint8_t> ( device_addr | I2C_ADDR_BIT::HOST_READ ));
    bool ack = write_byte(_device_addr);
    if (!ack) {
        terminate(I2C_TERMINATION::STOP);
        return false;
    }

    if (continue_dont_stop)
        terminate(I2C_TERMINATION::NO_STOP);
    else
        terminate(I2C_TERMINATION::STOP);

    return true;
}


bool I2CNX::transmit(const uint8_t data[M_BUS], const bool continue_dont_stop) {

    printf_debug("I2CNX: Transmitting data: ");
    for (int bus = N_BUS - 1; bus >= 0; bus--) {
        printf_debug_c("%u ", data[bus]);
    }
    printf_debug_c("\n");

    bool ack = write_byte(data);
    if (!ack) {
        terminate(I2C_TERMINATION::STOP);
        return false;
    }

    if (continue_dont_stop)
        terminate(I2C_TERMINATION::NO_STOP);
    else
        terminate(I2C_TERMINATION::STOP);

    return true;
}


void I2CNX::receive(uint8_t data[M_BUS], const bool acknowledge_continue_dont_stop) {

    printf_debug("I2CNX: Receiving data...\n");
    read_byte(data);

    printf_debug("I2CNX: Receiving data... Received: ");
    for (int bus = N_BUS - 1; bus >= 0; bus--) {
        printf_debug_c("%u ", data[bus]);
    }
    printf_debug_c("\n");

    if (acknowledge_continue_dont_stop)
        terminate(I2C_TERMINATION::ACK);
    else
        terminate(I2C_TERMINATION::NACK_STOP);

}

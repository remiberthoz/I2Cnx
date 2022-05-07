#pragma once
#include <stdint.h>
#include <avr/io.h>

#define M_BUS 8


/**
 * Termination that can be issued on an I2C bus after a byte is
 * exchanged.
 *
 * - STOP: stop the transaction
 * - NO_STOP: continue the transaction
 * - ACK: acknowledge the byte
 * - NACK_STOP: do not acknowledge the byte and stop the transaction
 */
enum class I2C_TERMINATION {
    STOP,
    NO_STOP,
    ACK,
    NACK_STOP
};


class I2CNX {

    private:
        const uint8_t N_BUS;
        PORT_t*const SCL_PORT;
        const uint8_t SCL_PIN;
        PORT_t*const SDA_PORT;
        const uint8_t SDA_FIRST_PIN;
        const uint8_t DELAY_US;
        const uint8_t SCL_MASK;
        const uint8_t SDA_MASK;
        bool stat_scl = 0;
        uint8_t stat_sda = 0;

    public:/**
        * @brief Create a group of I2C buses which share a common SCL line and separate
        * SDA lines.
        *
        * The SDA lines must be consecutive pins on a single AVR port (ie A2, A3, A4,
        * A5; but not A2, A3, A5; or not A2, A3, B4). Hence, the maximal number of SDA
        * lines available is 8.
        *
        * SDA and SCL lines must have external pull-up resistors (the internal pull-up
        * are not used by I2CNX.)
        *
        * @param[in] n_bus The number of busses to create within this group.
        * @param[in] scl_port The AVR I/O port to use for the SCL line.
        * @param[in] scl_pin The pin number to use for the SCL line on @p scl_port.
        *                    Example: `scl_port=PORT_E, scl_pin=4`, will use pin E4 as
        *                    SCL.
        * @param[in] sda_port The AVR I/O port to use for the SDA lines.
        * @param[in] sda_first_pin The pin number to use for the first SDA line on @p
        *                          sda_port. Example: `scl_port=PORT_A,
        *                          scl_first_pin=2, n_bus=4` will use pins A2, A3, A4,
        *                          A5 as SDA.
        * @param[in] delay_us Ignored.
        *
        * \remark Note that the intended use of I2CNX is to communicate
        * **simultaneously** to devices **at the same I2C address** and the **same
        * register address**. In particular this means that, if durring a **HOST
        * WRITE** transaction, one of the devices responds with a **NACK**, then all
        * other buses within the group will *stop* simulataneously.
        *
        * @todo Add option to use internal pull-up.
        * @todo Use @p delay_us parameter to set line switching delay, instead of the
        * hardcoded value.
        */
        I2CNX(const uint8_t n_bus, PORT_t*const scl_port, const uint8_t scl_pin, PORT_t*const sda_port, const uint8_t sda_first_pin, const uint8_t delay_us);
        /**
         * @brief Initialize all buses by forcing them into **IDLE** state.
         *
         * Buses are forced to **IDLE** state by issuing a *stop condition*. Depending
         * on the previous state of the bus, this may issue a *start condition* before
         * the *stop*.
         */
        virtual void init();
        /**
         * @brief Start a **HOST WRITE** transaction on all buses.
         *
         * @param[in] device_addr The device I2C address to communicate with within this
         * transaction.
         * @param[in] register_addr The device register address to write to within this
         * transaction.
         * @param[in] continue_dont_stop Whether to *stop* the transaction after
         * starting it (assuming no device replies with a **NACK**). This can be useful
         * ie to manipulate the device register address pointer.
         *
         * @returns `true` if all devices replied with **ACK** to both the I2C device
         * address and the register address, `false` otherwise.
         *
         * A **HOST WRITE** transaction is initiated for each bus with the device at I2C
         * address @p device_addr, and at the register address @p register_addr.
         *
         * If a device on any bus replies with a **NACK** to either the I2C address or
         * the register address, then the transaction is *stopped*. Otherwise, a *stop
         * condition* is issued only if @p continue_dont_stop is `false`.
         */
        virtual bool start_transmit(const uint8_t slave_addr, const uint8_t addr, const bool continue_dont_stop);
        /**
         * @brief Start a **HOST READ** transaction on all buses.
         *
         * @param[in] device_addr The device I2C address to communicate with within this
         * transaction.
         * @param[in] continue_dont_stop Whether to *stop* the transaction after
         * starting it (assuming no device replies with a **NACK**).
         *
         * @returns `true` if all devices replied with **ACK** to the I2C device
         * address, `false` otherwise.
         *
         * A **HOST READ** transation is initiated for each bus with the device at I2C
         * address @p device_addr, and at the register address that the device is
         * currently pointing to. The I2C protocol does not specify a way to set a
         * register address when starting **HOST READ** transactions. If you need to
         * manipulate the register being pointed to by the device, you can use a dummy
         * **HOST WRITE**.
         *
         * If a device on any bus replies with a **NACK** to the I2C address, then the
         * transaction is *stopped*. Otherwise, a *stop condition* is issued only if @p
         * continue_dont_stop is `false`.
         *
         * @see start_transmit
         */
        virtual bool start_receive(const uint8_t slave_addr, const bool continue_dont_stop);
        /**
         * @brief Transmit one data byte on each bus. This is intended to be used within
         * a **HOST WRITE** transaction.
         *
         * @param[in] data An array of data bytes to be sent to buses.
         * @param[in] continue_dont_stop Whether to *stop* the transaction after
         * transmitting the data byte (assuming no device replies with a **NACK**).
         *
         * @returns `true` if all devices replied with **ACK** to data bytes.
         *
         * Each bus can carry a different data byte. Bytes in @p data are mapped to
         * buses, and sent MSB-first. The byte at index zero of @p data goes to the SDA
         * line designated by `SDA_FIRST_PIN`.
         *
         * If a device on any bus replies with a **NACK** to the data byte, then the
         * transaction is *stopped*. Otherwise, a *stop condition* is issued only if @p
         * continue_dont_stop is `false`.
         *
         * @see start_transmit
         * @see SDA_FIRST_PIN
         */
        virtual bool transmit(const uint8_t data[8], const bool continue_dont_stop);
        /**
         * @brief Receive one byte on each bus. This is intended to be used within a
         * **HOST READ** transaction.
         *
         * @param[out] data An array to store data bytes read on buses.
         * @param[in] acknowledge_continue_dont_stop Whether to **ACK** (if `true`) or
         * to **NACK** (if `flase`) the data byte after receiving it.
         *
         * @see start_receive
         * @see SDA_FIRST_PIN
         */
        virtual void receive(uint8_t data[8], const bool acknowledge_continue_dont_stop);

    private:
        virtual inline void sda_to_bools(const uint8_t sda, bool bools[8]);
        virtual inline void bools_to_sda(uint8_t& sda, const bool bools[8]);
        virtual void delay();
        virtual void pulldwn_scl();
        virtual void release_scl();
        virtual void set_sda(const bool bits[M_BUS]);
        virtual void release_sda();
        virtual void get_sda(bool bits[M_BUS]);
        virtual void start();
        virtual void terminate(const I2C_TERMINATION termination);
        virtual bool write_byte(const uint8_t bytes[8]);
        virtual void read_byte(uint8_t bytes[8]);
};

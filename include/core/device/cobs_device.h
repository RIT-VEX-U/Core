#pragma once
#include "vex.h"

#include <cstddef>
#include <cstdint>
#include <vector>

class COBSSerialDevice {
  public:
    // Decoded packet containing the data one wishes to send
    using Packet = std::vector<uint8_t>;
    // Cobs Encoded packet containing 0 delimeters ready to be sent over the wire
    using WirePacket = std::vector<uint8_t>;

    /**
     * Create a serial device that communicates with 0-delimeted COBS encoded packets
     * @param port the vex::PORTXX that the device was created on
     * @param baud the baud rate to run the port at (i.e. 115200)
     */
    COBSSerialDevice(int32_t port, int32_t baud);
    /**
     * Send a packet of data to the wire. This function takes care of the encoding and sending
     * Blocks until the entire packet is written
     * NOTE: there is no flow control THAT NEEDS TO BE DONE ON YOUR END
     * @param data the data to encode then write
     * @param size the size of the data to write
     * @param leading_delimeter True to add a leading delimeter to the packet. This may help the packet deliver
     * uncorrupted if the previous packet failed to send completely
     * @return the number of bytes written (this will generally be > the size passed in due to encoding overhead)
     */
    int send_cobs_packet_blocking(const uint8_t *data, size_t size, bool leading_delimeter = false);
    /**
     * Recieve a packet from the wire
     * this function polls the port for incoming data and returns when a valid COBS packet has been delivered
     * @param buffer the buffer to write the decoded data into
     * @param max_size the maximum amount of data to read into buffer (buffer size)
     * @param timeout_us if nothing is received for this many us, return -2. The default value of 0 will never timeout
     * @return the number of bytes received or <0 if an error (-1 general error, -2 timeout)
     */
    int receive_cobs_packet_blocking(uint8_t *buffer, size_t max_size, uint32_t timeout_us = 0);

    /**
     * Encode a packet using consistent overhead byte stuffing
     * @param[in] in the data to send
     * @param[out] out the buffer to write the packet into
     * @param add_start_delimeter whether or not to add a leading delimeter to the packet. Adding a start delimeter may
     * help recover the system if the previous packet failed to completely send.
     */
    static void cobs_encode(const Packet &in, WirePacket &out, bool add_start_delimeter = false);
    /**
     * Decode a cobs encoded packet
     * @param[in] in the packet recieved from the wire (without delimeters)
     * @param[out] out the buffer to write the data into
     */
    static void cobs_decode(const WirePacket &in, Packet &out);
    /**
     * print hex data to the console
     * prints as 16 columns
     */
    static void hexdump(const uint8_t *data, size_t len);

    Packet get_last_decoded_packet();

  protected:
    /**
     * Poll port for incoming data
     * @return true if a packet was decoded in the most recent poll, false if no new packet is available
     */
    bool poll_incoming_data_once();

    /// @brief  process one byte at a time
    /// @param byte the incoming byte
    /// @return true if a packet was decoded
    bool handle_incoming_byte(uint8_t byte);

  private:
    vex::mutex serial_access_mut;
    int32_t port;
    int32_t baud;

    // Buffer to hold data about to be written
    Packet writing_buffer;
    // Buffer to hold encoded cobs data about to be written
    WirePacket encoded_write;

    // buffer used to get data from VEX OS land to userland. Scratch space
    std::vector<uint8_t> incoming_buffer;
    // buffer to read bytes in when building up a cobs packet
    WirePacket incoming_wire_packet;
    // contains the last packet that was received and decoded
    Packet last_decoded_packet;
};
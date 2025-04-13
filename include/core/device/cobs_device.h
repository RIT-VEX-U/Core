#pragma once
#include "vex.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>
/**
 * defines a Consistent Overhead Byte Stuffing device
 */
class COBSSerialDevice {
  public:
    using WirePacket = std::vector<uint8_t>; // 0x00 delimeted, cobs encoded
    using Packet = std::vector<uint8_t>;

    static constexpr int32_t NO_ACTIVITY_DELAY = 2; // ms
    static constexpr std::size_t MAX_OUT_QUEUE_SIZE = 50;
    static constexpr std::size_t MAX_IN_QUEUE_SIZE = 50;
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
    virtual int send_cobs_packet_blocking(const uint8_t *data, size_t size, bool leading_delimeter = false);
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
    bool write_packet_if_avail();
    /**
     * print hex data to the console
     * prints as 16 columns
     * @param data the byte to hexdump
     * @param len the size of how much you from the data to hexdump
     */
    static void hexdump(const uint8_t *data, size_t len);

  protected:
    /**
     * Poll port for incoming data
     * @return true if a packet was decoded in the most recent poll, false if no new packet is available
     */
    bool poll_incoming_data_once();
    bool send_cobs_packet(const Packet &pac);
    virtual void cobs_packet_callback(const Packet &pac) = 0;

  private:
    /**
     * @brief  process one byte at a time
     * @param byte the incoming byte
     * @return true if a packet was decoded
     */
    bool handle_incoming_byte(uint8_t byte);

    void handle_inbound_vdb_byte(uint8_t b);

    // Task that deals with the low level writing and reading bytes from the wire
    vex::task serial_task;
    /**
     * the thread for sending data to the wire
     */
    static int serial_thread(void *self);

    // Once the serial_task has read in an entire packet, it must be decoded
    // this thread decodes it back into its original binary form and calls the
    // user callback
    vex::task decode_task;
    /**
     * the thread for decoding data from the wire
     */
    static int decode_thread(void *self);

    vex::mutex serial_access_mut;
    int32_t port;
    int32_t baud;

    /// @brief Packets that have been encoded and are waiting for their turn
    /// to be sent out on the wire
    std::deque<WirePacket> outbound_packets{};
    vex::mutex outbound_mutex;

    /// @brief Packets that have been read from the wire and split up but that are
    /// still COBS encoded
    std::deque<WirePacket> inbound_packets;
    vex::mutex inbound_mutex;

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
#pragma once
#include <cstdint>
#include <deque>
#include <functional>
#include <vector>
#include <vex.h>

class COBSSerialDevice {
  public:
    using WirePacket = std::vector<uint8_t>; // 0x00 delimeted, cobs encoded
    using Packet = std::vector<uint8_t>;

    static constexpr int32_t NO_ACTIVITY_DELAY_MS = 2;
    static constexpr std::size_t MAX_OUT_QUEUE_SIZE = 50;
    static constexpr std::size_t MAX_IN_QUEUE_SIZE = 50;

    /**
     * Send a packet to the device
     * @param pac the non-encoded packet to send. It will be cobs encoded then sent
     * @param add_front_delimeter whether or not to add a 0 byte before the packet. May help deliver a packet if the
     * previous packet was corrupted
     * @return true if the packet was sent. false if the queue was full and the packet can't be sent
     */
    bool send_cobs_packet(const Packet &pac, bool add_front_delimeter = false);

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

  protected:
    /**
     * @param port the port that the peripheral is on (vex::PORTX)
     * @param baud_rate the baud rate of the serial communication
     */
    COBSSerialDevice(int32_t port, int32_t baud_rate);

    virtual ~COBSSerialDevice() {}

    /**
     * Callback when a packet comes in
     * Overload this with your child class to have this called when a packet comes in
     * @param pac the decoded packet that has been received
     */
    virtual void cobs_packet_callback(const Packet &pac) = 0;

  private:
    /**
     * Handle byte recieved from the wire. Will build up a packet until a delimeter is reached then send it to be
     * decoded
     * @param b the byte received
     */
    void handle_inbound_byte(uint8_t b);
    /**
     * Write a packet to the serial device if one is waiting in the queue
     * @return true if a packet was written
     */
    bool write_packet_if_avail();

    /// @brief Port on which the device is initialized
    int32_t port;
    /// @brief baud rate at which the device is running
    int32_t baud_rate;

    /// @brief Packets that have been encoded and are waiting for their turn
    /// to be sent out on the wire
    std::deque<WirePacket> outbound_packets{};
    vex::mutex outbound_mutex;

    /// @brief Packets that have been read from the wire and split up but that are
    /// still COBS encoded
    std::deque<WirePacket> inbound_packets;
    vex::mutex inbound_mutex;
    /// @brief Working buffer that the reading thread uses to assemble packets
    /// until it finds a full COBS packet
    WirePacket inbound_buffer;

    // Task that deals with the low level writing and reading bytes from the wire
    vex::task serial_task;
    static int serial_thread(void *self);

    // Once the serial_task has read in an entire packet, it must be decoded
    // this thread decodes it back into its original binary form and calls the
    // user callback
    vex::task decode_task;
    static int decode_thread(void *self);
};

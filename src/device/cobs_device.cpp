#include "../core/include/device/cobs_device.h"
/**
 * Create a serial device that communicates with 0-delimeted COBS encoded packets
 * @param port the vex::PORTXX that the device was created on
 * @param baud the baud rate to run the port at (i.e. 115200)
 */
COBSSerialDevice::COBSSerialDevice(int32_t port, int32_t baud) : port(port), baud(baud) {
    serial_task = vex::task(COBSSerialDevice::serial_thread, (void *)this, vex::thread::threadPriorityHigh);
    decode_task = vex::task(COBSSerialDevice::decode_thread, (void *)this, vex::thread::threadPriorityHigh);
    vexGenericSerialEnable(port, 0);
    vexGenericSerialBaudrate(port, baud);
}
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
int COBSSerialDevice::send_cobs_packet_blocking(const uint8_t *data, size_t size, bool leading_delimeter) {
    // lock the serial port
    serial_access_mut.lock();
    // sets the writing buffer to the size of the packet to be sent
    writing_buffer.resize(size);
    // copies the data to the beginning of the writing buffer
    std::copy(data, data + size, writing_buffer.begin());
    // encodes the packet to encoded_write
    COBSSerialDevice::cobs_encode(writing_buffer, encoded_write, leading_delimeter);
    // prints the encoded data as a hex
    // hexdump(encoded_write.data(), encoded_write.size());

    size_t write_head = 0;

    // write data to the device
    while (write_head < encoded_write.size()) {
        // checks the number of free bytes to write to at the port
        int32_t num_free = vexGenericSerialWriteFree(port);
        if (num_free < 0) {
            // error on port
            // if this happens, lord helps us
            serial_access_mut.unlock();
            return num_free;
        } else if (num_free == 0) {
            // if there are no bytes available to write to, flush the port
            vexGenericSerialFlush(port);
            continue;
        }
        // number of bytes to transmit to the device
        size_t num_to_transmit = encoded_write.size() - write_head;
        // if there arent enough free bytes to trasmit, just only send what we can for now
        // until the next flush of the serial port
        if (num_to_transmit > num_free) {
            num_to_transmit = num_free;
        }
        // send the data we can to the device
        int32_t sent = vexGenericSerialTransmit(port, encoded_write.data() + write_head, num_to_transmit);
        //
        if (sent != num_to_transmit) {
            // error on port
            // again, god safe us all
            serial_access_mut.unlock();
            return -1;
        }
        // add the sent data to the write head so it can continue writing from that point
        write_head += sent;
    }
    // unlock the serial port
    serial_access_mut.unlock();
    // returns the number of bytes written
    return write_head;
}
/**
 * Recieve a packet from the wire
 * this function polls the port for incoming data and returns when a valid COBS packet has been delivered
 * @param buffer the buffer to write the decoded data into
 * @param max_size the maximum amount of data to read into buffer (buffer size)
 * @param timeout_us if nothing is received for this many us, return -2. The default value of 0 will never timeout
 * @return the number of bytes received or <0 if an error (-1 general error, -2 timeout)
 */
int COBSSerialDevice::receive_cobs_packet_blocking(uint8_t *data, size_t max_size, uint32_t timeout_us) {
    // lock the serial port
    serial_access_mut.lock();
    // sets the start time
    size_t start_time = vexSystemHighResTimeGet();
    // waits to receive a packet
    while (true) {
        // Timed out
        size_t elapsed = vexSystemHighResTimeGet() - start_time;
        if (elapsed > timeout_us && timeout_us != 0) {
            serial_access_mut.unlock();
            return -2;
        }
        // we got a packet :)
        bool got_packet = poll_incoming_data_once();
        if (got_packet) {
            break;
        }
        vex::this_thread::yield();
    }
    // sets the data values to the packet data
    for (size_t index = 0; index < last_decoded_packet.size(); index++) {
        // if packet is bigger than the max size end the recieving and return the max size
        if (index >= max_size) {
            serial_access_mut.unlock();
            return max_size;
        }
        data[index] = last_decoded_packet[index];
    }
    // unlock the serial port
    serial_access_mut.unlock();
    // return the number of bytes recieved
    return last_decoded_packet.size();
}
/**
 * Encode a packet using consistent overhead byte stuffing
 * @param[in] in the data to send
 * @param[out] out the buffer to write the packet into
 * @param add_start_delimeter whether or not to add a leading delimeter to the packet. Adding a start delimeter may
 * help recover the system if the previous packet failed to completely send.
 */
void COBSSerialDevice::cobs_encode(const Packet &in, WirePacket &out, bool add_start_delimeter) {
    // clear the packet we are encoding to
    out.clear();
    // if you decided to be a funny man and encode nothing
    if (in.size() == 0) {
        return;
    }
    // calculates the size of the output
    size_t output_size = in.size() + (in.size() / 254) + 1;
    output_size += 1 + (add_start_delimeter ? 1 : 0); // delimeter bytes
    // resizes the packet we are encoding to be the same as the packet we are encoding from
    out.resize(output_size);

    size_t output_code_head = 0;
    // if we need a start delimiter add it to the output and set the output code location to be after the delimiter
    if (add_start_delimeter) {
        out[0] = 0;
        output_code_head = 1;
    }
    // sets the number of bytes in a block to start at 1 since the first byte is this value
    size_t code_value = 1;

    // sets the input head and output head start location and length of the packet
    size_t input_head = 0;
    size_t output_head = (add_start_delimeter ? 2 : 1);
    size_t length = 0;
    // sets the packet we are encoding to to the packet we are encoding from
    while (input_head < in.size()) {
        if (in[input_head] == 0 || length == 255) {
            // if we reached a delimiter (end of a block of bytes) or max length of a packet (255)
            // sets the first byte of the block to the number of bytes in the block
            out[output_code_head] = (uint8_t)code_value;
            // reset the number of bytes written
            // resets the output code location, the number of bytes in the block, length and the length of the packet
            code_value = 1;
            output_code_head = output_head;
            length = 0;
            // starts iterating the output head again
            output_head++;
            // if the input head is a delimiter then skip past it
            if (in[input_head] == 0) {
                input_head++;
            }
        } else {
            // otherwise set the output to the input directly, iterating the number of bytes in the block, the output
            // head, and the input head
            out[output_head] = in[input_head];
            code_value++;
            input_head++;
            output_head++;
        }
        // increase the length of bytes encoded
        length++;
    }
    // set the last output code head to the output code header value
    out[output_code_head] = code_value;

    // Trailing delimeter
    out[output_head] = 0;
    output_head++;
    out.resize(output_head);
}
/**
 * Decode a cobs encoded packet
 * @param[in] in the packet recieved from the wire (without delimeters)
 * @param[out] out the buffer to write the data into
 */
void COBSSerialDevice::cobs_decode(const WirePacket &in, Packet &out) {
    // clears the packet to decode to
    out.clear();
    // if you decided to be a funny man and decode nothing
    if (in.size() == 0) {
        return;
    }
    // resize the output using the calculated size from the input
    out.resize(in.size() + in.size() / 254);
    // sets the number of bytes in the block we are decoding from to 255, the number of bytes left to decode from that
    // block to 0, and sets the write head's starting location
    uint8_t code = 0xff;
    uint8_t left_in_block = 0;
    size_t write_head = 0;
    for (const uint8_t byte : in) {
        if (left_in_block) {
            // if we still have bytes to decode in this block decode them to the output
            out[write_head] = byte;
            write_head++;
        } else {
            // if the beginning of the block, the byte is equal to the number of bytes left in the block
            left_in_block = byte;
            if (left_in_block != 0 && (code != 0xff)) {
                // if the block length is not 255 and we are not at the beginning of the block, decode 0's to the
                // incomplete packet
                out[write_head] = 0;
                write_head++;
            }
            code = left_in_block;
            if (code == 0) {
                // hit a delimeter
                break;
            }
        }
        left_in_block--;
    }
    // resize the output to the number of bytes decoded
    out.resize(write_head);

    return;
}
/**
 * writes a packet to the device as soon as it is available
 */
bool COBSSerialDevice::write_packet_if_avail() {
    bool did_write = false;
    // packet to write to the device
    WirePacket outbound_packet = {};
    // lock the serial port
    outbound_mutex.lock();
    // check if we have a packet to write
    if (outbound_packets.size() > 0) {
        // of we do take the latest packet out of the vector of packets we have
        outbound_packet = outbound_packets.back();
        outbound_packets.pop_back();
    }
    // unlock
    outbound_mutex.unlock();
    if (outbound_packet.size() == 0) {
        return did_write;
    }
    const int avail = vexGenericSerialWriteFree(port);
    if (avail < (int)outbound_packet.size()) {
        // avail can be -1 for unknown reasons.
        // Signed comparison handles this correctly (as
        // correctly as I can think to).
        vexGenericSerialFlush(port);
    }
    const int32_t wrote = vexGenericSerialTransmit(port, outbound_packet.data(), (int32_t)outbound_packet.size());
    if (wrote < (int32_t)outbound_packet.size()) {
        printf("Unhandled state: Didn't write all that we wanted to, will "
               "probably get packet corruption\n");
    }
    did_write = true;

    return did_write;
}
/**
 * print hex data to the console
 * prints as 16 columns
 * @param data the byte to hexdump
 * @param len the size of how much you from the data to hexdump
 */
void COBSSerialDevice::hexdump(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", data[i]);
        if ((i % 16 == 0 && i != 0) || (i == len - 1)) {
            printf("\n");
        }
    }
}
/**
 * Poll port for incoming data
 * @return true if a packet was decoded in the most recent poll, false if no new packet is available
 */
bool COBSSerialDevice::poll_incoming_data_once() {
    // gets the data that is available to be recieved from the port
    int32_t num_to_read = vexGenericSerialReceiveAvail(port);
    // if there is none or somehow less than no data to recieve, then there is no incoming data
    if (num_to_read < 0) {
        return false;
    } else if (num_to_read == 0) {
        return false;
    }
    // set the buffer for incoming data to the size of the the data we should have recieved
    // copy the data we actually recieved and resize the buffer to what we actually got
    incoming_buffer.resize(num_to_read);
    int32_t actually_received = vexGenericSerialReceive(port, incoming_buffer.data(), num_to_read);
    incoming_buffer.resize(actually_received);
    // handle each bye in the incoming data buffer and return true once the whole packet has been handled
    for (uint8_t b : incoming_buffer) {
        bool finished_packet = handle_incoming_byte(b);
        if (finished_packet) {
            return true;
        }
    }
    // if we didnt finish the packet then return false
    return false;
}
/**
 * Handles incoming bytes coming from the wire
 * @param b the incoming byte
 * @return if we got a complete packet and sent it to the wire
 */
bool COBSSerialDevice::handle_incoming_byte(uint8_t b) {
    if (b == 0) {
        if (incoming_wire_packet.size() == 0) {
            // got delimeter but had no packet, just reading delimeters
            return false;
        } else {
            if (incoming_wire_packet.size() >= MAX_IN_QUEUE_SIZE) {
                // if the queue is full
                // send the packet to the buffer
                inbound_packets.push_front(incoming_buffer);
            }
            // decode the wire packet and clear wire packet buffer
            inbound_packets.push_front(incoming_wire_packet);
            cobs_decode(incoming_wire_packet, last_decoded_packet);
            incoming_wire_packet.clear();
            return true;
        }
    } else {
        // push the byte to the wire packet buffer
        incoming_wire_packet.push_back(b);
        return false;
    }
}
/**
 * the thread for sending data to the wire
 */
int COBSSerialDevice::serial_thread(void *vself) {
    // defines itself within the thread
    COBSSerialDevice &self = *(COBSSerialDevice *)vself;
    // enables the serial port and sets the baudrate
    vexGenericSerialEnable(self.port, 0x0);
    vexGenericSerialBaudrate(self.port, self.baud);

    // sets up a buffer
    static constexpr size_t buflen = 4096;
    static uint8_t buf[buflen] = {0};
    // loop for the thread
    while (1) {
        bool did_something = false;
        // Lame replacement for blocking IO. We can't just wait and tell the
        // scheduler to go work on something else while we wait for packets so
        // instead, if we're getting nothing in and have nothing to send, block
        // ourselves.

        // Writing
        if (self.write_packet_if_avail()) {
            did_something = true;
        }

        // Reading
        const int avail = vexGenericSerialReceiveAvail(self.port);
        if (avail > 0) {
            const int read = vexGenericSerialReceive(self.port, buf, buflen);
            if (read > 0 && read < (int)buflen) {
                // stops incoming data to handle it
                self.inbound_mutex.lock();
                for (int i = 0; i < read; i++) {
                    self.handle_incoming_byte(buf[i]);
                }
                self.inbound_mutex.unlock();
            }
            did_something = true;
        }

        if (!did_something) {
            vexDelay(NO_ACTIVITY_DELAY);
        }
    }
    return 0;
}
/**
 * the thread for decoding data from the wire
 */
int COBSSerialDevice::decode_thread(void *vself) {
    // defines itself within the thread
    COBSSerialDevice &self = *(COBSSerialDevice *)vself;
    Packet decoded = {};

    // loop for the thread
    while (true) {

        WirePacket inbound = {};
        {
            // stops incoming data so it can send data out
            self.inbound_mutex.lock();

            if (self.inbound_packets.size() > 0) {
                inbound = self.inbound_packets.back();
                self.inbound_packets.pop_back();
            }
            self.inbound_mutex.unlock();
        }
        // Theres no packet to decode
        if (inbound.size() == 0) {
            vexDelay(NO_ACTIVITY_DELAY);
            continue;
        }
        // decode the packet and run its callback function
        cobs_decode(inbound, decoded);

        self.cobs_packet_callback(decoded);
    }
    return 0;
}
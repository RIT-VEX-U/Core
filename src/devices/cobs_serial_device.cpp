#include "../core/include/devices/cobs_serial_device.h"

COBSSerialDevice::COBSSerialDevice(int32_t port, int32_t baud_rate)
    : port(port), baud_rate(baud_rate), outbound_packets() {

    serial_task = vex::task(COBSSerialDevice::serial_thread, (void *)this, vex::thread::threadPriorityHigh);
    decode_task = vex::task(COBSSerialDevice::decode_thread, (void *)this, vex::thread::threadPriorityHigh);
}

void COBSSerialDevice::handle_inbound_byte(uint8_t b) {

    if (b == 0x00 && inbound_buffer.size() > 0) {
        if (inbound_packets.size() < MAX_IN_QUEUE_SIZE) {
            inbound_packets.push_front(inbound_buffer);
        } else {
            printf(
              "COBSSerialDevice Port%d: Dropping inbound packet. inbound queue "
              "full\n",
              (int)(port + 1)
            );
        }
        // Starting a new packet now
        inbound_buffer.clear();
    } else if (b != 0x00) {
        inbound_buffer.push_back(b);
    }
}

int COBSSerialDevice::decode_thread(void *vself) {

    COBSSerialDevice &self = *(COBSSerialDevice *)vself;
    Packet decoded = {};

    while (true) {

        WirePacket inbound = {};
        {
            self.inbound_mutex.lock();

            if (self.inbound_packets.size() > 0) {
                inbound = self.inbound_packets.back();
                self.inbound_packets.pop_back();
            }
            self.inbound_mutex.unlock();
        }
        // Theres no packet to decode
        if (inbound.size() == 0) {
            vexDelay(NO_ACTIVITY_DELAY_MS);
            continue;
        }

        cobs_decode(inbound, decoded);

        self.cobs_packet_callback(decoded);
    }
    return 0;
}

bool COBSSerialDevice::write_packet_if_avail() {
    bool did_write = false;
    WirePacket outbound_packet = {};

    outbound_mutex.lock();
    if (outbound_packets.size() > 0) {
        outbound_packet = outbound_packets.back();
        outbound_packets.pop_back();
    }
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

int COBSSerialDevice::serial_thread(void *vself) {
    COBSSerialDevice &self = *(COBSSerialDevice *)vself;
    vexDelay(1000);
    vexGenericSerialEnable(self.port, 0x0);
    vexGenericSerialBaudrate(self.port, self.baud_rate);

    static constexpr size_t buflen = 4096;
    static uint8_t buf[buflen] = {0};

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

                self.inbound_mutex.lock();
                for (int i = 0; i < read; i++) {
                    self.handle_inbound_byte(buf[i]);
                }
                self.inbound_mutex.unlock();
            }
            did_something = true;
        }

        if (!did_something) {
            vexDelay(NO_ACTIVITY_DELAY_MS);
        }
    }
    return 0;
}

bool COBSSerialDevice::send_cobs_packet(const Packet &pac, bool add_front_delimeter) {
    outbound_mutex.lock();
    size_t size = outbound_packets.size();
    outbound_mutex.unlock();
    if (size >= MAX_OUT_QUEUE_SIZE) {
        return false;
    }

    std::vector<uint8_t> encoded;
    cobs_encode(pac, encoded, add_front_delimeter);

    outbound_mutex.lock();
    outbound_packets.push_front(encoded);
    outbound_mutex.unlock();
    return true;
}

void COBSSerialDevice::cobs_encode(const Packet &in, WirePacket &out, bool add_start_delimeter) {
    out.clear();
    if (in.size() == 0) {
        return;
    }
    size_t output_size = in.size() + (in.size() / 254) + 1;
    output_size += 1 + (add_start_delimeter ? 1 : 0); // delimeter bytes
    out.resize(output_size);

    size_t output_code_head = 0;
    if (add_start_delimeter) {
        out[0] = 0;
        output_code_head = 1;
    }

    size_t code_value = 1;

    size_t input_head = 0;
    size_t output_head = (add_start_delimeter ? 2 : 1);
    size_t length = 0;
    while (input_head < in.size()) {
        if (in[input_head] == 0 || length == 255) {
            out[output_code_head] = (uint8_t)code_value;
            code_value = 1;
            output_code_head = output_head;
            length = 0;
            output_head++;
            if (in[input_head] == 0) {
                input_head++;
            }
        } else {
            out[output_head] = in[input_head];
            code_value++;
            input_head++;
            output_head++;
        }
        length++;
    }
    out[output_code_head] = code_value;

    // Trailing delimeter
    out[output_head] = 0;
    output_head++;

    out.resize(output_head);
}

void COBSSerialDevice::cobs_decode(const WirePacket &in, Packet &out) {
    out.clear();
    if (in.size() == 0) {
        return;
    }

    out.resize(in.size() + in.size() / 254);
    uint8_t code = 0xff;
    uint8_t left_in_block = 0;
    size_t write_head = 0;
    for (const uint8_t byte : in) {
        if (left_in_block) {
            out[write_head] = byte;
            write_head++;
        } else {
            left_in_block = byte;
            if (left_in_block != 0 && (code != 0xff)) {
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
    out.resize(write_head);

    return;
}

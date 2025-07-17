#include "core/device/cobs_device.h"

COBSSerialDevice::COBSSerialDevice(int32_t port, int32_t baud) : port(port), baud(baud) {
    vexGenericSerialEnable(port, 0);
    vexGenericSerialBaudrate(port, baud);
}

void COBSSerialDevice::hexdump(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", data[i]);
        if ((i % 16 == 0 && i != 0) || (i == len - 1)) {
            printf("\n");
        }
    }
    fflush(stdout);
}

COBSSerialDevice::Packet COBSSerialDevice::get_last_decoded_packet() { return last_decoded_packet; }

int COBSSerialDevice::send_cobs_packet_blocking(const uint8_t *data, size_t size, bool leading_delimeter) {
    serial_access_mut.lock();

    writing_buffer.resize(size);
    std::copy(data, data + size, writing_buffer.begin());
    COBSSerialDevice::cobs_encode(writing_buffer, encoded_write, leading_delimeter);
    // printf("send: ");
    // hexdump(encoded_write.data(), encoded_write.size());

    size_t write_head = 0;
    int32_t num_free = vexGenericSerialWriteFree(port);
    while (write_head < encoded_write.size()) {
        if (num_free < 0) {
            // error on port
            serial_access_mut.unlock();
            return num_free;
        } else if (num_free == 0) {
            vexGenericSerialFlush(port);
            continue;
        }
        size_t num_to_transmit = encoded_write.size() - write_head;
        if (num_to_transmit > num_free) {
            num_to_transmit = num_free;
        }
        int32_t sent = vexGenericSerialTransmit(port, encoded_write.data() + write_head, num_to_transmit);
        if (sent != num_to_transmit) {
            // error on port

            serial_access_mut.unlock();
            return -1;
        }
        write_head += sent;
        num_free = vexGenericSerialWriteFree(port);
    }
    serial_access_mut.unlock();

    return write_head;
}
bool COBSSerialDevice::poll_incoming_data_once() {
    while (true) {
        int toRead = vexGenericSerialReceiveAvail(port);
        if (toRead <= 0) {
            return false;
        }
        int i = vexGenericSerialReadChar(port);
        if (i < 0) {
            return false;
        }
        bool finished = handle_incoming_byte((uint8_t)i);
        if (finished) {
            return finished;
        }
    }
    return false;
}

bool COBSSerialDevice::handle_incoming_byte(uint8_t b) {
    if (b == 0) {
        if (incoming_wire_packet.size() == 0) {
            // got delimeter but had no packet, just reading delimeters
            return false;
        } else {
            cobs_decode(incoming_wire_packet, last_decoded_packet);
            incoming_wire_packet.clear();
            return true;
        }
    } else {
        incoming_wire_packet.push_back(b);
        return false;
    }
}
int COBSSerialDevice::receive_cobs_packet_blocking(uint8_t *data, size_t max_size, uint32_t timeout_us) {
    serial_access_mut.lock();
    size_t start_time = vexSystemHighResTimeGet();
    while (true) {
        // Timed out
        size_t elapsed = vexSystemHighResTimeGet() - start_time;
        if (elapsed > timeout_us && timeout_us != 0) {
            serial_access_mut.unlock();
            return -2;
        }
        bool got_packet = poll_incoming_data_once();
        if (got_packet) {
            break;
        }
        vex::this_thread::yield();
    }

    for (size_t index = 0; index < last_decoded_packet.size(); index++) {
        if (index >= max_size) {
            serial_access_mut.unlock();
            return max_size;
        }
        data[index] = last_decoded_packet[index];
    }

    serial_access_mut.unlock();
    return last_decoded_packet.size();
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
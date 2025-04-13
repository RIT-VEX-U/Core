#pragma once
#include "../core/include/device/vdb/protocol.hpp"

namespace VDP {
/**
 * Defines different states of packet validility
 */
enum class PacketValidity : uint8_t {
    Ok,
    BadChecksum,
    TooSmall,
};
/**
 * defines a device registry for sending data or listening to data over a device
 */
class Registry {
  public:
    int num_bad = 0;
    int num_small = 0;
    enum Side {
        Controller,
        Listener,
    };
    using CallbackFn = std::function<void(const VDP::Channel &)>;
    /**
     * creates a device registry for sending data or listening to data over the device
     * @param device the device to send data to
     * @param reg_type the type of registry it is (Listener or Controller)
     */
    Registry(AbstractDevice *device, Side reg_type);
    /**
     * @return returns a char* of the registry type (Listener or Controller)
     */
    const char *identifier();

    /**
     * @brief Call this if you are a device who has a packet for the protocol to decode
     * @param pac the packet to take.
     */
    void take_packet(const Packet &pac);

    PartPtr get_remote_schema(ChannelID id);
    /**
     * installs a callback to a function that is called when the registry broadcasts the data schematic
     * @param on_broadcastf the callback to run when the registry broadcasts the schematic
     */
    void install_broadcast_callback(CallbackFn on_broadcast);
    /**
     * installs a callback to a function that is called when the registry broadbasts data
     * @param on_dataf the callback to run when the registry broadcasts data
     */
    void install_data_callback(CallbackFn on_data);

    /**
     * creates a new channel for the given VDP object, broadcasts it to the device
     * on the other end of the line. This channel is open to be written to
     * immediately, however it is not guaranteed to be sent to the other side
     * until broadcasting has been completed
     * Channel is only valid once negotiate has been callsed
     * @param for_data the Part Pointer to the data the channel should hold
     * @return the channel id for the new channel created
     */
    ChannelID open_channel(PartPtr for_data);
    /**
     * sets the data at the channel id to a Part Pointer and sends it to the device
     * @param id The id of the channel to hold the data
     * @param data the Part Pointer for the channel to hold and send to the device
     */
    bool send_data(ChannelID id, PartPtr data);
    /**
     * sends channel schematics to the Registry device and checks for ackowledgements
     * @return whether or not all channel's were acknowledgements
     */
    bool negotiate();

  private:
    ChannelID new_channel_id() {
        ChannelID id = next_channel_id;
        next_channel_id++;
        return id;
    }

    Side reg_type;
    bool needs_ack = false;
    static constexpr size_t ack_ms = 500;

    AbstractDevice *device;
    // Our channels (us -> them)
    std::vector<Channel> my_channels;
    ChannelID next_channel_id = 0;

    // The channels we know about from the other side
    // (them -> us)
    std::vector<Channel> remote_channels;

    CallbackFn on_broadcast = [&](VDP::Channel chan) {
        std::string schema_str = chan.data->pretty_print();
        printf(
          "VDB-%s: No Broadcast Callback installed: Received broadcast "
          "for channel id "
          "%d:\n%s\n",
          (reg_type == Side::Controller ? "Controller" : "Listener"), int(chan.id), schema_str.c_str()
        );
    };
    CallbackFn on_data = [](VDP::Channel chan) {
        printf(
          "VDB: No Data Callback installed: Received data for channel "
          "%d:\n%s\n",
          int(chan.id), chan.data->pretty_print_data().c_str()
        );
    };
};
} // namespace VDP

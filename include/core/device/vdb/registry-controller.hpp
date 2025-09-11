#pragma once
#include "core/device/vdb/protocol.hpp"
#include "vex.h"
#include <functional>
#include "core/device/vdb/visitor.hpp"
#include "core/device/vdb/builtins.hpp"

namespace VDP {
class RegistryController {
  public:
    int num_bad = 0;
    int num_small = 0;
    using CallbackFn = std::function<void(const VDP::Channel &)>;
    /**
     * creates a device registry for sending data or listening to data over the device
     * @param device the device to send data to
     * @param reg_type the type of registry it is (Listener or Controller)
     */
    RegistryController(AbstractDevice *device);
    /**
     * @brief Call this if you are a device who has a packet for the protocol to decode
     * @param pac the packet to take.
     */
    void take_packet(const Packet &pac);
    /**
     * gets the data currently stored at a channel
     * @param id the remote channel id to get data from
     * @param return the Part Pointer of schema data in the channel
     */
    PartPtr get_data(ChannelID id);

    /**
     * creates a new channel for the given VDP object, broadcasts it to the device
     * on the other end of the line. This channel is open to be written to
     * immediately, however it is not guaranteed to be sent to the other side
     * until broadcasting has been completed
     * Channel is only valid once negotiate has been callsed
     * @param for_data the Part Pointer to the data the channel should hold
     * @return the channel id for the new channel created
     */
    ChannelID open_channel(PartPtr &for_data);
    /**
     * sets the data at the channel id to a Part Pointer and sends it to the device
     * @param id The id of the channel to hold the data
     * @param data the Part Pointer for the channel to hold and send to the device
     */
    bool send_data(ChannelID id);

    /**
     * sends channel schematics to the Registry device and checks for ackowledgements
     * @return whether or not all channel's were acknowledgements
     */
    bool negotiate();
    int rec_switch_time = 1000;

  private:
  std::vector<Channel> channels;
    ChannelID new_channel_id() {
        ChannelID id = next_channel_id;
        next_channel_id++;
        return id;
    }

    int responses_in_queue;
    bool needs_ack = false;
    vex::timer timer;
    bool rec_mode = false;
    static constexpr size_t ack_ms = 500;

    AbstractDevice *device;
    // Our channels (us -> them)
    
    ChannelID next_channel_id = 0;
    /**
     * what the bot does when it recieves a broadcast packet to the board
     */
    CallbackFn on_broadcast = [&](VDP::Channel chan) {
        std::string schema_str = chan.data->pretty_print();
        printf(
          "VDB-Controller: No Broadcast Callback installed: Received broadcast "
          "for channel id "
          "%d:\n%s\n",
          int(chan.getID()), schema_str.c_str()
        );
    };
    /**
     * what the bot does when it receives a data packet from the board
     * essentially moves what they give us -> what we have and then updates the parts by calling response
     */
    CallbackFn on_data = [&](VDP::Channel new_data) {
        ResponsePacketVisitor RV(new_data.data);
        PartPtr original_data = channels[new_data.id].data;
        original_data->Visit(&RV);
        original_data->response();
    };
};
} // namespace VDP
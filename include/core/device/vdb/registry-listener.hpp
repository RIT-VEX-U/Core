#pragma once
#include "protocol.hpp"
#include <deque>

namespace VDP {
/**
 * defines a device registry for sending data or listening to data over a device
 */
template <typename MutexType> class RegistryListener {
public:
  int num_bad = 0;
  int num_small = 0;
  using CallbackFn = std::function<void(const VDP::Channel &)>;
  /**
   * creates a device registry for sending data or listening to data over the
   * device
   * @param device the device to send data to
   * @param reg_type the type of registry it is (Listener or Controller)
   */
  RegistryListener(AbstractDevice *device) : device(device) {
    device->register_receive_callback([&](const Packet &p) {
      printf("Listener: GOT PACKET\n");
      take_packet(p);
    });
  };
  /**
   * @brief Call this if you are a device who has a packet for the protocol to
   * decode
   * @param pac the packet to take.
   */
  void take_packet(const Packet &pac) {
    VDPTracef("Received packet of size %d", (int)pac.size());
    // checks the validity of the packet
    const VDP::PacketValidity status = validate_packet(pac);

    if (status == VDP::PacketValidity::BadChecksum) {
      VDPWarnf("Listener: Bad packet checksum. Skipping");
      num_bad++;
      return;
    } else if (status == VDP::PacketValidity::TooSmall) {
      num_small++;
      VDPWarnf("Listener: Packet too small to be valid (%d bytes). Skipping",
               (int)pac.size());
      return;
    } else if (status != VDP::PacketValidity::Ok) {
      VDPWarnf("Listener: Unknown validity of packet (BAD). Skipping");
      return;
    }
    // checks the packet function from the header
    const VDP::PacketHeader header = VDP::decode_header_byte(pac[0]);
    if (header.func == VDP::PacketFunction::Send) {
      VDPTracef("Listener: PacketFunction Send");

      if (header.type == VDP::PacketType::Data) {
        // if the packet is a data, get the data from the packet
        VDPTracef("Listener: PacketType Data");
        // get the channel id from the second byte of the packet
        const ChannelID id = pac[1];
        // stores the channel id's schema in a Part Pointer
        const PartPtr part = get_remote_schema(id);
        if (part == nullptr) {
          VDPDebugf("VDB-Listener: No channel information for id: %d", id);
          return;
        }
        // creates a PacketReader starting after the channel id location
        PacketReader reader{pac, 2};
        // stores the data read from the packet to the Registry Part
        part->read_data_from_message(reader);
        // runs the channel's on data callback
        on_data(Channel{part, id});
      } else if (header.type == VDP::PacketType::Broadcast) {
        printf("got broadcast packet\n");
        // if the packet is a broadcast, decode the packet
        VDPTracef("Listener: PacketType Broadcast", "");
        auto decoded = VDP::decode_broadcast(pac);
        // create a channel and give it the decoded packet
        VDP::Channel chan{decoded.second, decoded.first};
        // checks if the new channel is outside of the vector of remote channels
        if (channels.size() < chan.id) {
          VDPWarnf("Listener: Out of order broadcast. dropping");
          return;
        }
        // adds the channel to the vector of remote channels
        channels.push_back(chan);
        VDPTracef("Listener: Got broadcast of channel %d", int(chan.id));
        // runs the channel's on broadcast callback
        on_broadcast(chan);

        // creates a packet and writes the channel acknowledgement to it,
        // then sends it to the device
        Packet scratch;
        PacketWriter writer{scratch};
        writer.write_channel_acknowledge(chan);
        device->send_packet(writer.get_packet());
        printf("Listener: sent channel ack\n");
      }
    } else if (header.func == VDP::PacketFunction::Request) {
      printf("got request packet\n");
      // if the packet is a data, get the data from the packet
      VDPTracef("Listener: PacketType Request");
      // creates a PacketReader starting after the channel id location
      if(channel_response_queue.size() > 0){
        Packet scratch;
        PacketWriter writer{scratch};
        response_queue_mutex.lock();
        writer.write_response(channel_response_queue);
        response_queue_mutex.unlock();
        device->send_packet(writer.get_packet());
        printf("Listener: sent available data\n");
      }
      else{
        printf("no data available for response\n");
      }
      
    }
  };

  /**
   * @brief Submits a channel to respond to the board with
   * @param id the channel id to respond with
   * @return if the channel was submitted successfully or not
   */
  bool submit_response(PacketType type, ChannelID id, PartPtr data) {
    if (type != PacketType::Data) {
      printf("packet type is not data, not usable data\n");
      return false;
    }
    VDP::Channel channel_response = channels[id];
    channel_response.data = data;
    if (channels.size() < id) {
      printf("cannot respond to channel: %d, channel does not exist\n", id);
      return false;
    }
    response_queue_mutex.lock();
    channel_response_queue.push_back(channel_response);
    response_queue_mutex.unlock();
    return true;
  };

  PartPtr get_remote_schema(ChannelID id) {
    if (id >= channels.size()) {
      return nullptr;
    }
    return channels[id].data;
  };
  /**
   * installs a callback to a function that is called when the registry
   * broadcasts the data schematic
   * @param on_broadcastf the callback to run when the registry broadcasts the
   * schematic
   */
  void install_broadcast_callback(CallbackFn on_broadcastf) {
    VDPTracef("Listener: Installed broadcast callback for ");
    this->on_broadcast = (on_broadcastf);
  };
  /**
   * installs a callback to a function that is called when the registry
   * broadbasts data
   * @param on_dataf the callback to run when the registry broadcasts data
   */
  void install_data_callback(CallbackFn on_dataf) {
    VDPTracef("Listener: Installed data callback for ");
    this->on_data = (on_dataf);
  };
  /**
   * sets the data at the channel id to a Part Pointer and sends it to the
   * device
   * @param id The id of the channel to hold the data
   * @param data the Part Pointer for the channel to hold and send to the device
   */
  bool send_data(ChannelID id, PartPtr data) {
    // checks if the channel is actually stored in the Registry
    if (id > channels.size()) {
      printf("VDB-Listener: Channel with ID %d doesn't exist yet\n", (int)id);
      return false;
    }
    // sets the channel's data to the Part Pointer given
    Channel &chan = channels[id];
    chan.data = data;
    // checks if the channel has been acknowledged yet
    if (!chan.acked) {
      printf("VDB-Listener: Channel %d has not yet been negotiated. Dropping "
             "packet\n",
             (int)id);
      return false;
    }
    // if it has been acknowledged write the channel's data to a packet and send
    // it to the device
    VDP::Packet scratch;
    PacketWriter writ{scratch};

    writ.write_data_message(chan);
    VDP::Packet pac = writ.get_packet();

    return device->send_packet(pac);
  };
  /**
   * sends channel schematics to the Registry device and checks for
   * ackowledgements
   * @return whether or not all channel's were acknowledgements
   */
private:
  ChannelID new_channel_id() {
    ChannelID id = next_channel_id;
    next_channel_id++;
    return id;
  }
  bool needs_ack = false;
  static constexpr size_t ack_ms = 500;

  AbstractDevice *device;
  // Our channels (us -> them)
  std::vector<Channel> channels;
  ChannelID next_channel_id = 0;
  std::deque<Channel> chans_to_send;

  // The channels we know about from the other side
  // (them -> us)
  std::deque<Channel> channel_response_queue;

  MutexType response_queue_mutex;

  CallbackFn on_broadcast = [&](VDP::Channel chan) {
    std::string schema_str = chan.data->pretty_print();
    printf("VDB-Listener: No Broadcast Callback installed: Received broadcast "
           "for channel id "
           "%d:\n%s\n",
           int(chan.id), schema_str.c_str());
  };
  CallbackFn on_data = [](VDP::Channel chan) {
    printf("VDB Listener: No Data Callback installed: got data for channel "
           "%d:\n%s\n",
           int(chan.id), chan.data->pretty_print_data().c_str());
  };
  CallbackFn on_rec = [](VDP::Channel chan) {
    printf(
        "VDB Listener: No Data Callback installed: Received data for channel "
        "%d:\n%s\n",
        int(chan.id), chan.data->pretty_print_data().c_str());
  };
};
} // namespace VDP

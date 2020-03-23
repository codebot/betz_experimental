#include <cstdio>
#include <cstdlib>
#include <stdint.h>
#include <vector>
#include "lightweightserial.h"

extern "C" {
#include "parser.h"
}

#include "ros/ros.h"
#include "betz_node/SetIntParam.h"
#include "betz_node/GetIntParam.h"

class Betz
{
public:
  LightweightSerial *serial = nullptr;
  std::string rs485_device;
  ros::NodeHandle nh, nh_private;
  int num_params = 0;

  Betz()
  : nh(), nh_private("~")
  {
  }

  ~Betz()
  {
  }

  bool open_device()
  {
    nh_private.param<std::string>(
        "rs485_device",
        rs485_device,
        "/dev/ttyUSB0");
    serial = new LightweightSerial(rs485_device.c_str(), 1000000);
    if (!serial)
    {
      ROS_FATAL("couldn't open serial device");
      return false;
    }
    ROS_INFO("opened %s", rs485_device.c_str());
    return true;
  }

  bool send_pkt(const uint8_t *pkt, const uint32_t len)
  {
    if (len >= 252)
    {
      ROS_ERROR("refusing to send %d-byte packet. must be <252.",
          static_cast<int>(len));
      return false;
    }
    // ROS_INFO("sending %d-byte packet", static_cast<int>(len));
    uint8_t framed_pkt[len+5];
    framed_pkt[0] = 0xbe;
    framed_pkt[1] = 0xef;
    framed_pkt[2] = static_cast<uint8_t>(len);
    uint16_t csum = static_cast<uint16_t>(len);
    for (uint32_t i = 0; i < len; i++)
    {
      framed_pkt[i+3] = pkt[i];
      csum += pkt[i];  // so bad. do something smarter someday.
    }
    framed_pkt[len+3] = static_cast<uint8_t>(csum & 0xff);
    framed_pkt[len+4] = static_cast<uint8_t>(csum >> 8);
    return serial->write_block(framed_pkt, len+5);
  }

  // this function spins max_seconds or until expected_pkt_id arrives
  bool rs485_spin(const double max_seconds, const uint8_t expected_pkt_id = 0)
  {
    ros::Rate loop_rate(1000);
    ros::Time t_end = ros::Time::now() + ros::Duration(max_seconds);
    while (ros::ok())
    {
      if (max_seconds > 0 && t_end < ros::Time::now())
        break;
      loop_rate.sleep();
      ros::spinOnce();
      uint8_t rx_byte = 0;
      while (serial->read(&rx_byte))
      {
        //printf("rx: %02x\n", rx_byte);
        uint8_t rx_pkt_id = parser_process_byte(rx_byte);
        if (expected_pkt_id && expected_pkt_id == rx_pkt_id)
          return rx_pkt_id;
      }
    }
    ROS_WARN("rs485_spin timeout :(");
    return 0;
  }
  
  bool set_led(bool on)
  {
    uint8_t pkt[2] = { 0x10, 0x00 };
    if (on)
      pkt[1] = 1;
    return send_pkt(pkt, sizeof(pkt));
  }
  
  int get_num_params()
  {
    uint8_t pkt[1] = { 0x01 };
    send_pkt(pkt, sizeof(pkt));
    if (1 == rs485_spin(1.0, 0x01))
      return num_params;
    return -1;
  }
  
  void rs485_rx_num_params(const uint32_t payload_len, const uint8_t *payload)
  {
    ROS_INFO("rs485_rx_num_params()");
    if (payload_len != 4) {
      ROS_ERROR("unexpected payload len: %d", (int)payload_len);
      return;
    }
    num_params =
        payload[0]         |
        (payload[1] << 8)  |
        (payload[2] << 16) |
        (payload[3] << 24) ;
  }

  void rs485_rx_flash_read(const uint32_t payload_len, const uint8_t *payload)
  {
    ROS_INFO("rs485_rx_flash_read(%d)", payload_len);
  }
  
  void rs485_rx_pkt(const uint32_t len, const uint8_t *data)
  {
    if (len == 0)
    {
      ROS_ERROR("received zero-length rs485 packet");
      return;
    }
    const uint8_t pkt_id = data[0];
    switch(pkt_id)
    {
      case 0x01: rs485_rx_num_params(len-1, data+1); break;
      case 0xf0: rs485_rx_flash_read(len-1, data+1); break;
      default: ROS_INFO("unrecognized packet ID: %02x", (int)pkt_id);
    }
  }
  
  bool set_int_param(betz_node::SetIntParam::Request &request,
      betz_node::SetIntParam::Response &response)
  {
    int param_name_len = static_cast<int>(request.name.size());
    if (param_name_len > 250)
    {
      ROS_ERROR("param name len is too long: %s", request.name.c_str());
      return false;
    }
    ROS_INFO("set_int_param(%s, %d)",
        request.name.c_str(), request.value);
    uint8_t pkt[256] = {0};
    pkt[0] = 4;  // packet type 4 = set int param by name
    memcpy(&pkt[1], &request.value, 4);
    strcpy((char *)&pkt[5], request.name.c_str());
    return send_pkt(pkt, 6 + param_name_len);
  }
  
  bool get_int_param(betz_node::GetIntParam::Request &request,
      betz_node::GetIntParam::Response &response)
  {
    int param_name_len = static_cast<int>(request.name.size());
    if (param_name_len > 250)
    {
      ROS_ERROR("param name len is too long: %s", request.name.c_str());
      return false;
    }
    uint8_t pkt[256] = {0};
    pkt[0] = 3;  // packet type 3 = get int param by name
    strcpy((char *)&pkt[1], request.name.c_str());
    if (!send_pkt(pkt, 2 + param_name_len))
      return false;
    if (!rs485_spin(0.5, 3))
    {
      ROS_ERROR("didn't hear back from MCU for request for parameter [%s]",
          request.name.c_str());
      return false;
    }
    // todo: actually copy the returned value into the response message
    ROS_INFO("get_int_param(%s) = %d", request.name.c_str(), response.value);
    return true;
  }

  bool read_flash(const uint32_t start_addr, const uint32_t len)
  {
    uint8_t pkt[9] = {0};
    pkt[0] = 0xf0;
    memcpy(&pkt[1], &start_addr, sizeof(uint32_t));
    memcpy(&pkt[5], &len, sizeof(uint32_t));
    send_pkt(pkt, sizeof(pkt));

    if (rs485_spin(1.0, 0xf0) == 0xf0)
      return true;
    return false;
  }
};

Betz *g_betz = nullptr;
static void rs485_rx_pkt(const uint32_t len, const uint8_t *data)
{
  if (g_betz)
    g_betz->rs485_rx_pkt(len, data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "betz_node");
  Betz betz;
  g_betz = &betz;  // singleton to deal with C library function pointer
  parser_init();
  parser_set_rx_pkt_fptr(rs485_rx_pkt);

  if (!betz.open_device())
  {
    ROS_FATAL("could not open device");
    return 1;
  }

  const int num_params = betz.get_num_params();
  ROS_INFO("device has %d params", num_params);

  const bool read_ok = betz.read_flash(0x08000000, 128);
  ROS_INFO("read_flash(0x0, 128) = %d", read_ok ? 1 : 0);
  /*
  ros::ServiceServer int_param_srv =
      nh.advertiseService("set_int_param", set_int_param);
  ROS_INFO("entering slow_bldc spin loop");
  betz.rs485_spin(-1);  // spin forever. wooooahhh that makes me feel dizzy
  */
  return 0;
}

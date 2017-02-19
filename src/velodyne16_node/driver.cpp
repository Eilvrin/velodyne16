/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 University of Freiburg
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the Velodyne 3D LIDARs
 */

#include "driver.h"

VelodyneDriver::VelodyneDriver(ros::NodeHandle node,
                               ros::NodeHandle private_nh)
{
  // use private node handle to get parameters
  private_nh.param("frame_id", frame_id_, std::string("velodyne16"));
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  frame_id_ = tf::resolve(tf_prefix, frame_id_);

  private_nh.param("port", port_, (int) DATA_PORT_NUMBER);

  private_nh.param("device_ip", devip_str_, std::string(""));
  if (!devip_str_.empty())
    {
      ROS_INFO_STREAM("Only accepting packets from IP address: "
                      << devip_str_);
      int address;
      address = inet_addr(devip_str_.c_str());
      if (address == -1) 
        {
        ROS_ERROR_STREAM ("Error in IP adress: " << devip_str_);
        node.shutdown();
        exit(EXIT_FAILURE);
      } else {
        devip_.s_addr = address;
      }
    }  

  // Open UDP port
  sockfd_ = -1;

  // connect to Velodyne UDP port
  ROS_INFO_STREAM("Opening UDP socket: port " << port_);
  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ == -1)
    {
      ROS_ERROR ("Error openning socket.");
      perror("socket");
      node.shutdown();
      exit(EXIT_FAILURE);
    }

  sockaddr_in my_addr;                     // my address information
  memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
  my_addr.sin_family = AF_INET;            // host byte order
  my_addr.sin_port = htons(port_);      // port in network byte order
  my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP

  if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
    {
      ROS_ERROR ("Error binding socket.");
      perror("bind"); 
      node.shutdown();
      exit(EXIT_FAILURE);
    }

  if (fcntl(sockfd_,F_SETFL, O_NONBLOCK|FASYNC) < 0)
    {
      ROS_ERROR ("Error setting non-blocking socket.");
      perror("non-block");
      node.shutdown();
      exit(EXIT_FAILURE);
    }

  // Initialize dynamic reconfigure
  srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne16::
    Velodyne16NodeConfig> > (private_nh);
  dynamic_reconfigure::Server<velodyne16::Velodyne16NodeConfig>::
    CallbackType f;
  f = boost::bind (&VelodyneDriver::callback, this, _1, _2);
  srv_->setCallback (f); // Set callback function und call initially

  // raw packet output topic
  output_ =
    node.advertise<velodyne16::VelodynePacket>("velodyne16/packets", 500);

  previous_pkt_time = 0;

}

VelodyneDriver::~VelodyneDriver()
{
 if (sockfd_!= -1) close(sockfd_);
}

void VelodyneDriver::callback(velodyne16::Velodyne16NodeConfig &config,
              uint32_t level)
{
  ROS_INFO("Reconfigure Request");
  time_offset_ = config.time_offset;
}

//  poll the device
bool VelodyneDriver::publishPacket(void)
{
  velodyne16::VelodynePacketPtr packet (new velodyne16::VelodynePacket);
  packet->header.frame_id = frame_id_;
  bool receive_sucess = getPacket(packet, time_offset_);

  if (receive_sucess){
    output_.publish(packet);
    uint32_t packet_time = getPacketTime(packet);
    if (previous_pkt_time > packet_time) 
      ROS_WARN("The timestamp of packet is smaller than previous packet timestamp. This should happen only once per hour (velodyne time reset).");
    previous_pkt_time = packet_time;
  }

  return true;
}

//  poll the device
bool VelodyneDriver::getPacket(velodyne16::VelodynePacketPtr pkt, const double time_offset)
{
  size_t packet_size = sizeof(pkt->data);
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;
  fds[0].revents = 0;

  static const int POLL_TIMEOUT = 1000; // [ms]

  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);

  while (true)
    {
      // Unfortunately, the Linux kernel recvfrom() implementation
      // uses a non-interruptible sleep() when waiting for data,
      // which would cause this method to hang if the device is not
      // providing data.  We poll() the device first to make sure
      // the recvfrom() will not block.
      //
      // Note, however, that there is a known Linux kernel bug:
      //
      //   Under Linux, select() may report a socket file descriptor
      //   as "ready for reading", while nevertheless a subsequent
      //   read blocks.  This could for example happen when data has
      //   arrived but upon examination has wrong checksum and is
      //   discarded.  There may be other circumstances in which a
      //   file descriptor is spuriously reported as ready.  Thus it
      //   may be safer to use O_NONBLOCK on sockets that should not
      //   block.

      // poll() until input available
      do
      {
        int retval = poll(fds, 1, POLL_TIMEOUT);
        if (retval < 0)             // poll() error?
          {
            if (errno != EINTR)
              ROS_ERROR("poll() error: %s", strerror(errno));
            return false;
          }
        if (retval == 0)            // poll() timeout?
          {
            ROS_WARN("Velodyne poll() timeout");
            return false;
          }
        if ((fds[0].revents & POLLERR)
            || (fds[0].revents & POLLHUP)
            || (fds[0].revents & POLLNVAL)) // device error?
          {
            ROS_ERROR("poll() reports Velodyne error");
            return false;
          }
      } while ((fds[0].revents & POLLIN) == 0);

      // Time stamp is set to the start time of packet creation (time of firing the packet's first beam of the first firing sequence). 
      // ASSUMPTION: For the VLP-16 the time to accumulate one data packet equals 55.296µs * 24 = 1327.104 µs 
      // We neglect the transfer time. It is handled by the calibration value.
      pkt->header.stamp = ros::Time::now() - ros::Duration(1327.104*1.0e-6) + ros::Duration(time_offset);

      // Receive packets that should now be available from the
      // socket using a blocking read.
      ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0],
                                packet_size,  0,
                                (sockaddr*) &sender_address,
                                &sender_address_len);

      if (nbytes < 0)
        {
          if (errno != EWOULDBLOCK)
            {
              ROS_INFO("recvfail");
              return false;
            }
        }
      else if ((size_t) nbytes == packet_size)
        {
          // If packet is not from the lidar scanner we selected by IP,
          // continue; otherwise we are done.
          if(devip_str_ != ""
            && sender_address.sin_addr.s_addr != devip_.s_addr)
            continue;
          else
            break; //done
        }

    }
  return true;
}

uint32_t VelodyneDriver::getPacketTime(velodyne16::VelodynePacketPtr pkt){
  uint32_t timestamp;
  std::memcpy(&timestamp, &pkt->data[1200], sizeof(uint32_t));
  return timestamp;
}

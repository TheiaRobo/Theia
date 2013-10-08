/* Auto-generated by genmsg_cpp for file /home/diogo/Documents/DD2425_2013/Project/Theia/robot_messages/msg/coords.msg */
#ifndef ROBOT_MESSAGES_MESSAGE_COORDS_H
#define ROBOT_MESSAGES_MESSAGE_COORDS_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"

namespace robot_messages
{
template <class ContainerAllocator>
struct coords_ {
  typedef coords_<ContainerAllocator> Type;

  coords_()
  : header()
  , x(0.0)
  , y(0.0)
  , z(0.0)
  {
  }

  coords_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , x(0.0)
  , y(0.0)
  , z(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef float _x_type;
  float x;

  typedef float _y_type;
  float y;

  typedef float _z_type;
  float z;


  typedef boost::shared_ptr< ::robot_messages::coords_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robot_messages::coords_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct coords
typedef  ::robot_messages::coords_<std::allocator<void> > coords;

typedef boost::shared_ptr< ::robot_messages::coords> coordsPtr;
typedef boost::shared_ptr< ::robot_messages::coords const> coordsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::robot_messages::coords_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::robot_messages::coords_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace robot_messages

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::robot_messages::coords_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::robot_messages::coords_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::robot_messages::coords_<ContainerAllocator> > {
  static const char* value() 
  {
    return "75f40115cb5a29c0ceea66f491a5e1a3";
  }

  static const char* value(const  ::robot_messages::coords_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x75f40115cb5a29c0ULL;
  static const uint64_t static_value2 = 0xceea66f491a5e1a3ULL;
};

template<class ContainerAllocator>
struct DataType< ::robot_messages::coords_<ContainerAllocator> > {
  static const char* value() 
  {
    return "robot_messages/coords";
  }

  static const char* value(const  ::robot_messages::coords_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::robot_messages::coords_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float32 x\n\
float32 y\n\
float32 z\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::robot_messages::coords_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::robot_messages::coords_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::robot_messages::coords_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::robot_messages::coords_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.x);
    stream.next(m.y);
    stream.next(m.z);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct coords_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robot_messages::coords_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::robot_messages::coords_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ROBOT_MESSAGES_MESSAGE_COORDS_H


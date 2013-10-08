/* Auto-generated by genmsg_cpp for file /home/robo/DD2425_2013/Project/differential_drive/msg/PWM.msg */
#ifndef DIFFERENTIAL_DRIVE_MESSAGE_PWM_H
#define DIFFERENTIAL_DRIVE_MESSAGE_PWM_H
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

namespace differential_drive
{
template <class ContainerAllocator>
struct PWM_ {
  typedef PWM_<ContainerAllocator> Type;

  PWM_()
  : header()
  , PWM1(0)
  , PWM2(0)
  {
  }

  PWM_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , PWM1(0)
  , PWM2(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int32_t _PWM1_type;
  int32_t PWM1;

  typedef int32_t _PWM2_type;
  int32_t PWM2;


  typedef boost::shared_ptr< ::differential_drive::PWM_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::differential_drive::PWM_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PWM
typedef  ::differential_drive::PWM_<std::allocator<void> > PWM;

typedef boost::shared_ptr< ::differential_drive::PWM> PWMPtr;
typedef boost::shared_ptr< ::differential_drive::PWM const> PWMConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::differential_drive::PWM_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::differential_drive::PWM_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace differential_drive

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::differential_drive::PWM_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::differential_drive::PWM_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::differential_drive::PWM_<ContainerAllocator> > {
  static const char* value() 
  {
    return "7536f824b238f3bc8ae162f5a97c5bfc";
  }

  static const char* value(const  ::differential_drive::PWM_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x7536f824b238f3bcULL;
  static const uint64_t static_value2 = 0x8ae162f5a97c5bfcULL;
};

template<class ContainerAllocator>
struct DataType< ::differential_drive::PWM_<ContainerAllocator> > {
  static const char* value() 
  {
    return "differential_drive/PWM";
  }

  static const char* value(const  ::differential_drive::PWM_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::differential_drive::PWM_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# PWM should be between -255 and 255\n\
# 1 --> left\n\
# 2 --> right\n\
Header header\n\
\n\
int32 PWM1\n\
int32 PWM2\n\
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

  static const char* value(const  ::differential_drive::PWM_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::differential_drive::PWM_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::differential_drive::PWM_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::differential_drive::PWM_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.PWM1);
    stream.next(m.PWM2);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PWM_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::differential_drive::PWM_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::differential_drive::PWM_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "PWM1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.PWM1);
    s << indent << "PWM2: ";
    Printer<int32_t>::stream(s, indent + "  ", v.PWM2);
  }
};


} // namespace message_operations
} // namespace ros

#endif // DIFFERENTIAL_DRIVE_MESSAGE_PWM_H


/* Auto-generated by genmsg_cpp for file /home/robo/DD2425_2013/Theia/differential_drive/msg/KeyEvent.msg */
#ifndef DIFFERENTIAL_DRIVE_MESSAGE_KEYEVENT_H
#define DIFFERENTIAL_DRIVE_MESSAGE_KEYEVENT_H
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


namespace differential_drive
{
template <class ContainerAllocator>
struct KeyEvent_ {
  typedef KeyEvent_<ContainerAllocator> Type;

  KeyEvent_()
  : timestamp(0.0)
  , sym(0)
  , pressed(0)
  , name()
  {
  }

  KeyEvent_(const ContainerAllocator& _alloc)
  : timestamp(0.0)
  , sym(0)
  , pressed(0)
  , name(_alloc)
  {
  }

  typedef double _timestamp_type;
  double timestamp;

  typedef uint16_t _sym_type;
  uint16_t sym;

  typedef uint8_t _pressed_type;
  uint8_t pressed;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;


  typedef boost::shared_ptr< ::differential_drive::KeyEvent_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::differential_drive::KeyEvent_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct KeyEvent
typedef  ::differential_drive::KeyEvent_<std::allocator<void> > KeyEvent;

typedef boost::shared_ptr< ::differential_drive::KeyEvent> KeyEventPtr;
typedef boost::shared_ptr< ::differential_drive::KeyEvent const> KeyEventConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::differential_drive::KeyEvent_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::differential_drive::KeyEvent_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace differential_drive

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::differential_drive::KeyEvent_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::differential_drive::KeyEvent_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::differential_drive::KeyEvent_<ContainerAllocator> > {
  static const char* value() 
  {
    return "644813ba9e39b249cc8e56e2fa499967";
  }

  static const char* value(const  ::differential_drive::KeyEvent_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x644813ba9e39b249ULL;
  static const uint64_t static_value2 = 0xcc8e56e2fa499967ULL;
};

template<class ContainerAllocator>
struct DataType< ::differential_drive::KeyEvent_<ContainerAllocator> > {
  static const char* value() 
  {
    return "differential_drive/KeyEvent";
  }

  static const char* value(const  ::differential_drive::KeyEvent_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::differential_drive::KeyEvent_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 timestamp\n\
uint16 sym\n\
uint8 pressed\n\
string name\n\
\n\
";
  }

  static const char* value(const  ::differential_drive::KeyEvent_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::differential_drive::KeyEvent_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.timestamp);
    stream.next(m.sym);
    stream.next(m.pressed);
    stream.next(m.name);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct KeyEvent_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::differential_drive::KeyEvent_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::differential_drive::KeyEvent_<ContainerAllocator> & v) 
  {
    s << indent << "timestamp: ";
    Printer<double>::stream(s, indent + "  ", v.timestamp);
    s << indent << "sym: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.sym);
    s << indent << "pressed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.pressed);
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
  }
};


} // namespace message_operations
} // namespace ros

#endif // DIFFERENTIAL_DRIVE_MESSAGE_KEYEVENT_H


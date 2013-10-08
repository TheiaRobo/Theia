/* Auto-generated by genmsg_cpp for file /home/robo/DD2425_2013/Theia/HandFollow/msg/PidParams.msg */
#ifndef HANDFOLLOW_MESSAGE_PIDPARAMS_H
#define HANDFOLLOW_MESSAGE_PIDPARAMS_H
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


namespace HandFollow
{
template <class ContainerAllocator>
struct PidParams_ {
  typedef PidParams_<ContainerAllocator> Type;

  PidParams_()
  : p_x(0.0)
  , p_z(0.0)
  {
  }

  PidParams_(const ContainerAllocator& _alloc)
  : p_x(0.0)
  , p_z(0.0)
  {
  }

  typedef float _p_x_type;
  float p_x;

  typedef float _p_z_type;
  float p_z;


  typedef boost::shared_ptr< ::HandFollow::PidParams_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::HandFollow::PidParams_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PidParams
typedef  ::HandFollow::PidParams_<std::allocator<void> > PidParams;

typedef boost::shared_ptr< ::HandFollow::PidParams> PidParamsPtr;
typedef boost::shared_ptr< ::HandFollow::PidParams const> PidParamsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::HandFollow::PidParams_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::HandFollow::PidParams_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace HandFollow

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::HandFollow::PidParams_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::HandFollow::PidParams_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::HandFollow::PidParams_<ContainerAllocator> > {
  static const char* value() 
  {
    return "5a5e0c267dd7fe0632fbcd19cf9e2030";
  }

  static const char* value(const  ::HandFollow::PidParams_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x5a5e0c267dd7fe06ULL;
  static const uint64_t static_value2 = 0x32fbcd19cf9e2030ULL;
};

template<class ContainerAllocator>
struct DataType< ::HandFollow::PidParams_<ContainerAllocator> > {
  static const char* value() 
  {
    return "HandFollow/PidParams";
  }

  static const char* value(const  ::HandFollow::PidParams_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::HandFollow::PidParams_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float32 p_x\n\
float32 p_z\n\
\n\
";
  }

  static const char* value(const  ::HandFollow::PidParams_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::HandFollow::PidParams_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::HandFollow::PidParams_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.p_x);
    stream.next(m.p_z);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PidParams_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::HandFollow::PidParams_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::HandFollow::PidParams_<ContainerAllocator> & v) 
  {
    s << indent << "p_x: ";
    Printer<float>::stream(s, indent + "  ", v.p_x);
    s << indent << "p_z: ";
    Printer<float>::stream(s, indent + "  ", v.p_z);
  }
};


} // namespace message_operations
} // namespace ros

#endif // HANDFOLLOW_MESSAGE_PIDPARAMS_H


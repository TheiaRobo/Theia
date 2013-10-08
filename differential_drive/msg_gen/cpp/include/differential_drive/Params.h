/* Auto-generated by genmsg_cpp for file /home/robo/DD2425_2013/Project/differential_drive/msg/Params.msg */
#ifndef DIFFERENTIAL_DRIVE_MESSAGE_PARAMS_H
#define DIFFERENTIAL_DRIVE_MESSAGE_PARAMS_H
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
struct Params_ {
  typedef Params_<ContainerAllocator> Type;

  Params_()
  : K(0)
  , KI(0)
  , INT_MAX(0)
  , ticks(0)
  , r(0.0)
  , r_l(0.0)
  , r_r(0.0)
  , B(0.0)
  {
  }

  Params_(const ContainerAllocator& _alloc)
  : K(0)
  , KI(0)
  , INT_MAX(0)
  , ticks(0)
  , r(0.0)
  , r_l(0.0)
  , r_r(0.0)
  , B(0.0)
  {
  }

  typedef uint16_t _K_type;
  uint16_t K;

  typedef uint16_t _KI_type;
  uint16_t KI;

  typedef uint16_t _INT_MAX_type;
  uint16_t INT_MAX;

  typedef uint16_t _ticks_type;
  uint16_t ticks;

  typedef float _r_type;
  float r;

  typedef float _r_l_type;
  float r_l;

  typedef float _r_r_type;
  float r_r;

  typedef float _B_type;
  float B;


  typedef boost::shared_ptr< ::differential_drive::Params_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::differential_drive::Params_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct Params
typedef  ::differential_drive::Params_<std::allocator<void> > Params;

typedef boost::shared_ptr< ::differential_drive::Params> ParamsPtr;
typedef boost::shared_ptr< ::differential_drive::Params const> ParamsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::differential_drive::Params_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::differential_drive::Params_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace differential_drive

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::differential_drive::Params_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::differential_drive::Params_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::differential_drive::Params_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8950f824993d2867219a00186d4620f7";
  }

  static const char* value(const  ::differential_drive::Params_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8950f824993d2867ULL;
  static const uint64_t static_value2 = 0x219a00186d4620f7ULL;
};

template<class ContainerAllocator>
struct DataType< ::differential_drive::Params_<ContainerAllocator> > {
  static const char* value() 
  {
    return "differential_drive/Params";
  }

  static const char* value(const  ::differential_drive::Params_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::differential_drive::Params_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# PI Control parameters\n\
uint16 K\n\
uint16 KI\n\
uint16 INT_MAX\n\
uint16 ticks\n\
\n\
# Robot dimensions\n\
float32 r\n\
float32 r_l\n\
float32 r_r\n\
float32 B\n\
\n\
";
  }

  static const char* value(const  ::differential_drive::Params_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::differential_drive::Params_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::differential_drive::Params_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.K);
    stream.next(m.KI);
    stream.next(m.INT_MAX);
    stream.next(m.ticks);
    stream.next(m.r);
    stream.next(m.r_l);
    stream.next(m.r_r);
    stream.next(m.B);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct Params_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::differential_drive::Params_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::differential_drive::Params_<ContainerAllocator> & v) 
  {
    s << indent << "K: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.K);
    s << indent << "KI: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.KI);
    s << indent << "INT_MAX: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.INT_MAX);
    s << indent << "ticks: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.ticks);
    s << indent << "r: ";
    Printer<float>::stream(s, indent + "  ", v.r);
    s << indent << "r_l: ";
    Printer<float>::stream(s, indent + "  ", v.r_l);
    s << indent << "r_r: ";
    Printer<float>::stream(s, indent + "  ", v.r_r);
    s << indent << "B: ";
    Printer<float>::stream(s, indent + "  ", v.B);
  }
};


} // namespace message_operations
} // namespace ros

#endif // DIFFERENTIAL_DRIVE_MESSAGE_PARAMS_H


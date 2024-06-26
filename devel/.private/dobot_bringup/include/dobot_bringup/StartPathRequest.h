// Generated by gencpp from file dobot_bringup/StartPathRequest.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_STARTPATHREQUEST_H
#define DOBOT_BRINGUP_MESSAGE_STARTPATHREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dobot_bringup
{
template <class ContainerAllocator>
struct StartPathRequest_
{
  typedef StartPathRequest_<ContainerAllocator> Type;

  StartPathRequest_()
    : trace_name()
    , const_val(0)
    , cart(0)  {
    }
  StartPathRequest_(const ContainerAllocator& _alloc)
    : trace_name(_alloc)
    , const_val(0)
    , cart(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _trace_name_type;
  _trace_name_type trace_name;

   typedef int32_t _const_val_type;
  _const_val_type const_val;

   typedef int32_t _cart_type;
  _cart_type cart;





  typedef boost::shared_ptr< ::dobot_bringup::StartPathRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dobot_bringup::StartPathRequest_<ContainerAllocator> const> ConstPtr;

}; // struct StartPathRequest_

typedef ::dobot_bringup::StartPathRequest_<std::allocator<void> > StartPathRequest;

typedef boost::shared_ptr< ::dobot_bringup::StartPathRequest > StartPathRequestPtr;
typedef boost::shared_ptr< ::dobot_bringup::StartPathRequest const> StartPathRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dobot_bringup::StartPathRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dobot_bringup::StartPathRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dobot_bringup::StartPathRequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::StartPathRequest_<ContainerAllocator2> & rhs)
{
  return lhs.trace_name == rhs.trace_name &&
    lhs.const_val == rhs.const_val &&
    lhs.cart == rhs.cart;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dobot_bringup::StartPathRequest_<ContainerAllocator1> & lhs, const ::dobot_bringup::StartPathRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dobot_bringup

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::StartPathRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dobot_bringup::StartPathRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::StartPathRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dobot_bringup::StartPathRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::StartPathRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dobot_bringup::StartPathRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dobot_bringup::StartPathRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4951608306069c8a8b646ec8dd5aec26";
  }

  static const char* value(const ::dobot_bringup::StartPathRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4951608306069c8aULL;
  static const uint64_t static_value2 = 0x8b646ec8dd5aec26ULL;
};

template<class ContainerAllocator>
struct DataType< ::dobot_bringup::StartPathRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dobot_bringup/StartPathRequest";
  }

  static const char* value(const ::dobot_bringup::StartPathRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dobot_bringup::StartPathRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string trace_name\n"
"int32 const_val\n"
"int32 cart\n"
;
  }

  static const char* value(const ::dobot_bringup::StartPathRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dobot_bringup::StartPathRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.trace_name);
      stream.next(m.const_val);
      stream.next(m.cart);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StartPathRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dobot_bringup::StartPathRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dobot_bringup::StartPathRequest_<ContainerAllocator>& v)
  {
    s << indent << "trace_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.trace_name);
    s << indent << "const_val: ";
    Printer<int32_t>::stream(s, indent + "  ", v.const_val);
    s << indent << "cart: ";
    Printer<int32_t>::stream(s, indent + "  ", v.cart);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_STARTPATHREQUEST_H

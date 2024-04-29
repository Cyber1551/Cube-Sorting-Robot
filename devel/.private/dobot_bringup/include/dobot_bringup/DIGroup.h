// Generated by gencpp from file dobot_bringup/DIGroup.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_DIGROUP_H
#define DOBOT_BRINGUP_MESSAGE_DIGROUP_H

#include <ros/service_traits.h>


#include <dobot_bringup/DIGroupRequest.h>
#include <dobot_bringup/DIGroupResponse.h>


namespace dobot_bringup
{

struct DIGroup
{

typedef DIGroupRequest Request;
typedef DIGroupResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct DIGroup
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::DIGroup > {
  static const char* value()
  {
    return "808684a591f350916730d57d20e91134";
  }

  static const char* value(const ::dobot_bringup::DIGroup&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::DIGroup > {
  static const char* value()
  {
    return "dobot_bringup/DIGroup";
  }

  static const char* value(const ::dobot_bringup::DIGroup&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::DIGroupRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::DIGroup >
template<>
struct MD5Sum< ::dobot_bringup::DIGroupRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::DIGroup >::value();
  }
  static const char* value(const ::dobot_bringup::DIGroupRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::DIGroupRequest> should match
// service_traits::DataType< ::dobot_bringup::DIGroup >
template<>
struct DataType< ::dobot_bringup::DIGroupRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::DIGroup >::value();
  }
  static const char* value(const ::dobot_bringup::DIGroupRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::DIGroupResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::DIGroup >
template<>
struct MD5Sum< ::dobot_bringup::DIGroupResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::DIGroup >::value();
  }
  static const char* value(const ::dobot_bringup::DIGroupResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::DIGroupResponse> should match
// service_traits::DataType< ::dobot_bringup::DIGroup >
template<>
struct DataType< ::dobot_bringup::DIGroupResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::DIGroup >::value();
  }
  static const char* value(const ::dobot_bringup::DIGroupResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_DIGROUP_H
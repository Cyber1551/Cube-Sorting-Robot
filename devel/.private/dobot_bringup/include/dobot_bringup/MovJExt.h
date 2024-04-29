// Generated by gencpp from file dobot_bringup/MovJExt.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_MOVJEXT_H
#define DOBOT_BRINGUP_MESSAGE_MOVJEXT_H

#include <ros/service_traits.h>


#include <dobot_bringup/MovJExtRequest.h>
#include <dobot_bringup/MovJExtResponse.h>


namespace dobot_bringup
{

struct MovJExt
{

typedef MovJExtRequest Request;
typedef MovJExtResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct MovJExt
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::MovJExt > {
  static const char* value()
  {
    return "9c5f1f0d2ab66db2c121e88f04537959";
  }

  static const char* value(const ::dobot_bringup::MovJExt&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::MovJExt > {
  static const char* value()
  {
    return "dobot_bringup/MovJExt";
  }

  static const char* value(const ::dobot_bringup::MovJExt&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::MovJExtRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::MovJExt >
template<>
struct MD5Sum< ::dobot_bringup::MovJExtRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::MovJExt >::value();
  }
  static const char* value(const ::dobot_bringup::MovJExtRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::MovJExtRequest> should match
// service_traits::DataType< ::dobot_bringup::MovJExt >
template<>
struct DataType< ::dobot_bringup::MovJExtRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::MovJExt >::value();
  }
  static const char* value(const ::dobot_bringup::MovJExtRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::MovJExtResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::MovJExt >
template<>
struct MD5Sum< ::dobot_bringup::MovJExtResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::MovJExt >::value();
  }
  static const char* value(const ::dobot_bringup::MovJExtResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::MovJExtResponse> should match
// service_traits::DataType< ::dobot_bringup::MovJExt >
template<>
struct DataType< ::dobot_bringup::MovJExtResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::MovJExt >::value();
  }
  static const char* value(const ::dobot_bringup::MovJExtResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_MOVJEXT_H
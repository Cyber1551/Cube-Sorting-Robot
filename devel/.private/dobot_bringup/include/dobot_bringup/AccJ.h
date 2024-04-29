// Generated by gencpp from file dobot_bringup/AccJ.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_ACCJ_H
#define DOBOT_BRINGUP_MESSAGE_ACCJ_H

#include <ros/service_traits.h>


#include <dobot_bringup/AccJRequest.h>
#include <dobot_bringup/AccJResponse.h>


namespace dobot_bringup
{

struct AccJ
{

typedef AccJRequest Request;
typedef AccJResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct AccJ
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::AccJ > {
  static const char* value()
  {
    return "941d9ecd0f5402311261de883bef5059";
  }

  static const char* value(const ::dobot_bringup::AccJ&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::AccJ > {
  static const char* value()
  {
    return "dobot_bringup/AccJ";
  }

  static const char* value(const ::dobot_bringup::AccJ&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::AccJRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::AccJ >
template<>
struct MD5Sum< ::dobot_bringup::AccJRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::AccJ >::value();
  }
  static const char* value(const ::dobot_bringup::AccJRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::AccJRequest> should match
// service_traits::DataType< ::dobot_bringup::AccJ >
template<>
struct DataType< ::dobot_bringup::AccJRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::AccJ >::value();
  }
  static const char* value(const ::dobot_bringup::AccJRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::AccJResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::AccJ >
template<>
struct MD5Sum< ::dobot_bringup::AccJResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::AccJ >::value();
  }
  static const char* value(const ::dobot_bringup::AccJResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::AccJResponse> should match
// service_traits::DataType< ::dobot_bringup::AccJ >
template<>
struct DataType< ::dobot_bringup::AccJResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::AccJ >::value();
  }
  static const char* value(const ::dobot_bringup::AccJResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_ACCJ_H
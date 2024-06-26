// Generated by gencpp from file dobot_bringup/ClearError.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_CLEARERROR_H
#define DOBOT_BRINGUP_MESSAGE_CLEARERROR_H

#include <ros/service_traits.h>


#include <dobot_bringup/ClearErrorRequest.h>
#include <dobot_bringup/ClearErrorResponse.h>


namespace dobot_bringup
{

struct ClearError
{

typedef ClearErrorRequest Request;
typedef ClearErrorResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ClearError
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::ClearError > {
  static const char* value()
  {
    return "ca16cfbd5443ad97f6cc7ffd6bb67292";
  }

  static const char* value(const ::dobot_bringup::ClearError&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::ClearError > {
  static const char* value()
  {
    return "dobot_bringup/ClearError";
  }

  static const char* value(const ::dobot_bringup::ClearError&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::ClearErrorRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::ClearError >
template<>
struct MD5Sum< ::dobot_bringup::ClearErrorRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::ClearError >::value();
  }
  static const char* value(const ::dobot_bringup::ClearErrorRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::ClearErrorRequest> should match
// service_traits::DataType< ::dobot_bringup::ClearError >
template<>
struct DataType< ::dobot_bringup::ClearErrorRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::ClearError >::value();
  }
  static const char* value(const ::dobot_bringup::ClearErrorRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::ClearErrorResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::ClearError >
template<>
struct MD5Sum< ::dobot_bringup::ClearErrorResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::ClearError >::value();
  }
  static const char* value(const ::dobot_bringup::ClearErrorResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::ClearErrorResponse> should match
// service_traits::DataType< ::dobot_bringup::ClearError >
template<>
struct DataType< ::dobot_bringup::ClearErrorResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::ClearError >::value();
  }
  static const char* value(const ::dobot_bringup::ClearErrorResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_CLEARERROR_H

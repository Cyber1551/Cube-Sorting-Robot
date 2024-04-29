// Generated by gencpp from file dobot_bringup/RelMovJ.msg
// DO NOT EDIT!


#ifndef DOBOT_BRINGUP_MESSAGE_RELMOVJ_H
#define DOBOT_BRINGUP_MESSAGE_RELMOVJ_H

#include <ros/service_traits.h>


#include <dobot_bringup/RelMovJRequest.h>
#include <dobot_bringup/RelMovJResponse.h>


namespace dobot_bringup
{

struct RelMovJ
{

typedef RelMovJRequest Request;
typedef RelMovJResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct RelMovJ
} // namespace dobot_bringup


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dobot_bringup::RelMovJ > {
  static const char* value()
  {
    return "e279c0ec95d750d32f1bcdc49940f3d6";
  }

  static const char* value(const ::dobot_bringup::RelMovJ&) { return value(); }
};

template<>
struct DataType< ::dobot_bringup::RelMovJ > {
  static const char* value()
  {
    return "dobot_bringup/RelMovJ";
  }

  static const char* value(const ::dobot_bringup::RelMovJ&) { return value(); }
};


// service_traits::MD5Sum< ::dobot_bringup::RelMovJRequest> should match
// service_traits::MD5Sum< ::dobot_bringup::RelMovJ >
template<>
struct MD5Sum< ::dobot_bringup::RelMovJRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::RelMovJ >::value();
  }
  static const char* value(const ::dobot_bringup::RelMovJRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::RelMovJRequest> should match
// service_traits::DataType< ::dobot_bringup::RelMovJ >
template<>
struct DataType< ::dobot_bringup::RelMovJRequest>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::RelMovJ >::value();
  }
  static const char* value(const ::dobot_bringup::RelMovJRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dobot_bringup::RelMovJResponse> should match
// service_traits::MD5Sum< ::dobot_bringup::RelMovJ >
template<>
struct MD5Sum< ::dobot_bringup::RelMovJResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dobot_bringup::RelMovJ >::value();
  }
  static const char* value(const ::dobot_bringup::RelMovJResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dobot_bringup::RelMovJResponse> should match
// service_traits::DataType< ::dobot_bringup::RelMovJ >
template<>
struct DataType< ::dobot_bringup::RelMovJResponse>
{
  static const char* value()
  {
    return DataType< ::dobot_bringup::RelMovJ >::value();
  }
  static const char* value(const ::dobot_bringup::RelMovJResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DOBOT_BRINGUP_MESSAGE_RELMOVJ_H

// Generated by gencpp from file ublox_msgs/HnrPVT.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_HNRPVT_H
#define UBLOX_MSGS_MESSAGE_HNRPVT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ublox_msgs
{
template <class ContainerAllocator>
struct HnrPVT_
{
  typedef HnrPVT_<ContainerAllocator> Type;

  HnrPVT_()
    : iTOW(0)
    , year(0)
    , month(0)
    , day(0)
    , hour(0)
    , min(0)
    , sec(0)
    , valid(0)
    , nano(0)
    , gpsFix(0)
    , flags(0)
    , reserved0()
    , lon(0)
    , lat(0)
    , height(0)
    , hMSL(0)
    , gSpeed(0)
    , speed(0)
    , headMot(0)
    , headVeh(0)
    , hAcc(0)
    , vAcc(0)
    , sAcc(0)
    , headAcc(0)
    , reserved1()  {
      reserved0.assign(0);

      reserved1.assign(0);
  }
  HnrPVT_(const ContainerAllocator& _alloc)
    : iTOW(0)
    , year(0)
    , month(0)
    , day(0)
    , hour(0)
    , min(0)
    , sec(0)
    , valid(0)
    , nano(0)
    , gpsFix(0)
    , flags(0)
    , reserved0()
    , lon(0)
    , lat(0)
    , height(0)
    , hMSL(0)
    , gSpeed(0)
    , speed(0)
    , headMot(0)
    , headVeh(0)
    , hAcc(0)
    , vAcc(0)
    , sAcc(0)
    , headAcc(0)
    , reserved1()  {
  (void)_alloc;
      reserved0.assign(0);

      reserved1.assign(0);
  }



   typedef uint32_t _iTOW_type;
  _iTOW_type iTOW;

   typedef uint16_t _year_type;
  _year_type year;

   typedef uint8_t _month_type;
  _month_type month;

   typedef uint8_t _day_type;
  _day_type day;

   typedef uint8_t _hour_type;
  _hour_type hour;

   typedef uint8_t _min_type;
  _min_type min;

   typedef uint8_t _sec_type;
  _sec_type sec;

   typedef uint8_t _valid_type;
  _valid_type valid;

   typedef int32_t _nano_type;
  _nano_type nano;

   typedef uint8_t _gpsFix_type;
  _gpsFix_type gpsFix;

   typedef uint8_t _flags_type;
  _flags_type flags;

   typedef boost::array<uint8_t, 2>  _reserved0_type;
  _reserved0_type reserved0;

   typedef int32_t _lon_type;
  _lon_type lon;

   typedef int32_t _lat_type;
  _lat_type lat;

   typedef int32_t _height_type;
  _height_type height;

   typedef int32_t _hMSL_type;
  _hMSL_type hMSL;

   typedef int32_t _gSpeed_type;
  _gSpeed_type gSpeed;

   typedef int32_t _speed_type;
  _speed_type speed;

   typedef int32_t _headMot_type;
  _headMot_type headMot;

   typedef int32_t _headVeh_type;
  _headVeh_type headVeh;

   typedef uint32_t _hAcc_type;
  _hAcc_type hAcc;

   typedef uint32_t _vAcc_type;
  _vAcc_type vAcc;

   typedef uint32_t _sAcc_type;
  _sAcc_type sAcc;

   typedef uint32_t _headAcc_type;
  _headAcc_type headAcc;

   typedef boost::array<uint8_t, 4>  _reserved1_type;
  _reserved1_type reserved1;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(CLASS_ID)
  #undef CLASS_ID
#endif
#if defined(_WIN32) && defined(MESSAGE_ID)
  #undef MESSAGE_ID
#endif
#if defined(_WIN32) && defined(VALID_DATE)
  #undef VALID_DATE
#endif
#if defined(_WIN32) && defined(VALID_TIME)
  #undef VALID_TIME
#endif
#if defined(_WIN32) && defined(VALID_FULLY_RESOLVED)
  #undef VALID_FULLY_RESOLVED
#endif
#if defined(_WIN32) && defined(VALID_MAG)
  #undef VALID_MAG
#endif
#if defined(_WIN32) && defined(FIX_TYPE_NO_FIX)
  #undef FIX_TYPE_NO_FIX
#endif
#if defined(_WIN32) && defined(FIX_TYPE_DEAD_RECKONING_ONLY)
  #undef FIX_TYPE_DEAD_RECKONING_ONLY
#endif
#if defined(_WIN32) && defined(FIX_TYPE_2D)
  #undef FIX_TYPE_2D
#endif
#if defined(_WIN32) && defined(FIX_TYPE_3D)
  #undef FIX_TYPE_3D
#endif
#if defined(_WIN32) && defined(FIX_TYPE_GPS_DEAD_RECKONING_COMBINED)
  #undef FIX_TYPE_GPS_DEAD_RECKONING_COMBINED
#endif
#if defined(_WIN32) && defined(FIX_TYPE_TIME_ONLY)
  #undef FIX_TYPE_TIME_ONLY
#endif
#if defined(_WIN32) && defined(FLAGS_GNSS_FIX_OK)
  #undef FLAGS_GNSS_FIX_OK
#endif
#if defined(_WIN32) && defined(FLAGS_DIFF_SOLN)
  #undef FLAGS_DIFF_SOLN
#endif
#if defined(_WIN32) && defined(FLAGS_WKN_SET)
  #undef FLAGS_WKN_SET
#endif
#if defined(_WIN32) && defined(FLAGS_TOW_SET)
  #undef FLAGS_TOW_SET
#endif
#if defined(_WIN32) && defined(FLAGS_HEAD_VEH_VALID)
  #undef FLAGS_HEAD_VEH_VALID
#endif

  enum {
    CLASS_ID = 40u,
    MESSAGE_ID = 0u,
    VALID_DATE = 1u,
    VALID_TIME = 2u,
    VALID_FULLY_RESOLVED = 4u,
    VALID_MAG = 8u,
    FIX_TYPE_NO_FIX = 0u,
    FIX_TYPE_DEAD_RECKONING_ONLY = 1u,
    FIX_TYPE_2D = 2u,
    FIX_TYPE_3D = 3u,
    FIX_TYPE_GPS_DEAD_RECKONING_COMBINED = 4u,
    FIX_TYPE_TIME_ONLY = 5u,
    FLAGS_GNSS_FIX_OK = 1u,
    FLAGS_DIFF_SOLN = 2u,
    FLAGS_WKN_SET = 4u,
    FLAGS_TOW_SET = 8u,
    FLAGS_HEAD_VEH_VALID = 32u,
  };


  typedef boost::shared_ptr< ::ublox_msgs::HnrPVT_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::HnrPVT_<ContainerAllocator> const> ConstPtr;

}; // struct HnrPVT_

typedef ::ublox_msgs::HnrPVT_<std::allocator<void> > HnrPVT;

typedef boost::shared_ptr< ::ublox_msgs::HnrPVT > HnrPVTPtr;
typedef boost::shared_ptr< ::ublox_msgs::HnrPVT const> HnrPVTConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::HnrPVT_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::HnrPVT_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ublox_msgs::HnrPVT_<ContainerAllocator1> & lhs, const ::ublox_msgs::HnrPVT_<ContainerAllocator2> & rhs)
{
  return lhs.iTOW == rhs.iTOW &&
    lhs.year == rhs.year &&
    lhs.month == rhs.month &&
    lhs.day == rhs.day &&
    lhs.hour == rhs.hour &&
    lhs.min == rhs.min &&
    lhs.sec == rhs.sec &&
    lhs.valid == rhs.valid &&
    lhs.nano == rhs.nano &&
    lhs.gpsFix == rhs.gpsFix &&
    lhs.flags == rhs.flags &&
    lhs.reserved0 == rhs.reserved0 &&
    lhs.lon == rhs.lon &&
    lhs.lat == rhs.lat &&
    lhs.height == rhs.height &&
    lhs.hMSL == rhs.hMSL &&
    lhs.gSpeed == rhs.gSpeed &&
    lhs.speed == rhs.speed &&
    lhs.headMot == rhs.headMot &&
    lhs.headVeh == rhs.headVeh &&
    lhs.hAcc == rhs.hAcc &&
    lhs.vAcc == rhs.vAcc &&
    lhs.sAcc == rhs.sAcc &&
    lhs.headAcc == rhs.headAcc &&
    lhs.reserved1 == rhs.reserved1;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ublox_msgs::HnrPVT_<ContainerAllocator1> & lhs, const ::ublox_msgs::HnrPVT_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ublox_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::HnrPVT_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::HnrPVT_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::HnrPVT_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::HnrPVT_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::HnrPVT_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::HnrPVT_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::HnrPVT_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1803c15f4ff593453ea993864baf0f33";
  }

  static const char* value(const ::ublox_msgs::HnrPVT_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1803c15f4ff59345ULL;
  static const uint64_t static_value2 = 0x3ea993864baf0f33ULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::HnrPVT_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/HnrPVT";
  }

  static const char* value(const ::ublox_msgs::HnrPVT_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::HnrPVT_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# HNR-PVT (0x28 0x00)\n"
"# High Rate Output of PVT Solution\n"
"#\n"
"# Note that during a leap second there may be more (or less) than 60 seconds in\n"
"# a minute; see the description of leap seconds for details.\n"
"#\n"
"# This message provides the position, velocity and time solution with high \n"
"# output rate.\n"
"#\n"
"# Supported on ADR and UDR products.\n"
"#\n"
"uint8 CLASS_ID = 40\n"
"uint8 MESSAGE_ID = 0\n"
"\n"
"uint32 iTOW             # GPS Millisecond time of week [ms]\n"
"uint16 year             # Year (UTC)\n"
"uint8 month             # Month, range 1..12 (UTC)\n"
"uint8 day               # Day of month, range 1..31 (UTC)\n"
"uint8 hour              # Hour of day, range 0..23 (UTC)\n"
"uint8 min               # Minute of hour, range 0..59 (UTC)\n"
"uint8 sec               # Seconds of minute, range 0..60 (UTC)\n"
"\n"
"uint8 valid             # Validity flags\n"
"uint8 VALID_DATE = 1            # Valid UTC Date\n"
"uint8 VALID_TIME = 2            # Valid \n"
"uint8 VALID_FULLY_RESOLVED = 4  # UTC time of day has been fully resolved \n"
"                                # (no seconds uncertainty)\n"
"uint8 VALID_MAG = 8             # Valid Magnetic Declination\n"
"\n"
"int32 nano              # fraction of a second [ns], range -1e9 .. 1e9 (UTC)\n"
"\n"
"uint8 gpsFix            # GPS fix Type, range 0..5\n"
"uint8 FIX_TYPE_NO_FIX = 0\n"
"uint8 FIX_TYPE_DEAD_RECKONING_ONLY = 1\n"
"uint8 FIX_TYPE_2D = 2                           # Signal from only 3 SVs, \n"
"                                                # constant altitude assumed\n"
"uint8 FIX_TYPE_3D = 3\n"
"uint8 FIX_TYPE_GPS_DEAD_RECKONING_COMBINED = 4  # GPS + Dead reckoning\n"
"uint8 FIX_TYPE_TIME_ONLY = 5                    # Time only fix \n"
"\n"
"uint8 flags             # Fix Status Flags\n"
"uint8 FLAGS_GNSS_FIX_OK = 1          # i.e. within DOP & accuracy masks\n"
"uint8 FLAGS_DIFF_SOLN = 2            # DGPS used\n"
"uint8 FLAGS_WKN_SET = 4              # Valid GPS week number\n"
"uint8 FLAGS_TOW_SET = 8              # Valid GPS time of week (iTOW & fTOW)\n"
"uint8 FLAGS_HEAD_VEH_VALID = 32      # heading of vehicle is valid\n"
"\n"
"uint8[2] reserved0      # Reserved\n"
"\n"
"int32 lon               # Longitude [deg / 1e-7]\n"
"int32 lat               # Latitude [deg / 1e-7]\n"
"int32 height            # Height above Ellipsoid [mm]\n"
"int32 hMSL              # Height above mean sea level [mm]\n"
"\n"
"int32 gSpeed            # Ground Speed (2-D) [mm/s]\n"
"int32 speed             # Speed (3-D) [mm/s]\n"
"\n"
"int32 headMot           # Heading of motion (2-D) [deg / 1e-5]\n"
"int32 headVeh           # Heading of vehicle (2-D) [deg / 1e-5]\n"
"\n"
"uint32 hAcc             # Horizontal Accuracy Estimate [mm]\n"
"uint32 vAcc             # Vertical Accuracy Estimate [mm]\n"
"uint32 sAcc             # Speed Accuracy Estimate [mm/s]\n"
"uint32 headAcc          # Heading Accuracy Estimate (both motion & vehicle) \n"
"                        # [deg / 1e-5]\n"
"\n"
"uint8[4] reserved1      # Reserved\n"
;
  }

  static const char* value(const ::ublox_msgs::HnrPVT_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::HnrPVT_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.iTOW);
      stream.next(m.year);
      stream.next(m.month);
      stream.next(m.day);
      stream.next(m.hour);
      stream.next(m.min);
      stream.next(m.sec);
      stream.next(m.valid);
      stream.next(m.nano);
      stream.next(m.gpsFix);
      stream.next(m.flags);
      stream.next(m.reserved0);
      stream.next(m.lon);
      stream.next(m.lat);
      stream.next(m.height);
      stream.next(m.hMSL);
      stream.next(m.gSpeed);
      stream.next(m.speed);
      stream.next(m.headMot);
      stream.next(m.headVeh);
      stream.next(m.hAcc);
      stream.next(m.vAcc);
      stream.next(m.sAcc);
      stream.next(m.headAcc);
      stream.next(m.reserved1);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HnrPVT_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::HnrPVT_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::HnrPVT_<ContainerAllocator>& v)
  {
    s << indent << "iTOW: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.iTOW);
    s << indent << "year: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.year);
    s << indent << "month: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.month);
    s << indent << "day: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.day);
    s << indent << "hour: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.hour);
    s << indent << "min: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.min);
    s << indent << "sec: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.sec);
    s << indent << "valid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.valid);
    s << indent << "nano: ";
    Printer<int32_t>::stream(s, indent + "  ", v.nano);
    s << indent << "gpsFix: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gpsFix);
    s << indent << "flags: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.flags);
    s << indent << "reserved0[]" << std::endl;
    for (size_t i = 0; i < v.reserved0.size(); ++i)
    {
      s << indent << "  reserved0[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.reserved0[i]);
    }
    s << indent << "lon: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lon);
    s << indent << "lat: ";
    Printer<int32_t>::stream(s, indent + "  ", v.lat);
    s << indent << "height: ";
    Printer<int32_t>::stream(s, indent + "  ", v.height);
    s << indent << "hMSL: ";
    Printer<int32_t>::stream(s, indent + "  ", v.hMSL);
    s << indent << "gSpeed: ";
    Printer<int32_t>::stream(s, indent + "  ", v.gSpeed);
    s << indent << "speed: ";
    Printer<int32_t>::stream(s, indent + "  ", v.speed);
    s << indent << "headMot: ";
    Printer<int32_t>::stream(s, indent + "  ", v.headMot);
    s << indent << "headVeh: ";
    Printer<int32_t>::stream(s, indent + "  ", v.headVeh);
    s << indent << "hAcc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.hAcc);
    s << indent << "vAcc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.vAcc);
    s << indent << "sAcc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.sAcc);
    s << indent << "headAcc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.headAcc);
    s << indent << "reserved1[]" << std::endl;
    for (size_t i = 0; i < v.reserved1.size(); ++i)
    {
      s << indent << "  reserved1[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.reserved1[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_HNRPVT_H

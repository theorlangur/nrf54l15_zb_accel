#ifndef ZB_TYPES_HPP_
#define ZB_TYPES_HPP_

extern "C" {
#include <zboss_api.h>
}
#include <type_traits>

namespace zb
{
    enum class Access: zb_uint8_t
    {
        None = 0,
        Read = 0x01,
        Write = 0x02,
        Report = 0x04,
        Singleton = 0x08,
        Scene = 0x10,
        ManuSpec = 0x20,
        Internal = 0x40,

        RW = Read | Write,
        RWP = RW | Report,
        RP = Read | Report,
        RPS = RP | Report,
    };
    constexpr Access operator|(Access a1, Access a2) { return Access(zb_uint8_t(a1) | zb_uint8_t(a2)); }
    constexpr bool operator&(Access a1, Access a2) { return (zb_uint8_t(a1) & zb_uint8_t(a2)) != 0; }

    enum class Type: zb_uint8_t
    {
        Null          = ZB_ZCL_ATTR_TYPE_NULL,
        Raw8          = ZB_ZCL_ATTR_TYPE_8BIT,
        Raw16         = ZB_ZCL_ATTR_TYPE_16BIT,
        Raw24         = ZB_ZCL_ATTR_TYPE_24BIT,
        Raw32         = ZB_ZCL_ATTR_TYPE_32BIT,
        Raw40         = ZB_ZCL_ATTR_TYPE_40BIT,
        Raw48         = ZB_ZCL_ATTR_TYPE_48BIT,
        Raw56         = ZB_ZCL_ATTR_TYPE_56BIT,
        Raw64         = ZB_ZCL_ATTR_TYPE_64BIT,
        Bool          = ZB_ZCL_ATTR_TYPE_BOOL,
        Map8          = ZB_ZCL_ATTR_TYPE_8BITMAP,
        Map16         = ZB_ZCL_ATTR_TYPE_16BITMAP,
        Map24         = ZB_ZCL_ATTR_TYPE_24BITMAP,
        Map32         = ZB_ZCL_ATTR_TYPE_32BITMAP,
        Map40         = ZB_ZCL_ATTR_TYPE_40BITMAP,
        Map48         = ZB_ZCL_ATTR_TYPE_48BITMAP,
        Map56         = ZB_ZCL_ATTR_TYPE_56BITMAP,
        Map64         = ZB_ZCL_ATTR_TYPE_64BITMAP,
        U8            = ZB_ZCL_ATTR_TYPE_U8,
        U16           = ZB_ZCL_ATTR_TYPE_U16,
        U24           = ZB_ZCL_ATTR_TYPE_U24,
        U32           = ZB_ZCL_ATTR_TYPE_U32,
        U40           = ZB_ZCL_ATTR_TYPE_U40,
        U48           = ZB_ZCL_ATTR_TYPE_U48,
        U56           = ZB_ZCL_ATTR_TYPE_U56,
        U64           = ZB_ZCL_ATTR_TYPE_U64,
        S8            = ZB_ZCL_ATTR_TYPE_S8,
        S16           = ZB_ZCL_ATTR_TYPE_S16,
        S24           = ZB_ZCL_ATTR_TYPE_S24,
        S32           = ZB_ZCL_ATTR_TYPE_S32,
        S40           = ZB_ZCL_ATTR_TYPE_S40,
        S48           = ZB_ZCL_ATTR_TYPE_S48,
        S56           = ZB_ZCL_ATTR_TYPE_S56,
        S64           = ZB_ZCL_ATTR_TYPE_S64,
        E8            = ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
        E16           = ZB_ZCL_ATTR_TYPE_16BIT_ENUM,
        HalfFloat     = ZB_ZCL_ATTR_TYPE_SEMI,
        Float         = ZB_ZCL_ATTR_TYPE_SINGLE,
        Double        = ZB_ZCL_ATTR_TYPE_DOUBLE,
        OctetStr      = ZB_ZCL_ATTR_TYPE_OCTET_STRING,
        CharStr       = ZB_ZCL_ATTR_TYPE_CHAR_STRING,
        LongOctetStr  = ZB_ZCL_ATTR_TYPE_LONG_OCTET_STRING,
        LongCharStr   = ZB_ZCL_ATTR_TYPE_LONG_CHAR_STRING,
        Array         = ZB_ZCL_ATTR_TYPE_ARRAY,
        Struct        = ZB_ZCL_ATTR_TYPE_STRUCTURE,
        Set           = ZB_ZCL_ATTR_TYPE_SET,
        Bag           = ZB_ZCL_ATTR_TYPE_BAG,
        TimeOfDay     = ZB_ZCL_ATTR_TYPE_TIME_OF_DAY,
        Date          = ZB_ZCL_ATTR_TYPE_DATE,
        UTCTime       = ZB_ZCL_ATTR_TYPE_UTC_TIME,
        ClusterID     = ZB_ZCL_ATTR_TYPE_CLUSTER_ID,
        AttributeID   = ZB_ZCL_ATTR_TYPE_ATTRIBUTE_ID,
        BACnetOID     = ZB_ZCL_ATTR_TYPE_BACNET_OID,
        IEEEAddr      = ZB_ZCL_ATTR_TYPE_IEEE_ADDR,
        Sec128Key     = ZB_ZCL_ATTR_TYPE_128_BIT_KEY,
        Custom32Array = ZB_ZCL_ATTR_TYPE_CUSTOM_32ARRAY,
        Invalid       = ZB_ZCL_ATTR_TYPE_INVALID
    };

    enum class Role: zb_uint8_t
    {
        Invalid = 0x00,
        Server = 0x01,
        Client = 0x02,
        Any = Server | Client
    };

    template<class T>
    constexpr Type TypeToTypeId()
    {
        if constexpr (std::is_same_v<T,zb_uint8_t>)                return Type::U8;
        else if constexpr (std::is_same_v<T,zb_uint16_t>)          return Type::U16;
        else if constexpr (std::is_same_v<T,zb_uint32_t>)          return Type::U32;
        else if constexpr (std::is_same_v<T,zb_uint64_t>)          return Type::U64;
        else if constexpr (std::is_same_v<T,zb_int8_t>)            return Type::S8;
        else if constexpr (std::is_same_v<T,zb_int16_t>)           return Type::S16;
        else if constexpr (std::is_same_v<T,zb_int32_t>)           return Type::S32;
        else if constexpr (std::is_same_v<T,zb_int64_t>)           return Type::S64;
        else if constexpr (std::is_same_v<T,std::nullptr_t>)       return Type::Null;
        else if constexpr (std::is_enum_v<T> && sizeof(T) == 1)    return Type::E8;
        else if constexpr (std::is_enum_v<T> && sizeof(T) == 2)    return Type::E16;
        else if constexpr (std::is_same_v<T,float>)                return Type::Float;
        else if constexpr (std::is_same_v<T,double>)               return Type::Double;
        else if constexpr (std::is_same_v<T,bool>)                 return Type::Bool;
        else if constexpr (requires { T::TypeId(); })              return T::TypeId();
        else 
            static_assert(sizeof(T) == 0, "Unknown type");
        return Type::Invalid;
    }
}

#endif

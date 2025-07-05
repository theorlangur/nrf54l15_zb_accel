#ifndef ZB_DESC_HELPER_TYPES_EP_HPP_
#define ZB_DESC_HELPER_TYPES_EP_HPP_

#include "zb_desc_helper_types_cluster.hpp"

namespace zb
{
    template<size_t ServerCount, size_t ClientCount>
    struct ZB_PACKED_PRE TSimpleDesc: zb_af_simple_desc_1_1_t
    {
        zb_uint16_t app_cluster_list_ext[(ServerCount + ClientCount) - 2];
    } ZB_PACKED_STRUCT;
    
    template<cluster_info_t ci, auto mem_desc, bool withCheck> requires requires { typename decltype(mem_desc)::MemT; }
    struct AttributeAccess
    {
        using MemT = decltype(mem_desc)::MemT;

        auto operator=(MemT const& v)
        {
            return zb_zcl_set_attr_val(ep.ep_id, ci.id, (zb_uint8_t)ci.role, mem_desc.id, (zb_uint8_t*)&v, withCheck);
        }

        zb_af_endpoint_desc_t &ep;
    };

    struct EPClusterAttributeDesc_t
    {
        static constexpr uint16_t kANY_CLUSTER = 0xffff;
        static constexpr uint16_t kANY_ATTRIBUTE = 0xffff;
        static constexpr uint8_t  kANY_EP = 0xff;

        zb_uint8_t ep = kANY_EP;
        uint16_t cluster = kANY_CLUSTER;
        uint16_t attribute = kANY_ATTRIBUTE;

        constexpr bool fits(zb_uint8_t _ep, uint16_t _cluster, uint16_t _attr) const
        {
            return ((_ep == ep) || (ep == kANY_EP))
                && ((_cluster == cluster) || (cluster == kANY_CLUSTER))
                && ((_attr == attribute) || (attribute == kANY_ATTRIBUTE));
        }
    };

    struct EPBaseInfo
    {
        zb_uint8_t ep;
        zb_uint16_t dev_id;
        zb_uint8_t dev_ver;
    };

    template<EPBaseInfo i, class Clusters>
    struct EPDesc
    {
        using SimpleDesc = TSimpleDesc<Clusters::server_cluster_count(), Clusters::client_cluster_count()>;

        template<class T1, class T2, class... T> requires std::is_same_v<TClusterList<T1, T2, T...>, Clusters>
        constexpr EPDesc(TClusterList<T1, T2, T...> &clusters):
            simple_desc{ 
                {
                    .endpoint = i.ep, 
                    .app_profile_id = ZB_AF_HA_PROFILE_ID, 
                    .app_device_id = i.dev_id,
                    .app_device_version = i.dev_ver,
                    .reserved = 0,
                    .app_input_cluster_count = Clusters::server_cluster_count(),
                    .app_output_cluster_count = Clusters::client_cluster_count(),
                    .app_cluster_list = {T1::info().id, T2::info().id}
                },
                { T::info().id... }//rest
            },
            rep_ctx{},
            cvc_alarm_ctx{},
            ep{
                .ep_id = i.ep,
                .profile_id = ZB_AF_HA_PROFILE_ID,
                .device_handler = nullptr,
                .identify_handler = nullptr,
                .reserved_size = 0,
                .reserved_ptr = nullptr,
                .cluster_count = sizeof...(T) + 2,
                .cluster_desc_list = clusters.clusters,
                .simple_desc = &simple_desc,
                .rep_info_count = Clusters::reporting_attributes_count(),
                .reporting_info = rep_ctx,
                .cvc_alarm_count = Clusters::cvc_attributes_count(),
                .cvc_alarm_info = cvc_alarm_ctx
            }
        {
        }

    private:
        template<class _MemType, class _StructType, class _ClusterType>
        struct ClusterTypeInfo
        {
            using MemType = _MemType;
            using StructType = _StructType;
            using ClusterType = _ClusterType;
        };
        template<auto memPtr>
        static consteval auto validate_mem_ptr()
        {
            using MemPtrType = decltype(memPtr);
            static_assert(mem_ptr_traits<MemPtrType>::is_mem_ptr, "Only member pointer is allowed");

            using ClassType = mem_ptr_traits<MemPtrType>::ClassType;
            //using MemType = mem_ptr_traits<MemPtrType>::MemberType;
            using ClusterDescType = decltype(get_cluster_description<ClassType>());
            static_assert(Clusters::has_info(ClusterDescType::info()), "Requested cluster is not part of the EP");
            return ClusterTypeInfo<MemPtrType, ClassType, ClusterDescType>{};
        }

        template<auto memPtr, bool checked>
        auto attr_raw()
        {
            constexpr auto types = validate_mem_ptr<memPtr>();
            using ClusterDescType = decltype(types)::ClusterType;
            return AttributeAccess<ClusterDescType::info(), ClusterDescType::template get_member_description<memPtr>(), checked>{ep};
        }

    public:
        template<auto memPtr>
        auto attr() { return attr_raw<memPtr, false>(); }

        template<auto memPtr>
        auto attr_checked() { return attr_raw<memPtr, true>(); }

        //TODO: callback support!
        template<auto memPtr, class... Args>
        auto send_cmd(Args&&...args)
        {
            constexpr auto types = validate_mem_ptr<memPtr>();
            using ClusterDescType = decltype(types)::ClusterType;
            ClusterDescType::template get_cmd_description<memPtr>().template request<ClusterDescType::info(), {.ep = i.ep}>(std::forward<Args>(args)...);
        }

        template<auto memPtr, class... Args>
        auto send_cmd_to(uint16_t short_addr, uint8_t ep, Args&&...args)
        {
            constexpr auto types = validate_mem_ptr<memPtr>();
            using ClusterDescType = decltype(types)::ClusterType;
            ClusterDescType::template get_cmd_description<memPtr>().template request<ClusterDescType::info(), {.ep = i.ep}>(short_addr, ep, std::forward<Args>(args)...);
        }

        template<auto memPtr, class... Args>
        auto send_cmd_to(zb_ieee_addr_t long_addr, uint8_t ep, Args&&...args)
        {
            constexpr auto types = validate_mem_ptr<memPtr>();
            using ClusterDescType = decltype(types)::ClusterType;
            ClusterDescType::template get_cmd_description<memPtr>().template request<ClusterDescType::info(), {.ep = i.ep}>(long_addr, ep, std::forward<Args>(args)...);
        }

        template<auto memPtr, class... Args>
        auto send_cmd_to_group(uint16_t group, Args&&...args)
        {
            constexpr auto types = validate_mem_ptr<memPtr>();
            using ClusterDescType = decltype(types)::ClusterType;
            ClusterDescType::template get_cmd_description<memPtr>().template request<ClusterDescType::info(), {.ep = i.ep}>(group, std::forward<Args>(args)...);
        }

        template<auto memPtr, class... Args>
        auto send_cmd_to_binded(uint8_t bind_table_id, Args&&...args)
        {
            constexpr auto types = validate_mem_ptr<memPtr>();
            using ClusterDescType = decltype(types)::ClusterType;
            ClusterDescType::template get_cmd_description<memPtr>().template request<ClusterDescType::info(), {.ep = i.ep}>(bind_table_id, std::forward<Args>(args)...);
        }

        template<auto memPtr>
        constexpr EPClusterAttributeDesc_t handler_filter_for_attribute()
        {
            using MemPtrType = decltype(memPtr);
            static_assert(mem_ptr_traits<MemPtrType>::is_mem_ptr, "Only member pointer is allowed");

            using ClassType = mem_ptr_traits<MemPtrType>::ClassType;
            //using MemType = mem_ptr_traits<MemPtrType>::MemberType;
            using ClusterDescType = decltype(get_cluster_description<ClassType>());
            static_assert(Clusters::has_info(ClusterDescType::info()), "Requested cluster is not part of the EP");
            return {.ep = i.ep, .cluster = ClusterDescType::info().id, .attribute = ClusterDescType::template get_member_description<memPtr>().id};
        }

        template<class Cluster>
        constexpr EPClusterAttributeDesc_t handler_filter_for_cluster()
        {
            using ClusterDescType = decltype(get_cluster_description<Cluster>());
            static_assert(Clusters::has_info(ClusterDescType::info()), "Requested cluster is not part of the EP");
            return {.ep = i.ep, .cluster = ClusterDescType::info().id};
        }

        alignas(4) SimpleDesc simple_desc;
        alignas(4) zb_zcl_reporting_info_t rep_ctx[Clusters::reporting_attributes_count()];
        alignas(4) zb_zcl_cvc_alarm_variables_t cvc_alarm_ctx[Clusters::cvc_attributes_count()];
        alignas(4) zb_af_endpoint_desc_t ep;
    };
}
#endif

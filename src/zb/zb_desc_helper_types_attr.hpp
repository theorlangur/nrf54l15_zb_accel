#ifndef ZB_DESC_HELPER_TYPES_ATTR_HPP_
#define ZB_DESC_HELPER_TYPES_ATTR_HPP_

#include "zb_desc_helper_types.hpp"

namespace zb
{
    inline constexpr zb_zcl_attr_t g_LastAttribute{
        .id = ZB_ZCL_NULL_ID,
        .type = ZB_ZCL_ATTR_TYPE_NULL,
        .access = (zb_uint8_t)Access::None,
        .manuf_code = ZB_ZCL_NON_MANUFACTURER_SPECIFIC,
        .data_p = nullptr
    };

    template<class T>
    struct ADesc
    {
        using simplified_tag = void;
        zb_uint16_t id;
        Access a;
        T *pData;
        Type type = TypeToTypeId<T>();
    };

    template<class T>
    constexpr zb_zcl_attr_t AttrDesc(ADesc<T> d)
    {
        return {
            .id = d.id, 
            .type = (zb_uint8_t)d.type, 
            .access = (zb_uint8_t)d.a, 
            .manuf_code = ZB_ZCL_NON_MANUFACTURER_SPECIFIC, 
            .data_p = d.pData
        };
    }

    template<class StructTag, size_t N>
    struct TAttributeList
    {
        using Tag = decltype(get_cluster_description<StructTag>());

        TAttributeList(TAttributeList const&) = delete;
        TAttributeList(TAttributeList &&) = delete;
        void operator=(TAttributeList const&) = delete;
        void operator=(TAttributeList &&) = delete;


        template<class... T>
        constexpr TAttributeList(ADesc<T>... d):
            attributes{
                AttrDesc(zb::ADesc{ .id = ZB_ZCL_ATTR_GLOBAL_CLUSTER_REVISION_ID, .a = Access::Read, .pData = &rev }),
                AttrDesc(d)...
                , g_LastAttribute
            },
            rev(Tag::rev())
        {
        }

        operator zb_zcl_attr_t*() { return attributes; }

        constexpr static bool is_role(Role r) { return Tag::info().role == r; }
        constexpr static size_t attributes_with_access(Access r) { return Tag::count_members_with_access(r); }
        constexpr static size_t cvc_attributes() { return Tag::count_cvc_members(); }
        constexpr static auto info() { return Tag::info(); }

        constexpr zb_zcl_cluster_desc_t desc()
        {
            constexpr auto ci = Tag::info();
            return zb_zcl_cluster_desc_t{
                .cluster_id = ci.id,
                    .attr_count = N + 2,
                    .attr_desc_list = attributes,
                    .role_mask = (zb_uint8_t)ci.role,
                    .manuf_code = ci.manuf_code,
                    .cluster_init = get_cluster_init<Tag::info().id>(ci.role)
            };
        }

        alignas(4) zb_zcl_attr_t attributes[N + 2];
        zb_uint16_t rev;
    };

    template<class ClusterTag, class... T>
    constexpr auto MakeAttributeList(ClusterTag *t, ADesc<T>... d)->TAttributeList<ClusterTag, sizeof...(T)>
    {
        return TAttributeList<ClusterTag, sizeof...(T)>(d...);
    }
}
#endif

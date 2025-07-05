#ifndef ZB_DESC_HELPER_TYPES_SELF_CONTAINED_HPP_
#define ZB_DESC_HELPER_TYPES_SELF_CONTAINED_HPP_

#include "zb_desc_helper_types_device.hpp"

namespace zb
{
    template<class T>
    using ToAttributeListType = decltype(zb::cluster_struct_to_attr_list(std::declval<T&>(), zb::get_cluster_description<T>()));

    template<class T> struct mem_tag_t{};

    template<class T>
    struct AttributeListContainerMem
    {
        using AttrListType = ToAttributeListType<T>;
        AttrListType m;

        constexpr AttrListType& get(mem_tag_t<T>) { return m; }
    };

    template<class... Bases>
    struct AttributeListContainer: AttributeListContainerMem<Bases>... { using AttributeListContainerMem<Bases>::get...; };

    template<class T>
    struct EPArgsListContainerMem
    {
        T &m;
        constexpr T& get(mem_tag_t<T>) { return m; }
    };

    template<EPBaseInfo i, class... Bases>
    struct ep_args_t: EPArgsListContainerMem<Bases>... 
    {
        using EPArgsListContainerMem<Bases>::get...; 
    };

    template<EPBaseInfo i, class... Bases>
    constexpr auto make_ep_args(Bases&...b) { return ep_args_t<i, Bases...>{b...}; }

    template<EPBaseInfo i, ZigbeeClusterStruct... ClusterTypes>
    struct EPDescSelfContained
    {
        using ClusterListType = zb::TClusterList<ToAttributeListType<ClusterTypes>...>;

        static constexpr zb_uint8_t ep_id() { return i.ep; }

        constexpr EPDescSelfContained(ClusterTypes&...s):
            attributes{ zb::cluster_struct_to_attr_list(s, zb::get_cluster_description<ClusterTypes>())... },
            clusters{attributes.get(mem_tag_t<ClusterTypes>{})...},
            ep{clusters}
        {
        }

        constexpr EPDescSelfContained(ep_args_t<i, ClusterTypes...> arg):
            attributes{ zb::cluster_struct_to_attr_list(arg.get(mem_tag_t<ClusterTypes>{}), zb::get_cluster_description<ClusterTypes>())... },
            clusters{attributes.get(mem_tag_t<ClusterTypes>{})...},
            ep{clusters}
        {
        }

        AttributeListContainer<ClusterTypes...> attributes;
        ClusterListType clusters;
        zb::EPDesc<i, ClusterListType> ep;
    };
    //template<class... Clusters>
    //EPDescSelfContained(ep_args_t<Clusters...>) -> EPDescSelfContained<Clusters...>;

    template<class T>
    concept IsEPDescSelfContained = requires(T t) {
        typename T::ClusterListType;
        t.ep;
        t.clusters;
        t.attributes;
    };

    template<zb_uint8_t ep>
    struct ep_tag_t{};

    template<class EP>
    struct EPContainerMem
    {
        EP m;

        constexpr EP& get(mem_tag_t<EP>) { return m; }
        constexpr EP& get(ep_tag_t<EP::ep_id()>) { return m; }
    };

    template<class... EPs>
    struct EPListContainer: EPContainerMem<EPs>... 
    { 
        using EPContainerMem<EPs>::get...; 

        template<zb_uint8_t dummy>
        constexpr auto get(ep_tag_t<dummy>) { static_assert(sizeof(ep_tag_t<dummy>) == 0, "EP not found"); }
    };

    template<class T>
    struct EPDescTypeFromArg;

    template<EPBaseInfo i, class... Clusters>
    struct EPDescTypeFromArg<ep_args_t<i, Clusters...>>
    {
        using type = EPDescSelfContained<i, Clusters...>;
    };

    template<class T>
    using EPDescTypeFromArgT = EPDescTypeFromArg<T>::type;

    template<class... EPSelfContainedTypes>
    struct DeviceFull
    {
        static constexpr size_t N = sizeof...(EPSelfContainedTypes);

        template<class... EPArgs>
        constexpr DeviceFull(EPArgs..._eps):
            eps{_eps...},
            endpoints{&eps.get(mem_tag_t<EPSelfContainedTypes>{}).ep.ep...},
            ctx{.ep_count = N, .ep_desc_list = endpoints}
        {
        }

        template<zb_uint8_t _ep>
        constexpr auto& ep() { return eps.get(ep_tag_t<_ep>{}).ep; }

        operator zb_af_device_ctx_t*() { return &ctx; }

        EPListContainer<EPSelfContainedTypes...> eps;
        zb_af_endpoint_desc_t *endpoints[N];
        zb_af_device_ctx_t ctx;
    };

    template<class... EPArgs>
    DeviceFull(EPArgs...) -> DeviceFull<EPDescTypeFromArgT<EPArgs>...>;
}
#endif

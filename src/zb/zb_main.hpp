#ifndef ZB_MAIN_HPP_
#define ZB_MAIN_HPP_

#include "zb_desc_helper_types_device.hpp"
#include "zb_desc_helper_types_self_contained.hpp"
#include "zb_signals.hpp"
#include "zb_handers.hpp"

#include <zigbee/zigbee_error_handler.h>

namespace zb
{
    //returns TAttributeList with properly initialized array of zb_zcl_attr_t 
    //based on the information obtained via get_cluster_description for the passed structure type of ZbS
    //returns TAttributeList<...>
    template<class ZbS> requires ZigbeeClusterStruct<ZbS>
    constexpr auto to_attributes(ZbS &s) { return cluster_struct_to_attr_list(s, get_cluster_description<ZbS>()); }

    //given multiples of TAttributeList<> objects this constructs and returns a TClusterList<..> 
    //with a properly initialized array of zb_zcl_cluster_desc_t
    template<class... ClusterAttributesDesc> requires (ZigbeeAttributeList<ClusterAttributesDesc> && ...)
    constexpr auto to_clusters(ClusterAttributesDesc&... c) -> TClusterList<ClusterAttributesDesc...> { return {c...}; }

    //given multiples clusters in form of TClusterList<> this constructs and returns a EPDesc<..> 
    //with a proper zb_af_simple_desc_1_1_t-derived simple description and a properyl initialized zb_af_endpoint_desc_t
    //also with the automatically calculated and reserved space for reporting attributes and level control alarms
    template<EPBaseInfo i, class... T>
    constexpr auto configure_ep(TClusterList<T...> &clusters) -> EPDesc<i, TClusterList<T...>>
    {
        return {clusters};
    }

    //packs existing Endpoints together into a Device, with properly initialized zb_af_device_ctx_t
    template<EPBaseInfo... i, class... Clusters>
    constexpr auto configure_device(EPDesc<i, Clusters>&...eps) { return Device{eps...}; }

    template<class... SelfContainedEP> requires (IsEPDescSelfContained<SelfContainedEP> && ...)
    constexpr auto configure_device(SelfContainedEP&...eps) { return Device{eps.ep...}; }

    template<zb::EPBaseInfo i, class... Structs> requires (ZigbeeClusterStruct<Structs> && ...)
    constexpr auto make_endpoint(Structs&... s) { return EPDescSelfContained<i, Structs...>{s...}; }

    //Args==ep_args_t<...> -> EPDescTypeFromArgT<ep_args_t<...>> -> EPDescSelfContained<...>
    template<class... Args>
    constexpr auto make_device(Args... a) { return DeviceFull{a...}; }
}

#endif

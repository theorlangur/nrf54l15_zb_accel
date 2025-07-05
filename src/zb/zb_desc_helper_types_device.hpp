#ifndef ZB_DESC_HELPER_TYPES_DEVICE_HPP_
#define ZB_DESC_HELPER_TYPES_DEVICE_HPP_

#include "zb_desc_helper_types_ep.hpp"

namespace zb
{
    template<size_t N>
    struct Device
    {
        template<EPBaseInfo... i, class... Clusters> requires (sizeof...(Clusters) == N)
        constexpr Device(EPDesc<i, Clusters>&...eps):
            endpoints{&eps.ep...},
            ctx{.ep_count = N, .ep_desc_list = endpoints}
        {
        }

        operator zb_af_device_ctx_t*() { return &ctx; }

        zb_af_endpoint_desc_t *endpoints[N];
        zb_af_device_ctx_t ctx;
    };

    template<EPBaseInfo... i, class... Clusters>
    Device(EPDesc<i, Clusters>&...eps) -> Device<sizeof...(Clusters)>;
}
#endif

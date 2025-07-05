#ifndef ZB_SIGNALS_HPP_
#define ZB_SIGNALS_HPP_

#include "zb_buf.hpp"
#include <concepts>

extern "C"{
#include <zigbee/zigbee_app_utils.h>
}
namespace zb
{
    struct sig_handlers_t
    {
        using simple_handler_t = void(*)();
        using generic_handler_t = void(*)(zb_ret_t status);
        using dev_annce_handler_t = void(*)(zb_ret_t status, zb_zdo_signal_device_annce_params_t *p);
        using leave_handler_t = void(*)(zb_ret_t status, zb_zdo_signal_leave_params_t *p);
        using dev_associated_handler_t = void(*)(zb_ret_t status, zb_nwk_signal_device_associated_params_t *p);
        using leave_indication_handler_t = void(*)(zb_ret_t status, zb_zdo_signal_leave_indication_params_t *p);
        //using zgp_commisioning_handler_t = void(*)(zb_ret_t status, zb_zgp_signal_commissioning_params_t *p);
        using can_sleep_handler_t = void(*)(zb_ret_t status, zb_zdo_signal_can_sleep_params_t *p);
        using dev_authorized_handler_t = void(*)(zb_ret_t status, zb_zdo_signal_device_authorized_params_t *p);
        using dev_update_handler_t = void(*)(zb_ret_t status, zb_zdo_signal_device_update_params_t *p);
        //using pan_id_conflict_handler_t = void(*)(zb_ret_t status, zb_start_pan_id_conflict_resolution_t *p);
        using nlme_status_indication_handler_t = void(*)(zb_ret_t status, zb_nwk_command_status_t *p);
        using nwk_permit_join_status_handler_t = void(*)(zb_ret_t status, zb_uint8_t *p);

        template<typename T>
        struct handler_mem_t
        {
            T m_TypedHandler = nullptr;
            simple_handler_t m_SimpleHandler = nullptr;

            constexpr handler_mem_t() = default;
            constexpr handler_mem_t(simple_handler_t h):m_SimpleHandler(h) {}
            template<class Lambda> requires (!std::convertible_to<Lambda, T>)
            constexpr handler_mem_t(Lambda h):m_SimpleHandler(h) {}
            constexpr handler_mem_t(T h):m_TypedHandler(h) {}
            template<class LambdaT> requires std::convertible_to<LambdaT, T>
            constexpr handler_mem_t(LambdaT h):m_TypedHandler(h) {}

            constexpr void operator=(simple_handler_t h) { m_SimpleHandler = h; }
            constexpr void operator=(T h) { m_TypedHandler = h; }
            constexpr operator bool() const { return  m_TypedHandler || m_SimpleHandler; }
        };

        template<auto handler, class... Args>
        constexpr void invoke(Args... args) const
        {
            if constexpr (handler.m_TypedHandler)
                handler.m_TypedHandler(args...);
            else if constexpr (handler.m_SimpleHandler)
                handler.m_SimpleHandler();
        }

        handler_mem_t<generic_handler_t> on_skip_startup;
        handler_mem_t<dev_annce_handler_t> on_dev_annce;
        handler_mem_t<leave_handler_t> on_leave;
        handler_mem_t<generic_handler_t> on_error;
        handler_mem_t<generic_handler_t> on_dev_first_start;
        handler_mem_t<generic_handler_t> on_dev_reboot;
        handler_mem_t<generic_handler_t> on_steering;
        handler_mem_t<generic_handler_t> on_formation;
        handler_mem_t<generic_handler_t> on_finding_and_binding_target_finished;
        handler_mem_t<generic_handler_t> on_finding_and_binding_initiator_finished;
        handler_mem_t<dev_associated_handler_t> on_dev_associated;
        handler_mem_t<leave_indication_handler_t> on_leave_indication;
        handler_mem_t<generic_handler_t> on_wwah_rejoined_started;
        handler_mem_t<can_sleep_handler_t> on_can_sleep;
        handler_mem_t<generic_handler_t> on_production_config_ready;
        handler_mem_t<generic_handler_t> on_nwk_no_active_links_left;
        handler_mem_t<generic_handler_t> on_sub_ghz_suspend;
        handler_mem_t<generic_handler_t> on_sub_ghz_resume;
        handler_mem_t<dev_authorized_handler_t> on_dev_authorized;
        handler_mem_t<dev_update_handler_t> on_dev_update;
        handler_mem_t<nlme_status_indication_handler_t> on_nlme_status_indication;
        handler_mem_t<generic_handler_t> on_tcswap_db_backup_required;
        handler_mem_t<generic_handler_t> on_tcswapped;
        handler_mem_t<generic_handler_t> on_tc_rejoin_done;
        handler_mem_t<nwk_permit_join_status_handler_t> on_nwk_permit_join;
        handler_mem_t<generic_handler_t> on_steering_cancelled;
        handler_mem_t<generic_handler_t> on_formation_cancelled;
        handler_mem_t<generic_handler_t> on_ready_to_shut;
    };

    template<sig_handlers_t h = {}>
    zb_ret_t tpl_signal_handler(zb_bufid_t bufid)
    {
        BufPtr b{bufid};
        zb_zdo_app_signal_hdr_t *pHdr;
        auto signalId = zb_get_app_signal(bufid, &pHdr);
        zb_ret_t status = zb_buf_get_status(bufid);
        switch(signalId)
        {
            //case ZB_ZDO_SIGNAL_DEFAULT_START://deprecated
            //break;
            case ZB_ZDO_SIGNAL_SKIP_STARTUP:
                if constexpr (h.on_skip_startup) h.invoke<h.on_skip_startup>(status);
                break;
            case ZB_ZDO_SIGNAL_DEVICE_ANNCE://zb_zdo_signal_device_annce_params_t
                if constexpr (h.on_dev_annce) h.invoke<h.on_dev_annce>(status, ZB_ZDO_SIGNAL_GET_PARAMS(pHdr, zb_zdo_signal_device_annce_params_t));
                break;
            case ZB_ZDO_SIGNAL_LEAVE://zb_zdo_signal_leave_params_t
                if constexpr (h.on_leave) h.invoke<h.on_leave>(status, ZB_ZDO_SIGNAL_GET_PARAMS(pHdr, zb_zdo_signal_leave_params_t));
                break;
            case ZB_ZDO_SIGNAL_ERROR:
                if constexpr (h.on_error) h.invoke<h.on_error>(status);
                break;
            case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
                if constexpr (h.on_dev_first_start) h.invoke<h.on_dev_first_start>(status);
                break;
            case ZB_BDB_SIGNAL_DEVICE_REBOOT:
                if constexpr (h.on_dev_reboot) h.invoke<h.on_dev_reboot>(status);
                break;
            case ZB_BDB_SIGNAL_STEERING:
                if constexpr (h.on_steering) h.invoke<h.on_steering>(status);
                break;
            case ZB_BDB_SIGNAL_FORMATION:
                if constexpr (h.on_formation) h.invoke<h.on_formation>(status);
                break;
            case ZB_BDB_SIGNAL_FINDING_AND_BINDING_TARGET_FINISHED:
                if constexpr (h.on_finding_and_binding_target_finished) h.invoke<h.on_finding_and_binding_target_finished>(status);
                break;
            case ZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED:
                if constexpr (h.on_finding_and_binding_initiator_finished) h.invoke<h.on_finding_and_binding_initiator_finished>(status);
                break;
            case ZB_NWK_SIGNAL_DEVICE_ASSOCIATED://zb_nwk_signal_device_associated_params_t, deprecated
                if constexpr (h.on_dev_associated) h.invoke<h.on_dev_associated>(status, ZB_ZDO_SIGNAL_GET_PARAMS(pHdr, zb_nwk_signal_device_associated_params_t));
                break;
            case ZB_ZDO_SIGNAL_LEAVE_INDICATION://zb_zdo_signal_leave_indication_params_t
                if constexpr (h.on_leave_indication) h.invoke<h.on_leave_indication>(status, ZB_ZDO_SIGNAL_GET_PARAMS(pHdr, zb_zdo_signal_leave_indication_params_t));
                break;
            case ZB_BDB_SIGNAL_WWAH_REJOIN_STARTED:
                if constexpr (h.on_wwah_rejoined_started) h.invoke<h.on_wwah_rejoined_started>(status);
                break;
                //case ZB_ZGP_SIGNAL_COMMISSIONING://zb_zgp_signal_commissioning_params_s
                //break;
            case ZB_COMMON_SIGNAL_CAN_SLEEP://zb_zdo_signal_can_sleep_params_t
                if constexpr (h.on_can_sleep) h.invoke<h.on_can_sleep>(status, ZB_ZDO_SIGNAL_GET_PARAMS(pHdr, zb_zdo_signal_can_sleep_params_t));
                break;
            case ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
                if constexpr (h.on_production_config_ready) h.invoke<h.on_production_config_ready>(status);
                break;
            case ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
                if constexpr (h.on_nwk_no_active_links_left) h.invoke<h.on_nwk_no_active_links_left>(status);
                break;
            case ZB_SIGNAL_SUBGHZ_SUSPEND:
                if constexpr (h.on_sub_ghz_suspend) h.invoke<h.on_sub_ghz_suspend>(status);
                break;
            case ZB_SIGNAL_SUBGHZ_RESUME:
                if constexpr (h.on_sub_ghz_resume) h.invoke<h.on_sub_ghz_resume>(status);
                break;
            case ZB_ZDO_SIGNAL_DEVICE_AUTHORIZED://zb_zdo_signal_device_authorized_params_t
                if constexpr (h.on_dev_authorized) h.invoke<h.on_dev_authorized>(status, ZB_ZDO_SIGNAL_GET_PARAMS(pHdr, zb_zdo_signal_device_authorized_params_t));
                break;
            case ZB_ZDO_SIGNAL_DEVICE_UPDATE://zb_zdo_signal_device_update_params_t
                if constexpr (h.on_dev_update) h.invoke<h.on_dev_update>(status, ZB_ZDO_SIGNAL_GET_PARAMS(pHdr, zb_zdo_signal_device_update_params_t));
                break;
                //case ZB_NWK_SIGNAL_PANID_CONFLICT_DETECTED://zb_start_pan_id_conflict_resolution
                //break;
            case ZB_NLME_STATUS_INDICATION://zb_nwk_command_status_t
                if constexpr (h.on_nlme_status_indication) h.invoke<h.on_nlme_status_indication>(status, ZB_ZDO_SIGNAL_GET_PARAMS(pHdr, zb_nwk_command_status_t));
                break;
            case ZB_TCSWAP_DB_BACKUP_REQUIRED_SIGNAL:
                if constexpr (h.on_tcswap_db_backup_required) h.invoke<h.on_tcswap_db_backup_required>(status);
                break;
            case ZB_TC_SWAPPED_SIGNAL:
                if constexpr (h.on_tcswapped) h.invoke<h.on_tcswapped>(status);
                break;
            case ZB_BDB_SIGNAL_TC_REJOIN_DONE:
                if constexpr (h.on_tc_rejoin_done) h.invoke<h.on_tc_rejoin_done>(status);
                break;
            case ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS://zb_uint8_t
                if constexpr (h.on_nwk_permit_join) h.invoke<h.on_nwk_permit_join>(status, ZB_ZDO_SIGNAL_GET_PARAMS(pHdr, zb_uint8_t));
                break;
            case ZB_BDB_SIGNAL_STEERING_CANCELLED:
                if constexpr (h.on_steering_cancelled) h.invoke<h.on_steering_cancelled>(status);
                break;
            case ZB_BDB_SIGNAL_FORMATION_CANCELLED:
                if constexpr (h.on_formation_cancelled) h.invoke<h.on_formation_cancelled>(status);
                break;
            case ZB_SIGNAL_READY_TO_SHUT:
                if constexpr (h.on_ready_to_shut) h.invoke<h.on_ready_to_shut>(status);
                break;
        }

        return zigbee_default_signal_handler(bufid);
    }
}
#endif

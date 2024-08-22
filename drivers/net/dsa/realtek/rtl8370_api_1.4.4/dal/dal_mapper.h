#ifndef __DAL_MAPPER_H__
#define __DAL_MAPPER_H__

/*
 * Include Files
 */
#include <rtk_types.h>
#include <rtk_error.h>
#include <rtk_switch.h>
#include <led.h>
#include <cpu.h>
#include <stat.h>
#include <l2.h>
#include <interrupt.h>
#include <port.h>
#include <rate.h>
#include <vlan.h>


/*
 * Symbol Definition
 */

/*
 * Data Declaration
 */

typedef struct dal_mapper_s {

    /* switch */
    rtk_api_ret_t   (*switch_init)(void);
    rtk_api_ret_t   (*switch_portMaxPktLen_set)(rtk_port_t, rtk_switch_maxPktLen_linkSpeed_t, rtk_uint32);
    rtk_api_ret_t   (*switch_portMaxPktLen_get)(rtk_port_t, rtk_switch_maxPktLen_linkSpeed_t, rtk_uint32 *);
    rtk_api_ret_t   (*switch_maxPktLenCfg_set)(rtk_uint32, rtk_uint32);
    rtk_api_ret_t   (*switch_maxPktLenCfg_get)(rtk_uint32, rtk_uint32 *);
    rtk_api_ret_t   (*switch_greenEthernet_set)(rtk_enable_t);
    rtk_api_ret_t   (*switch_greenEthernet_get)(rtk_enable_t *);

    /* led */
    rtk_api_ret_t   (*led_enable_set)(rtk_led_group_t, rtk_portmask_t *);
    rtk_api_ret_t   (*led_enable_get)(rtk_led_group_t, rtk_portmask_t *);
    rtk_api_ret_t   (*led_operation_set)(rtk_led_operation_t );
    rtk_api_ret_t   (*led_operation_get)(rtk_led_operation_t *);
    rtk_api_ret_t   (*led_modeForce_set)(rtk_port_t, rtk_led_group_t, rtk_led_force_mode_t);
    rtk_api_ret_t   (*led_modeForce_get)(rtk_port_t, rtk_led_group_t, rtk_led_force_mode_t *);
    rtk_api_ret_t   (*led_blinkRate_set)(rtk_led_blink_rate_t);
    rtk_api_ret_t   (*led_blinkRate_get)(rtk_led_blink_rate_t *);
    rtk_api_ret_t   (*led_groupConfig_set)(rtk_led_group_t, rtk_led_congig_t);
    rtk_api_ret_t   (*led_groupConfig_get)(rtk_led_group_t, rtk_led_congig_t *);
    rtk_api_ret_t   (*led_groupAbility_set)(rtk_led_group_t, rtk_led_ability_t *);
    rtk_api_ret_t   (*led_groupAbility_get)(rtk_led_group_t, rtk_led_ability_t *);
    rtk_api_ret_t   (*led_serialMode_set)(rtk_led_active_t);
    rtk_api_ret_t   (*led_serialMode_get)(rtk_led_active_t *);
    rtk_api_ret_t   (*led_OutputEnable_set)(rtk_enable_t);
    rtk_api_ret_t   (*led_OutputEnable_get)(rtk_enable_t *);
    rtk_api_ret_t   (*led_serialModePortmask_set)(rtk_led_serialOutput_t, rtk_portmask_t *);
    rtk_api_ret_t   (*led_serialModePortmask_get)(rtk_led_serialOutput_t *, rtk_portmask_t *);

    /* cpu */
    rtk_api_ret_t (*cpu_enable_set)(rtk_enable_t);
    rtk_api_ret_t (*cpu_enable_get)(rtk_enable_t *);
    rtk_api_ret_t (*cpu_tagPort_set)(rtk_port_t, rtk_cpu_insert_t);
    rtk_api_ret_t (*cpu_tagPort_get)(rtk_port_t *, rtk_cpu_insert_t *);
    rtk_api_ret_t (*cpu_awarePort_set)(rtk_portmask_t *);
    rtk_api_ret_t (*cpu_awarePort_get)(rtk_portmask_t *);
    rtk_api_ret_t (*cpu_tagPosition_set)(rtk_cpu_position_t);
    rtk_api_ret_t (*cpu_tagPosition_get)(rtk_cpu_position_t *);
    rtk_api_ret_t (*cpu_tagLength_set)(rtk_cpu_tag_length_t);
    rtk_api_ret_t (*cpu_tagLength_get)(rtk_cpu_tag_length_t *);
    rtk_api_ret_t (*cpu_acceptLength_set)(rtk_cpu_rx_length_t);
    rtk_api_ret_t (*cpu_acceptLength_get)(rtk_cpu_rx_length_t *);
    rtk_api_ret_t (*cpu_priRemap_set)(rtk_pri_t, rtk_pri_t);
    rtk_api_ret_t (*cpu_priRemap_get)(rtk_pri_t, rtk_pri_t *);

    /* stat */
    rtk_api_ret_t (*stat_global_reset)(void);
    rtk_api_ret_t (*stat_port_reset)(rtk_port_t);
    rtk_api_ret_t (*stat_queueManage_reset)(void);
    rtk_api_ret_t (*stat_global_get)(rtk_stat_global_type_t, rtk_stat_counter_t *);
    rtk_api_ret_t (*stat_global_getAll)(rtk_stat_global_cntr_t *);
    rtk_api_ret_t (*stat_port_get)(rtk_port_t, rtk_stat_port_type_t, rtk_stat_counter_t *);
    rtk_api_ret_t (*stat_port_getAll)(rtk_port_t, rtk_stat_port_cntr_t *);
    rtk_api_ret_t (*stat_logging_counterCfg_set)(rtk_uint32, rtk_logging_counter_mode_t, rtk_logging_counter_type_t);
    rtk_api_ret_t (*stat_logging_counterCfg_get)(rtk_uint32, rtk_logging_counter_mode_t *, rtk_logging_counter_type_t *);
    rtk_api_ret_t (*stat_logging_counter_reset)(rtk_uint32);
    rtk_api_ret_t (*stat_logging_counter_get)(rtk_uint32, rtk_uint32 *);
    rtk_api_ret_t (*stat_lengthMode_set)(rtk_stat_lengthMode_t, rtk_stat_lengthMode_t);
    rtk_api_ret_t (*stat_lengthMode_get)(rtk_stat_lengthMode_t *, rtk_stat_lengthMode_t *);

    /* l2 */
    rtk_api_ret_t (*l2_init)(void);
    rtk_api_ret_t (*l2_addr_add)(rtk_mac_t *, rtk_l2_ucastAddr_t *);
    rtk_api_ret_t (*l2_addr_get)(rtk_mac_t *, rtk_l2_ucastAddr_t *);
    rtk_api_ret_t (*l2_addr_next_get)(rtk_l2_read_method_t, rtk_port_t, rtk_uint32 *, rtk_l2_ucastAddr_t *);
    rtk_api_ret_t (*l2_addr_del)(rtk_mac_t *, rtk_l2_ucastAddr_t *);
    rtk_api_ret_t (*l2_mcastAddr_add)(rtk_l2_mcastAddr_t *);
    rtk_api_ret_t (*l2_mcastAddr_get)(rtk_l2_mcastAddr_t *);
    rtk_api_ret_t (*l2_mcastAddr_next_get)(rtk_uint32 *, rtk_l2_mcastAddr_t *);
    rtk_api_ret_t (*l2_mcastAddr_del)(rtk_l2_mcastAddr_t *);
    rtk_api_ret_t (*l2_ipMcastAddr_add)(rtk_l2_ipMcastAddr_t *);
    rtk_api_ret_t (*l2_ipMcastAddr_get)(rtk_l2_ipMcastAddr_t *);
    rtk_api_ret_t (*l2_ipMcastAddr_next_get)(rtk_uint32 *, rtk_l2_ipMcastAddr_t *);
    rtk_api_ret_t (*l2_ipMcastAddr_del)(rtk_l2_ipMcastAddr_t *);
    rtk_api_ret_t (*l2_ipVidMcastAddr_add)(rtk_l2_ipVidMcastAddr_t *);
    rtk_api_ret_t (*l2_ipVidMcastAddr_get)(rtk_l2_ipVidMcastAddr_t *);
    rtk_api_ret_t (*l2_ipVidMcastAddr_next_get)(rtk_uint32 *, rtk_l2_ipVidMcastAddr_t *);
    rtk_api_ret_t (*l2_ipVidMcastAddr_del)(rtk_l2_ipVidMcastAddr_t *);
    rtk_api_ret_t (*l2_ucastAddr_flush)(rtk_l2_flushCfg_t *);
    rtk_api_ret_t (*l2_table_clear)(void);
    rtk_api_ret_t (*l2_table_clearStatus_get)(rtk_l2_clearStatus_t *);
    rtk_api_ret_t (*l2_flushLinkDownPortAddrEnable_set)(rtk_port_t, rtk_enable_t);
    rtk_api_ret_t (*l2_flushLinkDownPortAddrEnable_get)(rtk_port_t, rtk_enable_t *);
    rtk_api_ret_t (*l2_agingEnable_set)(rtk_port_t, rtk_enable_t);
    rtk_api_ret_t (*l2_agingEnable_get)(rtk_port_t, rtk_enable_t *);
    rtk_api_ret_t (*l2_limitLearningCnt_set)(rtk_port_t, rtk_mac_cnt_t);
    rtk_api_ret_t (*l2_limitLearningCnt_get)(rtk_port_t, rtk_mac_cnt_t *);
    rtk_api_ret_t (*l2_limitSystemLearningCnt_set)(rtk_mac_cnt_t);
    rtk_api_ret_t (*l2_limitSystemLearningCnt_get)(rtk_mac_cnt_t *);
    rtk_api_ret_t (*l2_limitLearningCntAction_set)(rtk_port_t, rtk_l2_limitLearnCntAction_t);
    rtk_api_ret_t (*l2_limitLearningCntAction_get)(rtk_port_t, rtk_l2_limitLearnCntAction_t *);
    rtk_api_ret_t (*l2_limitSystemLearningCntAction_set)(rtk_l2_limitLearnCntAction_t);
    rtk_api_ret_t (*l2_limitSystemLearningCntAction_get)(rtk_l2_limitLearnCntAction_t *);
    rtk_api_ret_t (*l2_limitSystemLearningCntPortMask_set)(rtk_portmask_t *);
    rtk_api_ret_t (*l2_limitSystemLearningCntPortMask_get)(rtk_portmask_t *);
    rtk_api_ret_t (*l2_learningCnt_get)(rtk_port_t port, rtk_mac_cnt_t *);
    rtk_api_ret_t (*l2_floodPortMask_set)(rtk_l2_flood_type_t, rtk_portmask_t *);
    rtk_api_ret_t (*l2_floodPortMask_get)(rtk_l2_flood_type_t, rtk_portmask_t *);
    rtk_api_ret_t (*l2_localPktPermit_set)(rtk_port_t, rtk_enable_t);
    rtk_api_ret_t (*l2_localPktPermit_get)(rtk_port_t, rtk_enable_t *);
    rtk_api_ret_t (*l2_aging_set)(rtk_l2_age_time_t);
    rtk_api_ret_t (*l2_aging_get)(rtk_l2_age_time_t *);
    rtk_api_ret_t (*l2_ipMcastAddrLookup_set)(rtk_l2_ipmc_lookup_type_t);
    rtk_api_ret_t (*l2_ipMcastAddrLookup_get)(rtk_l2_ipmc_lookup_type_t *);
    rtk_api_ret_t (*l2_ipMcastForwardRouterPort_set)(rtk_enable_t);
    rtk_api_ret_t (*l2_ipMcastForwardRouterPort_get)(rtk_enable_t *);
    rtk_api_ret_t (*l2_ipMcastGroupEntry_add)(ipaddr_t, rtk_uint32, rtk_portmask_t *);
    rtk_api_ret_t (*l2_ipMcastGroupEntry_del)(ipaddr_t, rtk_uint32);
    rtk_api_ret_t (*l2_ipMcastGroupEntry_get)(ipaddr_t, rtk_uint32, rtk_portmask_t *);
    rtk_api_ret_t (*l2_entry_get)(rtk_l2_addr_table_t *);
    rtk_api_ret_t (*l2_lookupHitIsolationAction_set)(rtk_l2_lookupHitIsolationAction_t);
    rtk_api_ret_t (*l2_lookupHitIsolationAction_get)(rtk_l2_lookupHitIsolationAction_t *);

    /* interrupt */
    rtk_api_ret_t (*int_polarity_set)(rtk_int_polarity_t);
    rtk_api_ret_t (*int_polarity_get)(rtk_int_polarity_t *);
    rtk_api_ret_t (*int_control_set)(rtk_int_type_t, rtk_enable_t);
    rtk_api_ret_t (*int_control_get)(rtk_int_type_t, rtk_enable_t *);
    rtk_api_ret_t (*int_status_set)(rtk_int_status_t *);
    rtk_api_ret_t (*int_status_get)(rtk_int_status_t *);
    rtk_api_ret_t (*int_advanceInfo_get)(rtk_int_advType_t, rtk_int_info_t *);

    /* port */
    rtk_api_ret_t (*port_phyAutoNegoAbility_set)(rtk_port_t, rtk_port_phy_ability_t *);
    rtk_api_ret_t (*port_phyAutoNegoAbility_get)(rtk_port_t, rtk_port_phy_ability_t *);
    rtk_api_ret_t (*port_phyForceModeAbility_set)(rtk_port_t, rtk_port_phy_ability_t *);
    rtk_api_ret_t (*port_phyForceModeAbility_get)(rtk_port_t, rtk_port_phy_ability_t *);
    rtk_api_ret_t (*port_phyStatus_get)(rtk_port_t, rtk_port_linkStatus_t *, rtk_port_speed_t *, rtk_port_duplex_t *);
    rtk_api_ret_t (*port_macForceLink_set)(rtk_port_t, rtk_port_mac_ability_t *);
    rtk_api_ret_t (*port_macForceLink_get)(rtk_port_t, rtk_port_mac_ability_t *);
    rtk_api_ret_t (*port_macForceLinkExt_set)(rtk_port_t, rtk_mode_ext_t, rtk_port_mac_ability_t *);
    rtk_api_ret_t (*port_macForceLinkExt_get)(rtk_port_t, rtk_mode_ext_t *, rtk_port_mac_ability_t *);
    rtk_api_ret_t (*port_macStatus_get)(rtk_port_t, rtk_port_mac_ability_t *);
    rtk_api_ret_t (*port_macLocalLoopbackEnable_set)(rtk_port_t, rtk_enable_t);
    rtk_api_ret_t (*port_macLocalLoopbackEnable_get)(rtk_port_t, rtk_enable_t *);
    rtk_api_ret_t (*port_phyReg_set)(rtk_port_t, rtk_port_phy_reg_t, rtk_port_phy_data_t);
    rtk_api_ret_t (*port_phyReg_get)(rtk_port_t, rtk_port_phy_reg_t, rtk_port_phy_data_t *);
    rtk_api_ret_t (*port_phyOCPReg_set)(rtk_port_t, rtk_uint32, rtk_uint32);
    rtk_api_ret_t (*port_phyOCPReg_get)(rtk_port_t, rtk_uint32, rtk_uint32 *);
    rtk_api_ret_t (*port_backpressureEnable_set)(rtk_port_t, rtk_enable_t);
    rtk_api_ret_t (*port_backpressureEnable_get)(rtk_port_t, rtk_enable_t *);
    rtk_api_ret_t (*port_adminEnable_set)(rtk_port_t, rtk_enable_t);
    rtk_api_ret_t (*port_adminEnable_get)(rtk_port_t, rtk_enable_t *);
    rtk_api_ret_t (*port_isolation_set)(rtk_port_t, rtk_portmask_t *);
    rtk_api_ret_t (*port_isolation_get)(rtk_port_t, rtk_portmask_t *);
    rtk_api_ret_t (*port_rgmiiDelayExt_set)(rtk_port_t, rtk_data_t, rtk_data_t);
    rtk_api_ret_t (*port_rgmiiDelayExt_get)(rtk_port_t, rtk_data_t *, rtk_data_t *);
    rtk_api_ret_t (*port_phyEnableAll_set)(rtk_enable_t);
    rtk_api_ret_t (*port_phyEnableAll_get)(rtk_enable_t *);
    rtk_api_ret_t (*port_efid_set)(rtk_port_t, rtk_data_t);
    rtk_api_ret_t (*port_efid_get)(rtk_port_t, rtk_data_t *);
    rtk_api_ret_t (*port_phyComboPortMedia_set)(rtk_port_t, rtk_port_media_t);
    rtk_api_ret_t (*port_phyComboPortMedia_get)(rtk_port_t, rtk_port_media_t *);
    rtk_api_ret_t (*port_rtctEnable_set)(rtk_portmask_t *);
    rtk_api_ret_t (*port_rtctDisable_set)(rtk_portmask_t *);
    rtk_api_ret_t (*port_rtctResult_get)(rtk_port_t, rtk_rtctResult_t *);
    rtk_api_ret_t (*port_sds_reset)(rtk_port_t);
    rtk_api_ret_t (*port_sgmiiLinkStatus_get)(rtk_port_t, rtk_data_t *, rtk_data_t *, rtk_port_linkStatus_t *);
    rtk_api_ret_t (*port_sgmiiNway_set)(rtk_port_t, rtk_enable_t);
    rtk_api_ret_t (*port_sgmiiNway_get)(rtk_port_t, rtk_enable_t *);
    rtk_api_ret_t (*port_fiberAbilityExt_set)(rtk_port_t, rtk_uint32, rtk_uint32);
    rtk_api_ret_t (*port_fiberAbilityExt_get)(rtk_port_t, rtk_uint32 *, rtk_uint32 *);
    rtk_api_ret_t (*port_autoDos_set)(rtk_port_autoDosType_t, rtk_enable_t);
    rtk_api_ret_t (*port_autoDos_get)(rtk_port_autoDosType_t, rtk_enable_t *);
    rtk_api_ret_t (*port_fiberAbility_set)(rtk_port_t, rtk_port_fiber_ability_t *);
    rtk_api_ret_t (*port_fiberAbility_get)(rtk_port_t, rtk_port_fiber_ability_t *);
    rtk_api_ret_t (*port_phyMdx_set)(rtk_port_t, rtk_port_phy_mdix_mode_t);
    rtk_api_ret_t (*port_phyMdx_get)(rtk_port_t, rtk_port_phy_mdix_mode_t *);
    rtk_api_ret_t (*port_phyMdxStatus_get)(rtk_port_t, rtk_port_phy_mdix_status_t *);
    rtk_api_ret_t (*port_phyTestMode_set)(rtk_port_t, rtk_port_phy_test_mode_t);
    rtk_api_ret_t (*port_phyTestMode_get)(rtk_port_t, rtk_port_phy_test_mode_t *);
    rtk_api_ret_t (*port_maxPacketLength_set)(rtk_port_t, rtk_uint32);
    rtk_api_ret_t (*port_maxPacketLength_get)(rtk_port_t, rtk_uint32 *);
    rtk_api_ret_t (*port_phyGreenEthernet_set)(rtk_port_t, rtk_enable_t);
    rtk_api_ret_t (*port_phyGreenEthernet_get)(rtk_port_t, rtk_enable_t *);
    rtk_api_ret_t (*port_phyLinkDownPowerSaving_set)(rtk_port_t, rtk_enable_t);
    rtk_api_ret_t (*port_phyLinkDownPowerSaving_get)(rtk_port_t, rtk_enable_t *);
    rtk_api_ret_t (*port_serdesReg_set)(rtk_uint32, rtk_uint32, rtk_uint32, rtk_uint32);
    rtk_api_ret_t (*port_serdesReg_get)(rtk_uint32, rtk_uint32, rtk_uint32, rtk_uint32 *);
    rtk_api_ret_t (*port_serdesPolarity_set)(rtk_uint32, rtk_port_sdsPolarity_t, rtk_port_sdsPolarity_t);
    rtk_api_ret_t (*port_serdesPolarity_get)(rtk_uint32, rtk_port_sdsPolarity_t *, rtk_port_sdsPolarity_t *);
    rtk_api_ret_t (*port_extPhyAutoPollingCfg_set)(rtk_port_t, rtk_port_extPhyAutoPollingCfg_t *);
    rtk_api_ret_t (*port_extPhyAutoPollingCfg_get)(rtk_port_t, rtk_port_extPhyAutoPollingCfg_t *);
    rtk_api_ret_t (*port_extPhySmiState_set)(rtk_enable_t);
    rtk_api_ret_t (*port_extPhySmiState_get)(rtk_enable_t *);
    rtk_api_ret_t (*port_extPhyReg_set)(rtk_uint32, rtk_port_phy_page_t, rtk_port_phy_reg_t, rtk_port_phy_data_t);
    rtk_api_ret_t (*port_extPhyReg_get)(rtk_uint32, rtk_port_phy_page_t, rtk_port_phy_reg_t, rtk_port_phy_data_t *);
    rtk_api_ret_t (*port_extC45PhyReg_set)(rtk_uint32,  rtk_uint32, rtk_port_phy_reg_t, rtk_port_phy_data_t);
    rtk_api_ret_t (*port_extC45PhyReg_get)(rtk_uint32, rtk_uint32, rtk_port_phy_reg_t, rtk_port_phy_data_t *);

    /* Rate */
    rtk_api_ret_t (*rate_shareMeter_set)(rtk_meter_id_t, rtk_meter_type_t, rtk_rate_t, rtk_enable_t);
    rtk_api_ret_t (*rate_shareMeter_get)(rtk_meter_id_t, rtk_meter_type_t *, rtk_rate_t *, rtk_enable_t *);
    rtk_api_ret_t (*rate_shareMeterBucket_set)(rtk_meter_id_t, rtk_uint32);
    rtk_api_ret_t (*rate_shareMeterBucket_get)(rtk_meter_id_t, rtk_uint32 *);
    rtk_api_ret_t (*rate_igrBandwidthCtrlRate_set)(rtk_port_t, rtk_rate_t,  rtk_enable_t, rtk_enable_t);
    rtk_api_ret_t (*rate_igrBandwidthCtrlRate_get)(rtk_port_t, rtk_rate_t *, rtk_enable_t *, rtk_enable_t *);
    rtk_api_ret_t (*rate_egrBandwidthCtrlRate_set)(rtk_port_t, rtk_rate_t,  rtk_enable_t);
    rtk_api_ret_t (*rate_egrBandwidthCtrlRate_get)(rtk_port_t, rtk_rate_t *, rtk_enable_t *);
    rtk_api_ret_t (*rate_egrQueueBwCtrlEnable_set)(rtk_port_t, rtk_qid_t, rtk_enable_t);
    rtk_api_ret_t (*rate_egrQueueBwCtrlEnable_get)(rtk_port_t, rtk_qid_t, rtk_enable_t *);
    rtk_api_ret_t (*rate_egrQueueBwCtrlRate_set)(rtk_port_t, rtk_qid_t, rtk_meter_id_t);
    rtk_api_ret_t (*rate_egrQueueBwCtrlRate_get)(rtk_port_t, rtk_qid_t, rtk_meter_id_t *);

    /*VLAN*/
    rtk_api_ret_t (*vlan_init)(void);
    rtk_api_ret_t (*vlan_set)(rtk_vlan_t, rtk_vlan_cfg_t *);
    rtk_api_ret_t (*vlan_get)(rtk_vlan_t, rtk_vlan_cfg_t *);
    rtk_api_ret_t (*vlan_egrFilterEnable_set)(rtk_enable_t);
    rtk_api_ret_t (*vlan_egrFilterEnable_get)(rtk_enable_t *);
    rtk_api_ret_t (*vlan_mbrCfg_set)(rtk_uint32, rtk_vlan_mbrcfg_t *);
    rtk_api_ret_t (*vlan_mbrCfg_get)(rtk_uint32, rtk_vlan_mbrcfg_t *);
    rtk_api_ret_t (*vlan_portPvid_set)(rtk_port_t, rtk_vlan_t, rtk_pri_t);
    rtk_api_ret_t (*vlan_portPvid_get)(rtk_port_t, rtk_vlan_t *, rtk_pri_t *);
    rtk_api_ret_t (*vlan_portIgrFilterEnable_set)(rtk_port_t, rtk_enable_t);
    rtk_api_ret_t (*vlan_portIgrFilterEnable_get)(rtk_port_t, rtk_enable_t *);
    rtk_api_ret_t (*vlan_portAcceptFrameType_set)(rtk_port_t, rtk_vlan_acceptFrameType_t);
    rtk_api_ret_t (*vlan_portAcceptFrameType_get)(rtk_port_t, rtk_vlan_acceptFrameType_t *);
    rtk_api_ret_t (*vlan_tagMode_set)(rtk_port_t, rtk_vlan_tagMode_t);
    rtk_api_ret_t (*vlan_tagMode_get)(rtk_port_t, rtk_vlan_tagMode_t *);
    rtk_api_ret_t (*vlan_transparent_set)(rtk_port_t, rtk_portmask_t *);
    rtk_api_ret_t (*vlan_transparent_get)(rtk_port_t , rtk_portmask_t *);
    rtk_api_ret_t (*vlan_keep_set)(rtk_port_t, rtk_portmask_t *);
    rtk_api_ret_t (*vlan_keep_get)(rtk_port_t, rtk_portmask_t *);
    rtk_api_ret_t (*vlan_stg_set)(rtk_vlan_t, rtk_stp_msti_id_t);
    rtk_api_ret_t (*vlan_stg_get)(rtk_vlan_t, rtk_stp_msti_id_t *);
    rtk_api_ret_t (*vlan_protoAndPortBasedVlan_add)(rtk_port_t, rtk_vlan_protoAndPortInfo_t *);
    rtk_api_ret_t (*vlan_protoAndPortBasedVlan_get)(rtk_port_t , rtk_vlan_proto_type_t, rtk_vlan_protoVlan_frameType_t, rtk_vlan_protoAndPortInfo_t *);
    rtk_api_ret_t (*vlan_protoAndPortBasedVlan_del)(rtk_port_t , rtk_vlan_proto_type_t, rtk_vlan_protoVlan_frameType_t);
    rtk_api_ret_t (*vlan_protoAndPortBasedVlan_delAll)(rtk_port_t);
    rtk_api_ret_t (*vlan_portFid_set)(rtk_port_t port, rtk_enable_t, rtk_fid_t);
    rtk_api_ret_t (*vlan_portFid_get)(rtk_port_t port, rtk_enable_t *, rtk_fid_t *);
    rtk_api_ret_t (*vlan_UntagDscpPriorityEnable_set)(rtk_enable_t);
    rtk_api_ret_t (*vlan_UntagDscpPriorityEnable_get)(rtk_enable_t *);
    rtk_api_ret_t (*stp_mstpState_set)(rtk_stp_msti_id_t, rtk_port_t, rtk_stp_state_t);
    rtk_api_ret_t (*stp_mstpState_get)(rtk_stp_msti_id_t, rtk_port_t, rtk_stp_state_t *);
    rtk_api_ret_t (*vlan_reservedVidAction_set)(rtk_vlan_resVidAction_t, rtk_vlan_resVidAction_t);
    rtk_api_ret_t (*vlan_reservedVidAction_get)(rtk_vlan_resVidAction_t *, rtk_vlan_resVidAction_t *);
    rtk_api_ret_t (*vlan_realKeepRemarkEnable_set)(rtk_enable_t );
    rtk_api_ret_t (*vlan_realKeepRemarkEnable_get)(rtk_enable_t *);
    rtk_api_ret_t (*vlan_reset)(void);

} dal_mapper_t;


#endif /* __DAL_MAPPER_H __ */

/*
 * Include Files
 */
#include <dal/dal_mapper.h>
#include <dal/rtl8367c/dal_rtl8367c_mapper.h>
#include <dal/rtl8367c/dal_rtl8367c_switch.h>
#include <dal/rtl8367c/dal_rtl8367c_led.h>
#include <dal/rtl8367c/dal_rtl8367c_cpu.h>
#include <dal/rtl8367c/dal_rtl8367c_stat.h>
#include <dal/rtl8367c/dal_rtl8367c_l2.h>
#include <dal/rtl8367c/dal_rtl8367c_interrupt.h>
#include <dal/rtl8367c/dal_rtl8367c_port.h>
#include <dal/rtl8367c/dal_rtl8367c_rate.h>
#include <dal/rtl8367c/dal_rtl8367c_vlan.h>

/*
 * Symbol Definition
 */

/*
 * Data Declaration
 */
static dal_mapper_t dal_rtl8367c_mapper =
{
    /* Switch */
    .switch_init = dal_rtl8367c_switch_init,
    .switch_portMaxPktLen_set = dal_rtl8367c_switch_portMaxPktLen_set,
    .switch_portMaxPktLen_get = dal_rtl8367c_switch_portMaxPktLen_get,
    .switch_maxPktLenCfg_set = dal_rtl8367c_switch_maxPktLenCfg_set,
    .switch_maxPktLenCfg_get = dal_rtl8367c_switch_maxPktLenCfg_get,
    .switch_greenEthernet_set = dal_rtl8367c_switch_greenEthernet_set,
    .switch_greenEthernet_get = dal_rtl8367c_switch_greenEthernet_get,

    /* led */
    .led_enable_set = dal_rtl8367c_led_enable_set,
    .led_enable_get = dal_rtl8367c_led_enable_get,
    .led_operation_set = dal_rtl8367c_led_operation_set,
    .led_operation_get = dal_rtl8367c_led_operation_get,
    .led_modeForce_set = dal_rtl8367c_led_modeForce_set,
    .led_modeForce_get = dal_rtl8367c_led_modeForce_get,
    .led_blinkRate_set = dal_rtl8367c_led_blinkRate_set,
    .led_blinkRate_get = dal_rtl8367c_led_blinkRate_get,
    .led_groupConfig_set = dal_rtl8367c_led_groupConfig_set,
    .led_groupConfig_get = dal_rtl8367c_led_groupConfig_get,
    .led_groupAbility_set = dal_rtl8367c_led_groupAbility_set,
    .led_groupAbility_get = dal_rtl8367c_led_groupAbility_get,
    .led_serialMode_set = dal_rtl8367c_led_serialMode_set,
    .led_serialMode_get = dal_rtl8367c_led_serialMode_get,
    .led_OutputEnable_set = dal_rtl8367c_led_OutputEnable_set,
    .led_OutputEnable_get = dal_rtl8367c_led_OutputEnable_get,
    .led_serialModePortmask_set = dal_rtl8367c_led_serialModePortmask_set,
    .led_serialModePortmask_get = dal_rtl8367c_led_serialModePortmask_get,

    /* cpu */
    .cpu_enable_set = dal_rtl8367c_cpu_enable_set,
    .cpu_enable_get = dal_rtl8367c_cpu_enable_get,
    .cpu_tagPort_set = dal_rtl8367c_cpu_tagPort_set,
    .cpu_tagPort_get = dal_rtl8367c_cpu_tagPort_get,
    .cpu_awarePort_set = dal_rtl8367c_cpu_awarePort_set,
    .cpu_awarePort_get = dal_rtl8367c_cpu_awarePort_get,
    .cpu_tagPosition_set = dal_rtl8367c_cpu_tagPosition_set,
    .cpu_tagPosition_get = dal_rtl8367c_cpu_tagPosition_get,
    .cpu_tagLength_set = dal_rtl8367c_cpu_tagLength_set,
    .cpu_tagLength_get = dal_rtl8367c_cpu_tagLength_get,
    .cpu_acceptLength_set = dal_rtl8367c_cpu_acceptLength_set,
    .cpu_acceptLength_get = dal_rtl8367c_cpu_acceptLength_get,
    .cpu_priRemap_set = dal_rtl8367c_cpu_priRemap_set,
    .cpu_priRemap_get = dal_rtl8367c_cpu_priRemap_get,

    /* stat */
    .stat_global_reset = dal_rtl8367c_stat_global_reset,
    .stat_port_reset = dal_rtl8367c_stat_port_reset,
    .stat_queueManage_reset = dal_rtl8367c_stat_queueManage_reset,
    .stat_global_get = dal_rtl8367c_stat_global_get,
    .stat_global_getAll = dal_rtl8367c_stat_global_getAll,
    .stat_port_get = dal_rtl8367c_stat_port_get,
    .stat_port_getAll = dal_rtl8367c_stat_port_getAll,
    .stat_logging_counterCfg_set = dal_rtl8367c_stat_logging_counterCfg_set,
    .stat_logging_counterCfg_get = dal_rtl8367c_stat_logging_counterCfg_get,
    .stat_logging_counter_reset = dal_rtl8367c_stat_logging_counter_reset,
    .stat_logging_counter_get = dal_rtl8367c_stat_logging_counter_get,
    .stat_lengthMode_set = dal_rtl8367c_stat_lengthMode_set,
    .stat_lengthMode_get = dal_rtl8367c_stat_lengthMode_get,

    /* l2 */
    .l2_init = dal_rtl8367c_l2_init,
    .l2_addr_add = dal_rtl8367c_l2_addr_add,
    .l2_addr_get = dal_rtl8367c_l2_addr_get,
    .l2_addr_next_get = dal_rtl8367c_l2_addr_next_get,
    .l2_addr_del = dal_rtl8367c_l2_addr_del,
    .l2_mcastAddr_add = dal_rtl8367c_l2_mcastAddr_add,
    .l2_mcastAddr_get = dal_rtl8367c_l2_mcastAddr_get,
    .l2_mcastAddr_next_get = dal_rtl8367c_l2_mcastAddr_next_get,
    .l2_mcastAddr_del = dal_rtl8367c_l2_mcastAddr_del,
    .l2_ipMcastAddr_add = dal_rtl8367c_l2_ipMcastAddr_add,
    .l2_ipMcastAddr_get = dal_rtl8367c_l2_ipMcastAddr_get,
    .l2_ipMcastAddr_next_get = dal_rtl8367c_l2_ipMcastAddr_next_get,
    .l2_ipMcastAddr_del = dal_rtl8367c_l2_ipMcastAddr_del,
    .l2_ipVidMcastAddr_add = dal_rtl8367c_l2_ipVidMcastAddr_add,
    .l2_ipVidMcastAddr_get = dal_rtl8367c_l2_ipVidMcastAddr_get,
    .l2_ipVidMcastAddr_next_get = dal_rtl8367c_l2_ipVidMcastAddr_next_get,
    .l2_ipVidMcastAddr_del = dal_rtl8367c_l2_ipVidMcastAddr_del,
    .l2_ucastAddr_flush = dal_rtl8367c_l2_ucastAddr_flush,
    .l2_table_clear = dal_rtl8367c_l2_table_clear,
    .l2_table_clearStatus_get = dal_rtl8367c_l2_table_clearStatus_get,
    .l2_flushLinkDownPortAddrEnable_set = dal_rtl8367c_l2_flushLinkDownPortAddrEnable_set,
    .l2_flushLinkDownPortAddrEnable_get = dal_rtl8367c_l2_flushLinkDownPortAddrEnable_get,
    .l2_agingEnable_set = dal_rtl8367c_l2_agingEnable_set,
    .l2_agingEnable_get = dal_rtl8367c_l2_agingEnable_get,
    .l2_limitLearningCnt_set = dal_rtl8367c_l2_limitLearningCnt_set,
    .l2_limitLearningCnt_get = dal_rtl8367c_l2_limitLearningCnt_get,
    .l2_limitSystemLearningCnt_set = dal_rtl8367c_l2_limitSystemLearningCnt_set,
    .l2_limitSystemLearningCnt_get = dal_rtl8367c_l2_limitSystemLearningCnt_get,
    .l2_limitLearningCntAction_set = dal_rtl8367c_l2_limitLearningCntAction_set,
    .l2_limitLearningCntAction_get = dal_rtl8367c_l2_limitLearningCntAction_get,
    .l2_limitSystemLearningCntAction_set = dal_rtl8367c_l2_limitSystemLearningCntAction_set,
    .l2_limitSystemLearningCntAction_get = dal_rtl8367c_l2_limitSystemLearningCntAction_get,
    .l2_limitSystemLearningCntPortMask_set = dal_rtl8367c_l2_limitSystemLearningCntPortMask_set,
    .l2_limitSystemLearningCntPortMask_get = dal_rtl8367c_l2_limitSystemLearningCntPortMask_get,
    .l2_learningCnt_get = dal_rtl8367c_l2_learningCnt_get,
    .l2_floodPortMask_set = dal_rtl8367c_l2_floodPortMask_set,
    .l2_floodPortMask_get = dal_rtl8367c_l2_floodPortMask_get,
    .l2_localPktPermit_set = dal_rtl8367c_l2_localPktPermit_set,
    .l2_localPktPermit_get = dal_rtl8367c_l2_localPktPermit_get,
    .l2_aging_set = dal_rtl8367c_l2_aging_set,
    .l2_aging_get = dal_rtl8367c_l2_aging_get,
    .l2_ipMcastAddrLookup_set = dal_rtl8367c_l2_ipMcastAddrLookup_set,
    .l2_ipMcastAddrLookup_get = dal_rtl8367c_l2_ipMcastAddrLookup_get,
    .l2_ipMcastForwardRouterPort_set = dal_rtl8367c_l2_ipMcastForwardRouterPort_set,
    .l2_ipMcastForwardRouterPort_get = dal_rtl8367c_l2_ipMcastForwardRouterPort_get,
    .l2_ipMcastGroupEntry_add = dal_rtl8367c_l2_ipMcastGroupEntry_add,
    .l2_ipMcastGroupEntry_del = dal_rtl8367c_l2_ipMcastGroupEntry_del,
    .l2_ipMcastGroupEntry_get = dal_rtl8367c_l2_ipMcastGroupEntry_get,
    .l2_entry_get = dal_rtl8367c_l2_entry_get,
    .l2_lookupHitIsolationAction_set = NULL,
    .l2_lookupHitIsolationAction_get = NULL,

    /* interrupt */
    .int_polarity_set = dal_rtl8367c_int_polarity_set,
    .int_polarity_get = dal_rtl8367c_int_polarity_get,
    .int_control_set = dal_rtl8367c_int_control_set,
    .int_control_get = dal_rtl8367c_int_control_get,
    .int_status_set = dal_rtl8367c_int_status_set,
    .int_status_get = dal_rtl8367c_int_status_get,
    .int_advanceInfo_get = dal_rtl8367c_int_advanceInfo_get,

    /* port */
    .port_phyAutoNegoAbility_set = dal_rtl8367c_port_phyAutoNegoAbility_set,
    .port_phyAutoNegoAbility_get = dal_rtl8367c_port_phyAutoNegoAbility_get,
    .port_phyForceModeAbility_set = dal_rtl8367c_port_phyForceModeAbility_set,
    .port_phyForceModeAbility_get = dal_rtl8367c_port_phyForceModeAbility_get,
    .port_phyStatus_get = dal_rtl8367c_port_phyStatus_get,
    .port_macForceLink_set = dal_rtl8367c_port_macForceLink_set,
    .port_macForceLink_get = dal_rtl8367c_port_macForceLink_get,
    .port_macForceLinkExt_set = dal_rtl8367c_port_macForceLinkExt_set,
    .port_macForceLinkExt_get = dal_rtl8367c_port_macForceLinkExt_get,
    .port_macStatus_get = dal_rtl8367c_port_macStatus_get,
    .port_macLocalLoopbackEnable_set = dal_rtl8367c_port_macLocalLoopbackEnable_set,
    .port_macLocalLoopbackEnable_get = dal_rtl8367c_port_macLocalLoopbackEnable_get,
    .port_phyReg_set = dal_rtl8367c_port_phyReg_set,
    .port_phyReg_get = dal_rtl8367c_port_phyReg_get,
    .port_phyOCPReg_set = dal_rtl8367c_port_phyOCPReg_set,
    .port_phyOCPReg_get = dal_rtl8367c_port_phyOCPReg_get,
    .port_backpressureEnable_set = dal_rtl8367c_port_backpressureEnable_set,
    .port_backpressureEnable_get = dal_rtl8367c_port_backpressureEnable_get,
    .port_adminEnable_set = dal_rtl8367c_port_adminEnable_set,
    .port_adminEnable_get = dal_rtl8367c_port_adminEnable_get,
    .port_isolation_set = dal_rtl8367c_port_isolation_set,
    .port_isolation_get = dal_rtl8367c_port_isolation_get,
    .port_rgmiiDelayExt_set = dal_rtl8367c_port_rgmiiDelayExt_set,
    .port_rgmiiDelayExt_get = dal_rtl8367c_port_rgmiiDelayExt_get,
    .port_phyEnableAll_set = dal_rtl8367c_port_phyEnableAll_set,
    .port_phyEnableAll_get = dal_rtl8367c_port_phyEnableAll_get,
    .port_efid_set = dal_rtl8367c_port_efid_set,
    .port_efid_get = dal_rtl8367c_port_efid_get,
    .port_phyComboPortMedia_set = dal_rtl8367c_port_phyComboPortMedia_set,
    .port_phyComboPortMedia_get = dal_rtl8367c_port_phyComboPortMedia_get,
    .port_rtctEnable_set = dal_rtl8367c_port_rtctEnable_set,
    .port_rtctDisable_set = dal_rtl8367c_port_rtctDisable_set,
    .port_rtctResult_get = dal_rtl8367c_port_rtctResult_get,
    .port_sds_reset = dal_rtl8367c_port_sds_reset,
    .port_sgmiiLinkStatus_get = dal_rtl8367c_port_sgmiiLinkStatus_get,
    .port_sgmiiNway_set = dal_rtl8367c_port_sgmiiNway_set,
    .port_sgmiiNway_get = dal_rtl8367c_port_sgmiiNway_get,
    .port_fiberAbilityExt_set = dal_rtl8367c_port_fiberAbilityExt_set,
    .port_fiberAbilityExt_get = dal_rtl8367c_port_fiberAbilityExt_get,
    .port_autoDos_set = dal_rtl8367c_port_autoDos_set,
    .port_autoDos_get = dal_rtl8367c_port_autoDos_get,
    .port_fiberAbility_set = NULL,
    .port_fiberAbility_get = NULL,
    .port_phyMdx_set = dal_rtl8367c_port_phyMdx_set,
    .port_phyMdx_get = dal_rtl8367c_port_phyMdx_get,
    .port_phyMdxStatus_get = dal_rtl8367c_port_phyMdxStatus_get,
    .port_phyTestMode_set = dal_rtl8367c_port_phyTestMode_set,
    .port_phyTestMode_get = dal_rtl8367c_port_phyTestMode_get,
    .port_maxPacketLength_set = NULL,
    .port_maxPacketLength_get = NULL,
    .port_phyGreenEthernet_set = dal_rtl8367c_port_phyGreenEthernet_set,
    .port_phyGreenEthernet_get = dal_rtl8367c_port_phyGreenEthernet_get,
    .port_phyLinkDownPowerSaving_set = dal_rtl8367c_port_phyLinkDownPowerSaving_set,
    .port_phyLinkDownPowerSaving_get = dal_rtl8367c_port_phyLinkDownPowerSaving_get,
    .port_serdesReg_set = dal_rtl8367c_port_serdesReg_set,
    .port_serdesReg_get = dal_rtl8367c_port_serdesReg_get,
    .port_serdesPolarity_set = dal_rtl8367c_port_serdesPolarity_set,
    .port_serdesPolarity_get = dal_rtl8367c_port_serdesPolarity_get,
    .port_extPhyAutoPollingCfg_set = dal_rtl8367c_port_extPhyAutoPollingCfg_set,
    .port_extPhyAutoPollingCfg_get = dal_rtl8367c_port_extPhyAutoPollingCfg_get,
    .port_extPhySmiState_set = dal_rtl8367c_port_extPhySmiState_set,
    .port_extPhySmiState_get = dal_rtl8367c_port_extPhySmiState_get,
    .port_extPhyReg_set = dal_rtl8367c_port_extPhyReg_set,
    .port_extPhyReg_get = dal_rtl8367c_port_extPhyReg_get,
    .port_extC45PhyReg_set = dal_rtl8367c_port_extC45PhyReg_set,
    .port_extC45PhyReg_get = dal_rtl8367c_port_extC45PhyReg_get,

    /* Rate */
    .rate_shareMeter_set = dal_rtl8367c_rate_shareMeter_set,
    .rate_shareMeter_get = dal_rtl8367c_rate_shareMeter_get,
    .rate_shareMeterBucket_set = dal_rtl8367c_rate_shareMeterBucket_set,
    .rate_shareMeterBucket_get = dal_rtl8367c_rate_shareMeterBucket_get,
    .rate_igrBandwidthCtrlRate_set = dal_rtl8367c_rate_igrBandwidthCtrlRate_set,
    .rate_igrBandwidthCtrlRate_get = dal_rtl8367c_rate_igrBandwidthCtrlRate_get,
    .rate_egrBandwidthCtrlRate_set = dal_rtl8367c_rate_egrBandwidthCtrlRate_set,
    .rate_egrBandwidthCtrlRate_get = dal_rtl8367c_rate_egrBandwidthCtrlRate_get,
    .rate_egrQueueBwCtrlEnable_set = dal_rtl8367c_rate_egrQueueBwCtrlEnable_set,
    .rate_egrQueueBwCtrlEnable_get = dal_rtl8367c_rate_egrQueueBwCtrlEnable_get,
    .rate_egrQueueBwCtrlRate_set = dal_rtl8367c_rate_egrQueueBwCtrlRate_set,
    .rate_egrQueueBwCtrlRate_get = dal_rtl8367c_rate_egrQueueBwCtrlRate_get,

    /*VLAN*/
    .vlan_init = dal_rtl8367c_vlan_init,
    .vlan_set = dal_rtl8367c_vlan_set,
    .vlan_get = dal_rtl8367c_vlan_get,
    .vlan_egrFilterEnable_set = dal_rtl8367c_vlan_egrFilterEnable_set,
    .vlan_egrFilterEnable_get = dal_rtl8367c_vlan_egrFilterEnable_get,
    .vlan_mbrCfg_set = dal_rtl8367c_vlan_mbrCfg_set,
    .vlan_mbrCfg_get = dal_rtl8367c_vlan_mbrCfg_get,
    .vlan_portPvid_set = dal_rtl8367c_vlan_portPvid_set,
    .vlan_portPvid_get = dal_rtl8367c_vlan_portPvid_get,
    .vlan_portIgrFilterEnable_set = dal_rtl8367c_vlan_portIgrFilterEnable_set,
    .vlan_portIgrFilterEnable_get = dal_rtl8367c_vlan_portIgrFilterEnable_get,
    .vlan_portAcceptFrameType_set = dal_rtl8367c_vlan_portAcceptFrameType_set,
    .vlan_portAcceptFrameType_get = dal_rtl8367c_vlan_portAcceptFrameType_get,
    .vlan_tagMode_set = dal_rtl8367c_vlan_tagMode_set,
    .vlan_tagMode_get = dal_rtl8367c_vlan_tagMode_get,
    .vlan_transparent_set = dal_rtl8367c_vlan_transparent_set,
    .vlan_transparent_get = dal_rtl8367c_vlan_transparent_get,
    .vlan_keep_set = dal_rtl8367c_vlan_keep_set,
    .vlan_keep_get = dal_rtl8367c_vlan_keep_get,
    .vlan_stg_set = dal_rtl8367c_vlan_stg_set,
    .vlan_stg_get = dal_rtl8367c_vlan_stg_get,
    .vlan_protoAndPortBasedVlan_add = dal_rtl8367c_vlan_protoAndPortBasedVlan_add,
    .vlan_protoAndPortBasedVlan_get = dal_rtl8367c_vlan_protoAndPortBasedVlan_get,
    .vlan_protoAndPortBasedVlan_del = dal_rtl8367c_vlan_protoAndPortBasedVlan_del,
    .vlan_protoAndPortBasedVlan_delAll = dal_rtl8367c_vlan_protoAndPortBasedVlan_delAll,
    .vlan_portFid_set = dal_rtl8367c_vlan_portFid_set,
    .vlan_portFid_get = dal_rtl8367c_vlan_portFid_get,
    .vlan_UntagDscpPriorityEnable_set = dal_rtl8367c_vlan_UntagDscpPriorityEnable_set,
    .vlan_UntagDscpPriorityEnable_get = dal_rtl8367c_vlan_UntagDscpPriorityEnable_get,
    .stp_mstpState_set = dal_rtl8367c_stp_mstpState_set,
    .stp_mstpState_get = dal_rtl8367c_stp_mstpState_get,
    .vlan_reservedVidAction_set = dal_rtl8367c_vlan_reservedVidAction_set,
    .vlan_reservedVidAction_get = dal_rtl8367c_vlan_reservedVidAction_get,
    .vlan_realKeepRemarkEnable_set = dal_rtl8367c_vlan_realKeepRemarkEnable_set,
    .vlan_realKeepRemarkEnable_get = dal_rtl8367c_vlan_realKeepRemarkEnable_get,
    .vlan_reset = dal_rtl8367c_vlan_reset,



};

/*
 * Macro Declaration
 */

/*
 * Function Declaration
 */


/* Module Name    :  */

/* Function Name:
 *      dal_rtl8367c_mapper_get
 * Description:
 *      Get DAL mapper function
 * Input:
 *      None
 * Output:
 *      None
 * Return:
 *      dal_mapper_t *     - mapper pointer
 * Note:
 */
dal_mapper_t *dal_rtl8367c_mapper_get(void)
{

    return &dal_rtl8367c_mapper;
} /* end of dal_rtl8367c_mapper_get */


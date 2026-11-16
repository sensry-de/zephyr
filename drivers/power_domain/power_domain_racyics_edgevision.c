/*
 * Copyright (c) 2026 Racyics GmbH.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #define DT_DRV_COMPAT racyics_edgevision_power_domain
 
 #include <zephyr/pm/device.h>
 #include <zephyr/logging/log.h>
 #include <zephyr/dt-bindings/power/edgevision_power.h>
 #include <soc.h>

 LOG_MODULE_REGISTER(edgevision_power_domain, CONFIG_POWER_DOMAIN_LOG_LEVEL);


 struct ri_pd_config {
    uint32_t domain_id;
 };

/*Internal*/

static int soc_pwrdomain_switch_on(const struct device *dev)
{
    const struct ri_pd_config *cfg = dev->config;
    uint32_t domain_id = cfg->domain_id;

    switch (domain_id) {
    case SOC_PWRDOMAIN_DISPLAY:
        ORREG(DISP_DPHY_DISP_DPHY_INIT_CFG_REG,
              RI_DISP_REG__DISP_DPHY_INIT_CFG__SW_RESET_DCU_N__M |
              RI_DISP_REG__DISP_DPHY_INIT_CFG__SW_RESET_DSI_PIXEL_N__M |
              RI_DISP_REG__DISP_DPHY_INIT_CFG__SW_RESET_DSI_SYS_N__M);
        break;
    case SOC_PWRDOMAIN_CM85:
        // Enable M85 clock
        ORREG(RBB_CTRL_ARM_M85_CLKRST_REG, RBB_REG__CTRL_ARM_M85_CLKRST__EN_CLK__M);
        ORREG(M85_RST_CTRL_REG,            M85_REG__RST_CTRL__SW_RESET_M85_N__M);
        break;
    case SOC_PWRDOMAIN_ETHOS:
        ORREG(ETHOS_REGFILE_RESET_REG_REG,
              ETHOS_REG__RESET_REG__SW_RESET_ETHOS_N__M);
        break;
    case SOC_PWRDOMAIN_ETHERNET:
        ORREG(ETHERNET_REG_SW_RESET_REG,
              ETHERNET_REG__SW_RESET__ETHERNET__M |
              ETHERNET_REG__SW_RESET__REF_FBIST__M);
        break;
    case SOC_PWRDOMAIN_H264:
        ORREG(H264_RESET_CTRL_REG,
              RI_H264_ADDRMAP__RESET_CTRL__RESET_H264_N__M);
        break;
    case SOC_PWRDOMAIN_EMMC:
        // enable clocks that are necessary to access EMMC_QSPI_REG_TOP_CTRL_REG
        ORREG(RBB_CTRL_DIV_CLK_EMMC_REG   , RBB_REG__CTRL_DIV_CLK_EMMC__EN_CLK__M);
        ORREG(RBB_CTRL_DIV_CLK_EMMC_TX_REG, RBB_REG__CTRL_DIV_CLK_EMMC_TX__EN_CLK__M);
        // release emmc toplevel resets
        ORREG(EMMC_QSPI_REG_TOP_CTRL_REG,
            RI_EMMC_TOP_REGFILE__CTRL__RESET_EMMC_N__M |
            RI_EMMC_TOP_REGFILE__CTRL__RESET_EMMC_TIMER_N__M |
            RI_EMMC_TOP_REGFILE__CTRL__RESET_EMMC_TX_N__M);
        break;
    default:
        LOG_ERR("Domain ID not supported for power on: %d", domain_id);
        return -EINVAL;
    }

    pm_device_children_action_run(dev, PM_DEVICE_ACTION_TURN_ON, NULL);
    LOG_DBG("Switched on power domain ID #(%d)", domain_id);

    return 0;
}

 /*External*/

 static int ri_edgevision_pm_action(const struct device *dev, enum pm_device_action action)
{
    LOG_DBG("Edgevision PM action %d", action);
    int ret = 0;
    switch (action) {
        case PM_DEVICE_ACTION_RESUME:
            ret = soc_pwrdomain_switch_on(dev);
        case PM_DEVICE_ACTION_SUSPEND:
            return 0; // TODO
        case PM_DEVICE_ACTION_TURN_ON:
        case PM_DEVICE_ACTION_TURN_OFF:
            break;
        default:
            return -ENOTSUP;
    }
    return ret;
}

static int ri_edgevision_pm_init(const struct device *dev)
{
    int ret;

    ret = pm_device_driver_init(dev, ri_edgevision_pm_action);

    if(ret < 0){
        LOG_ERR("Failed to enable runtime PM: %d", ret);
        return ret;
    }
    return 0;
}

 #define RI_EDGEVISION_PM_DEFINE(inst)                                              \
    static const struct ri_pd_config ri_pd_config_##inst = {                        \
        .domain_id = DT_INST_PROP(inst, domain_id),                                 \
    };                                                                              \
    PM_DEVICE_DT_INST_DEFINE(inst , ri_edgevision_pm_action);                       \
    DEVICE_DT_INST_DEFINE(  inst, ri_edgevision_pm_init,                            \
                            PM_DEVICE_DT_INST_GET(inst),                            \
                            NULL,                                                   \
                            &ri_pd_config_##inst,                                   \
                            POST_KERNEL,                                            \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, NULL);    
 DT_INST_FOREACH_STATUS_OKAY(RI_EDGEVISION_PM_DEFINE)
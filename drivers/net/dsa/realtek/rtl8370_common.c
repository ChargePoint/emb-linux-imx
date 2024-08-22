#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/mutex.h>
#include <linux/of_irq.h>
#include <linux/regmap.h>
#include <linux/if_bridge.h>

#include "realtek.h"
#include <linux/module.h>


static struct realtek_priv *g_priv;

void set_global_priv(struct realtek_priv *priv) {
    g_priv = priv;
}
EXPORT_SYMBOL(set_global_priv);

rtk_int32 rtl8367c_smi_read(rtk_uint32 m_addrs, rtk_uint32 *r_data) {
    u32 val;
    struct realtek_priv *priv = g_priv;
    rtk_uint32 ret = RT_ERR_OK;

    ret = regmap_read(priv->map_nolock, m_addrs, &val);
    *r_data = val & 0xFFFF;
    // if (g_print_flag)
    //     dev_info(priv->dev, "smi_r: 0x%X, d: 0x%X\n", m_addrs, val);
    return ret;
}
EXPORT_SYMBOL(rtl8367c_smi_read);

/*Replace the rtl8367c_smi_write function in rtl8367c_smi.c in the realtek api.*/
rtk_int32 rtl8367c_smi_write(rtk_uint32 m_addrs, rtk_uint32 r_data)
{
	struct realtek_priv *priv = g_priv;
	rtk_uint32 ret = RT_ERR_OK;

	//if (g_print_flag)
	//	dev_info(priv->dev, "smi_w: 0x%X, 0x%X\n", m_addrs, r_data);
	ret = regmap_write(priv->map_nolock, m_addrs, r_data);
	return ret;
}
EXPORT_SYMBOL(rtl8367c_smi_write);

static int __init rtl8370_common_init(void) {
    printk(KERN_INFO "rtl8370mb_common module inserted.\n");
    return 0;
}

static void __exit rtl8370_common_exit(void) {
    printk(KERN_INFO "rtl8370mb_common module removed.\n");
}

module_init(rtl8370_common_init);
module_exit(rtl8370_common_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("jimmy_liu");
MODULE_DESCRIPTION("RTK API based on v1.4.4 Module");
MODULE_VERSION("1.0.0");


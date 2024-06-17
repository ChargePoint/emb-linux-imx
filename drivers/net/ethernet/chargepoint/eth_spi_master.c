#include "linux/delay.h"
#include "linux/skbuff.h"
#include <linux/if_ether.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_arp.h>
#include <linux/if_vlan.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/spi/spi.h>


#define ETH_SPI_DRV_NAME    "chpt-eth-spi"
#define ETH_SPI_FRAG_LEN    (128u)
#define ETH_SPI_MTU         (1600u)
#define TX_RING_MAX_LEN     10
#define TX_RING_MIN_LEN     2

#define ETH_SPI_CLK_SPEED_MIN   (1000000u)
#define ETH_SPI_CLK_SPEED_MAX   (25000000u)
#define ETH_SPI_CLK_SPEED       (1000000u)
static int eth_spi_clkspeed;
module_param(eth_spi_clkspeed, int, 0);
MODULE_PARM_DESC(eth_spi_clkspeed, "SPI bus clock speed (Hz) (1000000 - 25000000).");

struct eth_spi_frame {
    u16 sof;
    u8 type;
    u8 frag_idx : 4;
    u8 frag_tot : 4;
    u16 len;
    u16 reserved_0;
    u8 buf[ETH_SPI_FRAG_LEN];
    u16 eof;
} __packed;

struct eth_spi_stats {
    u64 reset_timeout;
    u64 spi_err;
};

struct tx_ring {
    struct sk_buff *skb[TX_RING_MAX_LEN];
    u16 head;
    u16 tail;
    u16 size;
    u16 count;
};

struct chpt_eth_spi {
    struct net_device* net_dev;
    struct spi_device* spi_dev;
    struct eth_spi_stats* stats;
    struct task_struct *spi_thread;

    struct tx_ring txr;

    u8* rx_buffer;
    struct sk_buff* rx_skb;
};


static int chpt_eth_spi_netdev_init(struct net_device* dev)
{
    struct chpt_eth_spi* espi = netdev_priv(dev);

    dev_info(&espi->spi_dev->dev, "netdev_init");
    
    dev->mtu = ETH_SPI_MTU;
    dev->type = ARPHRD_ETHER;

    memset(&espi->stats, 0, sizeof(struct eth_spi_stats));

    espi->rx_buffer = kmalloc(sizeof(struct eth_spi_frame), GFP_KERNEL);
    if(!espi->rx_buffer)
        return -ENOBUFS;

    espi->rx_skb = netdev_alloc_skb_ip_align(dev, ETH_SPI_MTU + VLAN_ETH_HLEN);
    if(!espi->rx_skb) {
        kfree(espi->rx_buffer);
        netdev_info(espi->net_dev, "Failed to allocation RX sk_buff.\n");
        return -ENOBUFS;
    }

    return 0;
}

static void chpt_eth_spi_netdev_uninit(struct net_device* dev)
{
    struct chpt_eth_spi* espi = netdev_priv(dev);
    dev_info(&espi->spi_dev->dev, "netdev_uninit");

    kfree(espi->rx_buffer);
    dev_kfree_skb(espi->rx_skb);
}

static int chpt_eth_spi_thread(void* data)
{
    struct chpt_eth_spi* espi = data;
    netdev_info(espi->net_dev, "SPI thread created\n");

    while(!kthread_should_stop()) {
        set_current_state(TASK_INTERRUPTIBLE);

        // if nothing to tx:
        //  schedule();
        //
        set_current_state(TASK_RUNNING);

        netdev_dbg(espi->net_dev, "have work to do. int: , tx_skb: %p\n", NULL);

    }

    return 0;
}

static irqreturn_t eth_spi_intr_handler(int irq, void* data)
{
    struct chpt_eth_spi* espi = data;
    
    if(espi->spi_thread)
        wake_up_process(espi->spi_thread);

    return IRQ_HANDLED;
}

static int chpt_eth_spi_netdev_open(struct net_device* dev)
{
    struct chpt_eth_spi* espi = netdev_priv(dev);
    dev_info(&espi->spi_dev->dev, "netdev_open");

    int ret = 0;

    espi->spi_thread = kthread_run(chpt_eth_spi_thread, espi, "%s", dev->name);
    if(IS_ERR(espi->spi_thread)) {
        netdev_err(dev, "%s: unable to start kernel thread.\n", ETH_SPI_DRV_NAME, espi); 
        return PTR_ERR(espi->spi_thread);
    }

    ret = request_irq(espi->spi_dev->irq, eth_spi_intr_handler, 0, dev->name, espi);
    if(ret) {
        netdev_err(dev, "%s: unable to get IRQ %d (irqval=%d).\n", ETH_SPI_DRV_NAME, espi->spi_dev->irq, ret);
        kthread_stop(espi->spi_thread);
        return ret;
    }

    return 0;
}

static int chpt_eth_spi_netdev_stop(struct net_device* dev)
{
    struct chpt_eth_spi* espi = netdev_priv(dev);
    dev_info(&espi->spi_dev->dev, "netdev_stop");

    netif_stop_queue(dev);

    free_irq(espi->spi_dev->irq, espi);

    kthread_stop(espi->spi_thread);
    espi->spi_thread = NULL;

    // flush rx_ring.
    return 0;
}

static netdev_tx_t chpt_eth_spi_netdev_xmit(struct sk_buff* skb, struct net_device* dev)
{
    struct chpt_eth_spi* espi = netdev_priv(dev);
    dev_info(&espi->spi_dev->dev, "netdev_xmit");
    if(espi->spi_thread)
        wake_up_process(espi->spi_thread);


    return NETDEV_TX_OK;
}

static void chpt_eth_spi_netdev_tx_timeout(struct net_device* dev, unsigned int txqueue)
{
    struct chpt_eth_spi* espi = netdev_priv(dev);
    dev_info(&espi->spi_dev->dev, "netdev_tx_timeout");

    if(espi->spi_thread)
        wake_up_process(espi->spi_thread);
}

static const struct net_device_ops chpt_eth_spi_netdev_ops = {
    .ndo_init = chpt_eth_spi_netdev_init,
    .ndo_uninit = chpt_eth_spi_netdev_uninit,
    .ndo_open = chpt_eth_spi_netdev_open,
    .ndo_stop = chpt_eth_spi_netdev_stop,
    .ndo_start_xmit = chpt_eth_spi_netdev_xmit,
    .ndo_set_mac_address = eth_mac_addr,
    .ndo_tx_timeout = chpt_eth_spi_netdev_tx_timeout,
    .ndo_validate_addr = eth_validate_addr,
};

static const struct ethtool_ops chpt_eth_spi_ethtool_ops = {
};

static void chpt_eth_spi_netdev_setup(struct net_device *dev) {

    struct chpt_eth_spi* espi = NULL;

    dev->netdev_ops = &chpt_eth_spi_netdev_ops;
    dev->ethtool_ops = &chpt_eth_spi_ethtool_ops;
    //
    dev->watchdog_timeo = 250;
    dev->priv_flags &= ~IFF_TX_SKB_SHARING;
    dev->tx_queue_len = 100;

    dev->min_mtu = ETH_SPI_MTU;
    dev->max_mtu = ETH_SPI_MTU;

    espi = netdev_priv(dev);
    dev_info(&espi->spi_dev->dev, "netdev_setup");

    memset(espi, 0, sizeof(*espi));
    dev_info(&espi->spi_dev->dev, "netdev_setup :: Done!!");
    
};

static const struct of_device_id chpt_eth_spi_of_match[] = {
    { .compatible = "chpt,eth-spi" },
    { /* sentinel */ }
};

static int chpt_eth_spi_probe(struct spi_device* spi)
{
    struct chpt_eth_spi *espi = NULL;
    struct net_device* espi_netdev = NULL;

    dev_info(&spi->dev, "Probing ChargePoint Eth SPI device...");

    if(!spi->dev.of_node) {
        dev_err(&spi->dev, "Missing device tree\n");
        return -EINVAL;
    }
    
    if(eth_spi_clkspeed == 0) {
        if(spi->max_speed_hz)
            eth_spi_clkspeed = spi->max_speed_hz;
        else
            eth_spi_clkspeed = ETH_SPI_CLK_SPEED;
    }

    if((eth_spi_clkspeed < ETH_SPI_CLK_SPEED_MIN) || 
    (eth_spi_clkspeed > ETH_SPI_CLK_SPEED_MAX)) {
        dev_err(&spi->dev, "Invalid clkspeed: %d\n", eth_spi_clkspeed);
        return -EINVAL;
    }

    dev_info(&spi->dev, "clkspeed=%d\n", eth_spi_clkspeed);

    spi->mode = SPI_MODE_3;
    spi->max_speed_hz = eth_spi_clkspeed;
    if(spi_setup(spi) < 0) {
        dev_err(&spi->dev, "Unable to setup SPI device\n");
        return -EFAULT;
    }

    espi_netdev = alloc_etherdev(sizeof(struct chpt_eth_spi));
    if(!espi_netdev)
        return -ENOMEM;

    chpt_eth_spi_netdev_setup(espi_netdev);
    espi = netdev_priv(espi_netdev);

    espi->net_dev = espi_netdev;
    espi->spi_dev = spi;

    spi_set_drvdata(spi, espi_netdev);

    int ret = of_get_ethdev_address(spi->dev.of_node, espi->net_dev);
    if(ret) {
        eth_hw_addr_random(espi->net_dev);
        dev_info(&spi->dev, "Using random MAC address: %pM\n", espi->net_dev->dev_addr);
    }

    netif_carrier_off(espi_netdev);

    espi->net_dev->ethtool_ops = &chpt_eth_spi_ethtool_ops;

    if(register_netdev(espi_netdev)) {
        dev_err(&spi->dev, "Unable to register net device %s\n", espi_netdev->name);
        free_netdev(espi_netdev);
        return -EFAULT;
    }
    // init debugfs

    return 0;
}

static void chpt_eth_spi_remove(struct spi_device* spi)
{
    struct net_device* espi_devs = spi_get_drvdata(spi);
    //struct chpt_eth_spi* espi = netdev_priv(espi_devs);

    // remove debugfs
    //
    unregister_netdev(espi_devs);
    free_netdev(espi_devs);

}

static const struct spi_device_id chpt_eth_spi_id[] = {
    {"eth-spi", 0 },
    { /* sentinel */ }
};

static struct spi_driver chpt_eth_spi_driver = {
    .driver = {
        .name = ETH_SPI_DRV_NAME,
        .of_match_table = chpt_eth_spi_of_match,
    },
    .id_table = chpt_eth_spi_id,
    .probe = chpt_eth_spi_probe,
    .remove = chpt_eth_spi_remove,
};
module_spi_driver(chpt_eth_spi_driver);

MODULE_DESCRIPTION("ChargePoint Eth SPI Driver");
MODULE_AUTHOR("James Walmsley <james.walmsley@chargepoint.com");
MODULE_LICENSE("GPL");

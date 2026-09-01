#include "mv_net.h"
#include "MvErrorDefine.h"
#include <linux/version.h>

#define MIN_MTU         68
#define MAX_MTU         9000
#define DEFAULT_MTU     1500

struct net_device_ops netdev_ops = {
	.ndo_start_xmit		= MV_NetDeviceSendPkt,
    .ndo_change_mtu		= MV_NetDeviceChangeMTU, //mtu为1500存在负载过高丢包问题，不支持修改
};

struct ethtool_ops netdev_ethtool_ops = {
    .get_link = MV_GetLink,     //获取链路状态
};

int MV_NetDeviceSendPkt(struct sk_buff* skb, struct net_device* dev)
{
    //获取封装的私有指针
    PMV_POINTER_CONTAINER pstPrivate = MV_NETDEV_PRIV(dev);
    dev->stats.tx_packets++;
    dev->stats.tx_bytes += skb->len;
    NetDeviceSendPkt(skb->data, skb->len, skb, pstPrivate);
    return 0;
}

int MV_NetDeviceChangeMTU(struct net_device* dev, int new_mtu)
{
    if(new_mtu < MIN_MTU || new_mtu > MAX_MTU)
    {   
        return -ENAVAIL;
    }

    dev->mtu = new_mtu;

    return 0;
}

unsigned int MV_GetLink(struct net_device* dev)
{
    // 获取私有指针
    PMV_POINTER_CONTAINER pstPrivate = MV_NETDEV_PRIV(dev);
    if(NULL == pstPrivate)
    {
        return 0;
    }

    return NetGetLink(pstPrivate);
}

void MV_SkbDestructor(struct sk_buff* skb)
{
    void*   pstPrivate = NULL;
    pstPrivate = MV_GetSkbPrivate(skb);
    if(NULL == pstPrivate)
    {
        return;
    }

    NetResetRxNode(pstPrivate);
    return;
}

extern int MV_AllocNetDev(PMV_NET_DEVICE pstNetDevice, char* chDeviceName)
{
    struct net_device*      dev = NULL;
    PMV_POINTER_CONTAINER   pstPrivate = NULL;

    if(NULL == pstNetDevice)
    {
        return -ENAVAIL;
    }

    pstNetDevice->pNetDevice = alloc_netdev(sizeof(MV_POINTER_CONTAINER), chDeviceName, NET_NAME_USER, ether_setup);
    if(NULL == pstNetDevice->pNetDevice)
    {
        return -ENAVAIL;
    }

    dev = (struct net_device*)pstNetDevice->pNetDevice;

    pstPrivate = MV_NETDEV_PRIV(dev);

    pstPrivate->pPrivate = (void*)pstNetDevice->pstPort;

    return MV_OK;
}

extern void MV_FreeNetDev(PMV_NET_DEVICE pstNetDevice)
{
    if(NULL == pstNetDevice || NULL == pstNetDevice->pNetDevice)
    {
        return;
    }

    free_netdev((struct net_device*)pstNetDevice->pNetDevice);
}

extern int MV_RegisterNetDev(PMV_NET_DEVICE pstNetDevice)
{
    struct net_device*      dev = NULL; 
    int                     ret = 0;
    unsigned int            i = 0;
    
    if(NULL == pstNetDevice || NULL == pstNetDevice->pNetDevice)
    {
        return -ENAVAIL;
    }

    dev = pstNetDevice->pNetDevice;

    //初始化net_device相关字段，主要包含发包函数，mac地址等
    dev->netdev_ops = &netdev_ops;
    dev->ethtool_ops = &netdev_ethtool_ops;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,15,0)
    eth_hw_addr_set(dev, pstNetDevice->chMacAddr);
#else
    for(i = 0; i < MAC_ADDR_LEN; ++i)
    {
        dev->dev_addr[i] = pstNetDevice->chMacAddr[i];
    }
#endif

    //dev->flags |= IFF_NOARP;
    dev->flags |= IFF_BROADCAST;
    dev->max_mtu = MAX_MTU;
    dev->min_mtu = MIN_MTU;
    dev->mtu    = DEFAULT_MTU; //默认8192mtu，不支持修改
    //dev->priv_flags |= IFF_PROMISC;

    ret = register_netdev(dev);

    //注册给内核
    return ret;
}

extern void MV_UnRegisterNetDev(PMV_NET_DEVICE pstNetDevice)
{
    struct net_device*      dev = NULL;

    if(NULL == pstNetDevice || NULL == pstNetDevice->pNetDevice)
    {
        return;
    }

    dev = pstNetDevice->pNetDevice;

    unregister_netdev(dev);
}

extern void MV_NetifCarrierOn(PMV_NET_DEVICE pstNetDevice)
{
    struct net_device*      dev = NULL;

    if(NULL == pstNetDevice || NULL == pstNetDevice->pNetDevice)
    {
        return;
    }

    dev = pstNetDevice->pNetDevice;

    netif_carrier_on(dev);
}

extern void MV_NetifStartQueue(PMV_NET_DEVICE pstNetDevice)
{
    struct net_device*      dev = NULL;

    if(NULL == pstNetDevice || NULL == pstNetDevice->pNetDevice)
    {
        return;
    }

    dev = pstNetDevice->pNetDevice;

    netif_start_queue(dev);
}

extern void MV_NetifCarrierOff(PMV_NET_DEVICE pstNetDevice)
{
    struct net_device*      dev = NULL;

    if(NULL == pstNetDevice || NULL == pstNetDevice->pNetDevice)
    {
        return;
    }

    dev = pstNetDevice->pNetDevice;

    netif_carrier_off(dev);
}

extern void MV_NetifStopQueue(PMV_NET_DEVICE pstNetDevice)
{
    if(NULL == pstNetDevice || NULL == pstNetDevice->pNetDevice)
    {
        return;
    }   
    else
    {
        return netif_stop_queue((struct net_device*)pstNetDevice->pNetDevice);
    }
}

extern void MV_DevKfreeSkb(PMV_SK_BUFFER pstSkBuffer)
{
    if(NULL == pstSkBuffer || NULL == pstSkBuffer->pSkBuffer)
    {
        return;
    } 
    else
    {
        return dev_kfree_skb((struct sk_buff*)pstSkBuffer->pSkBuffer);
    }
}

extern void MV_NetifWakeQueue(PMV_NET_DEVICE pstNetDevice)
{
    if(NULL == pstNetDevice || NULL == pstNetDevice->pNetDevice)
    {
        return;
    }  
    else
    {
        return netif_wake_queue((struct net_device*)pstNetDevice->pNetDevice);
    }
}

/*获取有效载荷buffer 只处理ip udp协议*/ 
extern unsigned char* MV_GetSkbPayload(PMV_SK_BUFFER pstSkBuffer)
{
    struct sk_buff* skb;
    struct ethhdr*  eth_header;
    int             eth_len;
    struct iphdr*   ip_header;
    int             ip_len;
    struct udphdr*  udp_header;

    if(NULL == pstSkBuffer || NULL == pstSkBuffer->pSkBuffer)
    {
        return NULL;
    }

    skb = (struct sk_buff*)pstSkBuffer->pSkBuffer;

    eth_header = (struct ethhdr*)skb->data;
    eth_len = sizeof(struct ethhdr);
    if(eth_header->h_proto == htons(ETH_P_IP))
    {
        ip_header = (struct iphdr*)(skb->data + eth_len);
        ip_len = ip_header->ihl * 4;
        if(ip_header->protocol == IPPROTO_UDP)
        {
            udp_header = (struct udphdr*)((unsigned char*)ip_header + ip_len);
            return (unsigned char*)udp_header + 8;
        }
        else
        {
            return NULL;
        }
    }
    else
    {
        return NULL;
    }
}

extern int MV_GetHeaderLen(unsigned char* pRxBuffer, unsigned int* pHeadLen)
{
    struct ethhdr*   eth_header;
    struct iphdr*    ip_header;
    struct ipv6hdr*  ipv6_header;
    struct udphdr*   udp_header;
    struct tcphdr*   tcpheader;

    if(NULL == pRxBuffer || NULL == pHeadLen)
    {
        return -EINVAL;
    }

    *pHeadLen = 0;

    //以太网：mac层头部
    eth_header = (struct ethhdr*)pRxBuffer;
    *pHeadLen += ETH_HLEN;

    switch (ntohs(eth_header->h_proto))
    {
    case ETH_P_IP: //ipv4
        ip_header = (struct iphdr*)((unsigned char*)pRxBuffer + (*pHeadLen));
        *pHeadLen += ip_header->ihl * 4;

        if(ip_header->protocol == IPPROTO_UDP)
        {
            udp_header = (struct udphdr*)((unsigned char*)pRxBuffer + (*pHeadLen));
            *pHeadLen += sizeof(*udp_header);
        }
        else if(ip_header->protocol == IPPROTO_TCP)
        {
            tcpheader = (struct tcphdr*)((unsigned char*)pRxBuffer + (*pHeadLen));
            *pHeadLen += tcpheader->doff * 4;
        }
        else
        {
            return -ERANGE;
        }
        break;

    case ETH_P_IPV6: //ipv6
        ipv6_header = (struct ipv6hdr*)(pRxBuffer + (*pHeadLen));
        *pHeadLen += sizeof(*ipv6_header);

        //扩展头部
        if(ipv6_header->nexthdr == IPPROTO_UDP)
        {
            udp_header = (struct udphdr*)((unsigned char*)pRxBuffer + (*pHeadLen));
            *pHeadLen += sizeof(*udp_header);
        }
        else if(ipv6_header->nexthdr == IPPROTO_TCP)
        {
            tcpheader = (struct tcphdr*)((unsigned char*)pRxBuffer + (*pHeadLen));
            *pHeadLen += tcpheader->doff * 4;
        }
        else
        {
            return -ERANGE;
        }

        break;
    default:
        return -ERANGE;
    }

    return MV_OK;
}

extern int MV_DevAllocSkb(PMV_SK_BUFFER pstSkBuffer, unsigned int nLen)
{
    struct sk_buff* skb;
    
    if(NULL == pstSkBuffer)
    {
        return -EINVAL;
    }

    skb = dev_alloc_skb(nLen);
    if(NULL == skb)
    {
        return -EINVAL;
    }

    pstSkBuffer->pSkBuffer = skb;
    pstSkBuffer->pBuffer = skb->data;
    pstSkBuffer->nLen = nLen;

    return MV_OK;
}

extern int MV_SetSkbDestructor(PMV_SK_BUFFER pstSkBuffer, void (*destructor)(struct sk_buff *skb))
{
    struct sk_buff* skb;

    if(NULL == pstSkBuffer || NULL == pstSkBuffer->pSkBuffer)
    {
        return -EINVAL;
    }

    skb = pstSkBuffer->pSkBuffer;

    skb->destructor = destructor;

    return MV_OK;
}

extern int MV_SetSkbPrivate(PMV_SK_BUFFER pstSkBuffer, void* pstPrivate)
{
    struct sk_buff*             skb;
	struct skb_shared_info*     shinfo;

    if(NULL == pstSkBuffer || NULL == pstSkBuffer->pSkBuffer)
    {
        return -EINVAL;
    }

    skb = pstSkBuffer->pSkBuffer;
	shinfo = skb_shinfo(skb);

    shinfo->destructor_arg = pstPrivate;

    return MV_OK;
}

extern void* MV_GetSkbPrivate(struct sk_buff *skb)
{
	struct skb_shared_info*     shinfo;

    if(NULL == skb )
    {
        return NULL;
    }

    shinfo = skb_shinfo(skb);

    return shinfo->destructor_arg;
}

extern int MV_SkbPut(PMV_SK_BUFFER pstSkBuffer, unsigned char* pRxBuffer, unsigned int nLen)
{
    struct sk_buff* skb;
    struct net_device* dev;

    if(NULL == pstSkBuffer || NULL == pstSkBuffer->pSkBuffer || nLen > pstSkBuffer->nLen || NULL == pstSkBuffer->pNetDevice)
    {
        return -EINVAL;
    }

    skb = pstSkBuffer->pSkBuffer;
    dev = pstSkBuffer->pNetDevice;

    memcpy(skb_put(skb, pstSkBuffer->nLen), pRxBuffer, nLen);
    skb->protocol = eth_type_trans(skb, dev);

    return MV_OK;
}

extern void MV_SkbAddRxFrag(PMV_SK_BUFFER pstSkBuffer, void* pstPage, unsigned int nPageOffset, unsigned int nBlockSize)
{
    struct  sk_buff*    skb = NULL;

    if(NULL == pstSkBuffer || NULL == pstSkBuffer->pSkBuffer)
    {
        return;
    }

    skb = pstSkBuffer->pSkBuffer;

    return skb_add_rx_frag(skb, skb_shinfo(skb)->nr_frags, (struct page*)pstPage, nPageOffset, nBlockSize, nBlockSize);
}

extern int MV_NetifRx(PMV_SK_BUFFER pstSkBuffer)
{
    struct sk_buff* skb;
    struct net_device* dev;

    if(NULL == pstSkBuffer || NULL == pstSkBuffer->pSkBuffer || NULL == pstSkBuffer->pNetDevice)
    {
        return -EINVAL;
    }

    skb = pstSkBuffer->pSkBuffer;
    dev = pstSkBuffer->pNetDevice;

    skb->dev = dev;

    dev->stats.rx_packets++;
    dev->stats.rx_bytes += pstSkBuffer->nLen;

    // skb交由协议栈释放
    return netif_rx(skb);
}

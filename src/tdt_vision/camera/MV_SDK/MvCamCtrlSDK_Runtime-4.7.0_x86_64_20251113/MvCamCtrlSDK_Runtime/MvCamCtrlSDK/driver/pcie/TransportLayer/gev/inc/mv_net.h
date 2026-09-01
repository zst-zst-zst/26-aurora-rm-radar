#ifndef __MV_NET_H__
#define __MV_NET_H__

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>
#include <linux/if_ether.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <linux/udp.h>

#define MAC_ADDR_LEN                            6
#define MV_NETDEV_PRIV(dev)                netdev_priv(dev)

typedef struct _MV_POINTER_CONTAINER_
{
    void*          pPrivate;
}MV_POINTER_CONTAINER, *PMV_POINTER_CONTAINER;

typedef struct _MV_NET_DEVICE_
{
    void*           pNetDevice;
    void*           pstPort;        //与网络设备关联的port
    unsigned char   chMacAddr[MAC_ADDR_LEN];
}MV_NET_DEVICE, *PMV_NET_DEVICE;

typedef struct _MV_SK_BUFFER_
{
    void*               pSkBuffer;
    unsigned char*      pBuffer;    //包含协议头的buffer
    unsigned int        nLen;       //包含协议头的buffer长度
    void*               pNetDevice; //绑定的内核net_dev指针
}MV_SK_BUFFER, *PMV_SK_BUFFER;

/*修改网络设备的mtu属性*/
int MV_NetDeviceChangeMTU(struct net_device* dev, int new_mtu);
/*获取网络设备的link状态*/
unsigned int MV_GetLink(struct net_device* dev);
extern unsigned int NetGetLink(void* pstPrivate);
/*创建网络设备*/
extern int MV_AllocNetDev(PMV_NET_DEVICE pstNetDevice, char* chDeviceName);
/*销毁网络设备*/
extern void MV_FreeNetDev(PMV_NET_DEVICE pstNetDevice);
/*向内核注册网络设备*/
extern int MV_RegisterNetDev(PMV_NET_DEVICE pstNetDevice);
/*向内核解注册网络设备*/
extern void MV_UnRegisterNetDev(PMV_NET_DEVICE pstNetDevice);
/*通知内核激活链路*/
extern void MV_NetifCarrierOn(PMV_NET_DEVICE pstNetDevice);      
/*通知内核链路失效*/
extern void MV_NetifCarrierOff(PMV_NET_DEVICE pstNetDevice);  

/*发包*/
/*发包回调，捕获协议栈发出的数据*/
int MV_NetDeviceSendPkt(struct sk_buff* skb, struct net_device* dev);
extern void NetDeviceSendPkt(void* pSendBuffer, unsigned int nSendLen, void* pSkBuffer, void* pstPrivate);
/*阻止上层向驱动层继续发送数据包*/
extern void MV_NetifStopQueue(PMV_NET_DEVICE pstNetDevice);    
/*通知上层向驱动层继续发送数据包*/
extern void MV_NetifStartQueue(PMV_NET_DEVICE pstNetDevice);    
/*释放发送sk_buff缓冲区*/
extern void MV_DevKfreeSkb(PMV_SK_BUFFER pstSkBuffer);    
/*数据发送完成，唤醒网卡队列*/
extern void MV_NetifWakeQueue(PMV_NET_DEVICE pstNetDevice);    
/*获取有效载荷buffer*/
extern unsigned char* MV_GetSkbPayload(PMV_SK_BUFFER pstSkBuffer);   

/*收包*/
/*内核分配sk_buff*/
extern int MV_DevAllocSkb(PMV_SK_BUFFER pstSkBuffer, unsigned int nLen);
/*获取网络包头包长度（支持解析ipv4,ipv6），其余类型返回不支持*/
extern int MV_GetHeaderLen(unsigned char* pRxBuffer, unsigned int* nHeadLen);
/*注册sk_buff的析构回调*/
extern void MV_SkbDestructor(struct sk_buff* skb);
extern int MV_SetSkbDestructor(PMV_SK_BUFFER pstSkBuffer, void (*destructor)(struct sk_buff *skb));
/*重置sk_buff关联的收包结点*/
extern void NetResetRxNode(void* pstPrivate);
/*设置sk_buff的私有指针*/
extern int MV_SetSkbPrivate(PMV_SK_BUFFER pstSkBuffer, void* pstPrivate);
/*获取sk_buff的私有指针*/
extern void* MV_GetSkbPrivate(struct sk_buff *skb);
/*sk_buff线性区域处理*/
extern int MV_SkbPut(PMV_SK_BUFFER pstSkBuffer, unsigned char* pRxBuffer, unsigned int nLen);
/*sk_buff非线性区域处理*/
extern void MV_SkbAddRxFrag(PMV_SK_BUFFER pstSkBuffer, void* pstPage, unsigned int nPageOffset, unsigned int nBlockSize);
extern int MV_NetifRx(PMV_SK_BUFFER pstSkBuffer);
#endif  // __MV_NET_H__

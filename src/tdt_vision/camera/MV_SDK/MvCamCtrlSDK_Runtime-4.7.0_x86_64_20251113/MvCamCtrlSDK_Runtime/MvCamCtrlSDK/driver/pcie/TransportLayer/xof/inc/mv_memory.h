
#ifndef     __MEMORY_H__
#define     __MEMORY_H__ 

#include  <linux/types.h>  // 包含uint64_t

typedef struct _MV_MEMORYPAGELIST_
{
    uint64_t  		PageCount;       // page的数量
    void**			PageList;        // page的列表指针
    uint64_t  		OffsetPage;      // Buf在首个page中的偏移量
    void*      		KernelAddress;   // 内核态中的虚拟地址
    void*      		UserAddress;     // 用户态中的虚拟地址
    uint64_t   		Size;            // Buf的总大小
    unsigned long  	hMm;             // 所属进程内存空间

    uint64_t  		nUnSgMapPageCount;
    uint64_t  		nSgMapPageCount;
    uint64_t  		nNeedMapPageCount;
    uint64_t   		nUnSgMapSize;
    uint64_t   		nSgMapSize;
    uint64_t   		nLastPageSize; // 分块传输时记录上一块最后一页大小
} MV_MEMORYPAGELIST;


#ifdef __cplusplus
extern "C"
{
#endif 

//#include "misc.h"

typedef struct __TEST__
{
    int a;
    int b;
}MV_TEST;

extern void* MV_KMalloc(int aSize);
extern void* MV_KMallocAtomic(int aSize);
extern void  MV_KFree( void* aAddress );
extern void* MV_VMalloc(int aSize);
extern void  MV_VFree(void* aAddress);
extern void  MV_Memset(void* aAddress, unsigned int aValue, int aSize);
extern void  MV_Memcpy(void* aDestination, const void* aSource, int aSize);
extern unsigned int  MV_MemcpyToUser( void* aTo, const void* aFrom, int aSize );
extern unsigned int  MV_MemcpyFromUser(void* aTo, const void* aFrom, int aSize );
extern unsigned int  test(MV_TEST myTest, int e);
extern int MV_MallocPages(MV_MEMORYPAGELIST* pPageList, unsigned int nSize);
extern int MV_FreePages(MV_MEMORYPAGELIST* pPageList);
extern unsigned int MV_GetVmaSize(void* vma);

extern void* MV_KZalloc(int aSize);
extern void* MV_KZallocAtomic(int aSize);

extern int MV_Strlen(const char *str);


#ifdef __cplusplus
}
#endif 


#endif		//	__MEMORY_H__


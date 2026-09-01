#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include "asm-generic/io.h"

void MV_IOWRITE32(unsigned int value, /*volatile*/ void* addr)
{
    if (NULL == addr)
    {
        return;
    }
    
    iowrite32(value, addr);
}


unsigned int MV_IOREAD32(/*volatile*/ void* addr)
{
    if (NULL == addr)
    {
        return 0;
    }
    
    return ioread32(addr);
}
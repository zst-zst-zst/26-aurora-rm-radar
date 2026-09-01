
#ifndef __MV_MODULE_H__
#define __MV_MODULE_H__

#define MV_MODULE_LONG_NAME "Neu Gev Universal Pro For Ethernet"
#define MV_MODULE_NAME      "neugevfilter"
#define MV_COMPANY          "NA"
#define MV_COPYRIGHT        "GPL"
#define MV_VERSION          "4.4.0.1 build 20240422"

extern int ModuleRelease(void * pFile);
extern long ModuleRead(/*unsigned*/ char *pBuffer, unsigned long nCount);
extern long ModuleWrite(void * pFile, const char *pBuffer, unsigned long nCount);
extern long ModuleIoctl(void* pFile, unsigned int nCmd, void* nArg);
extern unsigned int ModuleHookPoint(void *pBuf);
extern int InitMemory(void);
extern int CleanMemory(void);

extern int ModuleMmap(void* , void*);

extern int InitModuleDevice(void** ppOutClassForDev, void** ppOutDevNode, const void* pMvDevMajor);

extern int DeinitModuleDevice(void);

#endif  // __MV_MODULE_H__

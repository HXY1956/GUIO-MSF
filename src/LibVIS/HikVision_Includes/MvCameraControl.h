
#ifndef _MV_CAMERA_CTRL_H_
#define _MV_CAMERA_CTRL_H_

#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvObsoleteInterfaces.h"

#ifndef MV_CAMCTRL_API

#if (defined (_WIN32) || defined(WIN64))
#if defined(MV_CAMCTRL_EXPORTS)
#define MV_CAMCTRL_API __declspec(dllexport)
#else
#define MV_CAMCTRL_API __declspec(dllimport)
#endif
#else
#ifndef __stdcall
#define __stdcall
#endif

#ifndef MV_CAMCTRL_API
#define  MV_CAMCTRL_API
#endif
#endif

#endif

#ifdef MV_CAMCTRL_API

#if (defined (_WIN32) || defined(WIN64))
	#if defined(MV_CAMCTRL_EXPORTS)
		#define MV_CAMCTRL_API __declspec(dllexport)
	#else
		#define MV_CAMCTRL_API __declspec(dllimport)
	#endif
	#else
		#ifndef __stdcall
			#define __stdcall
		#endif

		#if defined(MV_CAMCTRL_EXPORTS)
			#define  MV_CAMCTRL_API __attribute__((visibility("default")))
		#else
			#define  MV_CAMCTRL_API
		#endif
	#endif

#endif

#ifndef IN
    #define IN
#endif

#ifndef OUT
    #define OUT
#endif

#ifdef __cplusplus
extern "C" {
#endif 


MV_CAMCTRL_API int __stdcall MV_CC_Initialize();

MV_CAMCTRL_API int __stdcall MV_CC_Finalize();

MV_CAMCTRL_API unsigned int __stdcall MV_CC_GetSDKVersion();

MV_CAMCTRL_API int __stdcall MV_CC_EnumDevices(IN unsigned int nTLayerType, IN OUT MV_CC_DEVICE_INFO_LIST* pstDevList);

MV_CAMCTRL_API int __stdcall MV_CC_EnumDevicesEx(IN unsigned int nTLayerType, IN OUT MV_CC_DEVICE_INFO_LIST* pstDevList, IN const char* strManufacturerName);

MV_CAMCTRL_API int __stdcall MV_CC_EnumDevicesEx2(IN unsigned int nTLayerType, IN OUT MV_CC_DEVICE_INFO_LIST* pstDevList, IN const char* strManufacturerName, IN MV_SORT_METHOD enSortMethod);

MV_CAMCTRL_API bool __stdcall MV_CC_IsDeviceAccessible(IN MV_CC_DEVICE_INFO* pstDevInfo, IN unsigned int nAccessMode);

MV_CAMCTRL_API int __stdcall MV_CC_CreateHandle(IN OUT void ** handle, IN const MV_CC_DEVICE_INFO* pstDevInfo);

MV_CAMCTRL_API int __stdcall MV_CC_DestroyHandle(IN void * handle);

#ifndef __cplusplus
MV_CAMCTRL_API int __stdcall MV_CC_OpenDevice(IN void* handle, IN unsigned int nAccessMode, IN unsigned short nSwitchoverKey);
#else
MV_CAMCTRL_API int __stdcall MV_CC_OpenDevice(IN void* handle, IN unsigned int nAccessMode = MV_ACCESS_Exclusive, IN unsigned short nSwitchoverKey = 0);
#endif

MV_CAMCTRL_API int __stdcall MV_CC_CloseDevice(IN void* handle);

MV_CAMCTRL_API bool __stdcall MV_CC_IsDeviceConnected(IN void* handle);

MV_CAMCTRL_API int __stdcall MV_CC_RegisterImageCallBackEx(IN void* handle, 
                                                         IN void(__stdcall* cbOutput)(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser), IN void* pUser);
MV_CAMCTRL_API int __stdcall MV_CC_StartGrabbing(IN void* handle);

MV_CAMCTRL_API int __stdcall MV_CC_StopGrabbing(IN void* handle);

MV_CAMCTRL_API int __stdcall MV_CC_GetImageBuffer(IN void* handle, IN OUT MV_FRAME_OUT* pstFrame, IN unsigned int nMsec);

MV_CAMCTRL_API int __stdcall MV_CC_FreeImageBuffer(IN void* handle, IN MV_FRAME_OUT* pstFrame);

/********************************************************************//**
 *  @~english
 *  @brief  Timeout mechanism is used to get image, and the SDK waits inside until the data is returned
 *  @param  handle                      [IN]            Device handle
 *  @param  pData                       [IN][OUT]       Image data receiving buffer
 *  @param  nDataSize                   [IN]            Buffer size
 *  @param  pstFrameInfo                [IN][OUT]       Image information structure
 *  @param  nMsec                       [IN]            Waiting timeout
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Before calling this API to get image data frame, call MV_CC_StartGrabbing to start image acquisition.
             This API can get frame data actively, the upper layer program should control the frequency of calling this API according to the frame rate.
             This API supports setting timeout, SDK will wait to return until data appears. This function will increase the streaming stability, which can be used in the situation with high stability requirement.
             Both the USB3Vision and GIGE camera can support this API.
             This API is not supported by CameraLink device.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOneFrameTimeout(IN void* handle, IN OUT unsigned char* pData , IN unsigned int nDataSize, IN OUT MV_FRAME_OUT_INFO_EX* pstFrameInfo, IN unsigned int nMsec);

/********************************************************************//**
 
 *  @~english
 *  @brief  if Image buffers has retrieved the data��Clear them
 *  @param  handle                      [IN]            Device handle
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface allows user to clear the unnecessary images from the buffer memory without stopping acquisition.
             This interface allows user to clear previous data after switching from continuous mode to trigger mode. 
             This interface can only clear the image cache inside the SDK, and the cache in the Frame grabber cannot be cleared.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ClearImageBuffer(IN void* handle);

/********************************************************************//**
 
 *  @~english
 *  @brief  Get the number of valid images in the current image buffer
 *  @param  handle                      [IN]            Device handle
 *  @param  pnValidImageNum             [IN][OUT]       The number of valid images in the current image buffer
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface only counts the number of valid images inside the SDK, not including the number of valid images in the capture card cache.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetValidImageNum(IN void* handle, IN OUT unsigned int *pnValidImageNum);

/********************************************************************//**
 *  @~english
 *  @brief  Display one frame image
 *  @param  handle                      [IN]            Device handle
 *  @param  hWnd                        [IN]            HWND
 *  @param  pstDisplayInfo              [IN]            Frame Info
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks The rendering supports width and height to int type.
 *           When the render mode is D3D, the maximum resolution supported is 16384 * 163840.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DisplayOneFrameEx(IN void* handle, IN void* hWnd, IN MV_DISPLAY_FRAME_INFO_EX* pstDisplayInfo);

/********************************************************************//**
 *  @~english
 *  @brief  Display one frame image
 *  @param  handle                      [IN]            Device handle
 *  @param  hWnd                        [IN]            HWND
 *  @param  pstImage                    [IN]            Frame Info
 *  @param  enRenderMode                [IN]            Render mode, Windows:0-GDI 1-D3D 2-OpenGL Linux:0-OpenGL
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks OpenGL rendering mode can be selected, supporting three pixel formats: PixelType_Gvsp_RGB8_Packed��PixelType_Gvsp_BGR8_Packed��and PixelType_Gvsp_Mono8 for rendering images with a size exceeding 4GB. 
             Note that, other rendering modes are not supported.
			 If the image size does not exceed 4GB, the rendering supports width and height to int type.
			 When the render mode is D3D, the maximum resolution supported is 16384 * 163840.
             When calling, the value of nImageLen in the MV_CC_IMAGE structure needs to be input.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DisplayOneFrameEx2(IN void* handle, IN void* hWnd, IN MV_CC_IMAGE* pstImage, unsigned int enRenderMode);

/********************************************************************//**
 *  @~english
 *  @brief  Set the number of the internal image cache nodes in SDK, Greater than or equal to 1, to be called before the capture
 *  @param  handle                      [IN]            Device handle
 *  @param  nNum                        [IN]            Image Node Number
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Call this interface to set the number of SDK internal image buffer nodes. The interface should be called before calling MV_CC_StartGrabbing for capturing. 
			 Due to different stream retrieval methods, different cameras default to different cache nodes by default when not calling the MV_CC_SetImageNodeNum interface. For example, for dual U internal allocation, the default number of cache nodes is 3
             The actual number of nodes allocated by the SDK = the number of pre allocated nodes within the SDK + the number of nodes allocated by the user (MV_CC_SetImageNodeNum)
             If the system memory resources are insufficient, the SDK will recalculate and use it as the actual number of nodes.
             This interface does not support devices of type MV_CAMERALINK_DEVICE
             This interface is only valid for the SDK's internal allocation cache mode, and the external allocation cache mode (i.e., calling MV_CC_RegisterBuffer) is invalid;
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetImageNodeNum(IN void* handle, IN unsigned int nNum);

/********************************************************************//**
 *  @~english
 *  @brief  Set Grab Strategy
 *  @param  handle                      [IN]            Device handle
 *  @param  enGrabStrategy              [IN]            The value of Grab Strategy
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is set by four image acquisition approaches, the user may choose one as needed. Specific details are as followed: 
                OneByOne:Obtain image from output cache list frame by frame in order, this function is default strategy when device is on.
                LatestImagesOnly:Obtain the latest image from output cache list only, meanwhile clear output cache list.
                LatestImages:Obtain the latest OutputQueueSize image from output cache list, the range of OutputQueueSize is 1-ImageNodeNum, 
                          the user may set the value of MV_CC_SetOutputQueueSizeinterface,the default value of ImageNodeNum is 1,
                          If the user usesMV_CC_SetImageNodeNuminterface to set up OutputQueueSize,when the value of OutputQueueSize is set to be 1, 
                          the function will be same as LatestImagesOnly; if the value of OutputQueueSize is set to be ImageNodeNum, the function will be same as OneByOne.
                UpcomingImage:Ignore all images in output cache list when calling image acuiqisiotn interface, wait the next upcoming image generated.(This strategy does not support MV_USB_DEVICE device) 
			 This API only support MV_GIGE_DEVICE, MV_USB_DEVICE device on Windows, and only support MV_USB_DEVICE device on Linux.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGrabStrategy(IN void* handle, IN MV_GRAB_STRATEGY enGrabStrategy);

/********************************************************************//**
 *  @~english
 *  @brief  Set The Size of Output Queue(Only work under the strategy of MV_GrabStrategy_LatestImages��rang��1-ImageNodeNum)
 *  @param  handle                      [IN]            Device handle
 *  @param  nOutputQueueSize            [IN]            The Size of Output Queue
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface must be used with LatestImages Grab strategy, it is used for setting the maximum allowance queue size of the image under the LatestImages strategy.
             The user may change the output queue size while grabbing images.
 *           The DoubleUsb Device nOutputQueueSize at least 2
             This API only support MV_GIGE_DEVICE, MV_USB_DEVICE device on Windows, and only support MV_USB_DEVICE device on Linux.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetOutputQueueSize(IN void* handle, IN unsigned int nOutputQueueSize);

/********************************************************************//**
 *  @~english
 *  @brief  Get device information
 *  @param  handle                      [IN]            Device handle
 *  @param  pstDevInfo                  [IN][OUT]       Structure pointer of device information
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks The API support users to access device information after opening the device��don't support GenTL Devices
             If the device is a GigE camera, there is a blocking risk in calling the interface, so it is not recommended to call the interface during the fetching process. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetDeviceInfo(IN void * handle, IN OUT MV_CC_DEVICE_INFO* pstDevInfo);

/********************************************************************//**
 *  @~english
 *  @brief  Get various type of information
 *  @param  handle                      [IN]            Device handle
 *  @param  pstInfo                     [IN][OUT]       Structure pointer of various type of information
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Input required information type (specify nType in structure MV_ALL_MATCH_INFO) in the interface and get corresponding information (return in pInfo of structure MV_ALL_MATCH_INFO)
             The calling precondition of this interface is determined by obtained information type. Call after enabling capture to get MV_MATCH_TYPE_NET_DETECT information of GigE device,
             and call after starting device to get MV_MATCH_TYPE_USB_DETECT information of USB3Vision device.
             The information type MV_MATCH_TYPE_NET_DETECT corresponds to the structure MV_MATCH_INFO_NET_DETECT, which only supports  cameras of  MV_GIGE_DEVICE and MV_GENTL_GIGE_DEVICE types
             The information type MV_MATCH_TYPE_USB_DETECT corresponds to the structure MV_MATCH_INFO_USB_DETECT, which only supports cameras of MV_USB_DEVICE type
             This API is not supported by MV_CAMERALINK_DEVICE device. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetAllMatchInfo(IN void* handle, IN OUT MV_ALL_MATCH_INFO* pstInfo);

/********************************************************************//**

*  @~english
*  @brief   enum Frame grabber
*  @param   nTLayerType         [IN]             Frame grabber Type eg: (MV_GIGE_INTERFACE | MV_CAMERALINK_INTERFACE | MV_CXP_INTERFACE| MV_XOF_INTERFACE | MV_VIR_INTERFACE��
*  @param   pInterfaceInfoList   [IN][OUT]       Frame grabbe List
*  @return  Success, return MV_OK. Failure, return error code
*  @remarks This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumInterfaces(IN unsigned int nTLayerType, IN OUT MV_INTERFACE_INFO_LIST* pInterfaceInfoList);

/********************************************************************//**
*  @~english
*  @brief   create Frame grabber handle
*  @param   handle              [OUT]      Frame grabber handle
*  @param   pInterfaceInfo      [IN]       Frame grabber Info
*  @return  Success, return MV_OK. Failure, return error code
*  @remarks This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateInterface(IN OUT void ** handle, IN MV_INTERFACE_INFO* pInterfaceInfo);

/********************************************************************//**
*  @~english
*  @brief   create Frame grabber handle by ID
*  @param   handle             [IN][OUT]         Frame grabber handle
*  @param   pInterfaceID       [IN]              Frame grabber ID
*  @return  Success, return MV_OK. Failure, return error code
*  @remarks This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateInterfaceByID(IN OUT void ** handle, IN const char* pInterfaceID);

/********************************************************************//**

*  @~english
*  @brief   open Frame grabber
*  @param   handle         [IN]       Frame grabber handle
*  @param   pReserved      [IN]       Reserved��default NULL
*  @return   Success, return MV_OK. Failure, return error code
*  @remarks This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_OpenInterface(IN void* handle, IN char* pReserved);

/********************************************************************//**

*  @~english
*  @brief   close Frame grabber
*  @param   handle  [IN]          Frame grabber handle
*  @return   Success, return MV_OK. Failure, return error code
*  @remarks This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CloseInterface(IN void* handle);

/********************************************************************//**
*  @~english
*  @brief   Destroy Frame grabber handle
*  @param   handle  [IN] Frame grabber handle
*  @return  Success, return MV_OK. Failure, return error code
*  @remarks If MV_CC_DestroyInterface passes in "Device handle", the effect is the same as the MV_CC_DestroyHandle. This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DestroyInterface(IN void* handle);

/********************************************************************//**
*  @~english
*  @brief  Enumerate Devices with interface handle
*  @param  handle                   [IN]            Interface information
*  @param  pstDevList               [OUT]           Device List
*  @return Success, return MV_OK. Failure, return error code
*  @remarks The memory of the list is allocated within the SDK. When the interface is invoked by multiple threads, the memory of the device list will be released and applied
It is recommended to avoid multithreaded enumeration operations as much as possible.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevicesByInterface(IN void* handle, OUT MV_CC_DEVICE_INFO_LIST* pstDevList);



/*******************Part4 ch: ���/�ɼ��������������ýӿ� | en: Camera /Frame grabber attribute nodes universal interface*******************/

/********************************************************************//**
 *  @~english
 *  @brief  Get Integer value
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strKey                      [IN]            Key value, for example, using "Width" to get width
 *  @param  pstIntValue                 [IN][OUT]       Structure pointer of camera features
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks You can call this API to get the value of camera node with integer type after connecting the device. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetIntValueEx(IN void* handle,IN const char* strKey,IN OUT MVCC_INTVALUE_EX *pstIntValue);

/********************************************************************//**
 *  @~english
 *  @brief  Set Integer value
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strKey                      [IN]            Key value, for example, using "Width" to set width
 *  @param  nValue                      [IN]            Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks You can call this API to get the value of camera node with integer type after connecting the device. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetIntValueEx(IN void* handle,IN const char* strKey,IN int64_t nValue);

/********************************************************************//**
 *  @~english
 *  @brief  Get Enum value
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strKey                      [IN]            Key value, for example, using "PixelFormat" to get pixel format
 *  @param  pstEnumValue                [IN][OUT]       Structure pointer of camera features
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to get specified Enum nodes. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetEnumValue(IN void* handle,IN const char* strKey,IN OUT MVCC_ENUMVALUE *pstEnumValue);

/********************************************************************//**
*  @~english
*  @brief  Get Enum value
*  @param  handle                      [IN]            Device handle/Frame grabber handle
*  @param  strKey                      [IN]            Key value, for example, using "PixelFormat" to get pixel format
*  @param  pstEnumValue                [IN][OUT]       Structure pointer of camera features
*  @return Success, return MV_OK. Failure, return error code
*  @remarks After the device is connected, call this interface to get specified Enum nodes.
            Comparing with the API MV_CC_GetEnumValue, this API expands the number of enumeration values up to 256.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetEnumValueEx(IN void* handle, IN const char* strKey, IN OUT MVCC_ENUMVALUE_EX *pstEnumValue);

/********************************************************************//**
 *  @~english
 *  @brief  Set Enum value
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strKey                      [IN]            Key value, for example, using "PixelFormat" to set pixel format
 *  @param  nValue                      [IN]            Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to set specified Enum nodes. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValue(IN void* handle,IN const char* strKey,IN unsigned int nValue);

/********************************************************************//**
 *  @~english
 *  @brief  Get the symbolic of the specified value of the Enum type node
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strKey                      [IN]            Key value, for example, using "PixelFormat" to set pixel format
 *  @param  pstEnumEntry                [IN][OUT]           Symbolic to get
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Call this interface after connecting the device to obtain the symbol corresponding to the value of the specified node of Enum type.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetEnumEntrySymbolic(IN void* handle,IN const char* strKey,IN OUT MVCC_ENUMENTRY* pstEnumEntry);

/********************************************************************//**      
 *  @~english
 *  @brief  Set Enum value
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strKey                      [IN]            Key value, for example, using "PixelFormat" to set pixel format
 *  @param  strValue                    [IN]            Feature String to set
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to set specified Enum nodes. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetEnumValueByString(IN void* handle,IN const char* strKey,IN const char* strValue);

/********************************************************************//**
 *  @~english
 *  @brief  Get Float value
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strKey                      [IN]            Key value
 *  @param  pstFloatValue               [IN][OUT]       Structure pointer of camera features
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to get specified float node. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFloatValue(IN void* handle,IN const char* strKey,IN OUT MVCC_FLOATVALUE *pstFloatValue);

/********************************************************************//**
 *  @~english
 *  @brief  Set float value
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strKey                      [IN]            Key value
 *  @param  fValue                      [IN]            Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to set specified float node. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetFloatValue(IN void* handle,IN const char* strKey,IN float fValue);
    
/********************************************************************//**
 *  @~english
 *  @brief  Get Boolean value
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strKey                      [IN]            Key value
 *  @param  pbValue                     [IN][OUT]       Structure pointer of camera features
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to get specified bool nodes. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetBoolValue(IN void* handle,IN const char* strKey,IN OUT bool *pbValue);

/********************************************************************//**
 *  @~english
 *  @brief  Set Boolean value
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strKey                      [IN]            Key value
 *  @param  bValue                      [IN]            Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to set specified bool nodes. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBoolValue(IN void* handle,IN const char* strKey,IN bool bValue);

/********************************************************************//**
 *  @~english
 *  @brief  Get String value
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strKey                      [IN]            Key value
 *  @param  pstStringValue              [IN][OUT]       Structure pointer of camera features
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to get specified string nodes. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetStringValue(IN void* handle,IN const char* strKey,IN OUT MVCC_STRINGVALUE *pstStringValue);

/********************************************************************//**
 *  @~english
 *  @brief  Set String value
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strKey                      [IN]            Key value
 *  @param  strValue                    [IN]            Feature value to set
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to set specified string nodes. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetStringValue(IN void* handle,IN const char* strKey,IN const char* strValue);

/********************************************************************//**
 *  @~english
 *  @brief  Send Command
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strKey                      [IN]            Key value
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to set specified Command nodes. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetCommandValue(IN void* handle,IN const char* strKey);



/********************************************************************//**
 *  @~english
 *  @brief  Read Memory
 *  @param  handle                      [IN]            Device Handle/Frame grabber handle
 *  @param  pBuffer                     [IN][OUT]       Used as a return value, save the read-in memory value ( The memory value of GEV devices is stored in the big end mode, with the capture card device and the camera under the capture card stored in the big end mode, and other protocol devices stored in the small end mode)
 *  @param  nAddress                    [IN]            Memory address to be read, which can be obtained from the Camera.xml file of the device, the form xml node value of xxx_RegAddr
 *  @param  nLength                     [IN]            Length of the memory to be read
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Access device, read the data from certain register.
*************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ReadMemory(IN void* handle , IN OUT void *pBuffer, IN int64_t nAddress, IN int64_t nLength);

/********************************************************************//**
 *  @~english
 *  @brief  Write Memory
 *  @param  handle                      [IN]            Device Handle/Frame grabber handle
 *  @param  pBuffer                     [IN]            Memory value to be written ( Note The memory value of GEV devices is stored in the big end mode, with the capture card device and the camera under the capture card stored in the big end mode, and other protocol devices stored in the small end mode)
 *  @param  nAddress                    [IN]            Memory address to be written, which can be obtained from the Camera.xml file of the device, the form xml node value of xxx_RegAddr
 *  @param  nLength                     [IN]            Length of the memory to be written
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Access device, write a piece of data into a certain segment of register.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_WriteMemory(IN void* handle, IN const void *pBuffer, IN int64_t nAddress, IN int64_t nLength);

/********************************************************************//**

 *  @~english
 *  @brief  Invalidate GenICam Nodes
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_InvalidateNodes(IN void* handle);

/********************************************************************//**
 *  @~english
 *  @brief  Get camera feature tree XML
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  pData                       [IN][OUT]       XML data receiving buffer
 *  @param  nDataSize                   [IN]            Buffer size
 *  @param  pnDataLen                   [IN][OUT]       Actual data length
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks When pData is NULL or nDataSize than the actual XML file hours, do not copy the data, returned by pnDataLen XML file size.
             When pData is a valid cache address and the cache is large enough, copy the full data into the cache, and pnDataLen returns the actual size of the XML file.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetGenICamXML(IN void* handle, IN OUT unsigned char* pData, IN unsigned int nDataSize, IN OUT unsigned int* pnDataLen);

/********************************************************************//**
 *  @~english
 *  @brief  Get Access mode of cur node
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strName                     [IN]            Name of node
 *  @param  penAccessMode               [IN][OUT]       Access mode of the node
 *  @return Success, return MV_OK. Failure, return error code
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetNodeAccessMode(IN void* handle, IN const char * strName, IN OUT enum MV_XML_AccessMode *penAccessMode);

/********************************************************************//**
 *  @~english
 *  @brief  Get Interface Type of cur node
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strName                     [IN]            Name of node
 *  @param  penInterfaceType            [IN][OUT]       Interface Type of the node
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks The interface can know the node type in advance before calling the universal interface, so as to facilitate users to select the appropriate universal interface for setting and obtaining the node value.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_XML_GetNodeInterfaceType(IN void* handle, IN const char * strName, IN OUT enum MV_XML_InterfaceType *penInterfaceType);

/********************************************************************//**
 *  @~english
 *  @brief  Save camera feature
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strFileName                 [IN]            File name
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FeatureSave(IN void* handle, IN const char* strFileName);

/********************************************************************//**
 *  @~english
 *  @brief  Load camera feature
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strFileName                 [IN]            File name
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FeatureLoad(IN void* handle, IN const char* strFileName);

/********************************************************************//**
 *  @~english
 *  @brief  Load camera feature with error message list
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  strFileName                 [IN]            File name
 *  @param  pstNodeErrorList            [IN OUT]        Error message list, requested by the user externally and filled with data internally, \n
 *                                                      this parameter allows null to indicate that the user is not concerned about error information during import.
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks When some nodes fail to load, the interface returns MV_OK. \n
 *           The error node and the reason for the failure are obtained through stNodeError in the error message list.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FeatureLoadEx(IN void* handle, IN const char* strFileName, IN OUT MVCC_NODE_ERROR_LIST* pstNodeErrorList);

/********************************************************************//**
 *  @~english
 *  @brief  Read the file from the camera
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  pstFileAccess               [IN]            File access structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessRead(IN void* handle, IN MV_CC_FILE_ACCESS * pstFileAccess);


/********************************************************************//**
 *  @~english
 *  @brief  Read the file data from the camera
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  pstFileAccess               [IN]            File access structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessReadEx(IN void* handle, IN OUT MV_CC_FILE_ACCESS_EX * pstFileAccessEx);

/********************************************************************//**
 *  @~english
 *  @brief  Write the file to camera
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  pstFileAccess               [IN]            File access structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessWrite(IN void* handle, IN MV_CC_FILE_ACCESS * pstFileAccess);


/********************************************************************//**
 *  @~english
 *  @brief  Write the data(buffer) to camera
 *  @param  handle                        [IN]            Device handle/Frame grabber handle
 *  @param  pstFileAccessEx               [IN][OUT]       File access structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface uses cached data for read and write,solve the problem of no permissions in direct operation files, it's an extended interface of MV_CC_FileAccessWrite.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FileAccessWriteEx(IN void* handle, IN OUT MV_CC_FILE_ACCESS_EX * pstFileAccessEx);


/********************************************************************//**
 *  @~english
 *  @brief  Get File Access Progress 
 *  @param  handle                      [IN]            Device handle/Frame grabber handle
 *  @param  pstFileAccessProgress       [IN][OUT]       File access Progress
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetFileAccessProgress(IN void* handle, IN OUT MV_CC_FILE_ACCESS_PROGRESS * pstFileAccessProgress);

/********************************************************************//**
 *  @~english
 *  @brief  Device Local Upgrade
 *  @param  handle                      [IN]            Device handle
 *  @param  strFilePathName             [IN]            File name
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Call this API to send the upgrade firmware to the device for upgrade.
             This API will wait for return until the upgrade firmware is sent to the device, this response may take a long time.
             For CameraLink device, it keeps sending upgrade firmware continuously. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_LocalUpgrade(IN void* handle, IN const void* strFilePathName);

/********************************************************************//**
 *  @~english
 *  @brief  Get Upgrade Progress
 *  @param  handle                      [IN]            Device handle
 *  @param  pnProcess                   [IN][OUT]       Progress receiving address
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetUpgradeProcess(IN void* handle, IN OUT unsigned int* pnProcess);


/*******************Part6  ch: ����Ͳɼ��� ע���쳣�ص����¼��ӿ� | en:  Camera /Frame  Enrol abnormal callbacks and event interface*******************/

/********************************************************************//**
 *  @~english
 *  @brief  Register Exception Message CallBack, call after open device
 *  @param  handle                      [IN]            Device handle
 *  @param  cbException                 [IN]            Exception Message CallBack Function Pointer
 *  @param  pUser                       [IN]            User defined variable
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Call this interface after the device is opened by MV_CC_OpenDevice. 
             When device is exceptionally disconnected, the exception message can be obtained from callback function. For Disconnected GigE device,
             first call MV_CC_CloseDevice to shut device, and then call MV_CC_OpenDevice to reopen the device. 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterExceptionCallBack(IN void* handle, IN void(__stdcall* cbException)(unsigned int nMsgType, void* pUser), IN void* pUser);

/********************************************************************//**
 *  @~english
 *  @brief  Register event callback, which is called after the device is opened
 *  @param  handle                      [IN]            Device handle
 *  @param  cbEvent                     [IN]            Event CallBack Function Pointer
 *  @param  pUser                       [IN]            User defined variable
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Call this API to set the event callback function to get the event information, e.g., acquisition, exposure, and so on
             This API is not supported by CameraLink device.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterAllEventCallBack(IN void* handle, IN void(__stdcall* cbEvent)(MV_EVENT_OUT_INFO * pEventInfo, void* pUser), IN void* pUser);

/********************************************************************//**
 *  @~english
 *  @brief  Register single event callback, which is called after the device is opened
 *  @param  handle                      [IN]            Device handle
 *  @param  strEventName                [IN]            Event name
 *  @param  cbEvent                     [IN]            Event CallBack Function Pointer
 *  @param  pUser                       [IN]            User defined variable
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Call this API to set the event callback function to get the event information, e.g., acquisition, exposure, and so on.
             This API is not supported by CameraLink device .
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterEventCallBackEx(IN void* handle, IN const char* strEventName, IN void(__stdcall* cbEvent)(MV_EVENT_OUT_INFO * pEventInfo, void* pUser), IN void* pUser);

/********************************************************************//**
 *  @~english
 *  @brief  Enable specified event of device
 *  @param  handle                      [IN]            Device handle
 *  @param  strEventName                [IN]            Event name
 *  @return Success, return MV_OK. Failure, return error code 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EventNotificationOn(IN void* handle, IN const char* strEventName);

/********************************************************************//**
 *  @~english
 *  @brief  Disable specified event of device
 *  @param  handle                      [IN]            Device handle
 *  @param  strEventName                [IN]            Event name
 *  @return Success, return MV_OK. Failure, return error code 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EventNotificationOff(IN void* handle, IN const char* strEventName);


/********************************************************************//**
 *  @~english
 *  @brief  Set enumerate device timeout,only support GigE,range:[1, UINT_MAX)
 *  @param  nMilTimeout                 [IN]            time out,input of unsigned int,default 100ms
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Before calling enum device interfaces,call MV_GIGE_SetEnumDevTimeout to set max timeout,can reduce the maximum timeout to speed up the enumeration of GigE devices.
 *  @remarks This API only support GigE Vision Device.
             
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetEnumDevTimeout(IN unsigned int nMilTimeout);

/********************************************************************//**
 *  @~english
 *  @brief  Force IP
 *  @param  handle                      [IN]            Device handle
 *  @param  nIP                         [IN]            IP to set
 *  @param  nSubNetMask                 [IN]            Subnet mask
 *  @param  nDefaultGateWay             [IN]            Default gateway
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Force setting camera network parameter (including IP address, subnet mask, default gateway). After forced setting, device handle should be created again. 
             This API support GigEVision(MV_GIGE_DEVICE) and GenTL(MV_GENTL_GIGE_DEVICE) device.
             If device is in DHCP status, after calling this API to force setting camera network parameter, the device will restart.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_ForceIpEx(IN void* handle, IN unsigned int nIP, IN unsigned int nSubNetMask, IN unsigned int nDefaultGateWay);

/********************************************************************//**
 *  @~english
 *  @brief  IP configuration method
 *  @param  handle                      [IN]            Device handle
 *  @param  nType                       [IN]            IP type, refer to MV_IP_CFG_x
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Send command to set camera IP mode, such as DHCP and LLA, only supported by GigEVision(MV_GIGE_DEVICE) and GenTL(MV_GENTL_GIGE_DEVICE) Device.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetIpConfig(IN void* handle, IN unsigned int nType);

/********************************************************************//**
 *  @~english
 *  @brief  Set to use only one mode,type: MV_NET_TRANS_x. When do not set, priority is to use driver by default
 *  @param  handle                      [IN]            Device handle
 *  @param  nType                       [IN]            Net transmission mode, refer to MV_NET_TRANS_x
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarksSet SDK internal priority network mode through this interface, drive mode by default, only supported by GigEVision camera.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetNetTransMode(IN void* handle, IN unsigned int nType);

/********************************************************************//**
 *  @~english
 *  @brief  Get net transmission information
 *  @param  handle                      [IN]            Device handle
 *  @param  pstInfo                     [IN][OUT]       Information Structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Get network transmission information through this API, including received data size, number of lost frames.
             Call this API after starting image acquiring through MV_CC_StartGrabbing. This API is supported only by GigEVision Camera.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetNetTransInfo(IN void* handle, IN OUT MV_NETTRANS_INFO* pstInfo);

/********************************************************************//**
 *  @~english
 *  @brief  Setting the ACK mode of devices Discovery.
 *  @param  nMode                       [IN]            ACK mode��Default-Broadcast��,0-Unicast,1-Broadcast.
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This interface is ONLY effective on GigE cameras.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetDiscoveryMode(IN unsigned int nMode);

/********************************************************************//**
 *  @~english
 *  @brief  Set GVSP streaming timeout
 *  @param  handle                      [IN]            Device handle
 *  @param  nMillisec                   [IN]            Timeout, default 300ms, range:[10 - UINT_MAX)
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, and just before start streaming, 
 *           call this interface to set GVSP streaming timeout value.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGvspTimeout(IN void* handle, IN unsigned int nMillisec);

/********************************************************************//**
 *  @~english
 *  @brief  Get GVSP streaming timeout
 *  @param  handle                      [IN]            Device handle
 *  @param  pnMillisec                  [IN][OUT]       Timeout, ms as unit
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is used to get the current GVSP streaming timeout.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGvspTimeout(IN void* handle, IN OUT unsigned int* pnMillisec);

/********************************************************************//**
 *  @~english
 *  @brief  Set GVCP cammand timeout
 *  @param  handle                      [IN]            Device handle
 *  @param  nMillisec                   [IN]            Timeout(ms), default 500ms, range: [0,10000]
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks The API can set GVCP command timeout(ms) after device is connected .
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetGvcpTimeout(IN void* handle, IN unsigned int nMillisec);

/********************************************************************//**
 *  @~english
 *  @brief  Get GVCP cammand timeout
 *  @param  handle                      [IN]            Device handle
 *  @param  pnMillisec                  [IN][OUT]       Timeout, ms as unit
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is used to get the current GVCP timeout.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetGvcpTimeout(IN void* handle, IN OUT unsigned int* pnMillisec);

/********************************************************************//**
 *  @~english
 *  @brief  Set the number of retry GVCP cammand
 *  @param  handle                      [IN]            Device handle
 *  @param  nRetryGvcpTimes             [IN]            The number of retries��rang��0-100
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is used to increase The Times of retransmission when GVCP packet transmission is abnormal,and to some extent,
             it can avoid dropping the camera, with a range of 0-100.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetRetryGvcpTimes(IN void* handle, IN unsigned int nRetryGvcpTimes);

/********************************************************************//**
 *  @~english
 *  @brief  Get the number of retry GVCP cammand
 *  @param  handle                      [IN]            Device handle
 *  @param  pnRetryGvcpTimes            [IN][OUT]       The number of retries
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is used to get the current number of GVCP retransmissions, which defaults to 3.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetRetryGvcpTimes(IN void* handle, IN OUT unsigned int* pnRetryGvcpTimes);

/********************************************************************//**
 *  @~english
 *  @brief  Get the optimal Packet Size, Only support GigE Camera
 *  @param  handle                      [IN]            Device handle
 *  @return Optimal packetsize
 *  @remarks To get optimized packet size, for GigEVision device is SCPS
             and it is the size of a packet transported on the network. The interface should be called after MV_CC_OpenDevice and before MV_CC_StartGrabbing.
             This API is not supported by CameraLink device and U3V device. 
			 This interface does not support GenTL devices (protocol not supported). If a network camera is added in GenTL mode, it is recommended to configure GevSCPSPacketSize according to the actual network situation,or 1500.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetOptimalPacketSize(IN void* handle);

/********************************************************************//**
 *  @~english
 *  @brief  Set whethe to enable resend, and set resend
 *  @param  handle                      [IN]            Device handle
 *  @param  bEnable                     [IN]            enable resend
 *  @param  nMaxResendPercent           [IN]            Max resend persent
 *  @param  nResendTimeout              [IN]            Resend timeout, rang��0-10000ms
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After the device is connected, call this interface to set resend packet properties, only supported by GigEVision camera.
 ************************************************************************/
#ifndef __cplusplus
MV_CAMCTRL_API int __stdcall MV_GIGE_SetResend(IN void* handle, IN unsigned int bEnable, IN unsigned int nMaxResendPercent, IN unsigned int nResendTimeout);
#else
MV_CAMCTRL_API int __stdcall MV_GIGE_SetResend(IN void* handle, IN unsigned int bEnable, IN unsigned int nMaxResendPercent = 100, IN unsigned int nResendTimeout = 50);
#endif

/********************************************************************//**
 *  @~english
 *  @brief  set the max resend retry times
 *  @param  handle                      [IN]            Device handle
 *  @param  nRetryTimes                 [IN]            The max times to retry resending lost packets��default 20
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface MUST be called after enabling resending lost packets by calling MV_GIGE_SetResend,
 *           otherwise would fail and return MV_E_CALLORDER.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall  MV_GIGE_SetResendMaxRetryTimes(IN void* handle, IN unsigned int nRetryTimes);

/********************************************************************//**
 *  @~english
 *  @brief  get the max resend retry times
 *  @param  handle                      [IN]            Device handle
 *  @param  pnRetryTimes                [IN][OUT]       The max times to retry resending lost packets
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface MUST be called after enabling resending lost packets by calling MV_GIGE_SetResend,
 *           otherwise would fail and return MV_E_CALLORDER. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall  MV_GIGE_GetResendMaxRetryTimes(IN void* handle, IN OUT unsigned int* pnRetryTimes);

/********************************************************************//**
 *  @~english
 *  @brief  set time interval between same resend requests
 *  @param  handle                      [IN]            Device handle
 *  @param  nMillisec                   [IN]            The time interval between same resend requests,default 10ms
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface MUST be called after enabling resending lost packets by calling MV_GIGE_SetResend,
 *           otherwise would fail and return MV_E_CALLORDER. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall  MV_GIGE_SetResendTimeInterval(IN void* handle, IN unsigned int nMillisec);

/********************************************************************//**
 *  @~english
 *  @brief  get time interval between same resend requests
 *  @param  handle                      [IN]            Device handle
 *  @param  pnMillisec                  [IN][OUT]       The time interval between same resend requests
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface MUST be called after enabling resending lost packets by calling MV_GIGE_SetResend,
 *           otherwise would fail and return MV_E_CALLORDER. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall  MV_GIGE_GetResendTimeInterval(IN void* handle, IN OUT unsigned int* pnMillisec);

/********************************************************************//**
 *  @~english
 *  @brief  Set transmission type,Unicast or Multicast
 *  @param  handle                      [IN]            Device handle
 *  @param  stTransmissionType          [IN]            Struct of transmission type
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Call this API to set the transmission mode as single cast mode and multicast mode. And this API is only valid for GigEVision camera. 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_SetTransmissionType(IN void* handle, IN MV_TRANSMISSION_TYPE * pstTransmissionType);

/********************************************************************//**
 *  @~english
 *  @brief  Issue Action Command
 *  @param   pstActionCmdInfo           [IN]            Action Command
 *  @param   pstActionCmdResults        [IN][OUT]       Action Command Result List
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API is supported only by GigEVision camera.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_IssueActionCommand(IN MV_ACTION_CMD_INFO* pstActionCmdInfo, IN OUT MV_ACTION_CMD_RESULT_LIST* pstActionCmdResults);

/********************************************************************//**
 *  @~english
 *  @brief  Get Multicast Status
 *  @param  pstDevInfo                  [IN]            Device Information Structure
 *  @param  pbStatus                    [IN][OUT]       Status of Multicast
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is used to determine whether the camera is currently in multicast state, 
             and to solve the problem that the client needs to turn on the camera to determine multicast when enumerating.
			 This API only support GigE Vision Device.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_GIGE_GetMulticastStatus(IN MV_CC_DEVICE_INFO* pstDevInfo, IN OUT bool* pbStatus);


/*******************Part8 ch: ��CameraLink �豸֧�ֵĽӿ� | en: Only support camlink device interface*******************/
/********************************************************************//**
 *  @~english
 *  @brief  Get serial port information list
 *  @param  pstSerialPortList           [IN][OUT]       serial port information list
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is used to get local serial port information. This API do not support arm and Linux32 platform.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_GetSerialPortList(IN OUT MV_CAML_SERIAL_PORT_LIST* pstSerialPortList);

/********************************************************************//**
 *  @~english
 *  @brief  Set the specified enumeration serial port
 *  @param  pstSerialPortList           [IN]       serial port information list
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface is used to set the specified enumeration serial port. This API do not support arm and Linux32 platform.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_SetEnumSerialPorts(IN MV_CAML_SERIAL_PORT_LIST* pstSerialPortList);

/***********************************************************************************************************//**
 *  @~english
 *  @brief  Set device baudrate using one of the CL_BAUDRATE_XXXX value   
 *  @param  handle                      [IN]            Device handle
 *  @param  nBaudrate                   [IN]            baud rate to set. Refer to the 'CameraParams.h' for parameter definitions, for example, #define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This API is supported only by CameraLink device.
             This API support calls when devices are not connected. But it is necessary to connect to the device first when accessing a CameraLink Device through the GenTL protocol.
             Due to hardware/system/external interference and other factors, configuring a high baud rate may cause abnormal communication. 
             It is recommended to configure a baud rate of less than 115200
             This API do not support arm and Linux32 platform.
************************************************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_SetDeviceBaudrate(IN void* handle, IN unsigned int nBaudrate);

/********************************************************************//**
 *  @~english
 *  @brief  Returns the current device baudrate, using one of the CL_BAUDRATE_XXXX value
 *  @param  handle                      [IN]            Device handle
 *  @param  pnCurrentBaudrate           [IN][OUT]       Return pointer of baud rate to user. Refer to the 'CameraParams.h' for parameter definitions, for example, #define MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This API is supported only by CameraLink device.
             This API support calls when devices are not connected.
             This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_GetDeviceBaudrate(IN void* handle,IN OUT unsigned int* pnCurrentBaudrate);

/********************************************************************//**
 *  @~english
 *  @brief  Returns supported baudrates of the combined device and host interface
 *  @param  handle                      [IN]            Device handle
 *  @param  pnBaudrateAblity            [IN][OUT]       Return pointer of the supported baudrates to user. 'OR' operation results of the supported baudrates. Refer to the 'CameraParams.h' for single value definitions, for example, MV_CAML_BAUDRATE_9600  0x00000001
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This API is supported only by CameraLink device.
             This API support calls when devices are not connected.
             This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_GetSupportBaudrates(IN void* handle,IN OUT unsigned int* pnBaudrateAblity);

/********************************************************************//**
 *  @~english
 *  @brief  Sets the timeout for operations on the serial port
 *  @param  handle                      [IN]            Device handle
 *  @param  nMillisec                   [IN]            Timeout in [ms] for operations on the serial port.
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This API do not support arm and Linux32 platform.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CAML_SetGenCPTimeOut(IN void* handle, IN unsigned int nMillisec);


/*******************Part9 ch: ��U3V�豸֧�ֵĽӿ� | en: Only support U3V device interface*******************/

/********************************************************************//**
 *  @~english
 *  @brief  Set transfer size of U3V device
 *  @param  handle                      [IN]            Device handle
 *  @param  nTransferSize               [IN]            Transfer size��Byte��default��1M��rang��>=0x400��Recommended maximum: [windows] rang <= 0x400000; [Linux] rang <= 0x200000
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Increasing the transmission packet size can reduce the CPU utilization at the time of fetching. However, different PCS and different USB extension CARDS have different compatibility, and if this parameter is set too large, there may be the risk of not getting the image.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_SetTransferSize(IN void* handle, IN unsigned int nTransferSize);

/********************************************************************//**
 *  @~english
 *  @brief  Get transfer size of U3V device
 *  @param  handle                      [IN]            Device handle
 *  @param  pnTransferSize              [IN][OUT]           Transfer size��Byte
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This interface is used to get the current U3V transfer packet size, default 1M.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_GetTransferSize(IN void* handle, IN OUT unsigned int* pnTransferSize);

/********************************************************************//**
 *  @~english
 *  @brief  Set transfer ways of U3V device
 *  @param  handle                      [IN]            Device handle
 *  @param  nTransferWays               [IN]            Transfer ways��rang��1-10
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Users can adjust this parameter according to PC performance, camera image frame rate, image size, memory utilization and other factors. But different PCS and different USB expansion CARDS have different compatibility.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_SetTransferWays(IN void* handle, IN unsigned int nTransferWays);

/********************************************************************//**
 *  @~english
 *  @brief  Get transfer ways of U3V device
 *  @param  handle                      [IN]            Device handle
 *  @param  pnTransferWays              [IN][OUT]       Transfer ways
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This interface is used to get the current number of U3V asynchronous feed nodes.
	For U3V camera, The number of transmission channels is related to the size of the payload size corresponding to the pixel format, which is calculated by the maximum asynchronous registration length / the payload size corresponding to pixel format.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_GetTransferWays(IN void* handle, IN OUT unsigned int* pnTransferWays);

/********************************************************************//**
 *  @~english
 *  @brief  Register exception stream callBack, call after open device (only support U3V Camera, don't support GenTL Device)
 *  @param  handle                      [IN]            Device handle
 *  @param  cbException                 [IN]            Exception callback function pointer
 *  @param  pUser                       [IN]            User defined variable
 *  @return Success, return MV_OK. Failure, return error code 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_RegisterStreamExceptionCallBack(IN void* handle, IN void(__stdcall* cbException)(MV_CC_STREAM_EXCEPTION_TYPE enExceptionType, void* pUser), IN void* pUser);

/********************************************************************//**
 *  @~english
 *  @brief  Set the number of U3V device event cache nodes
 *  @param  handle                      [IN]            Device handle
 *  @param  nEventNodeNum               [IN]            Event Node Number
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This interface is used to set the current number of U3V event nodes. default to 5 nodes.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_SetEventNodeNum(IN void* handle, IN unsigned int nEventNodeNum);


/********************************************************************//**
 *  @~english
 *  @brief  Set U3V Synchronisation timeout,range:[1000, UINT_MAX)
 *  @param  handle               [IN]            Device handle
 *  @param  nMills               [IN]            set synchronisation timeout(ms),default 1000ms
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Increasing the SetSyncTimeOut can compatible with some camera configuretion parameters very slow,more than 1000ms 
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_SetSyncTimeOut(IN void* handle, IN unsigned int nMills);

/********************************************************************//**
 *  @~english
 *  @brief  Get U3V Camera Synchronisation timeout
 *  @param  handle                      [IN]            Device handle
 *  @param  pnMills                     [IN][OUT]       Get Synchronisation time(ms)
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks This interface is used to get the current U3V timeout, default 1000ms.
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_USB_GetSyncTimeOut(IN void* handle, IN OUT unsigned int* pnMills);


/******************************************************************************//**
       
 *  @~english
 *  @brief  Enumerate Interfaces with GenTL
 *  @param  pstIFList                   [IN][OUT]       Interfaces List
 *  @param  strGenTLPath                [IN]            GenTL cti file path
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks The memory of the Interfaces list is allocated within the SDK. When the interface is invoked by multiple threads, the memory of the device list will be released and applied.\n
             It is recommended to avoid multithreaded enumeration operations as much as possible.
             Currently not supported for SDK to directly call MvProducerU3V. cti and MvProducerGEV. cti. supports calling other. cti
 *******************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumInterfacesByGenTL(IN OUT MV_GENTL_IF_INFO_LIST* pstIFList, IN const char * strGenTLPath);

/********************************************************************//**
 *  @~english
 *  @brief  Unload cti library
 *  @param  pGenTLPath                [IN]            GenTL cti file path
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks Make sure that all devices enumerated by this cti are already closed.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_UnloadGenTLLibrary(IN const char * pGenTLPath);

/*****************************************************************************************************//**
 *  @~english
 *  @brief  Enumerate Devices with GenTL interface
 *  @param  pstIFInfo                   [IN]            Interface information
 *  @param  pstDevList                  [IN][OUT]           Device List
 *  @return Success, return MV_OK. Failure, return error code 
 *  @remarks The memory of the list is allocated within the SDK. When the interface is invoked by multiple threads, the memory of the device list will be released and applied.\n
             It is recommended to avoid multithreaded enumeration operations as much as possible.
 *****************************************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_EnumDevicesByGenTL(IN MV_GENTL_IF_INFO* pstIFInfo, IN OUT MV_GENTL_DEV_INFO_LIST* pstDevList);

/********************************************************************//**
 *  @~english
 *  @brief  Create Device Handle with GenTL Device Info
 *  @param  handle                      [IN][OUT]       Device handle
 *  @param  pstDevInfo                  [IN]            Device Information
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Create required resources within library and initialize internal module according to input device information.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_CreateHandleByGenTL(IN OUT void ** handle, IN const MV_GENTL_DEV_INFO* pstDevInfo);

/********************************************************************//**
 *  @~english
 *  @brief  Save image, support Bmp and Jpeg.
 *  @param  handle                      [IN]            Device handle
 *  @param  pstSaveParam                [IN][OUT]       Save image parameters structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Once there is image data, you can call this API to convert the data.
             You can also call MV_CC_GetOneFrameTimeout or MV_CC_RegisterImageCallBackEx or MV_CC_GetImageBuffer to get one image frame and set the callback function, and then call this API to convert the format.
             Comparing with the API MV_CC_SaveImageEx2, this API support the parameter nWidth/nHeight/nDataLen to UINT_MAX. 
			 JPEG format supports a maximum width and height of 65500
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageEx3(IN void* handle, IN OUT MV_SAVE_IMAGE_PARAM_EX3* pstSaveParam);

/********************************************************************//**
 *  @~english
 *  @brief  Save the image file.
 *  @param  handle                      [IN]            Device handle
 *  @param  pstSaveFileParam            [IN][OUT]       Save the image file parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API support BMP/JPEG/PNG/TIFF.
             this API support the parameter nWidth/nHeight/nDataLen to UINT_MAX. 
			 JPEG format supports a maximum width and height of 65500
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageToFileEx(IN void* handle, IN OUT MV_SAVE_IMAGE_TO_FILE_PARAM_EX* pstSaveFileParam);

/********************************************************************//**
 *  @~english
 *  @brief  Save the image file.
 *  @param  handle                      [IN]            Device handle
 *  @param  pstImage                    [IN]            Image information
 *  @param  pSaveImageParam             [IN]            Save the image file parameter structure
 *  @param  pcImagePath                 [IN]            Image path
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks When the image size exceeds 4GB, only PNG and TIFF are supported. Otherwise, BMP,JPEG,TIFF and PNG are supported.
			 JPEG format supports a maximum width and height of 65500
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SaveImageToFileEx2(IN void* handle, IN MV_CC_IMAGE* pstImage, IN MV_CC_SAVE_IMAGE_PARAM* pSaveImageParam, IN const char* pcImagePath);

/********************************************************************//**
 
 *  @~english
 *  @brief  Rotate Image
 *  @param  handle                      [IN]            Device handle
 *  @param  pstRotateParam              [IN][OUT]       Rotate image parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API only support 90/180/270 rotation of data in the MONO8/RGB24/BGR24 format.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RotateImage(IN void* handle, IN OUT MV_CC_ROTATE_IMAGE_PARAM* pstRotateParam);

/********************************************************************//**
 *  @~english
 *  @brief  Flip Image
 *  @param  handle                      [IN]            Device handle
 *  @param  pstFlipParam                [IN][OUT]       Flip image parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API only support vertical and horizontal reverse of data in the MONO8/RGB24/BGR24 format.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FlipImage(IN void* handle, IN OUT MV_CC_FLIP_IMAGE_PARAM* pstFlipParam);


/********************************************************************//**
 *  @~english
 *  @brief  Pixel format conversion
 *  @param  handle                      [IN]            Device handle
 *  @param  pstCvtParam                 [IN][OUT]       Convert Pixel Type parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This API is used to transform the collected original data to pixel format and save to specified memory. 
             There is no order requirement to call this API, the transformation will execute when there is image data. 
             First call MV_CC_GetOneFrameTimeout or MV_CC_RegisterImageCallBackEx to set callback function, and get a frame of image data,
             then call this API to transform the format.
             Comparing with the API MV_CC_ConvertPixelType, this API support the parameter nWidth/nHeight/nSrcDataLen to UINT_MAX. 

 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ConvertPixelTypeEx(IN void* handle, IN OUT MV_CC_PIXEL_CONVERT_PARAM_EX* pstCvtParam);

/********************************************************************//**
 *  @~english
 *  @brief  Interpolation algorithm type setting
 *  @param  handle                      [IN]            Device handle
 *  @param  nBayerCvtQuality            [IN]            Bayer interpolation method  0-Fast 1-Equilibrium 2-Optimal
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Set the bell interpolation quality parameters of the internal image conversion interface, 
             and the interpolation algorithm used in the MV_CC_ConvertPixelTypeEx and MV_CC_GetImageForRGB/BGR interfaces is set by this interface.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerCvtQuality(IN void* handle, IN unsigned int nBayerCvtQuality);

/********************************************************************//**
 *  @~english
 *  @brief  Filter type of the bell interpolation quality algorithm setting
 *  @param  handle                      [IN]            Device handle
 *  @param  bFilterEnable               [IN]            Filter type enable
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Set the bell interpolation filter type parameters of the internal image conversion interface, 
             and the interpolation algorithm used in the MV_CC_ConvertPixelTypeEx and MV_CC_SaveImageEx3 interfaces is set by this interface.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerFilterEnable(IN void* handle, IN bool bFilterEnable);

/********************************************************************//**
 *  @~english
 *  @brief  Set Gamma value
 *  @param  handle                      [IN]            Device handle
 *  @param  fBayerGammaValue            [IN]            Gamma value[0.1,4.0]
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After setting the value, it works when calling MV_CC_ConvertPixelTypeEx\MV_CC_SaveImageEx3\MV_CC_SaveImageToFileEx API convert Bayer8/10/12/16 to RGB24/48�� RGBA32/64��BGR24/48��BGRA32/64.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerGammaValue(IN void* handle, IN float fBayerGammaValue);

/********************************************************************//**
 *  @~english
 *  @brief  Set Gamma value
 *  @param  handle                           [IN]            Device handle
 *  @param  MvGvspPixelType enSrcPixelType   [IN]            PixelType,support PixelType_Gvsp_Mono8,Bayer8/10/12/16
 *  @param  fGammaValue                      [IN]            Gamma value:0.1~ 4.0
 *  @remarks After setting the gamma of Mono8 ��the gamma value takes effect when calling MV_CC_ConvertPixelTypeEx converts Mono8 to Mono8.
 *  @remarks After setting the gamma of Bayer8/10/12/16, the gamma value takes effect when calling MV_CC_ConvertPixelTypeEx\MV_CC_SaveImageToFileEx\MV_CC_SaveImageEx3 converts Bayer8/10/12/16 to RGB24/48,RGBA32/64,BGR24/48,BGRA32/64.
 *  @remarks This API compatible with MV_CC_SetBayerGammaValue, adds Mono8 PixelType.
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetGammaValue(IN void* handle, IN enum MvGvspPixelType enSrcPixelType, IN float fGammaValue);

/********************************************************************//**
 *  @~english
 *  @brief  Set Gamma param
 *  @param  handle                      [IN]            Device handle
 *  @param  pstGammaParam               [IN]            Gamma param
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After setting the param, it work in the calling MV_CC_ConvertPixelTypeEx\MV_CC_SaveImageEx3\MV_CC_SaveImageToFileEx API convert Bayer8/10/12/16 to RGB24/48�� RGBA32/64��BGR24/48��BGRA32/64.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerGammaParam(IN void* handle, IN MV_CC_GAMMA_PARAM* pstGammaParam);

/********************************************************************//**
 *  @~english
 *  @brief  Set CCM param,Scale default 1024
 *  @param  handle                      [IN]            Device handle
 *  @param  pstCCMParam                 [IN]            CCM parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After enable the color correction and set the color correction matrix, It work in the calling MV_CC_ConvertPixelTypeEx\MV_CC_SaveImageEx3 API convert Bayer8/10/12/16 to RGB24/48�� RGBA32/64��BGR24/48��BGRA32/64.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerCCMParam(IN void* handle, IN MV_CC_CCM_PARAM* pstCCMParam);

/********************************************************************//**
 *  @~english
 *  @brief  Set CCM param
 *  @param  handle                      [IN]            Device handle
 *  @param  pstCCMParam                 [IN]            CCM parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks After enable the color correction and set the color correction matrix, It work in the calling MV_CC_ConvertPixelTypeEx\MV_CC_SaveImageEx3 API convert Bayer8/10/12/16 to RGB24/48�� RGBA32/64��BGR24/48��BGRA32/64.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetBayerCCMParamEx(IN void* handle, IN MV_CC_CCM_PARAM_EX* pstCCMParam);

/********************************************************************//**
 *  @~english
 *  @brief  Adjust image contrast
 *  @param  handle                      [IN]            Device handle
 *  @param  pstContrastParam            [IN][OUT]       Contrast parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks 
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ImageContrast(IN void* handle, IN OUT MV_CC_CONTRAST_PARAM* pstContrastParam);

/********************************************************************//**
 *  @~english
 *  @brief  Remove the purple edge from the image.
 *  @param  handle                      [IN]            Device handle
 *  @param  pstPurpleFringingParam      [IN][OUT]       PurpleFringing parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Only supports PixelType_Gvsp_RGB8_Packed and PixelType_Gvsp_BGR8_Packed.
 *  ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_PurpleFringing(IN void* handle, IN MV_CC_PURPLE_FRINGING_PARAM* pstPurpleFringingParam);

/********************************************************************//**
 *  @~english
 *  @brief  Set ISP configuration.
 *  @param  handle                      [IN]            Device handle
 *  @param  pstParam                    [IN][OUT]       ISP parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_SetISPConfig(void* handle, IN MV_CC_ISP_CONFIG_PARAM* pstParam);

/********************************************************************//**
 *  @~english
 *  @brief  ISP process.
 *  @param  handle                      [IN]            Device handle
 *  @param  pstInputImage               [IN]            Input image structure
 *  @param  pstOutputImage              [IN][OUT]       Output image structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks The Interface should be called after MV_CC_SetISPConfig.
 *  ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ISPProcess(void* handle, IN MV_CC_IMAGE* pstInputImage, MV_CC_IMAGE* pstOutputImage);

/********************************************************************//**
 *  @~english
 *  @brief  High Bandwidth Decode
 *  @param  handle                      [IN]            Device handle
 *  @param  pstDecodeParam              [IN][OUT]       High Bandwidth Decode parameter structure
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks Decode the lossless compressed data from the camera into raw data��At the same time, it supports parsing the watermark information of the real-time image of the current camera (if the input lossless code stream is not the current camera or is not real-time streaming, the watermark parsing may be abnormal);
			 If decoding fails, please check the following: (1) The CPU is required to support the SSE AVX instruction set. (2) If the current frame is abnormal (packet loss, etc.), it may cause decoding exceptions. (3) The camera plot is abnormal, even if there is no packet loss, it may cause exceptions
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_HB_Decode(IN void* handle, IN OUT MV_CC_HB_DECODE_PARAM* pstDecodeParam);

/********************************************************************//**
 *  @~english
 *  @brief  Draw Rect Auxiliary Line
 *  @param  handle                      [IN]            Device handle
 *  @param  pRectInfo                   [IN]            Rect Auxiliary Line Info
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface only supports windows platform.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DrawRect(IN void* handle, IN MVCC_RECT_INFO* pRectInfo);

/********************************************************************//**
 *  @~english
 *  @brief  Draw Circle Auxiliary Line
 *  @param  handle                      [IN]            Device Handle
 *  @param  pCircleInfo                 [IN]            Circle Auxiliary Line Info
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks This interface only supports windows platform.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DrawCircle(IN void* handle, IN MVCC_CIRCLE_INFO* pCircleInfo);

/********************************************************************//**
 *  @~english
 *  @brief  Draw Line Auxiliary Line
 *  @param  handle                      [IN]            Device Handle
 *  @param  pLinesInfo                  [IN]            Linear Auxiliary Line Info
 *  @return Success, return MV_OK. Failure, return error code
 *  @remarks  This interface only supports windows platform.
 ***********************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_DrawLines(IN void* handle, IN MVCC_LINES_INFO* pLinesInfo);

/********************************************************************//**
 *  @~english
 *  @brief  Start Record
 *  @param  handle                      [IN]            Device handle
 *  @param  pstRecordParam              [IN]            Record param structure
 *  @return Success, return MV_OK. Failure, return error code
	The maximum supported width * height of this interface is 8000 * 8000, otherwise it will result in calling MV_ CC_ InputOneFrame interface error.
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StartRecord(IN void* handle, IN MV_CC_RECORD_PARAM* pstRecordParam);

/********************************************************************//**
 *  @~english
 *  @brief  Input RAW data to Record
 *  @param  handle                      [IN]            Device handle
 *  @param  pstInputFrameInfo           [IN]            Record data structure
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_InputOneFrame(IN void* handle, IN MV_CC_INPUT_FRAME_INFO * pstInputFrameInfo);

/********************************************************************//**
 *  @~english
 *  @brief  Stop Record
 *  @param  handle                      [IN]            Device handle
 *  @return Success, return MV_OK. Failure, return error code
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_StopRecord(IN void* handle);

/********************************************************************//**
 *  @~english
 *  @brief  Reconstruct Image(For time-division exposure function)
 *  @param  handle                      [IN]            Device handle
 *  @param  pstReconstructParam         [IN][OUT]       Reconstruct image parameters
 *  @return Success, return MV_OK, Failure, return error code.
 *  @remarks Image segmentation supports any pixel format. Image segmentation should be used with the "MultiLightControl" node of the linear array camera. This node can set multiple different exposure values, such as MultiLightControl=2, 
             The camera will overlap and merge two images corresponding to two different exposure values into one image (the actual height is the height of the two images) and send it to the upper application. 
             Call the interface and pass in nExposureNum is two. One image sent by the camera can be divided into two images, each of which corresponds to an exposure value. 
             If an ordinary camera is used or the "MultiLightControl" node of the linear array camera is not turned on, the image segmentation is meaningless, but the image is divided into 2, 3, and 4 images by line. 
             The height of each image becomes 1/2, 1/3, 1/4 of the original image (determined by nExposureNum).
 ************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_ReconstructImage(IN void* handle, IN OUT MV_RECONSTRUCT_IMAGE_PARAM* pstReconstructParam);



/********************************************************************//**
*  @~english
*  @brief  
*  @param  nBufLen                      [IN]        memory bytes     
*  @param  nAlignment                   [IN]        memory aligned bytes (Must be an integer power of 2 greater than 0)
*  @return Success, return memory address, Failure, return NULL.
*  @remarks 
************************************************************************/
MV_CAMCTRL_API void *  __stdcall MV_CC_AllocAlignedBuffer(IN uint64_t  nBufSize, IN unsigned int nAlignment);

/********************************************************************//**
*  @~english 
*  @brief   release aligned memory
*  @param  pBuffer                      [IN]        memory address
*  @return Success, return MV_OK, Failure, return error code.
*  @remarks
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_FreeAlignedBuffer(IN void* pBuffer);

/********************************************************************//**
*  @~english
*  @brief  Obtain the device payload size (payload includes image data and Chunk data) and memory alignment method, 
    which is used by the application layer to allocate sufficient cache and correct memory alignment when registering external cache for SDK
*  @param  handle                      [IN]            Device Handle
*  @param  pnPayloadSize               [IN OUT]        Payload size
*  @param  pnAlignment                 [IN OUT]        Alignment bytes
*  @return Success, return MV_OK, Failure, return error code.
*  @remarks
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_GetPayloadSize(IN void* handle, IN OUT uint64_t* pnPayloadSize, IN OUT unsigned int* pnAlignment);

/********************************************************************//**
*  @~english
*  @brief  The application allocates memory and registers it within the SDK for use by the SDK
*  @param  handle                      [IN]            Device Handle
*  @param  pBuffer                     [IN]            external memory address
*  @param  nBufSize                    [IN]            external memory len 
*  @param  pUser                       [IN]            User Pointer
*  @return Success, return MV_OK, Failure, return error code.
*  @remarks Registering memory can be done by using MV_CC_GetPayloadSize to obtain the memory size, and allocating the memory size using MV_CC_AllocAlignedBuffer
            The registered memory needs to be notified by the application layer to the SDK to cancel the registration (MV_CC_UnregisterBuffer) and then released (MV_CC_FreeAlignedBuffer)
            After using this interface, only MV_CC_GetImageBuffer��MV_CC_FreeImageBuffer/MV_CC_RegisterImageCallBackEx is supported for image retrieval, and other interfaces are not supported for image retrieval
            After using this interface, if the SDK internal node (MV_CC_SetImageNodeNum) was previously configured, it is invalid
            The dual USB interface camera requires at least 3 spaces to be registered inside the SDK;There is no limit for other cameras for the time being, but to avoid insufficient cache, please configure sufficient cache into the SDK
************************************************************************/
MV_CAMCTRL_API int __stdcall MV_CC_RegisterBuffer(IN void* handle, IN void *pBuffer, IN uint64_t nBufSize, IN void* pUser);

/********************************************************************//**

*  @~english
*  @brief   revoke external memory
*  @param  handle                      [IN]            Device Handle
*  @param  pBuffer                     [IN]            external memory address
*  @return Success, return MV_OK, Failure, return error code.
*  @remarks
************************************************************************/
MV_CAMCTRL_API int __stdcall  MV_CC_UnRegisterBuffer(IN void* handle, IN void* pBuffer);


#ifdef __cplusplus
}
#endif 

#endif //_MV_CAMERA_CTRL_H_

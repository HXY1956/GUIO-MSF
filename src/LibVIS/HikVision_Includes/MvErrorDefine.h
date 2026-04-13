
#ifndef _MV_ERROR_DEFINE_H_
#define _MV_ERROR_DEFINE_H_

#include "MvISPErrorDefine.h"

/********************************************************************/
///  \~chinese
///  @{
///  \~english
///  \name Definition of correct code
///  @{
#define MV_OK                       0x00000000  
/// @}

/********************************************************************/
///  \~chinese
///  @{
///  \~english
///  \name Definition of General error code
///  @{
#define MV_E_HANDLE                 0x80000000  
#define MV_E_SUPPORT                0x80000001  ///< \~           \~english Not supported function
#define MV_E_BUFOVER                0x80000002  ///< \~                \~english Buffer overflow
#define MV_E_CALLORDER              0x80000003  ///< \~         \~english Function calling order error
#define MV_E_PARAMETER              0x80000004  ///< \~         \~english Incorrect parameter
#define MV_E_RESOURCE               0x80000006  ///< \~       \~english Applying resource failed
#define MV_E_NODATA                 0x80000007  ///< \~            \~english No data
#define MV_E_PRECONDITION           0x80000008  ///< \~ \~english Precondition error, or running environment changed
#define MV_E_VERSION                0x80000009  ///< \~        \~english Version mismatches
#define MV_E_NOENOUGH_BUF           0x8000000A  ///< \~   \~english Insufficient memory
#define MV_E_ABNORMAL_IMAGE         0x8000000B  ///< \~chinese   \~english Abnormal image, maybe incomplete image because of lost packet
#define MV_E_LOAD_LIBRARY           0x8000000C  ///< \~chinese     \~english Load library failed
#define MV_E_NOOUTBUF               0x8000000D  ///< \~chinese       \~english No Avaliable Buffer
#define MV_E_ENCRYPT                0x8000000E  ///< \~chinese         \~english Encryption error
#define MV_E_OPENFILE               0x8000000F  ///< \~chinese     \~english open file error
#define MV_E_BUF_IN_USE             0x80000010  ///< \~chinese  use
#define MV_E_BUF_INVALID            0x80000011  ///< \~chinese   \~english Buffer address invalid
#define MV_E_NOALIGN_BUF            0x80000012  ///< \~chinese       \~english Buffer alignmenterror error
#define MV_E_NOENOUGH_BUF_NUM       0x80000013  ///< \~chinese      \~english Insufficient cache count
#define MV_E_PORT_IN_USE            0x80000014  ///< \~chinese       \~english Port is in use
#define MV_E_IMAGE_DECODEC          0x80000015  ///< \~chinese ~english Decoding error (SDK verification image exception)
#define MV_E_UINT32_LIMIT           0x80000016  ///  \~chinese unsigned int 
#define MV_E_IMAGE_HEIGHT           0x80000017   ///  \~chinese \~english image height anomaly (discard incomplete images)

#define MV_E_UNKNOW                 0x800000FF  ///< \~chinese         \~english Unknown error
/// @}

/********************************************************************/
///  \~chinese
///  @{
///  \~english
///  \name GenICam Series Error Codes: Range from 0x80000100 to 0x800001FF
///  @{
#define MV_E_GC_GENERIC             0x80000100  
#define MV_E_GC_ARGUMENT            0x80000101  
#define MV_E_GC_RANGE               0x80000102  
#define MV_E_GC_PROPERTY            0x80000103  
#define MV_E_GC_RUNTIME             0x80000104  
#define MV_E_GC_LOGICAL             0x80000105  
#define MV_E_GC_ACCESS              0x80000106 
#define MV_E_GC_TIMEOUT             0x80000107  
#define MV_E_GC_DYNAMICCAST         0x80000108  
#define MV_E_GC_UNKNOW              0x800001FF

/********************************************************************/
///  \~chinese
///  @{
///  \~english
///  \name GigE_STATUS Error Codes: Range from 0x80000200 to 0x800002FF
///  @{
#define MV_E_NOT_IMPLEMENTED        0x80000200 
#define MV_E_INVALID_ADDRESS        0x80000201  
#define MV_E_WRITE_PROTECT          0x80000202  
#define MV_E_ACCESS_DENIED          0x80000203  
#define MV_E_BUSY                   0x80000204  ///< \~chinese     \~english Device is busy, or network disconnected
#define MV_E_PACKET                 0x80000205  ///< \~chinese           \~english Network data packet error
#define MV_E_NETER                  0x80000206  ///< \~chinese            \~english Network error
#define MV_E_SUPPORT_MODIFY_DEVICE_IP   0x8000020E 
#define MV_E_KEY_VERIFICATION       0x8000020F  ///< \~chinese              \~english SwitchKey error
#define MV_E_IP_CONFLICT            0x80000221  ///< \~chinese           \~english Device IP conflict
/// @}

/********************************************************************/
///  \~chinese
///  @{
///  \~english
///  \name USB_STATUS Error Codes: Range from 0x80000300 to 0x800003FF
///  @{
#define MV_E_USB_READ               0x80000300  ///< \~chinese              \~english Reading USB error
#define MV_E_USB_WRITE              0x80000301  ///< \~chinese             \~english Writing USB error
#define MV_E_USB_DEVICE             0x80000302  ///< \~chinese              \~english Device exception
#define MV_E_USB_GENICAM            0x80000303  ///< \~chinese       \~english GenICam error
#define MV_E_USB_BANDWIDTH          0x80000304  ///< \~chinese        \~english Insufficient bandwidth
#define MV_E_USB_DRIVER             0x80000305  ///< \~chinese   \~english Driver mismatch or unmounted drive
#define MV_E_USB_UNKNOW             0x800003FF  ///< \~chinese        \~english USB unknown error
/// @}

/********************************************************************/
///  \~chinese
///  @{
///  \~english
///  \name Upgrade Error Codes: Range from 0x80000400 to 0x800004FF
///  @{
#define MV_E_UPG_FILE_MISMATCH      0x80000400  ///< \~chinese          \~english Firmware mismatches
#define MV_E_UPG_LANGUSGE_MISMATCH  0x80000401  ///< \~chinese      \~english Firmware language mismatches
#define MV_E_UPG_CONFLICT           0x80000402  ///< \~chinese \~english Upgrading conflicted (repeated upgrading requests during device upgrade)
#define MV_E_UPG_INNER_ERR          0x80000403  ///< \~chinese    \~english Camera internal error during upgrade
#define MV_E_UPG_UNKNOW             0x800004FF  ///< \~chinese        \~english Unknown error during upgrade
/// @}

#endif //_MV_ERROR_DEFINE_H_

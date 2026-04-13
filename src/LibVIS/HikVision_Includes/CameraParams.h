
#ifndef _MV_CAMERA_PARAMS_H_
#define _MV_CAMERA_PARAMS_H_

#include "PixelType.h"

#ifndef __cplusplus
typedef char    bool;
#define true    1
#define false   0
#endif

typedef enum _MV_SORT_METHOD_
{
    SortMethod_SerialNumber   = 0,
    SortMethod_UserID         = 1,
    SortMethod_CurrentIP_ASC  = 2,
    SortMethod_CurrentIP_DESC = 3,
} MV_SORT_METHOD;

#define INFO_MAX_BUFFER_SIZE 64

typedef struct _MV_GIGE_DEVICE_INFO_
{
    unsigned int  nIpCfgOption;
    unsigned int  nIpCfgCurrent;
    unsigned int  nCurrentIp;
    unsigned int  nCurrentSubNetMask;
    unsigned int  nDefultGateWay;
    unsigned char chManufacturerName[32];
    unsigned char chModelName[32];
    unsigned char chDeviceVersion[32];
    unsigned char chManufacturerSpecificInfo[48];
    unsigned char chSerialNumber[16];
    unsigned char chUserDefinedName[16];
    unsigned int  nNetExport;
    unsigned int  nReserved[4];
} MV_GIGE_DEVICE_INFO;

typedef struct _MV_USB3_DEVICE_INFO_
{
    unsigned char CrtlInEndPoint;
    unsigned char CrtlOutEndPoint;
    unsigned char StreamEndPoint;
    unsigned char EventEndPoint;
    unsigned short idVendor;
    unsigned short idProduct;
    unsigned int  nDeviceNumber;
    unsigned char chDeviceGUID[INFO_MAX_BUFFER_SIZE];
    unsigned char chVendorName[INFO_MAX_BUFFER_SIZE];
    unsigned char chModelName[INFO_MAX_BUFFER_SIZE];
    unsigned char chFamilyName[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceVersion[INFO_MAX_BUFFER_SIZE];
    unsigned char chManufacturerName[INFO_MAX_BUFFER_SIZE];
    unsigned char chSerialNumber[INFO_MAX_BUFFER_SIZE];
    unsigned char chUserDefinedName[INFO_MAX_BUFFER_SIZE];
    unsigned int  nbcdUSB;
    unsigned int  nDeviceAddress;
    unsigned int  nReserved[2];
} MV_USB3_DEVICE_INFO;

typedef struct _MV_CamL_DEV_INFO_
{
    unsigned char chPortID[INFO_MAX_BUFFER_SIZE];
    unsigned char chModelName[INFO_MAX_BUFFER_SIZE];
    unsigned char chFamilyName[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceVersion[INFO_MAX_BUFFER_SIZE];
    unsigned char chManufacturerName[INFO_MAX_BUFFER_SIZE];
    unsigned char chSerialNumber[INFO_MAX_BUFFER_SIZE];
    unsigned int  nReserved[38];
} MV_CamL_DEV_INFO;

typedef struct _MV_CXP_DEVICE_INFO_
{
    unsigned char chInterfaceID[INFO_MAX_BUFFER_SIZE];
    unsigned char chVendorName[INFO_MAX_BUFFER_SIZE];
    unsigned char chModelName[INFO_MAX_BUFFER_SIZE];
    unsigned char chManufacturerInfo[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceVersion[INFO_MAX_BUFFER_SIZE];
    unsigned char chSerialNumber[INFO_MAX_BUFFER_SIZE];
    unsigned char chUserDefinedName[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceID[INFO_MAX_BUFFER_SIZE];
    unsigned int  nReserved[7];
} MV_CXP_DEVICE_INFO;

typedef struct _MV_CML_DEVICE_INFO_
{
    unsigned char chInterfaceID[INFO_MAX_BUFFER_SIZE];
    unsigned char chVendorName[INFO_MAX_BUFFER_SIZE];
    unsigned char chModelName[INFO_MAX_BUFFER_SIZE];
    unsigned char chManufacturerInfo[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceVersion[INFO_MAX_BUFFER_SIZE];
    unsigned char chSerialNumber[INFO_MAX_BUFFER_SIZE];
    unsigned char chUserDefinedName[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceID[INFO_MAX_BUFFER_SIZE];
    unsigned int  nReserved[7];
} MV_CML_DEVICE_INFO;

typedef struct _MV_XOF_DEVICE_INFO_
{
    unsigned char chInterfaceID[INFO_MAX_BUFFER_SIZE];
    unsigned char chVendorName[INFO_MAX_BUFFER_SIZE];
    unsigned char chModelName[INFO_MAX_BUFFER_SIZE];
    unsigned char chManufacturerInfo[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceVersion[INFO_MAX_BUFFER_SIZE];
    unsigned char chSerialNumber[INFO_MAX_BUFFER_SIZE];
    unsigned char chUserDefinedName[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceID[INFO_MAX_BUFFER_SIZE];
    unsigned int  nReserved[7];
} MV_XOF_DEVICE_INFO;

typedef struct _MV_GENTL_VIR_DEVICE_INFO_
{
    unsigned char chInterfaceID[INFO_MAX_BUFFER_SIZE];
    unsigned char chVendorName[INFO_MAX_BUFFER_SIZE];
    unsigned char chModelName[INFO_MAX_BUFFER_SIZE];
    unsigned char chManufacturerInfo[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceVersion[INFO_MAX_BUFFER_SIZE];
    unsigned char chSerialNumber[INFO_MAX_BUFFER_SIZE];
    unsigned char chUserDefinedName[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceID[INFO_MAX_BUFFER_SIZE];
    unsigned char chTLType[INFO_MAX_BUFFER_SIZE];
    unsigned int  nReserved[7];
} MV_GENTL_VIR_DEVICE_INFO;

#define MV_UNKNOW_DEVICE                0x00000000
#define MV_GIGE_DEVICE                  0x00000001
#define MV_1394_DEVICE                  0x00000002
#define MV_USB_DEVICE                   0x00000004
#define MV_CAMERALINK_DEVICE            0x00000008
#define MV_VIR_GIGE_DEVICE              0x00000010
#define MV_VIR_USB_DEVICE               0x00000020
#define MV_GENTL_GIGE_DEVICE            0x00000040
#define MV_GENTL_CAMERALINK_DEVICE      0x00000080
#define MV_GENTL_CXP_DEVICE             0x00000100
#define MV_GENTL_XOF_DEVICE             0x00000200
#define MV_GENTL_VIR_DEVICE             0x00000800

typedef struct _MV_CC_DEVICE_INFO_
{
    unsigned short nMajorVer;
    unsigned short nMinorVer;
    unsigned int   nMacAddrHigh;
    unsigned int   nMacAddrLow;
    unsigned int   nTLayerType;
    unsigned int   nDevTypeInfo;
    unsigned int   nReserved[3];
    union {
        MV_GIGE_DEVICE_INFO      stGigEInfo;
        MV_USB3_DEVICE_INFO      stUsb3VInfo;
        MV_CamL_DEV_INFO         stCamLInfo;
        MV_CML_DEVICE_INFO       stCMLInfo;
        MV_CXP_DEVICE_INFO       stCXPInfo;
        MV_XOF_DEVICE_INFO       stXoFInfo;
        MV_GENTL_VIR_DEVICE_INFO stVirInfo;
    } SpecialInfo;
} MV_CC_DEVICE_INFO;

#define MV_MAX_TLS_NUM                  8
#define MV_MAX_DEVICE_NUM               256

typedef struct _MV_CC_DEVICE_INFO_LIST_
{
    unsigned int        nDeviceNum;
    MV_CC_DEVICE_INFO*  pDeviceInfo[MV_MAX_DEVICE_NUM];
} MV_CC_DEVICE_INFO_LIST;

#define MV_GIGE_INTERFACE               0x00000001
#define MV_CAMERALINK_INTERFACE         0x00000004
#define MV_CXP_INTERFACE                0x00000008
#define MV_XOF_INTERFACE                0x00000010
#define MV_VIR_INTERFACE                0x00000020

#define MV_MAX_INTERFACE_NUM            64

typedef struct _MV_INTERFACE_INFO_
{
    unsigned int  nTLayerType;
    unsigned int  nPCIEInfo;
    unsigned char chInterfaceID[INFO_MAX_BUFFER_SIZE];
    unsigned char chDisplayName[INFO_MAX_BUFFER_SIZE];
    unsigned char chSerialNumber[INFO_MAX_BUFFER_SIZE];
    unsigned char chModelName[INFO_MAX_BUFFER_SIZE];
    unsigned char chManufacturer[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceVersion[INFO_MAX_BUFFER_SIZE];
    unsigned char chUserDefinedName[INFO_MAX_BUFFER_SIZE];
    unsigned int  nReserved[64];
} MV_INTERFACE_INFO;

typedef struct _MV_INTERFACE_INFO_LIST_
{
    unsigned int nInterfaceNum;
    MV_INTERFACE_INFO* pInterfaceInfos[MV_MAX_INTERFACE_NUM];
} MV_INTERFACE_INFO_LIST;

typedef struct _MV_GENTL_IF_INFO_
{
    unsigned char chInterfaceID[INFO_MAX_BUFFER_SIZE];
    unsigned char chTLType[INFO_MAX_BUFFER_SIZE];
    unsigned char chDisplayName[INFO_MAX_BUFFER_SIZE];
    unsigned int  nCtiIndex;
    unsigned int  nReserved[8];
} MV_GENTL_IF_INFO;

#define MV_MAX_GENTL_IF_NUM             256

typedef struct _MV_GENTL_IF_INFO_LIST_
{
    unsigned int      nInterfaceNum;
    MV_GENTL_IF_INFO* pIFInfo[MV_MAX_GENTL_IF_NUM];
} MV_GENTL_IF_INFO_LIST;

typedef struct _MV_GENTL_DEV_INFO_
{
    unsigned char chInterfaceID[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceID[INFO_MAX_BUFFER_SIZE];
    unsigned char chVendorName[INFO_MAX_BUFFER_SIZE];
    unsigned char chModelName[INFO_MAX_BUFFER_SIZE];
    unsigned char chTLType[INFO_MAX_BUFFER_SIZE];
    unsigned char chDisplayName[INFO_MAX_BUFFER_SIZE];
    unsigned char chUserDefinedName[INFO_MAX_BUFFER_SIZE];
    unsigned char chSerialNumber[INFO_MAX_BUFFER_SIZE];
    unsigned char chDeviceVersion[INFO_MAX_BUFFER_SIZE];
    unsigned int  nCtiIndex;
    unsigned int  nReserved[8];
} MV_GENTL_DEV_INFO;

#define MV_MAX_GENTL_DEV_NUM            256

typedef struct _MV_GENTL_DEV_INFO_LIST_
{
    unsigned int       nDeviceNum;
    MV_GENTL_DEV_INFO* pDeviceInfo[MV_MAX_GENTL_DEV_NUM];
} MV_GENTL_DEV_INFO_LIST;

#define MV_ACCESS_Exclusive                     1
#define MV_ACCESS_ExclusiveWithSwitch           2
#define MV_ACCESS_Control                       3
#define MV_ACCESS_ControlWithSwitch             4
#define MV_ACCESS_ControlSwitchEnable           5
#define MV_ACCESS_ControlSwitchEnableWithKey    6
#define MV_ACCESS_Monitor                       7

typedef struct _MV_CHUNK_DATA_CONTENT_
{
    unsigned char*      pChunkData;                                 ///< [OUT] \~chinese Chunk           \~english Chunk Data
    unsigned int        nChunkID;                                   ///< [OUT] \~chinese ChunkID               \~english Chunk ID
    unsigned int        nChunkLen;                                  ///< [OUT] \~chinese Chunk           \~english Chunk Length

    unsigned int        nReserved[8];                               ///<       \~chinese                 \~english Reserved

}MV_CHUNK_DATA_CONTENT;


typedef struct _MV_CC_IMAGE_
{
    unsigned int        nWidth;                                     ///< \~chinese       \~english Width
    unsigned int        nHeight;                                    ///< \~chinese        \~english Height
    enum MvGvspPixelType enPixelType;                                ///< ~chinese      \~english Pixel type

    unsigned char*      pImageBuf;                                  ///< \~chinese   \~english Image buffer
    uint64_t            nImageBufSize;                              ///< \~chinese  \~english Image buffer size
    uint64_t            nImageLen;                                  ///< \~chinese    \~english Image length

    unsigned int        nReserved[4];                               ///<        \~chinese     \~english Reserved

}MV_CC_IMAGE;

typedef struct _MV_FRAME_OUT_INFO_EX_
{
    unsigned short          nWidth;                                 ///< [OUT] \~chinese nExtendWidth)    \~english Image Width (over 65535, use nExtendWidth)
    unsigned short          nHeight;                                
    enum MvGvspPixelType    enPixelType;                            

    unsigned int            nFrameNum;                              ///< [OUT] \~chinese             \~english Frame Number
    unsigned int            nDevTimeStampHigh;                      ///< [OUT] \~chinese           \~english Timestamp high 32 bits
    unsigned int            nDevTimeStampLow;                       ///< [OUT] \~chinese       \~english Timestamp low 32 bits
    unsigned int            nReserved0;                             ///< [OUT] \~chinese        \~english Reserved, 8-byte aligned
    int64_t                 nHostTimeStamp;                         ///< [OUT] \~chinese       \~english Host-generated timestamp

    unsigned int            nFrameLen;                              ///< [OUT] \~chinese              \~english The Length of Frame

    
    unsigned int            nSecondCount;                           ///< [OUT] \~chinese               \~english The Seconds
    unsigned int            nCycleCount;                            ///< [OUT] \~chinese                  \~english The Count of Cycle
    unsigned int            nCycleOffset;                           ///< [OUT] \~chinese            \~english The Offset of Cycle

    float                   fGain;                                  ///< [OUT] \~chinese                \~english Gain
    float                   fExposureTime;                          ///< [OUT] \~chinese             \~english Exposure Time
    unsigned int            nAverageBrightness;                     ///< [OUT] \~chinese              \~english Average brightness

    /// \~chinese       \~english White balance
    unsigned int            nRed;                                   ///< [OUT] \~chinese                 \~english Red
    unsigned int            nGreen;                                 ///< [OUT] \~chinese                  \~english Green
    unsigned int            nBlue;                                  ///< [OUT] \~chinese                   \~english Blue

    unsigned int            nFrameCounter;                          ///< [OUT] \~chinese                  \~english Frame Counter
    unsigned int            nTriggerIndex;                          ///< [OUT] \~chinese                \~english Trigger Counting

    unsigned int            nInput;                                 ///< [OUT] \~chinese                   \~english Input
    unsigned int            nOutput;                                ///< [OUT] \~chinese                  \~english Output

    unsigned short          nOffsetX;                               ///< [OUT] \~chinese              \~english OffsetX
    unsigned short          nOffsetY;                               ///< [OUT] \~chinese             \~english OffsetY
    unsigned short          nChunkWidth;                            ///< [OUT] \~chinese              \~english The Width of Chunk
    unsigned short          nChunkHeight;                           ///< [OUT] \~chinese                \~english The Height of Chunk

    unsigned int            nLostPacket;                            ///< [OUT] \~chinese             \~english Lost Packet Number In This Frame

    unsigned int            nUnparsedChunkNum;                      ///< [OUT] \~chinese \~english Unparsed Chunk Number
    union
    {
        MV_CHUNK_DATA_CONTENT*  pUnparsedChunkContent;              ///< [OUT] \~chinese        \~english Unparsed Chunk Content
        int64_t                 nAligning;                          ///< [OUT] \~chinese                  \~english Aligning
    }UnparsedChunkList;

    unsigned int            nExtendWidth;                           ///< [OUT] \~chinese        \~english Image Width
    unsigned int            nExtendHeight;                          ///< [OUT] \~chinese     \~english Image Height

	uint64_t                nFrameLenEx;                            ///< [OUT] \~chinese         \~english The Length of Frame

    unsigned int            nReserved1;                             ///< [OUT] \~chinese       \~english Reserved
    unsigned int            nSubImageNum;                           ///< [OUT] \~chinese 
    union
    {
        MV_CC_IMAGE*        pstSubImage;                            ///< [OUT] \~chinese               \~english Sub image info
        int64_t             nAligning;                              ///< [OUT] \~chinese                \~english Aligning
    }SubImageList;


    union
    {
        void*               pUser;                                  ///< [OUT] \~chinese           \~english Custom pointer (user-defined pointer corresponding to memory address when registering external cache)
        int64_t             nAligning;                              ///< [OUT] \~chinese                   \~english Aligning
    }UserPtr;

	unsigned int            nReserved[26];                          ///<       \~chinese                 \~english Reserved               

}MV_FRAME_OUT_INFO_EX;

/// \~chinese    \~english Image Struct, output the pointer of Image and the information of the specific image
typedef struct _MV_FRAME_OUT_
{
    unsigned char*          pBufAddr;                               ///< [OUT] \~chinese        \~english  pointer of image
    MV_FRAME_OUT_INFO_EX    stFrameInfo;                            ///< [OUT] \~chinese               \~english information of the specific image

    unsigned int            nRes[16];                               ///<       \~chinese                   \~english Reserved

}MV_FRAME_OUT;

/// \~chinese              \~english The strategy of Grabbing
typedef enum _MV_GRAB_STRATEGY_
{
    MV_GrabStrategy_OneByOne            = 0,                        ///< \~chinese    \~english Grab One By One
    MV_GrabStrategy_LatestImagesOnly    = 1,                        ///< \~chinese   \~english Grab The Latest Image
    MV_GrabStrategy_LatestImages        = 2,                        ///< \~chinese         \~english Grab The Latest Images
    MV_GrabStrategy_UpcomingImage       = 3,                        ///< \~chinese              \~english Grab The Upcoming Image

}MV_GRAB_STRATEGY;

/// \~chinese       \~english Network transmission information
typedef struct _MV_NETTRANS_INFO_
{
    int64_t             nReceiveDataSize;                           ///< [OUT] \~chinese  \~english Received Data Size
    int                 nThrowFrameCount;                           ///< [OUT] \~chinese             \~english Throw frame number
    unsigned int        nNetRecvFrameCount;                         ///< [OUT] \~chinese                \~english Received Frame Count
    int64_t             nRequestResendPacketCount;                  ///< [OUT] \~chinese                      \~english Request Resend Packet Count
    int64_t             nResendPacketCount;                         ///< [OUT] \~chinese                        \~english Resend Packet Count

}MV_NETTRANS_INFO;

/// \~chinese                   \~english Information Type
#define MV_MATCH_TYPE_NET_DETECT        0x00000001                  ///< \~chinese             \~english Network traffic and packet loss information
#define MV_MATCH_TYPE_USB_DETECT        0x00000002                  ///< \~chinese  \~english The total number of bytes host received from U3V device

/// \~chinese    \~english A fully matched information structure
typedef struct _MV_ALL_MATCH_INFO_
{
    unsigned int        nType;                                      ///< [IN]  \~chinese . MV_MATCH_TYPE_NET_DETECT��MV_MATCH_TYPE_USB_DETECT  \~english Information type need to output ,e.g. MV_MATCH_TYPE_NET_DETECT��MV_MATCH_TYPE_USB_DETECT
    void*               pInfo;                                      ///< [OUT] \~chinese                      \~english Output information cache, which is allocated by the caller
    unsigned int        nInfoSize;                                  ///< [IN]  \~chinese 
}MV_ALL_MATCH_INFO;

/// \~chinese    \~english Network traffic and packet loss feedback structure, the corresponding type is MV_MATCH_TYPE_NET_DETECT
typedef struct _MV_MATCH_INFO_NET_DETECT_
{
    int64_t             nReceiveDataSize;                           ///< [OUT] \~chinese     \~english Received data size 
    int64_t             nLostPacketCount;                           ///< [OUT] \~chinese                          \~english Number of packets lost
    unsigned int        nLostFrameCount;                            ///< [OUT] \~chinese                     \~english Number of frames lost
    unsigned int        nNetRecvFrameCount;                         ///< [OUT] \~chinese                      \~english Received Frame Count
    int64_t             nRequestResendPacketCount;                  ///< [OUT] \~chinese                         \~english Request Resend Packet Count
    int64_t             nResendPacketCount;                         ///< [OUT] \~chinese                       \~english Resend Packet Count

}MV_MATCH_INFO_NET_DETECT;

typedef struct _MV_MATCH_INFO_USB_DETECT_
{
    int64_t             nReceiveDataSize;                           ///< [OUT] \~chinese 
    unsigned int        nReceivedFrameCount;                        ///< [OUT] \~chinese 
    unsigned int        nErrorFrameCount;                           ///< [OUT] \~chinese 

    unsigned int        nReserved[2];                               ///<       \~chinese 

}MV_MATCH_INFO_USB_DETECT;

typedef struct _MV_DISPLAY_FRAME_INFO_EX_ {
    unsigned int            nWidth;
    unsigned int            nHeight;
    enum MvGvspPixelType    enPixelType;
    unsigned char*          pImageBuf;
    unsigned int            nImageBufLen;
    unsigned int            enRenderMode;
    unsigned int            nRes[3];
} MV_DISPLAY_FRAME_INFO_EX;

enum MV_SAVE_IAMGE_TYPE {
    MV_Image_Undefined = 0,
    MV_Image_Bmp       = 1,
    MV_Image_Jpeg      = 2,
    MV_Image_Png       = 3,
    MV_Image_Tif       = 4,
};

typedef struct _MV_SAVE_IMAGE_PARAM_EX3_ {
    unsigned char*          pData;
    unsigned int            nDataLen;
    enum MvGvspPixelType    enPixelType;
    unsigned int            nWidth;
    unsigned int            nHeight;
    unsigned char*          pImageBuffer;
    unsigned int            nImageLen;
    unsigned int            nBufferSize;
    enum MV_SAVE_IAMGE_TYPE enImageType;
    unsigned int            nJpgQuality;
    unsigned int            iMethodValue;
    unsigned int            nReserved[3];
} MV_SAVE_IMAGE_PARAM_EX3;

typedef struct _MV_SAVE_IMAGE_TO_FILE_PARAM_EX_ {
    unsigned int            nWidth;
    unsigned int            nHeight;
    enum MvGvspPixelType    enPixelType;
    unsigned char*          pData;
    unsigned int            nDataLen;
    enum MV_SAVE_IAMGE_TYPE enImageType;
    char*                   pcImagePath;
    unsigned int            nQuality;
    int                     iMethodValue;
    unsigned int            nReserved[8];
} MV_SAVE_IMAGE_TO_FILE_PARAM_EX;

typedef struct _MV_CC_SAVE_IMAGE_PARAM_ {
    enum MV_SAVE_IAMGE_TYPE enImageType;
    unsigned int            nQuality;
    int                     iMethodValue;
    unsigned int            nReserved[8];
} MV_CC_SAVE_IMAGE_PARAM;

typedef enum _MV_IMG_ROTATION_ANGLE_ {
    MV_IMAGE_ROTATE_90  = 1,
    MV_IMAGE_ROTATE_180 = 2,
    MV_IMAGE_ROTATE_270 = 3,
}MV_IMG_ROTATION_ANGLE;

typedef struct _MV_CC_ROTATE_IMAGE_PARAM_T_ {
    enum MvGvspPixelType    enPixelType;
    unsigned int            nWidth;
    unsigned int            nHeight;
    unsigned char*          pSrcData;
    unsigned int            nSrcDataLen;
    unsigned char*          pDstBuf;
    unsigned int            nDstBufLen;
    unsigned int            nDstBufSize;
    MV_IMG_ROTATION_ANGLE   enRotationAngle;
    unsigned int            nRes[8];
} MV_CC_ROTATE_IMAGE_PARAM;

enum MV_IMG_FLIP_TYPE {
    MV_FLIP_VERTICAL   = 1,
    MV_FLIP_HORIZONTAL = 2,
};

typedef struct _MV_CC_FLIP_IMAGE_PARAM_T_ {
    enum MvGvspPixelType    enPixelType;
    unsigned int            nWidth;
    unsigned int            nHeight;
    unsigned char*          pSrcData;
    unsigned int            nSrcDataLen;
    unsigned char*          pDstBuf;
    unsigned int            nDstBufLen;
    unsigned int            nDstBufSize;
    MV_IMG_FLIP_TYPE        enFlipType;
    unsigned int            nRes[8];
} MV_CC_FLIP_IMAGE_PARAM;

typedef struct _MV_CC_PIXEL_CONVERT_PARAM_EX_ {
    unsigned int            nWidth;
    unsigned int            nHeight;
    enum MvGvspPixelType    enSrcPixelType;
    unsigned char*          pSrcData;
    unsigned int            nSrcDataLen;
    enum MvGvspPixelType    enDstPixelType;
    unsigned char*          pDstBuffer;
    unsigned int            nDstLen;
    unsigned int            nDstBufferSize;
    unsigned int            nRes[4];
} MV_CC_PIXEL_CONVERT_PARAM_EX;

typedef enum _MV_CC_GAMMA_TYPE_ {
    MV_CC_GAMMA_TYPE_NONE        = 0,
    MV_CC_GAMMA_TYPE_VALUE       = 1,
    MV_CC_GAMMA_TYPE_USER_CURVE  = 2,
    MV_CC_GAMMA_TYPE_LRGB2SRGB   = 3,
    MV_CC_GAMMA_TYPE_SRGB2LRGB   = 4,
}MV_CC_GAMMA_TYPE;

typedef struct _MV_CC_GAMMA_PARAM_T_ {
    MV_CC_GAMMA_TYPE enGammaType;
    float            fGammaValue;
    unsigned char*   pGammaCurveBuf;
    unsigned int     nGammaCurveBufLen;
    unsigned int     nRes[8];
} MV_CC_GAMMA_PARAM;

typedef struct _MV_CC_CCM_PARAM_T_ {
    bool        bCCMEnable;
    int         nCCMat[9];
    unsigned int nRes[8];
} MV_CC_CCM_PARAM;

typedef struct _MV_CC_CCM_PARAM_EX_T_ {
    bool        bCCMEnable;
    int         nCCMat[9];
    unsigned int nCCMScale;
    unsigned int nRes[8];
} MV_CC_CCM_PARAM_EX;

typedef struct _MV_CC_CONTRAST_PARAM_T_ {
    unsigned int            nWidth;
    unsigned int            nHeight;
    unsigned char*          pSrcBuf;
    unsigned int            nSrcBufLen;
    enum MvGvspPixelType    enPixelType;
    unsigned char*          pDstBuf;
    unsigned int            nDstBufSize;
    unsigned int            nDstBufLen;
    unsigned int            nContrastFactor;
    unsigned int            nRes[8];
} MV_CC_CONTRAST_PARAM;

typedef struct _MV_CC_FRAME_SPEC_INFO_ {
    unsigned int nSecondCount;
    unsigned int nCycleCount;
    unsigned int nCycleOffset;
    float        fGain;
    float        fExposureTime;
    unsigned int nAverageBrightness;
    unsigned int nRed;
    unsigned int nGreen;
    unsigned int nBlue;
    unsigned int nFrameCounter;
    unsigned int nTriggerIndex;
    unsigned int nInput;
    unsigned int nOutput;
    unsigned short nOffsetX;
    unsigned short nOffsetY;
    unsigned short nFrameWidth;
    unsigned short nFrameHeight;
    unsigned int nReserved[16];
} MV_CC_FRAME_SPEC_INFO;

typedef struct _MV_CC_PURPLE_FRINGING_PARAM_T_ {
    unsigned int            nWidth;
    unsigned int            nHeight;
    unsigned char*          pSrcBuf;
    unsigned int            nSrcBufLen;
    enum MvGvspPixelType    enPixelType;
    unsigned char*          pDstBuf;
    unsigned int            nDstBufSize;
    unsigned int            nDstBufLen;
    unsigned int            nKernelSize;
    unsigned int            nEdgeThreshold;
    unsigned int            nRes[8];
} MV_CC_PURPLE_FRINGING_PARAM;

typedef struct _MV_CC_ISP_CONFIG_PARAM_T_ {
    char* pcConfigPath;
    unsigned int nRes[16];
} MV_CC_ISP_CONFIG_PARAM;

typedef struct _MV_CC_HB_DECODE_PARAM_T_
{
    unsigned char*          pSrcBuf;                                ///< [IN]  \~chinese         \~english Input data buffer
    unsigned int            nSrcLen;                                ///< [IN]  \~chinese      \~english Input data size

    unsigned int            nWidth;                                 ///< [OUT] \~chinese               \~english Width
    unsigned int            nHeight;                                ///< [OUT] \~chinese        \~english Height
    unsigned char*          pDstBuf;                                ///< [OUT] \~chinese       \~english Output data buffer
    unsigned int            nDstBufSize;                            ///< [IN]  \~chinese    \~english Provided output buffer size
    unsigned int            nDstBufLen;                             ///< [OUT] \~chinese        \~english Output data size
    enum MvGvspPixelType    enDstPixelType;                         ///< [OUT] \~chinese         \~english Output pixel format

    MV_CC_FRAME_SPEC_INFO   stFrameSpecInfo;                        ///< [OUT] \~chinese              \~english Frame Spec Info

    unsigned int            nRes[8];                                ///<       \~chinese                  \~english Reserved

}MV_CC_HB_DECODE_PARAM;

typedef enum _MV_RECORD_FORMAT_TYPE_
{
    MV_FormatType_Undefined             = 0,                        ///< \~chinese            \~english Undefined Recode Format Type
    MV_FormatType_AVI                   = 1,                        ///< \~chinese           \~english AVI Recode Format Type

}MV_RECORD_FORMAT_TYPE;

/// \~chinese                  \~english Record Parameters
typedef struct _MV_CC_RECORD_PARAM_T_
{
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese
    unsigned short          nWidth;                                 ///< [IN]  \~chinese 
    unsigned short          nHeight;                                ///< [IN]  \~chinese

    float                   fFrameRate;                             ///< [IN]  \~chinese 
    unsigned int            nBitRate;                               ///< [IN]  \~chinese 

    MV_RECORD_FORMAT_TYPE   enRecordFmtType;                        ///< [IN]  \~chinese 

    char*                   strFilePath;                            ///< [IN]  \~chinese 

    unsigned int            nRes[8];                                ///<       \~chinese 
}MV_CC_RECORD_PARAM;

/// \~chinese           \~english Input Data
typedef struct _MV_CC_INPUT_FRAME_INFO_T_
{
    unsigned char*      pData;                                      ///< [IN]  \~chinese         \~english Record Data
    unsigned int        nDataLen;                                   ///< [IN]  \~chinese            \~english The Length of Record Data

    unsigned int        nRes[8];                                    ///<       \~chinese                   \~english Reserved

}MV_CC_INPUT_FRAME_INFO;

/// \~chinese                 \~english Acquisition mode
typedef enum _MV_CAM_ACQUISITION_MODE_
{
    MV_ACQ_MODE_SINGLE                  = 0,                        ///< \~chinese                     \~english Single Mode
    MV_ACQ_MODE_MUTLI                   = 1,                        ///< \~chinese                  \~english Multi Mode
    MV_ACQ_MODE_CONTINUOUS              = 2,                        ///< \~chinese                 \~english Continuous Mode

}MV_CAM_ACQUISITION_MODE;

/// \~chinese            \~english Gain Mode
typedef enum _MV_CAM_GAIN_MODE_
{
    MV_GAIN_MODE_OFF                    = 0,                        ///< \~chinese                        \~english Single Mode
    MV_GAIN_MODE_ONCE                   = 1,                        ///< \~chinese                        \~english Multi Mode
    MV_GAIN_MODE_CONTINUOUS             = 2,                        ///< \~chinese                         \~english Continuous Mode

}MV_CAM_GAIN_MODE;

/// \~chinese                \~english Exposure Mode
typedef enum _MV_CAM_EXPOSURE_MODE_
{
    MV_EXPOSURE_MODE_TIMED              = 0,                        ///< \~chinese                      \~english Timed
    MV_EXPOSURE_MODE_TRIGGER_WIDTH      = 1,                        ///< \~chinese               \~english TriggerWidth
}MV_CAM_EXPOSURE_MODE;

/// \~chinese         \~english Auto Exposure Mode
typedef enum _MV_CAM_EXPOSURE_AUTO_MODE_
{
    MV_EXPOSURE_AUTO_MODE_OFF           = 0,                        ///< \~chinese                       \~english Off
    MV_EXPOSURE_AUTO_MODE_ONCE          = 1,                        ///< \~chinese                       \~english Once
    MV_EXPOSURE_AUTO_MODE_CONTINUOUS    = 2,                        ///< \~chinese                         \~english Continuous

}MV_CAM_EXPOSURE_AUTO_MODE;

/// \~chinese                  \~english Trigger Mode
typedef enum _MV_CAM_TRIGGER_MODE_
{
    MV_TRIGGER_MODE_OFF                 = 0,                        ///< \~chinese                       \~english Off
    MV_TRIGGER_MODE_ON                  = 1,                        ///< \~chinese                         \~english ON

}MV_CAM_TRIGGER_MODE;

/// \~chinese              \~english Gamma Selector
typedef enum _MV_CAM_GAMMA_SELECTOR_
{
    MV_GAMMA_SELECTOR_USER              = 1,                        ///< \~chinese                         \~english Gamma Selector User
    MV_GAMMA_SELECTOR_SRGB              = 2,                        ///< \~chinese sRGB                         \~english Gamma Selector sRGB

}MV_CAM_GAMMA_SELECTOR;

/// \~chinese           \~english White Balance
typedef enum _MV_CAM_BALANCEWHITE_AUTO_
{
    MV_BALANCEWHITE_AUTO_OFF            = 0,                        ///< \~chinese                        \~english Off
    MV_BALANCEWHITE_AUTO_ONCE           = 2,                        ///< \~chinese                       \~english Once
    MV_BALANCEWHITE_AUTO_CONTINUOUS     = 1,                        ///< \~chinese                     \~english Continuous

}MV_CAM_BALANCEWHITE_AUTO;

/// \~chines                   \~english Trigger Source
typedef enum _MV_CAM_TRIGGER_SOURCE_
{
    MV_TRIGGER_SOURCE_LINE0             = 0,                        ///< \~chinese Line0                        \~english Line0
    MV_TRIGGER_SOURCE_LINE1             = 1,                        ///< \~chinese Line1                        \~english Line1
    MV_TRIGGER_SOURCE_LINE2             = 2,                        ///< \~chinese Line2                        \~english Line2
    MV_TRIGGER_SOURCE_LINE3             = 3,                        ///< \~chinese Line3                        \~english Line3
    MV_TRIGGER_SOURCE_COUNTER0          = 4,                        ///< \~chinese Conuter0                     \~english Conuter0

    MV_TRIGGER_SOURCE_SOFTWARE          = 7,                        ///< \~chinese                      \~english Software
    MV_TRIGGER_SOURCE_FrequencyConverter= 8,                        ///< \~chinese                     \~english Frequency Converter

}MV_CAM_TRIGGER_SOURCE;

/// \~chinese GigEVisio       \~english GigEVision IP Configuration
#define MV_IP_CFG_STATIC                0x05000000                  ///< \~chinese                       \~english Static
#define MV_IP_CFG_DHCP                  0x06000000                  ///< \~chinese DHCP                         \~english DHCP
#define MV_IP_CFG_LLA                   0x04000000                  ///< \~chinese LLA                          \~english LLA

/// \~chinese GigEVision    \~english GigEVision Net Transfer Mode
#define MV_NET_TRANS_DRIVER             0x00000001                  ///< \~chinese                       \~english Driver
#define MV_NET_TRANS_SOCKET             0x00000002                  ///< \~chinese Socket                       \~english Socket

/// \~chinese CameraLink          \~english CameraLink Baud Rates (CLUINT32)
#define MV_CAML_BAUDRATE_9600           0x00000001                  ///< \~chinese 9600                         \~english 9600
#define MV_CAML_BAUDRATE_19200          0x00000002                  ///< \~chinese 19200                        \~english 19200
#define MV_CAML_BAUDRATE_38400          0x00000004                  ///< \~chinese 38400                        \~english 38400
#define MV_CAML_BAUDRATE_57600          0x00000008                  ///< \~chinese 57600                        \~english 57600
#define MV_CAML_BAUDRATE_115200         0x00000010                  ///< \~chinese 115200                       \~english 115200
#define MV_CAML_BAUDRATE_230400         0x00000020                  ///< \~chinese 230400                       \~english 230400
#define MV_CAML_BAUDRATE_460800         0x00000040                  ///< \~chinese 460800                       \~english 460800
#define MV_CAML_BAUDRATE_921600         0x00000080                  ///< \~chinese 921600                       \~english 921600
#define MV_CAML_BAUDRATE_AUTOMAX        0x40000000                  ///< \~chinese                      \~english Auto Max

/// \~chinese          \~english Exception message type
#define MV_EXCEPTION_DEV_DISCONNECT     0x00008001                  ///< \~chinese                \~english The device is disconnected
#define MV_EXCEPTION_VERSION_CHECK      0x00008002                  ///< \~chinese          \~english SDK does not match the driver version

/// \~chinese 
typedef enum _MV_CC_STREAM_EXCEPTION_TYPE_
{
    MV_CC_STREAM_EXCEPTION_ABNORMAL_IMAGE   = 0x4001,               ///< \~chinese 
    MV_CC_STREAM_EXCEPTION_LIST_OVERFLOW    = 0x4002,               ///< \~chinese 
    MV_CC_STREAM_EXCEPTION_LIST_EMPTY       = 0x4003,               ///< \~chinese 
    MV_CC_STREAM_EXCEPTION_RECONNECTION     = 0x4004,               ///< \~chinese 
    MV_CC_STREAM_EXCEPTION_DISCONNECTED     = 0x4005,               ///< \~chinese 
    MV_CC_STREAM_EXCEPTION_DEVICE           = 0x4006,               ///< \~chinese 

}MV_CC_STREAM_EXCEPTION_TYPE;

#define MAX_EVENT_NAME_SIZE             128

typedef struct _MV_EVENT_OUT_INFO_ {
    char           EventName[MAX_EVENT_NAME_SIZE];
    unsigned short nEventID;
    unsigned short nStreamChannel;
    unsigned int   nBlockIdHigh;
    unsigned int   nBlockIdLow;
    unsigned int   nTimestampHigh;
    unsigned int   nTimestampLow;
    void*          pEventData;
    unsigned int   nEventDataSize;
    unsigned int   nReserved[16];
} MV_EVENT_OUT_INFO;

typedef struct _MV_CC_FILE_ACCESS_T {
    const char*  pUserFileName;
    const char*  pDevFileName;
    unsigned int nReserved[32];
} MV_CC_FILE_ACCESS;

typedef struct _MV_CC_FILE_ACCESS_E {
    char*        pUserFileBuf;
    unsigned int pFileBufSize;
    unsigned int pFileBufLen;
    const char*  pDevFileName;
    unsigned int nReserved[32];
} MV_CC_FILE_ACCESS_EX;

typedef struct _MV_CC_FILE_ACCESS_PROGRESS_T {
    int64_t      nCompleted;
    int64_t      nTotal;
    unsigned int nReserved[8];
} MV_CC_FILE_ACCESS_PROGRESS;

typedef enum _MV_GIGE_TRANSMISSION_TYPE_ {
    MV_GIGE_TRANSTYPE_UNICAST = 0x0,
    MV_GIGE_TRANSTYPE_MULTICAST = 0x1,
    MV_GIGE_TRANSTYPE_LIMITEDBROADCAST = 0x2,
    MV_GIGE_TRANSTYPE_SUBNETBROADCAST = 0x3,
    MV_GIGE_TRANSTYPE_CAMERADEFINED = 0x4,
    MV_GIGE_TRANSTYPE_UNICAST_DEFINED_PORT = 0x5,
    MV_GIGE_TRANSTYPE_UNICAST_WITHOUT_RECV = 0x00010000,
    MV_GIGE_TRANSTYPE_MULTICAST_WITHOUT_RECV = 0x00010001,
} MV_GIGE_TRANSMISSION_TYPE;

typedef struct _MV_TRANSMISSION_TYPE_T {
    MV_GIGE_TRANSMISSION_TYPE enTransmissionType;
    unsigned int              nDestIp;
    unsigned short            nDestPort;
    unsigned int              nReserved[32];
} MV_TRANSMISSION_TYPE;

typedef struct _MV_ACTION_CMD_INFO_T {
    unsigned int  nDeviceKey;
    unsigned int  nGroupKey;
    unsigned int  nGroupMask;
    unsigned int  bActionTimeEnable;
    int64_t       nActionTime;
    const char*   pBroadcastAddress;
    unsigned int  nTimeOut;
    unsigned int  bSpecialNetEnable;
    unsigned int  nSpecialNetIP;
    unsigned int  nReserved[14];
} MV_ACTION_CMD_INFO;

typedef struct _MV_ACTION_CMD_RESULT_T {
    unsigned char strDeviceAddress[16];
    int           nStatus;
    unsigned int  nReserved[4];
} MV_ACTION_CMD_RESULT;

typedef struct _MV_ACTION_CMD_RESULT_LIST_T {
    unsigned int          nNumResults;
    MV_ACTION_CMD_RESULT* pResults;
} MV_ACTION_CMD_RESULT_LIST;

enum MV_XML_InterfaceType {
    IFT_IValue,
    IFT_IBase,
    IFT_IInteger,
    IFT_IBoolean,
    IFT_ICommand,
    IFT_IFloat,
    IFT_IString,
    IFT_IRegister,
    IFT_ICategory,
    IFT_IEnumeration,
    IFT_IEnumEntry,
    IFT_IPort,
};

enum MV_XML_AccessMode {
    AM_NI,
    AM_NA,
    AM_WO,
    AM_RO,
    AM_RW,
    AM_Undefined,
    AM_CycleDetect,
};

#define MV_MAX_NODE_NUM        1024
#define MV_MAX_NODE_NAME_LEN   64
typedef struct _MVCC_NODE_NAME_T {
    char           strName[MV_MAX_NODE_NAME_LEN];
    unsigned int   nReserved[4];
} MVCC_NODE_NAME;

typedef struct _MVCC_NODE_NAME_LIST_T {
    unsigned int      nNodeNum;
    MVCC_NODE_NAME    stNodeName[MV_MAX_NODE_NUM];
    unsigned int      nReserved[4];
} MVCC_NODE_NAME_LIST;

#define MV_MAX_NODE_ERROR_NUM  64
typedef enum _MVCC_NODE_ERR_TYPE_ {
    MVCC_NODE_ERR_NODE_INVALID = 1,
    MVCC_NODE_ERR_ACCESS = 2,
    MVCC_NODE_ERR_OUT_RANGE = 3,
    MVCC_NODE_ERR_VERIFY_FAILD = 4,
    MVCC_NODE_ERR_OTHER = 100,
}MVCC_NODE_ERR_TYPE;

typedef struct _MVCC_NODE_ERROR_T {
    char               strName[MV_MAX_NODE_NAME_LEN];
    MVCC_NODE_ERR_TYPE enErrType;
    unsigned int       nReserved[4];
} MVCC_NODE_ERROR;

typedef struct _MVCC_NODE_ERROR_LIST_T {
    unsigned int      nErrorNum;
    MVCC_NODE_ERROR   stNodeError[MV_MAX_NODE_ERROR_NUM];
    unsigned int      nReserved[4];
} MVCC_NODE_ERROR_LIST;

#define MV_MAX_XML_SYMBOLIC_NUM 64
typedef struct _MVCC_ENUMVALUE_T {
    unsigned int nCurValue;
    unsigned int nSupportedNum;
    unsigned int nSupportValue[MV_MAX_XML_SYMBOLIC_NUM];
    unsigned int nReserved[4];
} MVCC_ENUMVALUE;

#define MV_MAX_ENUM_SYMBOLIC_NUM 256
typedef struct _MVCC_ENUMVALUE_EX_T {
    unsigned int nCurValue;
    unsigned int nSupportedNum;
    unsigned int nSupportValue[MV_MAX_ENUM_SYMBOLIC_NUM];
    unsigned int nReserved[4];
} MVCC_ENUMVALUE_EX;

#define MV_MAX_SYMBOLIC_LEN 64
typedef struct _MVCC_ENUMENTRY_T {
    unsigned int nValue;
    char         chSymbolic[MV_MAX_SYMBOLIC_LEN];
    unsigned int nReserved[4];
} MVCC_ENUMENTRY;

typedef struct _MVCC_INTVALUE_T {
    unsigned int nCurValue;
    unsigned int nMax;
    unsigned int nMin;
    unsigned int nInc;
    unsigned int nReserved[4];
} MVCC_INTVALUE;

typedef struct _MVCC_INTVALUE_EX_T {
    int64_t      nCurValue;
    int64_t      nMax;
    int64_t      nMin;
    int64_t      nInc;
    unsigned int nReserved[16];
} MVCC_INTVALUE_EX;

typedef struct _MVCC_FLOATVALUE_T
{
    float               fCurValue;                                 
    float               fMax;                                       
    float               fMin;                                      

    unsigned int        nReserved[4];                             

}MVCC_FLOATVALUE;

typedef struct _MVCC_STRINGVALUE_T
{
    char                chCurValue[256];                   

    int64_t             nMaxLength;                                 
    unsigned int        nReserved[2];                              
}MVCC_STRINGVALUE;


typedef struct _MVCC_COLORF
{
	float           fR;             
    float           fG;            
    float           fB;           
    float           fAlpha;        
    unsigned int    nReserved[4];   

}MVCC_COLORF;

typedef struct _MVCC_POINTF
{
    float           fX;            
    float           fY;             
    unsigned int    nReserved[4];  

}MVCC_POINTF;

typedef struct _MVCC_RECT_INFO
{
    float           fTop;           
    float           fBottom;       
    float           fLeft;         
    float           fRight;        
    MVCC_COLORF     stColor;        
    unsigned int    nLineWidth;     
    unsigned int    nReserved[4];  

}MVCC_RECT_INFO;


typedef struct _MVCC_CIRCLE_INFO
{
    MVCC_POINTF     stCenterPoint;  

    float           fR1;            
    float           fR2;           
    MVCC_COLORF     stColor;
    unsigned int    nLineWidth;     
    unsigned int    nReserved[4];   

}MVCC_CIRCLE_INFO;

typedef struct _MVCC_LINES_INFO
{
    MVCC_POINTF     stStartPoint;   ///< [IN] \~chinese         \~english The Start Point of Auxiliary Line
    MVCC_POINTF     stEndPoint;     ///< [IN] \~chinese            \~english The End Point of Auxiliary Line
    MVCC_COLORF     stColor;        ///< [IN] \~chinese                  \~english Color of Auxiliary Line
    unsigned int    nLineWidth;     ///< [IN] \~chinese      \~english Width of Auxiliary Line, width is 1 or 2
    unsigned int    nReserved[4];   ///<  \~chinese                         \~english Reserved

}MVCC_LINES_INFO;

///< \~chinese  \~english The maximum number of source image to be split in time-division exposure
#define MV_MAX_SPLIT_NUM                  8

/// \~chinese      \~english Image reconstruction method
typedef enum _MV_IMAGE_RECONSTRUCTION_METHOD_
{
	MV_SPLIT_BY_LINE = 1,         ///< \~chinese        \~english Source image split into multiple images by line

}MV_IMAGE_RECONSTRUCTION_METHOD;

/// \~chinese      \~english List of images after image reconstruction
typedef struct _MV_OUTPUT_IMAGE_INFO_
{
    unsigned int                    nWidth;        ///< [OUT]       \~chinese              \~english Source Image Width
    unsigned int                    nHeight;       ///< [OUT]       \~chinese               \~english Source Image Height
    enum MvGvspPixelType            enPixelType;   ///< [OUT]       \~chinese               \~english Pixel format

    unsigned char*                  pBuf;          ///< [IN][OUT]   \~chinese           \~english Output data buffer
    unsigned int                    nBufLen;       ///< [OUT]       \~chinese          \~english Output data length
    unsigned int                    nBufSize;      ///< [IN]        \~chinese   \~english Provided output buffer size

    unsigned int                    nRes[8];       ///<             \~chinese                 \~english Reserved
}MV_OUTPUT_IMAGE_INFO;

typedef struct _MV_RECONSTRUCT_IMAGE_PARAM_
{
    unsigned int                    nWidth;                             ///< [IN]  \~chinese          \~english Source Image Width
    unsigned int                    nHeight;                            ///< [IN]  \~chinese          \~english Source Image Height
    enum MvGvspPixelType            enPixelType;                        ///< [IN]  \~chinese          \~english Pixel format

    unsigned char*                  pSrcData;                           ///< [IN]  \~chinese      \~english Input data buffer
    unsigned int                    nSrcDataLen;                        ///< [IN]  \~chinese      \~english Input data length

    unsigned int                    nExposureNum;                       ///< [IN]  \~chinese     \~english Exposure number
    MV_IMAGE_RECONSTRUCTION_METHOD  enReconstructMethod;                ///< [IN]  \~chinese     \~english Image restructuring method

    MV_OUTPUT_IMAGE_INFO            stDstBufList[MV_MAX_SPLIT_NUM];     ///< [OUT] \~chinese  \~english Output data info

    unsigned int                    nRes[4];
}MV_RECONSTRUCT_IMAGE_PARAM;

typedef struct _MV_CAML_SERIAL_PORT_
{
    unsigned char       chSerialPort[INFO_MAX_BUFFER_SIZE];             ///< [OUT] \~chinese              \~english Serial Port

    unsigned int        nRes[4];                                       ///<\~chinese                         \~english Reserved
}MV_CAML_SERIAL_PORT;

#define MV_MAX_SERIAL_PORT_NUM     64

typedef struct _MV_CAML_SERIAL_PORT_LIST_
{
    unsigned int                  nSerialPortNum;                        ///< [OUT] \~chinese       \~english Serial Port Num
    MV_CAML_SERIAL_PORT           stSerialPort[MV_MAX_SERIAL_PORT_NUM];  ///< [IN][OUT] \~chinese          \~english Serial Port Information

    unsigned int                  nRes[4];                               ///<\~chinese                            \~english Reserved
}MV_CAML_SERIAL_PORT_LIST;

#endif /* _MV_CAMERA_PARAMS_H_ */

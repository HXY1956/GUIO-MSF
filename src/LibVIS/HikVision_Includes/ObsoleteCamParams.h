
#ifndef _MV_OBSOLETE_CAM_PARAMS_H_
#define _MV_OBSOLETE_CAM_PARAMS_H_

#include "PixelType.h"

typedef struct _MV_FRAME_OUT_INFO_ {
    unsigned short      nWidth;
    unsigned short      nHeight;
    enum MvGvspPixelType enPixelType;
    unsigned int        nFrameNum;
    unsigned int        nDevTimeStampHigh;
    unsigned int        nDevTimeStampLow;
    unsigned int        nReserved0;
    int64_t             nHostTimeStamp;
    unsigned int        nFrameLen;
    unsigned int        nLostPacket;
    unsigned int        nReserved[2];
} MV_FRAME_OUT_INFO;

typedef struct _MV_SAVE_IMAGE_PARAM_T_ {
    unsigned char*      pData;
    unsigned int        nDataLen;
    enum MvGvspPixelType enPixelType;
    unsigned short      nWidth;
    unsigned short      nHeight;
    unsigned char*      pImageBuffer;
    unsigned int        nImageLen;
    unsigned int        nBufferSize;
    enum MV_SAVE_IAMGE_TYPE enImageType;
} MV_SAVE_IMAGE_PARAM;

typedef struct _MV_IMAGE_BASIC_INFO_ {
    unsigned short nWidthValue;
    unsigned short nWidthMin;
    unsigned int   nWidthMax;
    unsigned int   nWidthInc;
    unsigned int   nHeightValue;
    unsigned int   nHeightMin;
    unsigned int   nHeightMax;
    unsigned int   nHeightInc;
    float          fFrameRateValue;
    float          fFrameRateMin;
    float          fFrameRateMax;
    unsigned int   enPixelType;
    unsigned int   nSupportedPixelFmtNum;
    unsigned int   enPixelList[64];
    unsigned int   nReserved[8];
} MV_IMAGE_BASIC_INFO;


/// \~chinese   \~english Noise feature type
typedef enum _MV_CC_BAYER_NOISE_FEATURE_TYPE
{
    MV_CC_BAYER_NOISE_FEATURE_TYPE_INVALID = 0, ///<        \~chinese                   \~english Invalid
    MV_CC_BAYER_NOISE_FEATURE_TYPE_PROFILE = 1, ///<        \~chinese                    \~english Noise curve
    MV_CC_BAYER_NOISE_FEATURE_TYPE_LEVEL   = 2, ///<        \~chinese                   \~english Noise level
    MV_CC_BAYER_NOISE_FEATURE_TYPE_DEFAULT = 1, ///<        \~chinese                     \~english Default

}MV_CC_BAYER_NOISE_FEATURE_TYPE;

/// \~chinese Bayer  \~english Denoise profile info
typedef struct _MV_CC_BAYER_NOISE_PROFILE_INFO_T_
{
    unsigned int        nVersion;           ///<  \~chinese                     \~english version
    MV_CC_BAYER_NOISE_FEATURE_TYPE enNoiseFeatureType;  ///<  \~chinese        \~english noise feature type
    enum MvGvspPixelType    enPixelType;    ///<  \~chinese       \~english image format
    int                 nNoiseLevel;        ///<  \~chinese               \~english noise level
    unsigned int        nCurvePointNum;     ///<  \~chinese                  \~english curve point number
    int*                nNoiseCurve;        ///<  \~chinese                        \~english noise curve
    int*                nLumCurve;          ///<  \~chinese                       \~english luminance curve

    unsigned int        nRes[8];            ///<       \~chinese                           \~english Reserved

}MV_CC_BAYER_NOISE_PROFILE_INFO;

/// \~chinese Bayer \~english Bayer noise estimate param
typedef struct _MV_CC_BAYER_NOISE_ESTIMATE_PARAM_T_
{
    unsigned int        nWidth;             ///< [IN]  \~chinese            \~english Width
    unsigned int        nHeight;            ///< [IN]  \~chinese               \~english Height
    enum MvGvspPixelType    enPixelType;    ///< [IN]  \~chinese                       \~english Pixel format

    unsigned char*      pSrcData;           ///< [IN]  \~chinese                  \~english Input data buffer
    unsigned int        nSrcDataLen;        ///< [IN]  \~chinese                \~english Input data size

    unsigned int        nNoiseThreshold;    ///< [IN]  \~chinese              \~english Noise Threshold

    unsigned char*      pCurveBuf;          ///< [IN]  \~chinese     \~english Buffer used to store noise and brightness curves, size:4096 * sizeof(int) * 2)
    MV_CC_BAYER_NOISE_PROFILE_INFO stNoiseProfile; ///< [OUT]  \~chinese              \~english Denoise profile

    unsigned int        nThreadNum;         ///< [IN]  \~chinese     \~english Thread number, 0 means that the library is adaptive to the hardware, 1 means single thread(Default value), Greater than 1 indicates the number of threads

    unsigned int        nRes[8];            ///<       \~chinese                       \~english Reserved

}MV_CC_BAYER_NOISE_ESTIMATE_PARAM;

/// \~chinese Baye    \~english Bayer spatial Denoise param
typedef struct _MV_CC_BAYER_SPATIAL_DENOISE_PARAM_T_
{
    unsigned int        nWidth;             ///< [IN]  \~chinese             \~english Width
    unsigned int        nHeight;            ///< [IN]  \~chinese              \~english Height
    enum MvGvspPixelType    enPixelType;        ///< [IN]  \~chinese                    \~english Pixel format

    unsigned char*      pSrcData;           ///< [IN]  \~chinese                   \~english Input data buffer
    unsigned int        nSrcDataLen;        ///< [IN]  \~chinese                 \~english Input data size

    unsigned char*      pDstBuf;            ///< [OUT] \~chinese               \~english Output data buffer
    unsigned int        nDstBufSize;        ///< [IN]  \~chinese          \~english Provided output buffer size
    unsigned int        nDstBufLen;         ///< [OUT] \~chinese         \~english Output data length

    MV_CC_BAYER_NOISE_PROFILE_INFO stNoiseProfile; ///< [IN]  \~chinese \~english Denoise profile
    unsigned int        nDenoiseStrength;   ///< [IN]  \~chinese                \~english nDenoise Strength   
    unsigned int        nSharpenStrength;   ///< [IN]  \~chinese                  \~english Sharpen Strength
    unsigned int        nNoiseCorrect;      ///< [IN]  \~chinese         \~english Noise Correct  

    unsigned int        nThreadNum;         ///< [IN]  \~chinese       \~english Thread number, 0 means that the library is adaptive to the hardware, 1 means single thread(Default value), Greater than 1 indicates the number of threads

    unsigned int        nRes[8];            ///<       \~chinese                    \~english Reserved

}MV_CC_BAYER_SPATIAL_DENOISE_PARAM;

/// \~chinese CLUT             \~english CLUT param
typedef struct _MV_CC_CLUT_PARAM_T_
{
    bool                bCLUTEnable;                                ///< [IN]  \~chinese           \~english CLUT enable
    unsigned int        nCLUTScale;                                 ///< [IN]  \~chinese   \~english Quantitative scale(Integer power of 2, <= 65536)
    unsigned int        nCLUTSize;                                  ///< [IN]  \~chinese    \~english CLUT size, currently only supports 17
    unsigned char*      pCLUTBuf;                                   ///< [IN]  \~chinese          \~english CLUT buffer
    unsigned int        nCLUTBufLen;                                ///< [IN]  \~chinese (nCLUTSize*nCLUTSize*nCLUTSize*sizeof(int)*3)  \~english CLUT buffer length(nCLUTSize*nCLUTSize*nCLUTSize*sizeof(int)*3)

    unsigned int        nRes[8];                                    ///<       \~chinese                  \~english Reserved

}MV_CC_CLUT_PARAM;

/// \~chinese       \~english Sharpen structure
typedef struct _MV_CC_SHARPEN_PARAM_T_
{
    unsigned int            nWidth;                                 ///< [IN]  \~chinese       \~english Image Width
    unsigned int            nHeight;                                ///< [IN]  \~chinese       \~english Image Height
    unsigned char*          pSrcBuf;                                ///< [IN]  \~chinese          \~english Input data buffer
    unsigned int            nSrcBufLen;                             ///< [IN]  \~chinese         \~english Input data length
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese               \~english Pixel format

    unsigned char*          pDstBuf;                                ///< [OUT] \~chinese           \~english Output data buffer
    unsigned int            nDstBufSize;                            ///< [IN]  \~chinese  \~english Provided output buffer size
    unsigned int            nDstBufLen;                             ///< [OUT] \~chinese      \~english Output data length

    unsigned int            nSharpenAmount;                         ///< [IN]  \~chinese   \~english Sharpen amount,[0,500]  //   [nSharpenAmount nSharpenPosAmount & nSharpenNegAmount ]
    unsigned int            nSharpenRadius;                         ///< [IN]  \~chinese  \~english Sharpen radius(The larger the radius, the longer it takes),[1,21]
    unsigned int            nSharpenThreshold;                      ///< [IN]  \~chinese   \~english Sharpen threshold,[0,255]


    unsigned int        nSharpenPosAmount;     // [IN]     
    unsigned int        nSharpenNegAmount;     // [IN]    

    unsigned int            nRes[6];                                ///<                       \~english Reserved

}MV_CC_SHARPEN_PARAM;

/// \~chinese     \~english Color correct structure
typedef struct _MV_CC_COLOR_CORRECT_PARAM_T_
{
    unsigned int            nWidth;                                 ///< [IN]  \~chinese            \~english Image Width
    unsigned int            nHeight;                                ///< [IN]  \~chinese           \~english Image Height
    unsigned char*          pSrcBuf;                                ///< [IN]  \~chinese        \~english Input data buffer
    unsigned int            nSrcBufLen;                             ///< [IN]  \~chinese        \~english Input data length
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese        \~english Pixel format

    unsigned char*          pDstBuf;                                ///< [OUT] \~chinese         \~english Output data buffer
    unsigned int            nDstBufSize;                            ///< [IN]  \~chinese  \~english Provided output buffer size
    unsigned int            nDstBufLen;                             ///< [OUT] \~chinese         \~english Output data length

    unsigned int            nImageBit;                              ///< [IN]  \~chinese  \~english Image bit(8 or 10 or 12 or 16)
    MV_CC_GAMMA_PARAM       stGammaParam;                           ///< [IN]  \~chinese     \~english Gamma info
    MV_CC_CCM_PARAM_EX      stCCMParam;                             ///< [IN]  \~chinese           \~english CCM info
    MV_CC_CLUT_PARAM        stCLUTParam;                            ///< [IN]  \~chinese            \~english CLUT info

    unsigned int            nRes[8];                                ///<       \~chinese                 \~english Reserved

}MV_CC_COLOR_CORRECT_PARAM;

/// \~chinese          \~english Rect ROI structure
typedef struct _MV_CC_RECT_I_
{
    unsigned int nX;                                                ///< \~chinese           \~english X Position
    unsigned int nY;                                                ///< \~chinese             \~english Y Position
    unsigned int nWidth;                                            ///< \~chinese                   \~english Rect Width
    unsigned int nHeight;                                           ///< \~chinese                    \~english Rect Height

}MV_CC_RECT_I;

/// \~chinese          \~english Noise estimate structure
typedef struct _MV_CC_NOISE_ESTIMATE_PARAM_T_
{
    unsigned int            nWidth;                                 ///< [IN]  \~chinese       \~english Image Width
    unsigned int            nHeight;                                ///< [IN]  \~chinese       \~english Image Height
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese           \~english Pixel format
    unsigned char*          pSrcBuf;                                ///< [IN]  \~chinese         \~english Input data buffer
    unsigned int            nSrcBufLen;                             ///< [IN]  \~chinese           \~english Input data length

    MV_CC_RECT_I*           pstROIRect;                             ///< [IN]  \~chinese              \~english Image ROI
    unsigned int            nROINum;                                ///< [IN]  \~chinese               \~english ROI number

    ///< \~chinese Bayer  \~english Bayer Noise estimate param,Mono8/RGB formats are invalid
    unsigned int            nNoiseThreshold;                        ///< [IN]  \~chinese [0,4095]       \~english Noise threshold[0,4095]
    ///< \~chinese ֵ:8bit,0xE0;10bit,0x380;12bit,0xE00     \~english Suggestive value:8bit,0xE0;10bit,0x380;12bit,0xE00

    unsigned char*          pNoiseProfile;                          ///< [OUT] \~chinese            \~english Output Noise Profile
    unsigned int            nNoiseProfileSize;                      ///< [IN]  \~chinese   \~english Provided output buffer size
    unsigned int            nNoiseProfileLen;                       ///< [OUT] \~chinese      \~english Output Noise Profile length

    unsigned int            nRes[8];                                ///<       \~chinese             \~english Reserved

}MV_CC_NOISE_ESTIMATE_PARAM;

/// \~chinese        \~english Spatial denoise structure
typedef struct _MV_CC_SPATIAL_DENOISE_PARAM_T_
{
    unsigned int            nWidth;                                 ///< [IN]  \~chinese        \~english Image Width
    unsigned int            nHeight;                                ///< [IN]  \~chinese       \~english Image Height
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese               \~english Pixel format
    unsigned char*          pSrcBuf;                                ///< [IN]  \~chinese           \~english Input data buffer
    unsigned int            nSrcBufLen;                             ///< [IN]  \~chinese            \~english Input data length

    unsigned char*          pDstBuf;                                ///< [OUT] \~chinese        \~english Output data buffer
    unsigned int            nDstBufSize;                            ///< [IN]  \~chinese   \~english Provided output buffer size
    unsigned int            nDstBufLen;                             ///< [OUT] \~chinese   \~english Output data length

    unsigned char*          pNoiseProfile;                          ///< [IN]  \~chinese           \~english Input Noise Profile
    unsigned int            nNoiseProfileLen;                       ///< [IN]  \~chinese      \~english Input Noise Profile length

    ///< \~chinese Bayer   \~english Bayer Spatial denoise param,Mono8/RGB formats are invalid
    unsigned int            nBayerDenoiseStrength;                  ///< [IN]  \~chinese         \~english Denoise Strength[0,100]
    unsigned int            nBayerSharpenStrength;                  ///< [IN]  \~chinese      \~english Sharpen Strength[0,32]
    unsigned int            nBayerNoiseCorrect;                     ///< [IN]  \~chinese   \~english Noise Correct[0,1280]

    ///< \~chinese Mono8/RGB   \~english Mono8/RGB Spatial denoise param,Bayer formats are invalid
    unsigned int            nNoiseCorrectLum;                       ///< [IN]  \~chinese   \~english Noise Correct Lum[1,2000]
    unsigned int            nNoiseCorrectChrom;                     ///< [IN]  \~chinese   \~english Noise Correct Chrom[1,2000]
    unsigned int            nStrengthLum;                           ///< [IN]  \~chinese     \~english Strength Lum[0,100]
    unsigned int            nStrengthChrom;                         ///< [IN]  \~chinese   \~english Strength Chrom[0,100]
    unsigned int            nStrengthSharpen;                       ///< [IN]  \~chinese        \~english Strength Sharpen[1,1000]

    unsigned int            nRes[8];                                ///<       \~chinese                   \~english Reserved

}MV_CC_SPATIAL_DENOISE_PARAM;

/// \~chinese LSC          \~english LSC calib structure
typedef struct _MV_CC_LSC_CALIB_PARAM_T_
{
    unsigned int            nWidth;                                 ///< [IN]  \~chinese     \~english Image Width
    unsigned int            nHeight;                                ///< [IN]  \~chinese     \~english Image Height
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese              \~english Pixel format
    unsigned char*          pSrcBuf;                                ///< [IN]  \~chinese          \~english Input data buffer
    unsigned int            nSrcBufLen;                             ///< [IN]  \~chinese      \~english Input data length

    unsigned char*          pCalibBuf;                              ///< [OUT] \~chinese      \~english Output calib buffer
    unsigned int            nCalibBufSize;                          ///< [IN]  \~chinese nWidth*nHeight*sizeof(unsigned short))    \~english Provided output buffer size
    unsigned int            nCalibBufLen;                          
    unsigned int            nSecNumW;                          
    unsigned int            nSecNumH;                              
    unsigned int            nPadCoef;                              
    unsigned int            nCalibMethod;                         
    unsigned int            nTargetGray;                          


    unsigned int            nRes[8];                                ///<       \~chinese               \~english Reserved

}MV_CC_LSC_CALIB_PARAM;

/// \~chinese LSC            \~english LSC correct structure
typedef struct _MV_CC_LSC_CORRECT_PARAM_T_
{
    unsigned int            nWidth;                                 ///< [IN]  \~chinese    \~english Image Width
    unsigned int            nHeight;                                ///< [IN]  \~chinese     \~english Image Height
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese         \~english Pixel format
    unsigned char*          pSrcBuf;                                ///< [IN]  \~chinese           \~english Input data buffer
    unsigned int            nSrcBufLen;                             ///< [IN]  \~chinese         \~english Input data length

    unsigned char*          pDstBuf;                                ///< [OUT] \~chinese        \~english Output data buffer
    unsigned int            nDstBufSize;                            ///< [IN]  \~chinese    \~english Provided output buffer size
    unsigned int            nDstBufLen;                             ///< [OUT] \~chinese       \~english Output data length

    unsigned char*          pCalibBuf;                              ///< [IN]  \~chinese      \~english Input calib buffer
    unsigned int            nCalibBufLen;                           ///< [IN]  \~chinese    \~english Input calib buffer length

    unsigned int            nRes[8];                                ///<       \~chinese                \~english Reserved

}MV_CC_LSC_CORRECT_PARAM;

/// \~chinese ĳ  \~english The maximum number of child nodes corresponding to a node
#define MV_MAX_XML_NODE_NUM_C       128

/// \~chinese        \~english The maximum length of node name string
#define MV_MAX_XML_NODE_STRLEN_C    64

/// \~chinese             \~english The maximum length of Node String
#define MV_MAX_XML_STRVALUE_STRLEN_C 64

/// \~chinese          \~english The maximum length of the node description field
#define MV_MAX_XML_DISC_STRLEN_C    512

/// \~chinese                     \~english The maximum number of units
#define MV_MAX_XML_ENTRY_NUM        10

/// \~chinese               \~english The maximum number of parent nodes
#define MV_MAX_XML_PARENTS_NUM      8

/// \~chinese    \~english The length of the name of each unit that has been implemented
#define MV_MAX_XML_SYMBOLIC_STRLEN_C 64

enum MV_XML_Visibility
{
    V_Beginner      = 0,    ///< Always visible
    V_Expert        = 1,    ///< Visible for experts or Gurus
    V_Guru          = 2,    ///< Visible for Gurus
    V_Invisible     = 3,    ///< Not Visible
    V_Undefined     = 99    ///< Object is not yet initialized
};

/// \~chinese  | en:Single Node Basic Attributes
typedef struct _MV_XML_NODE_FEATURE_
{
    enum MV_XML_InterfaceType   enType;                             ///< \~chinese        \~english Node Type
    enum MV_XML_Visibility  enVisivility;                           ///< \~chinese        \~english Is visibility
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   ///< \~chinese  \~english Node Description, NOT SUPPORT NOW
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];   ///< \~chinese         \~english Display Name
    char                strName[MV_MAX_XML_NODE_STRLEN_C];          ///< \~chinese         \~english Node Name
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];       ///< \~chinese        \~english Notice

    unsigned int        nReserved[4];
}MV_XML_NODE_FEATURE;


typedef struct _MV_XML_NODES_LIST_
{
    unsigned int        nNodeNum;       ///< \~chinese         \~english Node Number
    MV_XML_NODE_FEATURE stNodes[MV_MAX_XML_NODE_NUM_C];
}MV_XML_NODES_LIST;

typedef struct _MV_XML_FEATURE_Value_
{
    enum MV_XML_InterfaceType   enType;                             ///< \~chinese       \~english Node Type
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   ///< \~chinese  \~english Node Description, NOT SUPPORT NOW
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];   ///< \~chinese       \~english Display Name
    char                strName[MV_MAX_XML_NODE_STRLEN_C];          ///< \~chinese        \~english Node Name
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];       ///< \~chinese         \~english Notice
    unsigned int        nReserved[4];
}MV_XML_FEATURE_Value;

typedef struct _MV_XML_FEATURE_Base_
{
    enum MV_XML_AccessMode enAccessMode;                           ///< \~chinese     \~english Access Mode
}MV_XML_FEATURE_Base;

typedef struct _MV_XML_FEATURE_Integer_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   ///< \~chinese ~english NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];

    enum MV_XML_Visibility  enVisivility;                           ///< \~chinese       \~english Visible
    enum MV_XML_AccessMode  enAccessMode;                           ///< \~chinese    \~english Access Mode
    int                 bIsLocked;                                  ///< \~chinese    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    int64_t             nValue;                                     ///< \~chinese           \~english Current Value
    int64_t             nMinValue;                                  ///< \~chinese         \~english Min Value
    int64_t             nMaxValue;                                  ///< \~chinese       \~english Max Value
    int64_t             nIncrement;                                 ///< \~chinese 
    unsigned int        nReserved[4];

}MV_XML_FEATURE_Integer;

typedef struct _MV_XML_FEATURE_Boolean_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   ///< \~chinese    \~english NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];

    enum MV_XML_Visibility  enVisivility;                           ///< \~chinese     \~english Visible
    enum MV_XML_AccessMode  enAccessMode;                           ///< \~chinese   \~english Access Mode
    int                 bIsLocked;                                  ///< \~chinese    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    bool                bValue;                                     ///< \~chinese       \~english Current Value

    unsigned int        nReserved[4];
}MV_XML_FEATURE_Boolean;

typedef struct _MV_XML_FEATURE_Command_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   ///< \~chinese   \~english NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];

    enum MV_XML_Visibility  enVisivility;                           ///< \~chinese    \~english Visible
    enum MV_XML_AccessMode  enAccessMode;                           ///< \~chinese    \~english Access Mode
    int                 bIsLocked;                                  ///< \~chinese    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW

    unsigned int        nReserved[4];
}MV_XML_FEATURE_Command;

typedef struct _MV_XML_FEATURE_Float_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   ///< \~chinese     \~english NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];

    enum MV_XML_Visibility  enVisivility;                           ///< \~chinese    \~english Visible
    enum MV_XML_AccessMode  enAccessMode;                           ///< \~chinese   \~english Access Mode
    int                 bIsLocked;                                  ///< \~chinese    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    double              dfValue;                                    ///< \~chinese         \~english Current Value
    double              dfMinValue;                                 ///< \~chinese      \~english Min Value
    double              dfMaxValue;                                 ///< \~chinese        \~english Max Value
    double              dfIncrement;                                ///< \~chinese        \~english Increment

    unsigned int        nReserved[4];
}MV_XML_FEATURE_Float;

typedef struct _MV_XML_FEATURE_String_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   ///< \~chinese    \~english NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];

    enum MV_XML_Visibility  enVisivility;                           ///< \~chinese      \~english Visible
    enum MV_XML_AccessMode  enAccessMode;                           ///< \~chinese   \~english Access Mode
    int                 bIsLocked;                                  ///< \~chinese  \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    char                strValue[MV_MAX_XML_STRVALUE_STRLEN_C];     ///< \~chinese          \~english Current Value

    unsigned int        nReserved[4];
}MV_XML_FEATURE_String;

typedef struct _MV_XML_FEATURE_Register_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   ///< \~chinese   \~english NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];

    enum MV_XML_Visibility  enVisivility;                           ///< \~chinese       \~english Visible
    enum MV_XML_AccessMode  enAccessMode;                           ///< \~chinese   \~english Access Mode
    int                 bIsLocked;                                  ///< \~chinese   \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    int64_t             nAddrValue;                                 ///< \~chinese         \~english Current Value

    unsigned int        nReserved[4];
}MV_XML_FEATURE_Register;

typedef struct _MV_XML_FEATURE_Category_
{
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   ///< \~chinese    \~english Node Description, NOT SUPPORT NOW
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];   ///< \~chinese        \~english Display Name
    char                strName[MV_MAX_XML_NODE_STRLEN_C];          ///< \~chinese           \~english Node Name
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];       ///< \~chinese           \~english Notice

    enum MV_XML_Visibility  enVisivility;                           ///< \~chinese     \~english Visible

    unsigned int        nReserved[4];
}MV_XML_FEATURE_Category;

typedef struct _MV_XML_FEATURE_EnumEntry_
{
    char                strName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   ///< \~chinese    \~english NOT SUPPORT NOW
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];
    int                 bIsImplemented;
    int                 nParentsNum;
    MV_XML_NODE_FEATURE stParentsList[MV_MAX_XML_PARENTS_NUM];

    enum MV_XML_Visibility  enVisivility;                           ///< \~chinese         \~english Visible
    int64_t             nValue;                                     ///< \~chinese       \~english Current Value
    enum MV_XML_AccessMode  enAccessMode;                           ///< \~chinese   \~english Access Mode
    int                 bIsLocked;                                  ///< \~chinese   \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    int                 nReserved[8];

}MV_XML_FEATURE_EnumEntry;

typedef struct _MV_XML_FEATURE_Enumeration_
{
    enum MV_XML_Visibility  enVisivility;                           ///< \~chinese     \~english Visible
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   ///< \~chinese  \~english Node Description, NOT SUPPORT NOW
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];   ///< \~chinese        \~english Display Name
    char                strName[MV_MAX_XML_NODE_STRLEN_C];          ///< \~chinese         \~english Node Name
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];       ///< \~chinese            \~english Notice

    int                 nSymbolicNum;                               ///< \~chinese ymbolic   \~english Symbolic Number
    char                strCurrentSymbolic[MV_MAX_XML_SYMBOLIC_STRLEN_C];///< \~chinese    \~english Current Symbolic Index
    char                strSymbolic[MV_MAX_XML_SYMBOLIC_NUM][MV_MAX_XML_SYMBOLIC_STRLEN_C];
    enum MV_XML_AccessMode   enAccessMode;                          ////< \~chinese    \~english Access Mode
    int                 bIsLocked;                                  ///< \~chinese    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW
    int64_t             nValue;                                     ///< \~chinese           \~english Current Value

    unsigned int        nReserved[4];
}MV_XML_FEATURE_Enumeration;

typedef struct _MV_XML_FEATURE_Port_
{
    enum MV_XML_Visibility  enVisivility;                           ///< \~chinese  \~english Visible
    char                strDescription[MV_MAX_XML_DISC_STRLEN_C];   ///< \~chinese   \~english Node Description, NOT SUPPORT NOW
    char                strDisplayName[MV_MAX_XML_NODE_STRLEN_C];   ///< \~chinese         \~english Display Name
    char                strName[MV_MAX_XML_NODE_STRLEN_C];          ///< \~chinese           \~english Node Name
    char                strToolTip[MV_MAX_XML_DISC_STRLEN_C];       ///< \~chinese              \~english Notice

    enum MV_XML_AccessMode  enAccessMode;                           ///< \~chinese      \~english Access Mode
    int                 bIsLocked;                                  ///< \~chinese    \~english Locked. 0-NO; 1-YES, NOT SUPPORT NOW

    unsigned int        nReserved[4];
}MV_XML_FEATURE_Port;

typedef struct _MV_XML_CAMERA_FEATURE_
{
    enum MV_XML_InterfaceType    enType;
    union
    {
        MV_XML_FEATURE_Integer      stIntegerFeature;
        MV_XML_FEATURE_Float        stFloatFeature;
        MV_XML_FEATURE_Enumeration  stEnumerationFeature;
        MV_XML_FEATURE_String       stStringFeature;
    }SpecialFeature;

}MV_XML_CAMERA_FEATURE;



/// \~chinese          \~english Save Image Parameters
typedef struct _MV_SAVE_IMAGE_PARAM_T_EX_
{
    unsigned char*          pData;                                  ///< [IN]  \~chinese          \~english Input Data Buffer
    unsigned int            nDataLen;                               ///< [IN]  \~chinese        \~english Input Data length
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese   \~english Input Data Pixel Format
    unsigned short          nWidth;                                 ///< [IN]  \~chinese            \~english Image Width
    unsigned short          nHeight;                                ///< [IN]  \~chinese               \~english Image Height

    unsigned char*          pImageBuffer;                           ///< [OUT] \~chinese           \~english Output Image Buffer
    unsigned int            nImageLen;                              ///< [OUT] \~chinese         \~english Output Image length
    unsigned int            nBufferSize;                            ///< [IN]  \~chinese  \~english Output buffer size provided
    enum MV_SAVE_IAMGE_TYPE enImageType;                            ///< [IN]  \~chinese           \~english Output Image Format
    unsigned int            nJpgQuality;                            ///< [IN]  \~chinese   \~english Encoding quality(50-99]��Other formats are invalid

    unsigned int            iMethodValue;                           ///< [IN]  \~chinese  \~english Bayer interpolation method  0-Fast 1-Equilibrium 2-Optimal 3-Optimal+

    unsigned int            nReserved[3];                           ///<       \~chinese                 \~english Reserved

}MV_SAVE_IMAGE_PARAM_EX;



/// \~chinese            \~english Save Image Parameters
typedef struct _MV_SAVE_IMG_TO_FILE_PARAM_
{
    enum MvGvspPixelType    enPixelType;                            ///< [IN]  \~chinese    \~english The pixel format of the input data
    unsigned char*          pData;                                  ///< [IN]  \~chinese          \~english Input Data Buffer
    unsigned int            nDataLen;                               ///< [IN]  \~chinese         \~english Input Data length
    unsigned short          nWidth;                                 ///< [IN]  \~chinese              \~english Image Width
    unsigned short          nHeight;                                ///< [IN]  \~chinese           \~english Image Height
    enum MV_SAVE_IAMGE_TYPE enImageType;                            ///< [IN]  \~chinese           \~english Input Image Format
    unsigned int            nQuality;                               ///< [IN]  \~chinese JPG \~english JPG Encoding quality(50-99],Other formats are invalid
    char                    pImagePath[256];                        ///< [IN]  \~chinese           \~english Input file path

    int                     iMethodValue;                           ///< [IN]  \~chinese+ \~english Bayer interpolation method  0-Fast 1-Equilibrium 2-Optimal 3-Optimal+

    unsigned int            nReserved[8];                           ///<       \~chinese               \~english Reserved

}MV_SAVE_IMG_TO_FILE_PARAM;


// \~chinese        \~english Pixel convert structure
typedef struct _MV_CC_PIXEL_CONVERT_PARAM_
{
    unsigned short          nWidth;                                 ///< [IN]  \~chinese               \~english Width
    unsigned short          nHeight;                                ///< [IN]  \~chinese                \~english Height

    enum MvGvspPixelType    enSrcPixelType;                         ///< [IN]  \~chinese              \~english Source pixel format
    unsigned char*          pSrcData;                               ///< [IN]  \~chinese           \~english Input data buffer
    unsigned int            nSrcDataLen;                            ///< [IN]  \~chinese           \~english Input data length

    enum MvGvspPixelType    enDstPixelType;                         ///< [IN]  \~chinese          \~english Destination pixel format
    unsigned char*          pDstBuffer;                             ///< [OUT] \~chinese            \~english Output data buffer
    unsigned int            nDstLen;                                ///< [OUT] \~chinese       \~english Output data length
    unsigned int            nDstBufferSize;                         ///< [IN]  \~chinese    \~english Provided output buffer size

    unsigned int            nRes[4];                                ///<       \~chinese                  \~english Reserved

}MV_CC_PIXEL_CONVERT_PARAM;

/// \~chinese         \~english The saved format for 3D data
enum MV_SAVE_POINT_CLOUD_FILE_TYPE
{
    MV_PointCloudFile_Undefined         = 0,                        ///< \~chinese            \~english Undefined point cloud format
    MV_PointCloudFile_PLY               = 1,                        ///< \~chinese PLY                 \~english The point cloud format named PLY
    MV_PointCloudFile_CSV               = 2,                        ///< \~chinese CSV                 \~english The point cloud format named CSV
    MV_PointCloudFile_OBJ               = 3,                        ///< \~chinese OBJ                 \~english The point cloud format named OBJ

};

/// \~chinese        \~english Save 3D data to buffer
typedef struct _MV_SAVE_POINT_CLOUD_PARAM_
{
    unsigned int                    nLinePntNum;                    ///< [IN]  \~chinese       \~english The number of points in each row,which is the width of the image
    unsigned int                    nLineNum;                       ///< [IN]  \~chinese       \~english The number of rows,which is the height of the image

    enum MvGvspPixelType            enSrcPixelType;                 ///< [IN]  \~chinese   \~english The pixel format of the input data
    unsigned char*                  pSrcData;                       ///< [IN]  \~chinese          \~english Input data buffer
    unsigned int                    nSrcDataLen;                    ///< [IN]  \~chinese \~english Input data length

    unsigned char*                  pDstBuf;                        ///< [OUT] \~chinese    \~english Output pixel data buffer
    unsigned int                    nDstBufSize;                    ///< [IN]  \~chinese С(nLinePntNum * nLineNum * (16*3 + 4) + 2048)   \~english Output buffer size provided(nLinePntNum * nLineNum * (16*3 + 4) + 2048) 
    unsigned int                    nDstBufLen;                     ///< [OUT] \~chinese  \~english Output pixel data buffer size
    enum MV_SAVE_POINT_CLOUD_FILE_TYPE   enPointCloudFileType;      ///< [IN]  \~chinese \~english Output point data file type provided

    unsigned int        nReserved[8];                               ///<       \~chinese       \~english Reserved

}MV_SAVE_POINT_CLOUD_PARAM;

/// \~chinese           \~english Display frame information
typedef struct _MV_DISPLAY_FRAME_INFO_
{
	void*                   hWnd;                                   ///< [IN] \~chinese 
	unsigned char*          pData;                                  ///< [IN] \~chinese 
	unsigned int            nDataLen;                               ///< [IN] \~chinese 
	unsigned short          nWidth;                                 ///< [IN] \~chinese 
	unsigned short          nHeight;                                ///< [IN] \~chinese 
	enum MvGvspPixelType    enPixelType;                            ///< [IN] \~chinese

	unsigned int            enRenderMode;                             ///  [IN] \~chinese 
	unsigned int            nRes[3];                                ///<      \~chinese              \~english Reserved

}MV_DISPLAY_FRAME_INFO;




#endif /* _MV_OBSOLETE_CAM_PARAMS_H_ */

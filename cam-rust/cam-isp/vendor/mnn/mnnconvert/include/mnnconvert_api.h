//
//  mnnconvert_api.h
//  mnnconvert - C API for ONNX-to-MNN conversion (shared library)
//
//  Usage:
//    MnnConvert_Result result = MnnConvert_OnnxToMnn("model.onnx", "model.mnn", "bizCode");
//    if (result.code == 0) printf("Success: %s\n", result.message);
//    MnnConvert_FreeResult(&result);
//

#ifndef MNN_CONVERT_API_H
#define MNN_CONVERT_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

/* Version */
#define MNN_CONVERT_VERSION_MAJOR 1
#define MNN_CONVERT_VERSION_MINOR 0
#define MNN_CONVERT_VERSION_PATCH 0

/* Conversion result */
typedef struct {
    int    code;      /* 0 = success, non-zero = error */
    char*  message;   /* caller must free via MnnConvert_FreeResult */
} MnnConvert_Result;

/* Options for conversion */
typedef struct {
    const char* bizCode;            /* business code, default "MNN" */
    int         optimizeLevel;      /* 0=no optimize, 1=safe, 2=aggresive, default 1 */
    int         fp16;               /* save weights in fp16, default 0 */
    int         weightQuantBits;    /* 0=no quant, 2-8 bits, default 0 */
    int         weightQuantBlock;   /* block size for weight quant, -1=channel-wise */
    int         saveStaticModel;    /* save with fixed shape, default 0 */
    float       targetVersion;      /* target MNN version, default 0 (auto) */
    int         transformerFuse;    /* fuse attention/transformer ops, default 0 */
    int         allowCustomOp;      /* allow unknown ops, default 0 */
    int         useGeluApproximation; /* use gelu approximation, default 1 */
    const char* inputConfigFile;    /* input config for static model */
} MnnConvert_Options;

/* Default options */
static const MnnConvert_Options MNN_CONVERT_OPTIONS_DEFAULT = {
    "MNN",     /* bizCode */
    1,         /* optimizeLevel */
    0,         /* fp16 */
    0,         /* weightQuantBits */
    -1,        /* weightQuantBlock */
    0,         /* saveStaticModel */
    0.0f,      /* targetVersion */
    0,         /* transformerFuse */
    0,         /* allowCustomOp */
    1,         /* useGeluApproximation */
    NULL       /* inputConfigFile */
};

/**
 * Convert an ONNX model to MNN format.
 * @param onnxPath   Path to input .onnx file
 * @param mnnPath    Path to output .mnn file
 * @param options    Conversion options (pass NULL for defaults)
 * @return           Result struct; check code and free message
 */
MnnConvert_Result MnnConvert_OnnxToMnn(const char* onnxPath, const char* mnnPath,
                                       const MnnConvert_Options* options);

/**
 * Free the message string inside a result.
 */
void MnnConvert_FreeResult(MnnConvert_Result* result);

#ifdef __cplusplus
}
#endif

#endif /* MNN_CONVERT_API_H */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "cam-isp/mnn_sys/mnn_wrapper.h"

int main() {
    printf("Loading test_mnn_work.mnn...\n"); fflush(stdout);
    MnnInterpreter interp = mnn_interpreter_create_from_file("test_mnn_work.mnn");
    if (!interp) { printf("FAIL: load\n"); return 1; }
    printf("Creating session...\n"); fflush(stdout);
    MnnSession sess = mnn_session_create(interp, MNN_BACKEND_CPU, 4);
    if (!sess) { printf("FAIL: session\n"); return 1; }
    printf("Getting input...\n"); fflush(stdout);
    MnnTensor in = mnn_session_get_input_v2(interp, sess, NULL);
    if (!in) { printf("FAIL: get input\n"); return 1; }
    int dims[4]; int ndim = mnn_tensor_get_shape(in, dims, 4);
    printf("Input shape ndim=%d:", ndim); for(int i=0;i<ndim;i++) printf(" %d", dims[i]); printf("\n"); fflush(stdout);
    printf("Input type=%d\n", mnn_tensor_get_type(in)); fflush(stdout);
    
    // Use mnn_run_host_tensors to run inference
    int H = 48, W = 64;
    float* in_data = (float*)malloc(H * W * sizeof(float));
    for (int i = 0; i < H*W; i++) in_data[i] = (float)(i % 256);
    
    float* out_data = (float*)malloc(H * W * sizeof(float) * 4); // enough for 4 channels
    int in_shape[] = {1, 1, H, W};
    printf("Running...\n"); fflush(stdout);
    int nout = mnn_run_host_tensors(interp, sess, in_data, in_shape, 4, out_data, H*W*4);
    printf("mnn_run_host_tensors returned: %d\n", nout); fflush(stdout);
    
    if (nout > 0) {
        printf("First 8 outputs: ");
        for (int i = 0; i < 8 && i < nout; i++) printf("%f ", out_data[i]);
        printf("\nExpected: 0 255 510 765 1020 1275 1530 1785\n");
    }
    
    mnn_session_release(interp, sess);
    mnn_interpreter_destroy(interp);
    free(in_data);
    free(out_data);
    printf("Done\n");
    return 0;
}

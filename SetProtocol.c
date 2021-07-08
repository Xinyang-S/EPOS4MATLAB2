/* This a library for communication with Maxon EPOS4 motor controllers
 * using MATLAB.
 *
 * Copyright, Eugenio Yime Rodriguez, 2015
 *  
 */

#include "mex.h"
#include "Definitions.h"

#ifdef _LINUX_
#include "Win2Linux.h"
#endif

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    DWORD ErrCode = 0;
    HANDLE mHandle;
    DWORD pBaudrate;
    DWORD pTimeout = 500;
    BOOL Fault = FALSE;
    
    WORD NodeID;
    long lHandle;
    char ErrorInfo[255];

    /* Examine input (right-hand-side) arguments. */
    if (nrhs < 1) {
        mexPrintf("Error: this function should be use with four input arguments \n");
        return;
    }
    /* Check first input */
    if (mxGetM(prhs[0]) != 1 || mxGetM(prhs[0]) != 1) {
        mexPrintf("Error: this function requires four input scalar\n");
        return;
    }
    /* Check second input */
    if (mxGetM(prhs[1]) != 1 || mxGetM(prhs[1]) != 1) {
        mexPrintf("Error: this function requires four input scalar\n");
        return;
    }
    /* Check third input */
//     if (mxGetM(prhs[2]) != 1 || mxGetM(prhs[2]) != 1) {
//         mexPrintf("Error: this function requires four input scalar\n");
//         return;
//     }

    /* Examine output (left-hand-side) arguments. */
    if (nlhs > 1) {
        mexPrintf("Error: this function should be use with only one output argument\n");
        return;
    }

    /* create output matrix */
    plhs[0] = mxCreateDoubleScalar(0.0);

    /* first input */
    lHandle = (long) *mxGetPr(prhs[0]);
    mHandle = LongToHandle(lHandle);
    /* second input */
    pBaudrate = (WORD) * mxGetPr(prhs[1]);
    /* third input */
//     pTimeout = (WORD) * mxGetPr(prhs[2]);

    /* Call Function */
    if (!VCS_SetProtocolStackSettings(mHandle, 19200, 500, &ErrCode)>0) {
        VCS_GetErrorInfo(ErrCode, ErrorInfo, 255);
        mexPrintf("Error SetProtocolStackSettings: %s \n", ErrorInfo);
        *mxGetPr(plhs[0]) = 1;
    }
}

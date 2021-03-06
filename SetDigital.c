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
    BOOL Fault = FALSE;
    HANDLE mHandle;
    WORD NodeID;
    long lHandle;
    WORD OutputNumber; // subindex //firmware spec pg 192 (1 or 2) // cable colour hardware ref pg 40
    WORD Configuration; // command library pg 59 (15 or 14)
    BOOL DigitalValue; // 0 or 1
    BOOL Mask = 1; // always 1
    BOOL Polarity = 0; // high active = 0, low active = 1;
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
    if (mxGetM(prhs[2]) != 1 || mxGetM(prhs[2]) != 1) {
        mexPrintf("Error: this function requires four input scalar\n");
        return;
    }
    /* Check four input */
    if (mxGetM(prhs[3]) != 1 || mxGetM(prhs[3]) != 1) {
        mexPrintf("Error: this function requires four input scalar\n");
        return;
    }

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
    NodeID = (WORD) * mxGetPr(prhs[1]);
    /* third input */
    OutputNumber = (WORD) * mxGetPr(prhs[2]);
    /* fourth input */
    if (OutputNumber == 1){ Configuration = 15;}
    else if (OutputNumber == 2){ Configuration = 14;}
    else {Configuration = 0;}
    /* fifth input */
    DigitalValue = (WORD) * mxGetPr(prhs[3]);

    /* Call Function */
    if (!VCS_DigitalOutputConfiguration(mHandle, NodeID, OutputNumber, Configuration, DigitalValue, Mask, Polarity, &ErrCode)) {
        VCS_GetErrorInfo(ErrCode, ErrorInfo, 255);
        mexPrintf("Error DigitalConfig: %s \n", ErrorInfo);
        *mxGetPr(plhs[0]) = 1;;
    }
}

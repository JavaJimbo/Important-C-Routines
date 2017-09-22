/***********************************************************************************************************
 * 6-2-17: Window Filter
 ************************************************************************************************************/

#define MAXFILTER 17


short windowFilter (short newValue);
unsigned char swap(short *ptrDataA, short *ptrDataB);
unsigned char sortData(short *arrSortData);


unsigned char swap(short *ptrDataA, short *ptrDataB){
short tempVal;
    if (ptrDataA == NULL || ptrDataB == NULL) return(FALSE);
    tempVal = *ptrDataA;
    *ptrDataA = *ptrDataB;
    *ptrDataB = tempVal;
    return(TRUE);
}

unsigned char sortData(short *arrSortData){
short i, j; 

    if (arrSortData == NULL) return(FALSE);
    for(i = 0; i < (MAXFILTER-1); i++){   
        for (j = i+1; j < MAXFILTER; j++){
            if (arrSortData[i] > arrSortData[j]){
                swap(&arrSortData[i], &arrSortData[j]);
            }
        }       
    }
    return(TRUE);
}

#define CENTERINDEX ((MAXFILTER-1)/2)

short windowFilter (short newValue){
short centerValue;
static short arrInData[MAXFILTER]; // = {103, 102, 101, 108, 100, 105, 107, 106, 104};
short arrFilterData[MAXFILTER];
static unsigned short i = 0, j = 0, k = 0;
static unsigned char startFlag = TRUE;

    if (startFlag){
        startFlag = FALSE;
        for (i = 0; i < MAXFILTER; i++) arrInData[i] = 0;
        i = 0;
    } 
    if (i >= MAXFILTER) i = 0;
    arrInData[i++] = newValue;
    
    for (j = 0; j < MAXFILTER; j++) arrFilterData[j] = arrInData[j];
    if (sortData(arrFilterData) == FALSE) printf("\rERROR");

    // printf("\rSORT ARRAY: ");
    // for (k = 0; k < MAXFILTER; k++) printf ("%d ", arrFilterData[k]);
    centerValue = arrFilterData[CENTERINDEX];
    return(centerValue);
}



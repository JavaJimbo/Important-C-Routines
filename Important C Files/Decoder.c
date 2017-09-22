
// DECODER ROUTINE

#define STX 2
#define ETX 3
#define DLE 6
#define PLUS  43
#define ROBOT_ID 1
#define MAXBUFFER 64

#define THIS_BOARD_NUMBER 0x81
#define SETBOARD 0x80


	// Store next char if packet is valid and board number matches
	if (startFlag && j<MAXDATA) 
		InData[j]=ch;            
			
	// If preceding character wasn't an escape char:
    // check whether it is STX, ETX or DLE,
    // otherwise if board number matches then store and advance for next char
    if (escapeFlag==false){
		if (ch==DLE) escapeFlag=true;
		else if (ch==STX){
        	j = 0;
			startFlag = true;			
		}
		else if (ch==ETX) {
			startFlag = FALSE;
			dataLength = j;
			j = 0;
		}
		else if (startFlag) j++;
	}       
		
	// Otherwise if preceding character was an escape char:	
	else { 
    	escapeFlag=false;
        if (startFlag) j++;            	
	}
}	


// This version was used in Roomba for Raspberry Pi
unsigned int decodePacket (unsigned char *ptrInPacket, unsigned int inPacketSize, unsigned char *ptrData) {	
	static unsigned char startFlag = false, escapeFlag = false;
	static unsigned int dataIndex = 0;
	unsigned char ch;
	unsigned int i = 0, dataLength;

	if (ptrInPacket == NULL || inPacketSize == 0 || ptrData == NULL || inPacketSize >= MAXBUFFER || dataIndex >= MAXBUFFER)
	{
		startFlag = false;
		escapeFlag = false;
		dataIndex = 0;
		return (0);
	}		
		
	do {		
		ch = ptrInPacket[i++];
	
		// Store next char if packet is valid and board number matches
		if (startFlag && dataIndex < MAXBUFFER) ptrData[dataIndex] = ch;   
			
		// If preceding character wasn't an escape char:
		// check whether it is STX, ETX or DLE,
		// otherwise if board number matches then store and advance for next char
		if (escapeFlag == false || startFlag == false) {
			if (ch == DLE) escapeFlag = true;
			else if (ch == STX) {
				dataIndex = 0;
				startFlag = true;			
			}
			else if (ch == ETX) {
				startFlag = false;				
				dataLength = dataIndex;
				// dataIndex = 0;
				return (dataLength);
			}
			else if (startFlag) dataIndex++;
		}		
		// Otherwise if preceding character was an escape char:	
		else { 
			escapeFlag = false;
			if (startFlag) dataIndex++;  	
		}
	
	} while (i < inPacketSize && i < MAXBUFFER && dataIndex < MAXBUFFER);
	return (0);
}	

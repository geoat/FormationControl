#include <stdlib.h>

#include "packet.h"	
#include "crc16.h"

Packet pktObj;
Packet TpktObj;
Packet fpktObj;

unsigned char serialPacket[LP_SIZE+10];
//Priyanka Bhat
Packet *Create_Packet(unsigned char type,unsigned char length, unsigned char *value)
{

	
	pktObj.startByte = START_BYTE;
	pktObj.type = type;
	for(int i = 0;i<length;i++)
	{
		pktObj.value[i] = value[i];
	}

	pktObj.valueLength = length;
	pktObj.dataLength=Get_DataLength(length);
	pktObj.packetLength=1+1+pktObj.dataLength+(CRC_BYTE_SIZE);
	Set_CRCValue(&pktObj);
	return &pktObj;
}



//Arun Geo Thomas
unsigned char Get_DataLength(unsigned char length) //length is the length of value array
{
	return length+1;
}

void Set_CRCValue(Packet *pkt)
{

	uint8_t DataCRC[LP_SIZE+5];
	DataCRC[0]=pkt->type;
	for(int i =0;i<pkt->valueLength;i++)
	{
		DataCRC[i+1]=pkt->value[i];
	}
	uint16_t CRC = crc16_compute(DataCRC, pkt->dataLength, NULL);
	pkt->CRC[1]=(unsigned char)CRC & 0x00FF;
	pkt->CRC[0]=(unsigned char)((CRC & 0xFF00)>>8);

}


void Destroy_Packet(Packet *pkt)
{
}

//Priyanka Bhat
unsigned char *Get_Byte_Stream(Packet *pkt)
{
	serialPacket[0]=pkt->startByte;
	serialPacket[1]=pkt->dataLength;
	serialPacket[2]=pkt->type;
	for(int i =0;i<pkt->valueLength;i++)
	{
		serialPacket[3+i]=pkt->value[i];
	}
	serialPacket[pkt->packetLength-1]=pkt->CRC[1];
	serialPacket[pkt->packetLength-2]=pkt->CRC[0];
	serialPacket[pkt->packetLength]='\n';
	return serialPacket;
}

Packet *Create_Flash_Data_Packet(uint8_t *flashBuffer) //Maria G
{
	fpktObj.startByte = START_BYTE;
	fpktObj.type = T_FLASHMEM;

	fpktObj.value[0]=*(flashBuffer + 0);
	fpktObj.value[1]=*(flashBuffer + 1);
	fpktObj.value[2]=*(flashBuffer + 2);
	fpktObj.value[3]=*(flashBuffer + 3);
	fpktObj.value[4]=*(flashBuffer + 4);
	fpktObj.value[5]=*(flashBuffer + 5);
	fpktObj.value[6]=*(flashBuffer + 6);
	fpktObj.value[7]=*(flashBuffer + 7);
	fpktObj.value[8]=*(flashBuffer + 8);
	fpktObj.value[9]=*(flashBuffer + 9);
	fpktObj.value[10]=*(flashBuffer + 10);
	fpktObj.value[11]=*(flashBuffer + 11);
	fpktObj.value[12]=*(flashBuffer + 12);
	fpktObj.value[13]=*(flashBuffer + 13);
	fpktObj.value[14]=*(flashBuffer + 14);
	fpktObj.value[15]=*(flashBuffer + 15);
	for (int i = 16; i<22; i++){
		fpktObj.value[i] = 0;
	}
	fpktObj.value[22]=*(flashBuffer + 16);

	fpktObj.valueLength = 23;
	fpktObj.dataLength=Get_DataLength(fpktObj.valueLength );
	fpktObj.packetLength=1+1+fpktObj.dataLength+(CRC_BYTE_SIZE);
	Set_CRCValue(&fpktObj);
	return &fpktObj;
}
//Arun Geo Thomas
Packet *Create_Telemetery_Packet(uint16_t bat_volt, int16_t *MotorValues, int16_t phi, int16_t theta, int16_t psi, int16_t sp, int16_t sq, int16_t sr, unsigned char _msgCode, int16_t PArray[], uint32_t pressure, unsigned char droneMode)
{
	TpktObj.startByte = START_BYTE;
	TpktObj.type = T_DATA;
	unsigned char* charTemp = (unsigned char* )MotorValues;
	for(uint8_t i =0;i<8;i++)
	{
		TpktObj.value[i]=charTemp[i];
		
	}

	//REMINDER: Consider Endianess
	TpktObj.value[9]=bat_volt>>8;
	TpktObj.value[8]=bat_volt&0x00FF;

	TpktObj.value[11]=phi>>8;
	TpktObj.value[10]=phi&0x00FF;
	TpktObj.value[13]=theta>>8;
	TpktObj.value[12]=theta&0x00FF;
	TpktObj.value[15]=psi>>8;
	TpktObj.value[14]=psi&0x00FF;
	TpktObj.value[17]=sp>>8;
	TpktObj.value[16]=sp&0x00FF;
	TpktObj.value[19]=sq>>8;
	TpktObj.value[18]=sq&0x00FF;
	TpktObj.value[21]=sr>>8;
	TpktObj.value[20]=sr&0x00FF;
	TpktObj.value[22]=_msgCode;
	charTemp = (unsigned char* )PArray;
	for(uint8_t i =0;i<8;i++)
	{
		TpktObj.value[23+i]=charTemp[i];
		
	}
	TpktObj.value[31]=(pressure>>16)&0xFF;
	TpktObj.value[32]=(pressure>>8)&0xFF;
	TpktObj.value[33]=pressure&0xFF;
	TpktObj.value[34]=droneMode;

	TpktObj.valueLength = 35;
	TpktObj.dataLength=Get_DataLength(TpktObj.valueLength );
	TpktObj.packetLength=1+1+TpktObj.dataLength+(CRC_BYTE_SIZE);
	Set_CRCValue(&TpktObj);
	return &TpktObj;
}
//Arun Geo Thomas
Packet *Create_adMSG_Packet(char *additionalMessage){
	TpktObj.startByte = START_BYTE;
	TpktObj.type = T_adMSG;
	for(int i =0;i<15;i++){
			TpktObj.value[i]=additionalMessage[i];
	}
	TpktObj.valueLength = 15;
	TpktObj.dataLength=Get_DataLength(TpktObj.valueLength);
	TpktObj.packetLength=1+1+TpktObj.dataLength+(CRC_BYTE_SIZE);
	Set_CRCValue(&TpktObj);
	return &TpktObj;

}
//Arun Geo Thomas
Packet *Create_HeartBeatPacket()
{
	pktObj.startByte = START_BYTE;
	pktObj.type = T_HEARTBEAT;
	pktObj.value[0] = T_HEARTBEAT;
	pktObj.valueLength = 1;
	pktObj.dataLength=Get_DataLength(pktObj.valueLength);
	pktObj.packetLength=1+1+pktObj.dataLength+(CRC_BYTE_SIZE);
	Set_CRCValue(&pktObj);
	return &pktObj;
}

#ifndef packet__
#define packet__

#include <stdint.h>
#include "../Data/dronedata.hpp"
#include "../Data/configuration.hpp"
#include <cstring>
#include "byteconverter.hpp"
#include "../Data/autopilotstates.hpp"
#include "../Data/pathplannerdata.hpp"

#include "crc16.hpp"

#define START_BYTE 255
#define READ_BUFFER_SIZE 1024

//packet Tag Names
#define T_MODE 1
#define T_TYPE 2
#define T_DATA 3
#define T_EXIT 4
#define T_ACK 5
#define T_CONFIG_KPKV 6
#define T_CONFIG_SGAIN 7
#define T_CONFIG_PGAIN 8
#define T_CONFIG_COMM_GRAPH 9
#define T_PATHTYPE 10
#define MESSAGE_TYPE_MAX 10

//Payload sizes
#define T_DATA_SIZE (18 * sizeof(Packet::data_type))
#define T_CONFIG_KPKV_SIZE ((12 * sizeof(Packet::data_type)))
#define T_ACK_SIZE (3)
#define T_MODE_SIZE (1)
#define T_CONFIG_SGAIN_SIZE (6*(sizeof(Packet::data_type)+2))

#define CRC_BIT_SIZE 16
#define CRC_BYTE_SIZE (CRC_BIT_SIZE / 8)

#define DATA_SIZE 257

class Packet
{


public:
	typedef float data_type;
	unsigned char startByte;
	unsigned char type;
	unsigned char droneID;
	unsigned char data[DATA_SIZE];
	unsigned char dataLength;
	unsigned char CRC[CRC_BYTE_SIZE];

	inline unsigned char GetPayLoadLength()
	{
		return 1 + 1 + 1 + dataLength;
	}

	inline unsigned char GetPacketLength()
	{
		return 1 + GetPayLoadLength() + 2;
	}

	Packet() {}

	template <typename T>
	Packet(const DroneData<T> &droneData)
	{
		startByte = START_BYTE;
		type = T_DATA;
		droneID = droneData.droneID;
		Converter<float> convertor;
		for (size_t i = 0; i < 12; i++)
		{
			convertor.m_value = (data_type)droneData.droneStates[i];
			memcpy(data + i * 4, convertor.m_bytes, 4);
			//((float *)(data))[i] = (data_type)droneData.droneStates[i];
		}
		unsigned char *ciData = data + 12 * 4;
		for (size_t i = 0; i < 6; i++)
		{
			convertor.m_value = (data_type)droneData.controlInput[i];
			memcpy(ciData + i * 4, convertor.m_bytes, 4);
			//((float *)(data))[12 + i] = (data_type)droneData.controlInput[i];
		}
		//std::cout << "droneData.controlInput[0]:" << droneData.controlInput[0] << std::endl;

		dataLength = T_DATA_SIZE;
		SetCRCValue();
	}

	void GetByteStream(char stream[], int &streamLength)
	{
		stream[0] = startByte;
		stream[1] = type;
		stream[2] = droneID;
		stream[3] = dataLength;
		memcpy(stream + 4, data, dataLength);
		// int f = 0;
		// f = f | data[40];
		// f = f | ((long int)data[41] << 8);
		// f = f | ((long int)data[42] << 16);
		// f = f | ((long int)data[43] << 24);

		// //printf("Drone%u:%f\n", droneID, *((float *)&f));
		streamLength = GetPacketLength();
		memcpy(stream + streamLength - 2, CRC, 2);
		stream[streamLength] = '\n';
		// for (size_t i = 0; i < streamLength; i++)
		// {
		// 	printf("[%d]=%d;", i, stream[i]);
		// }
		// printf("\n");
	}
	void SetCRCValue()
	{
		uint8_t DataCRC[DATA_SIZE + 6];
		DataCRC[0] = this->type;
		DataCRC[1] = this->droneID;
		DataCRC[2] = this->dataLength;
		for (int i = 0; i < this->dataLength; i++)
		{
			DataCRC[i + 3] = this->data[i];
		}
		uint16_t CRC = crc16_compute(DataCRC, this->GetPayLoadLength(), NULL);
		this->CRC[1] = (unsigned char)CRC & 0x00FF;
		this->CRC[0] = (unsigned char)((CRC & 0xFF00) >> 8);
	}
	template <typename T>
	void CreateKpKvPacket(Configuration<T>& configuration, _droneID_type droneID)
	{

		startByte = START_BYTE;
		type = T_CONFIG_KPKV;
		this->droneID = droneID;

		for (size_t i = 0; i < 6; i++)
		{

			((float *)(data))[i] = (data_type)configuration.KpGain[i];
		}

		for (size_t i = 0; i < 6; i++)
		{
			((float *)(data))[6 + i] = (data_type)configuration.KvGain[i];
		}
		//std::cout << "droneData.controlInput[0]:" << droneData.controlInput[0] << std::endl;

		dataLength = sizeof(data_type) * 12;
		SetCRCValue();
	}

	void CreateAckPacket(const Packet &packet)
	{

		this->startByte = START_BYTE;
		this->droneID = packet.droneID;
		this->type = T_ACK;
		this->data[0] = packet.type;
		this->data[1] = packet.CRC[0];
		this->data[2] = packet.CRC[1];
		this->dataLength = T_ACK_SIZE;
		this->SetCRCValue();
	}


	void CreateModePacket(_droneID_type droneID, AutoPilotStates stateTo)
	{
		this->startByte = START_BYTE;
		this->droneID = droneID;
		this->type = T_MODE;
		this->data[0] = (unsigned char)stateTo;
		this->dataLength = T_MODE_SIZE;
		this->SetCRCValue();
	}

	void CreateExitPacket(_droneID_type droneID)
	{
		this->startByte = START_BYTE;
		this->droneID = droneID;
		this->type = T_EXIT;
		this->dataLength = 0;
		this->SetCRCValue();
	}

	template <typename T>
	void CreateSGainPacket(Configuration<T>& configuration, _droneID_type droneID)
	{
		startByte = START_BYTE;
		type = T_CONFIG_SGAIN;
		this->droneID = droneID;

		size_t i = 0;
		for (auto &&entry : configuration.SEntries)
		{
			unsigned char *data_start = data + i * 6;
			data_start[0] = (unsigned char)entry.row();
			data_start[1] = (unsigned char)entry.col();
			*((float *)(data_start+2)) = (data_type)entry.value();
			i++;
		}	

		
		dataLength = T_CONFIG_SGAIN_SIZE;
		SetCRCValue();
	}

	template <typename T>
	void CreatePGainPacket(Configuration<T> &configuration, _droneID_type droneID)
	{
		startByte = START_BYTE;
		type = T_CONFIG_PGAIN;
		this->droneID = droneID;

		size_t i = 0;
		for (auto &&entry : configuration.PEntries)
		{
			unsigned char *data_start = data + i * 6;
			data_start[0] = (unsigned char)entry.row();
			data_start[1] = (unsigned char)entry.col();
			*((float *)(data_start + 2)) = (data_type)entry.value();
			i++;
		}

		dataLength = (sizeof(Packet::data_type) + 2) * configuration.PEntries.size();
		SetCRCValue();
	}

	template <typename T>
	void CreateCommGraphPacket(Configuration<T> &configuration, _droneID_type droneID)
	{
		startByte = START_BYTE;
		type = T_CONFIG_COMM_GRAPH;
		this->droneID = droneID;
		//searching to find droneiD index
		auto search = configuration.dronesAndTypes.find(droneID);

		if(search!=configuration.dronesAndTypes.end()){
			
			//got drone index
			size_t currentDroneIndex = search->second.second; //index
			//store drone type
			data[0] = search->second.first; //drone type
			//already one data stored
			size_t i = 1;
			for (auto &&entry : configuration.communicationEntries)
			{
				//search the data link entry with this drone as target
				if ((unsigned char)entry.col() == currentDroneIndex)
				{
					//now we no the data link

					//search for the drone with this source index					
					for (auto &&drone : configuration.dronesAndTypes)
					{
						if (drone.second.second == entry.row())
						{
							//drone ID obtained
							data[i] = drone.first;
							//increase one data count
							i++;
						}
					}
					
				}
			}
			dataLength = i;
		}
		SetCRCValue();
	}

	template<typename T>
	void CreatePathInformationPacket(const PathPlannerData<T>& pathPlannerData, _droneID_type droneID)
	{
		this->startByte = START_BYTE;
		this->droneID = droneID;
		this->type = T_PATHTYPE;
		//send type first
		this->data[0] = (unsigned char) pathPlannerData.pathType;
		auto otherDatas = data+1;
		switch(pathPlannerData.pathType)
		{
			case PathType::Line:
				((data_type *)(otherDatas))[0] = (data_type)pathPlannerData.lineOrigin.x;
				((data_type *)(otherDatas))[1] = (data_type)pathPlannerData.lineOrigin.y;
				((data_type *)(otherDatas))[2] = (data_type)pathPlannerData.lineSlope.x;
				((data_type *)(otherDatas))[3] = (data_type)pathPlannerData.lineSlope.y;
				this->dataLength = sizeof(data_type) * 4 + 1;
				break;

			case PathType::Orbit:
				((data_type *)(otherDatas))[0] = (data_type)pathPlannerData.radius;
				((data_type *)(otherDatas))[1] = (data_type)pathPlannerData.orbitCentre.x;
				((data_type *)(otherDatas))[2] = (data_type)pathPlannerData.orbitCentre.y;
				this->dataLength = sizeof(data_type)*3+1;
			break;

			case PathType::WayPoints:
				{
					((data_type *)(otherDatas))[0] = (data_type)pathPlannerData.radius;
					auto numberOfWayPoints = pathPlannerData.wayPoints.size();
					for (size_t i = 0; i < numberOfWayPoints; i++)
					{
						((data_type *)(otherDatas))[1+i*2] = (data_type)pathPlannerData.wayPoints[i].x;
						((data_type *)(otherDatas))[2 + i * 2] = (data_type)pathPlannerData.wayPoints[i].y;
					}
					this->dataLength = sizeof(data_type) * (numberOfWayPoints*2+1) + 1;
				}
				
			break;

			default:
			break;		

		} 		


		this->SetCRCValue();
	}

private:
};

#endif

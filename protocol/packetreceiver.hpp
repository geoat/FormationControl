#pragma once

#include <stdlib.h>
#include <stdint.h>

#include "packet.hpp"
#include <memory>
#include <functional>

enum class PacketReceiverState
{
    checkStartByte,
    checkMessageType,
    getDroneID,
    getDataLength,
    getMsg,
    checkCRC0,
    checkCRC1
};

class PacketReceiver
{

private:
    PacketReceiverState currentState;
    PacketReceiverState nextState;
#define PACKET_RECEIVER_INTERNAL_BUFFER_SIZE (READ_BUFFER_SIZE * 2)
    unsigned char readBuffer[PACKET_RECEIVER_INTERNAL_BUFFER_SIZE];
    unsigned int bufferDataCount = 0;

    unsigned int dataReceptionCounter = 0;
    unsigned int startByteLocation;

private:
    //combine remnant data and incoming data
    void SetBuffer(const char buffer[], const unsigned int count)
    {
        unsigned int totalDataCount = bufferDataCount + count;
        if (totalDataCount >= PACKET_RECEIVER_INTERNAL_BUFFER_SIZE)
        {
            bufferDataCount = 0;
        }
        memcpy(readBuffer + bufferDataCount, buffer, count);
        bufferDataCount += count;
    }

public:
    void Finalize(unsigned int start)
    {
        if (start < bufferDataCount)
        {
            memcpy(readBuffer, &(readBuffer[start]), bufferDataCount - start);
            bufferDataCount = bufferDataCount - start;
        }
        else
        {
            bufferDataCount = 0;
        }
    }

    inline void StartByteDetected(unsigned int byteNumber)
    {
        currentPacket.startByte = START_BYTE;
        startByteLocation = byteNumber;
        nextState = PacketReceiverState::checkMessageType;
        //printf("Start Byte Detected\n");
    }

    bool StateHandler(const unsigned int &byteNumber, const unsigned char &currentByte, int &readFailed)
    {

        switch (currentState)
        {
        case PacketReceiverState::checkStartByte:
            if (currentByte != START_BYTE)
            {

                nextState = PacketReceiverState::checkStartByte;
            }
            else
            {
                StartByteDetected(byteNumber);
            }
            break;

        case PacketReceiverState::checkMessageType:

            if (currentByte <= MESSAGE_TYPE_MAX)
            {
                currentPacket.type = currentByte;
                currentPacket.dataLength = 0;
                // if (currentByte == T_DATA)
                //     currentPacket.dataLength = T_DATA_SIZE;
                // else if (currentByte == T_CONFIG_KPKV)
                //     currentPacket.dataLength = T_CONFIG_KPKV_SIZE;
                // else if (currentByte == T_ACK)
                //     currentPacket.dataLength = T_ACK_SIZE;
                // else if (currentByte == T_MODE)
                //     currentPacket.dataLength = T_MODE_SIZE;
                // else if (currentByte == T_CONFIG_SGAIN)
                //     currentPacket.dataLength = T_CONFIG_SGAIN_SIZE;
                
                //printf("MsgType:%u\n", currentByte);

                    nextState = PacketReceiverState::getDroneID;
            }
            else
            {
                if (currentByte == START_BYTE)
                {
                    StartByteDetected(byteNumber);
                }
                else
                {

                    nextState = PacketReceiverState::checkStartByte;
                }
            }

            break;
        case PacketReceiverState::getDroneID:
            currentPacket.droneID = currentByte;
            //printf("DroneID:%u\n", currentByte);

            nextState = PacketReceiverState::getDataLength;            

            break;
        case PacketReceiverState::getDataLength:
            currentPacket.dataLength = currentByte;
            //printf("DataLength:%u\n", currentByte);
            if (currentPacket.dataLength>0)
                nextState = PacketReceiverState::getMsg;
            else
            {
                nextState = PacketReceiverState::checkCRC0;
            }
            break;
        case PacketReceiverState::getMsg:

            storeValues(currentByte);

            break;
        case PacketReceiverState::checkCRC0:

            currentPacket.SetCRCValue();
            //printf("CRC0c:%u\n", currentPacket.CRC[0]);
            //printf("CRC0:%u\n", currentByte);
            if (currentPacket.CRC[0] == currentByte)
            {
                nextState = PacketReceiverState::checkCRC1;
            }
            else
            {
                currentPacket.CRC[0] = currentByte;
                //reception failed
                readFailed = startByteLocation;
            }

            break;
        case PacketReceiverState::checkCRC1:
            //printf("CRC1c:%u\n", currentPacket.CRC[1]);
            //printf("CRC1:%u\n", currentByte);
            if (currentPacket.CRC[1] == currentByte)
            {
                nextState = PacketReceiverState::checkStartByte;
                return true;
            }
            else
            {
                currentPacket.CRC[1] = currentByte;
                readFailed = startByteLocation;
            }

            break;
        }
        currentState = nextState;

        return false;
    }

    void storeValues(unsigned char currentByte)
    {
        currentPacket.data[dataReceptionCounter] = currentByte;

        dataReceptionCounter++;
        if (dataReceptionCounter >= currentPacket.dataLength)
        {
            nextState = PacketReceiverState::checkCRC0;
            dataReceptionCounter = 0;
        }
        else
        {
            nextState = PacketReceiverState::getMsg;
        }
    }

public:
    Packet currentPacket;
    PacketReceiver() : currentState(PacketReceiverState::checkStartByte)
    {
    }

    bool HandleDataAndGetLastPacket(const char buffer[], const unsigned int count)
    {

        bool packetReady = false;
        SetBuffer(buffer, count);

        for (int i = 0; i < bufferDataCount; i++)
        {
            //printf("i:%d\n", i);
            //intialize variable to detect CRCFailed (>=0) & know value of last start byte location
            int readFailed = -1;

            //call StateHandler to proccess the data byte
            bool packetRecieved = StateHandler(i, readBuffer[i], readFailed);
            if (packetRecieved)
            {
                //received a packet. save it and go for next itertion
                //this->currentPacket;
                //mark that there is anyway a succesful reception
                packetReady = true;
            }
            else
            {
                //no packets recieved:
                //1. Still Reading
                //2. Read failed - need a new byte search
                //3. Data to read exhausted

                //2. Read failed wrong CRC - need a new byte search
                if (readFailed >= 0 && readFailed < (bufferDataCount - 1))
                {
                    //search for new start byte from last detected stare byte
                    i = readFailed + 1;
                    continue;
                }
                //3. Data to read exhausted
                if (i == (bufferDataCount - 1))
                {
                    //save data for next read - from the start byte of current read

                    //save data in buffer
                    Finalize(startByteLocation);
                    //refresh packetreceiver to check for start byte in next run
                    startByteLocation = 0;
                    currentState = PacketReceiverState::checkStartByte;
                }

                //1. Still Reading - continue reading to next byte
            }
        }
        //announce packet is ready
        return packetReady;
    }
    template <typename PacketHandler, typename ClientPacketQueue>
    bool HandleDataWithPacketCallbacks(const char buffer[], const unsigned int count, PacketHandler& packetHandler, ClientPacketQueue& clientPacketQueue)
    {

        bool packetReady = false;
        SetBuffer(buffer, count);
        //printf("bufferDataCount:%u\n", bufferDataCount);
        for (int i = 0; i < bufferDataCount; i++)
        {
            //printf("i:%d\n", i);
            //intialize variable to detect CRCFailed (>=0) & know value of last start byte location
            int readFailed = -1;

            //call StateHandler to proccess the data byte
            bool packetRecieved = StateHandler(i, readBuffer[i], readFailed);
            if (packetRecieved)
            {
                //received a packet. save it and go for next itertion
                //this->currentPacket;
                //mark that there is anyway a succesful reception
                packetReady = true;
                packetHandler(currentPacket, clientPacketQueue);
                startByteLocation = i + 1;
            }
            else
            {
                //no packets recieved:
                //1. Still Reading
                //2. Read failed - need a new byte search
                //3. Data to read exhausted

                //2. Read failed wrong CRC - need a new byte search
                if (readFailed >= 0 && readFailed < (bufferDataCount - 1))
                {
                    //search for new start byte from last detected stare byte
                    i = readFailed + 1;
                    continue;
                }

                //1. Still Reading - continue reading to next byte
            }
            //3. Data to read exhausted
            if (i == (bufferDataCount - 1))
            {
                //save data for next read - from the start byte of current read

                //save data in buffer
                Finalize(startByteLocation);
                //refresh packetreceiver to check for start byte in next run
                startByteLocation = 0;
                currentState = PacketReceiverState::checkStartByte;
            }
        }
        //announce packet is ready
        return packetReady;
    }
};

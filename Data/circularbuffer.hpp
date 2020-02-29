#pragma once
#include <cstring>
#include <iostream>
template <unsigned int SIZE, typename data_type = char>
class CircularBuffer
{
    public:
    data_type data[SIZE];
    unsigned int readStart = 0;
    unsigned int dataCount= 0;

    public:
    data_type GetCandidate(unsigned int location)
    {
        return data[(readStart + location) % SIZE];
    }
    //SIZE should be always set higher than or equal to twice the READ_BUFFER_SIZE
    void InsertData(char (&bufferIn)[SIZE/2], unsigned int count)
    {
        unsigned int totalDataCount = dataCount + count;
        std::cout << "totalDataCount:" << totalDataCount << std::endl;
        

        int spaceInEnd = (int)SIZE -dataCount;
        std::cout << "spaceInEnd:" << spaceInEnd << std::endl;
        unsigned int toCopy = (spaceInEnd >= count) ? count : spaceInEnd;
        memcpy(data + readStart + dataCount, bufferIn, toCopy);
        if (spaceInEnd < count)
        {
            readStart = count - toCopy;
            memcpy(data, bufferIn + toCopy, readStart);
            }
            dataCount = (totalDataCount > (unsigned int)SIZE) ? (unsigned int)SIZE : totalDataCount;
            std::cout << "dataCount:" << dataCount << std::endl;
    }

    data_type Finalize(unsigned int location)
    {
        readStart = (readStart + location) % SIZE;
    }
};
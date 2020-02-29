

#define test_size  5
#define buffer_size (test_size*2)

#include "circularbuffer.hpp"
#include <iostream>

int main(){
    CircularBuffer <test_size*2> cBuffer;

    char inputbuffer[] = {1,2,3,0,0};


    for (size_t i = 0; i < 10; i++)
    {
        cBuffer.InsertData(inputbuffer,3);
        std::cout << cBuffer.readStart << std::endl;
        std::cout << cBuffer.dataCount<<std::endl;

        for (size_t j = 0; j < buffer_size; j++)
        {
            std::cout << (int)cBuffer.data[j] <<",";
        }
        std::cout << std::endl;
    }
    


        return 0;
}
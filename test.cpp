#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <poll.h>
ssize_t ngetc (char *c)
{
  return read (0, c, 1);
}
int main()
{
    char c = 's';
    while(c!='q')
    {

        //std::cout<<"running"<<std::endl;
        struct pollfd pollfds;
        pollfds.fd = 0;
        pollfds.events = POLLIN;

        poll(&pollfds, 1, 0);

        if(pollfds.revents & POLLIN)
        {
            ngetc(&c);
            std::cout<<":"<<c<<std::endl;
        }

    }
}
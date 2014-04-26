#include "optical_flow.h"

#include <thread>
#include <iostream>

int main()
{
    OpticalFlowSensor ofs;
    std::thread ofs_thread(ofs.loop, "/dev/ttyO0");
    
    while (true) 
    {
        OpticalFlowSensor::FlowData fd;
        if (ofs.dataReady())
        {
            fd = ofs.getFlowData();
            
            std:cout << fd.flow_x << " " << fd.flow_y << std::endl;
        }        
    }
}
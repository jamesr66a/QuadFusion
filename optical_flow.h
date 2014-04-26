#ifndef OPTICAL_FLOW_H
#define OPTICAL_FLOW_H

#include <string>
#include <vector>

class OpticalFlowSensor
{
public:
    void loop(std::string device_file);
    bool dataReady();
    FlowData getFlowData();
    
    struct FlowData{
        int16_t flow_x;
        int16_t flow_y;
        FlowData(int16_t flow_x, int16_t flow_y)
            : flow_x(flow_x), flow_y(flow_y)
        {}
    }
};

#endif
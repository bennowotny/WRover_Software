#ifndef FREQUENCY_HEADER_GUARD
#define FREQUENCY_HEADER_GUARD

#include <ros/ros.h>

struct FrequencyCharacteristics{
    const double nominalFrequency;
    const double nominalTolerance;
    static FrequencyCharacteristics makeFrequencyCharacteristics(XmlRpc::XmlRpcValue &rosParamSettings);
    FrequencyCharacteristics(double nominalFrequency, double nominalTolerance);
    double getPeriod() const;

    FrequencyCharacteristics(const FrequencyCharacteristics &other) = default;
    ~FrequencyCharacteristics() = default;
    FrequencyCharacteristics& operator=(const FrequencyCharacteristics &other) = delete;
    FrequencyCharacteristics(FrequencyCharacteristics &&other) = default;
    FrequencyCharacteristics& operator=(FrequencyCharacteristics &&other) = delete;
};
#endif
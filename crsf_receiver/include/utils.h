#ifndef CRSF_RECEIVER_UTILS_HPP
#define CRSF_RECEIVER_UTILS_HPP

#include <chrono>
#include <algorithm>

#include "crsf_structs.h"
#include "crsf_receiver_msg/CRSFChannels16.h"
#include "crsf_receiver_msg/CRSFLinkInfo.h"

using namespace crsf_receiver_msg;


clock_t millis(std::chrono::time_point<std::chrono::high_resolution_clock> start_time);

long convert_range(long x, double in_min, double in_max, double out_min, double out_max);

CRSFChannels16 convert_to_channels_message(int* channels);
CRSFLinkInfo convert_to_link_info(LinkStatisticsFrame info);


#endif
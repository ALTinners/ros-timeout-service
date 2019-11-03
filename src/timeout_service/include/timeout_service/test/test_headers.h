#include <string>
#include <chrono>

namespace timeout_service
{

namespace testing
{

const std::string signaller_topic_name{"/signaller"};
const std::string reference_service_name{"/reference"};

const std::chrono::milliseconds sleep_time{2000};

}

}
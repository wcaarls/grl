#include <grl/configurable.h>
#include <grl/visualization.h>

using namespace grl;

DEFINE_FACTORY(Configurable)

DECLARE_TYPE_NAME(bool)
DECLARE_TYPE_NAME(char)
DECLARE_TYPE_NAME(unsigned char)
DECLARE_TYPE_NAME(short int)
DECLARE_TYPE_NAME(short unsigned int)
DECLARE_TYPE_NAME(int)
DECLARE_TYPE_NAME(unsigned int)
DECLARE_TYPE_NAME(long int)
DECLARE_TYPE_NAME(long unsigned int)
DECLARE_TYPE_NAME(long long int)
DECLARE_TYPE_NAME(long long unsigned int)
DECLARE_TYPE_NAME(float)
DECLARE_TYPE_NAME(double)
DECLARE_TYPE_NAME(Vector)
DECLARE_TYPE_NAME(std::string)

Visualizer *Visualizer::instance_;

pthread_once_t RandGen::once_ = PTHREAD_ONCE_INIT;
pthread_mutex_t RandGen::mutex_;
pthread_key_t RandGen::key_;

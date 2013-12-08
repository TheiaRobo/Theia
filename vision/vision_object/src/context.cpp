#include "context.h"

Context::Context(const Config & config) :
camera(config.camera),
candidate(config.candidate),
colorImage(config.colorImage)
{
	path = config.path;
}
#include "context.h"

Context::Context(const Config & config) :
camera(config.camera),
colorImage(config.colorImage)
{
	path = config.path;
}
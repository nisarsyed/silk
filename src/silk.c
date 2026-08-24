#include <silk/silk.h>

int sl_version(void)
{
    return (SL_VERSION_MAJOR << 16) | (SL_VERSION_MINOR << 8) | SL_VERSION_PATCH;
}

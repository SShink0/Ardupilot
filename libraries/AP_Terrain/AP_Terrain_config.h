#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_Filesystem/AP_Filesystem_config.h>

#ifndef AP_TERRAIN_AVAILABLE
#if HAVE_FILESYSTEM_SUPPORT && defined(HAL_BOARD_TERRAIN_DIRECTORY)
#define AP_TERRAIN_AVAILABLE 1
#else
#define AP_TERRAIN_AVAILABLE 0
#endif
#endif

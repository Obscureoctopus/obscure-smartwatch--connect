#pragma once
#include <stdint.h>

/* NOAA low-precision solar position and a low-precision lunar azimuth. */
void sunmoon_positions(double lat, double lon, int64_t epoch_ms,
                       uint16_t *sun_az, uint16_t *moon_az);

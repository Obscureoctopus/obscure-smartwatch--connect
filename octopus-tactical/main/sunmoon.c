#include <math.h>
#include "sunmoon.h"

static double rad(double d) { return d * M_PI / 180.0; }
static double deg(double r) { return r * 180.0 / M_PI; }

static void altaz(double h, double dec, double lat, uint16_t *az_out)
{
    double alt = asin(sin(dec) * sin(lat) + cos(dec) * cos(lat) * cos(h));
    double x = (sin(dec) - sin(alt) * sin(lat)) / (cos(alt) * cos(lat) + 1e-10);
    if (x > 1.0) {
        x = 1.0;
    }
    if (x < -1.0) {
        x = -1.0;
    }
    double az = acos(x);
    double az_deg = (sin(h) > 0) ? 360.0 - deg(az) : deg(az);
    while (az_deg < 0) {
        az_deg += 360.0;
    }
    while (az_deg >= 360.0) {
        az_deg -= 360.0;
    }
    *az_out = (uint16_t)az_deg;
}

void sunmoon_positions(double lat, double lon, int64_t epoch_ms,
                       uint16_t *sun_az, uint16_t *moon_az)
{
    double jd = epoch_ms / 86400000.0 + 2440587.5;
    double n = jd - 2451545.0;
    double lat_r = rad(lat);

    double L = fmod(280.460 + 0.9856474 * n, 360.0);
    double g = rad(fmod(357.528 + 0.9856003 * n, 360.0));
    double lambda = rad(L + 1.915 * sin(g) + 0.020 * sin(2 * g));
    double eps = rad(23.439 - 0.0000004 * n);
    double alpha = atan2(cos(eps) * sin(lambda), cos(lambda));
    double dec_s = asin(sin(eps) * sin(lambda));
    double gmst = fmod(18.697374556 + 24.06570982441908 * (n - (long)n), 24.0) * 15.0;
    double h_s = rad(gmst + lon) - alpha;
    while (h_s > M_PI) {
        h_s -= 2 * M_PI;
    }
    while (h_s < -M_PI) {
        h_s += 2 * M_PI;
    }
    altaz(h_s, dec_s, lat_r, sun_az);

    double ml = rad(fmod(218.316 + 13.176396 * n, 360.0));
    double mm = rad(fmod(134.963 + 13.064993 * n, 360.0));
    double mf = rad(fmod(93.272 + 13.229350 * n, 360.0));
    double lambda_m = ml + rad(6.289) * sin(mm);
    double beta = rad(5.128) * sin(mf);
    double dec_m = asin(sin(beta) * cos(eps) + cos(beta) * sin(eps) * sin(lambda_m));
    double alpha_m = atan2(sin(lambda_m) * cos(eps) - tan(beta) * sin(eps), cos(lambda_m));
    double h_m = rad(gmst + lon) - alpha_m;
    while (h_m > M_PI) {
        h_m -= 2 * M_PI;
    }
    while (h_m < -M_PI) {
        h_m += 2 * M_PI;
    }
    altaz(h_m, dec_m, lat_r, moon_az);
}

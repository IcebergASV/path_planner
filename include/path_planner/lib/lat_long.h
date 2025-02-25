#ifndef LAT_LONG_H
#define LAT_LONG_H


class LatLong
{
    public:

    LatLong() = delete; // Disable default constructor
    LatLong(double latitude, double longitude) : latitude(latitude), longitude(longitude) {}
    
    const double latitude;
    const double longitude;
};

#endif // LAT_LONG_H
#pragma once

#include <GeographicLib/LocalCartesian.hpp>

struct gps_coordinate {
    double latitude;
    double longitude;
    double altitude;
};

struct cartesian_coordinate {
    double x_position;
    double y_position;
    double z_position;
};

class gps_converter {

private:

    GeographicLib::LocalCartesian local_cartesian_projection;

public:
    gps_converter(double origin_latitude, double origin_longitude, double origin_altitude = 0.0)
        : local_cartesian_projection(origin_latitude,origin_longitude,origin_altitude) {}

    cartesian_coordinate to_cartesian(double latitude,double longitude,double altitude = 0.0) const {
        
        cartesian_coordinate coordinate_position;

        local_cartesian_projection.Forward(latitude,
            longitude,
            altitude,
            coordinate_position.x_position,
            coordinate_position.y_position,
            coordinate_position.z_position);
        return coordinate_position;
    }

    cartesian_coordinate to_cartesian(const gps_coordinate& gps) const {
        return to_cartesian(gps.latitude, gps.longitude, gps.altitude);
    }

    gps_coordinate to_gps(double x_position,
                          double y_position,
                          double z_position = 0.0) const {
        gps_coordinate coordinate_position;

        local_cartesian_projection.Reverse(x_position,
                                           y_position,
                                           z_position,
                                           coordinate_position.latitude,
                                           coordinate_position.longitude,
                                           coordinate_position.altitude);
        return coordinate_position;
    }

    gps_coordinate to_gps(const cartesian_coordinate& coordinate) const {
        return to_gps(coordinate.x_position,
                      coordinate.y_position,
                      coordinate.z_position);
    }

    void set_origin(double latitude,
                    double longitude,
                    double altitude = 0.0) {
        local_cartesian_projection.Reset(latitude,
                                         longitude,
                                         altitude);
    }

    gps_coordinate get_origin() const {
        return {
            local_cartesian_projection.LatitudeOrigin(),
            local_cartesian_projection.LongitudeOrigin(),
            local_cartesian_projection.HeightOrigin()
        };
    }
};
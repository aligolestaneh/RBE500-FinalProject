// Helper functions
#ifndef RBE500_FINAL_PROJECT_PKG_HELPERS_HPP_
#define RBE500_FINAL_PROJECT_PKG_HELPERS_HPP_

// Standard Core C++ libs
#include <iostream>
#include <memory>
#include <string>
#include <cmath>

namespace helpers
{
    /** @brief Function to normalize angle to  make it between -pi to +pi range
     * @param rad: Angle in radians
     * @return Normalized angle in radians
     */
    template <typename T>
    inline T normalize(const T &rad)
    {
        return atan2(sin(rad), cos(rad));
    }

    /** @brief Function to convert degrees to radians
     * @param def: Angle in degrees
     * @return Normalized Angle in radians (-pi to pi range)
     */
    template <typename T>
    inline T getRad(const T &deg)
    {
        return normalize((deg * M_PI) / (180.0));
    }

    /** @brief Function to convert radians to degress
     * @param def: Angle in radians
     * @return Normalized Angle in degrees (-pi to pi range)
     */
    template <typename T>
    inline T getDegrees(const T &rad)
    {
        return ((normalize(rad) * 180.0) / (M_PI));
    }
}

#endif
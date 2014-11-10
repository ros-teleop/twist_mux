/*********************************************************************
 * Software License Agreement (CC BY-NC-SA 4.0 License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  This work is licensed under the Creative Commons
 *  Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 *  To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-nc-sa/4.0/
 *  or send a letter to
 *  Creative Commons, 444 Castro Street, Suite 900,
 *  Mountain View, California, 94041, USA.
 *********************************************************************/

/*
 * @author Enrique Fernandez
 * @author Siegfried Gevatter
 */

#ifndef UTILS_H
#define UTILS_H

// This could be taken from #include <boost/algorithm/clamp.hpp>
// but it seems that all versions of Boost have it.

/**
 * @brief Clamp a value to the range [min, max]
 * @param x Value
 * @param min Min value of the range [min, max]
 * @param max Max value of the range [min, max]
 * @return Value clamped to the range [min, max]
 */
template<typename T>
static T clamp(T x, T min, T max)
{
       if (  x < min) x = min;
  else if (max <   x) x = max;
  return x;
}

#endif // UTILS_H


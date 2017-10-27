/*
    Copyright (c) 2017 Antonia Reiter

    Permission is hereby granted, free of charge, to any person obtaining
    a copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
    CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
    TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
    OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "AbstractVehicle.h"
#include <algorithm>
#include "helper.h"

using namespace std;

/**
 * @brief get current lane number from vehicle
 * 
 * @return int lane number
 */
int AbstractVehicle::getLane() { 

  /**
   * \note 
   * - lane width is 4 meters 
   * - left lane = lane 0 
   * - middle lane = lane 1 
   * - right lane = lane 2  
   */

  if (frenet_d < 0) {
    // error handling --> this is the other traffic direction lane!!
    return -1;
  }
    
  if (frenet_d > 12) {
    // rror handling --> this is the hard shoulder!!
    return 3;
  }

  // simple lane detection because of d value
  if (frenet_d < 4) {
    return 0;
  } else if (frenet_d < 8) {
    return 1;
  } else if (frenet_d < 12) {
    return 2;
  }
}
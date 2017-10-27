#include "helper.h"
#include <algorithm>
#include <math.h>
#include <vector>

#include "EgoVehicle.h"

using namespace std;


int AbstractVehicle::getLane() { 
  // remarks: 
  // * lane width is 4 meters
  // * left lane = lane 0 
  // * middle lane = lane 1 
  // * right lane = lane 2


  if (frenet_d < 0) {
    // TODO: error handling --> this is the other traffic direction lane!!
    return -1;
  }
    
  if (frenet_d > 12) {
    // TODO: error handling --> this is the hard shoulder!!
    return 3;
  }


  if (frenet_d < 4) {
    return 0;
  } else if (frenet_d < 8) {
    return 1;
  } else if (frenet_d < 12) {
    return 2;
  }
}
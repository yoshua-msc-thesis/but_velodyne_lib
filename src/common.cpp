/*
 * common.cpp
 *
 *  Created on: Oct 10, 2017
 *      Author: ivelas
 */

#include <but_velodyne/common.h>
#include <algorithm>

using namespace std;

namespace but_velodyne {

void random_indices(const int total_count, const int new_count, vector<int> &output) {
  output.resize(total_count);
  for(int i = 0; i < total_count; i++) {
    output[i] = i;
  }
  random_shuffle(output.begin(), output.end());
  output.resize(new_count);
}

}


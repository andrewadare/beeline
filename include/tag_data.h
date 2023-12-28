#ifndef __TAG_DATA_H__
#define __TAG_DATA_H__

#include <iostream>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "manif/manif.h"

struct TagData {
  int frame_id;  // video frame
  int tag_id;
  manif::SE3d transform;
  double error;

  friend auto operator<<(std::ostream& os, const TagData& t) -> std::ostream& {
    return os << t.frame_id << " " << t.tag_id << " " << t.transform << " "
              << t.error;
  }
};

using IntSet = std::set<int>;
using IntMap = std::unordered_map<int, IntSet>;

// Map of frame ID => {tag ID => TagData} maps
using ObsMap = std::unordered_map<int, std::unordered_map<int, TagData>>;

// Read a text file with the space-separated fields
// pose_id tag_id x y z qw qx qy qz uncertainty
// The IDs are ints, the rest are doubles.
// The coords are in a x-y = forward-left basis in the camera's body frame.
// The poses_by_tag map relates tag ID => {pose IDs} (observers)
int read_tag_observations(const char* path, ObsMap* obs_list,
                          IntMap* poses_by_tag);

#endif  // __TAG_DATA_H__
#include <stdio.h>

#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "pose_graph.h"
#include "tag_data.h"

using manif::SE3d;
using std::cout, std::endl;

namespace {

// Line schema:
// pose_id tag_id x y z qw qx qy qz uncertainty
TagData parse(const std::string& csv_line) {
  std::vector<double> a;
  std::stringstream ss(csv_line);
  std::string numeric_val;

  while (std::getline(ss, numeric_val, ' ')) {
    a.push_back(std::stod(numeric_val));
  }
  assert(a.size() == 10);

  const int frame_id = static_cast<int>(a[0]);
  const int tag_id = static_cast<int>(a[1]);
  const auto t = SE3d::Translation(a[2], a[3], a[4]);
  const auto q = Eigen::Quaterniond(a[5], a[6], a[7], a[8]);

  return {frame_id, tag_id, SE3d(t, q), a[9]};
}

}  // namespace

int read_tag_observations(const char* path, ObsMap* obs_map,
                          IntMap* poses_by_tag) {
  std::ifstream infile(path);
  std::string line;
  int line_count = 0;

  if (!infile.is_open()) {
    cout << "Unable to open file: " << path << endl;
    return -1;
  }

  while (std::getline(infile, line)) {
    if (line_count > 0) {  // Skip header

      // Read line from file
      TagData tag = parse(line);

      // Add TagData object to the dataset
      (*obs_map)[tag.frame_id].emplace(tag.tag_id, tag);

      // Add this tag-to-frame association to the observation history
      (*poses_by_tag)[tag.tag_id].emplace(tag.frame_id);
    }
    line_count++;
  }
  infile.close();

  cout << "Read " << line_count << " lines from " << path << endl;
  return line_count;
}

int save_g2o(const PoseGraph& g, const char* path) {
  std::ofstream outfile(path);

  if (!outfile.is_open()) {
    cout << "Unable to open file: " << path << endl;
    return -1;
  }

  // Upper triangle of 6 dof precision matrix contains
  // sum(1..6) = 6(6+1)/2 = 21 elements flattened row-wise
  std::array<double, 6> precisions = {1, 1, 1, 1, 1, 1};  // TODO pass these in
  std::array<double, 21> ut;
  int k = 0;
  for (int i = 0; i < 6; ++i) {
    for (int j = i; j < 6; ++j) {
      ut[k] = (i == j) ? precisions[i] : 0;
      ++k;
    }
  }

  // Write nodes
  for (const auto& [frame_id, node] : g.nodes) {
    outfile << "VERTEX_SE3:QUAT " << node.id << " " << node.transform << "\n";
  }

  // Write camera -> tag edges, including covariances.
  // manif::SE3d::coeffs() returns x y z qx qy qz qw, matching g2o.
  // Beware!
  // Manif uses Eigen::Quaternion internally, which has an inconsistent API:
  // The constructor takes wxyz, but internal storage / coeffs() is xyzw.
  for (const auto& [index_pair, edge] : g.edges) {
    outfile << "EDGE_SE3:QUAT " << edge.i << " " << edge.j << " "
            << edge.transform;
    for (const auto& prec : ut) {  // precisions
      outfile << " " << prec;
    }
    outfile << "\n";
  }

  outfile.close();
  return 0;
}

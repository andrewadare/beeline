#include <iostream>

#include "averaging.h"
#include "manif/manif.h"
#include "pose_graph.h"
#include "tag_data.h"

using manif::SE3d;
using std::cout, std::endl;

void add_tags(const ObsMap& observations, const int i, PoseGraph* g) {
  for (const auto& [k, tag] : observations.at(i)) {
    g->add({i, k, EdgeType::DETECTION, tag.transform});
    if (!g->contains(k)) {
      // TODO: averaging over redundant estimates may be more accurate
      cout << "Adding tag " << k << endl;
      g->add({k, NodeType::TAG, g->nodes[i].transform * tag.transform});
    }
  }
}

int main(int argc, const char** argv) {
  if (argc != 2) {
    cout << "Usage: " << argv[0] << " <input-json>" << endl;
    return 1;
  }

  ObsMap observations;
  IntMap poses_by_tag;
  read_tag_observations(argv[1], &observations, &poses_by_tag);

  const int first = 300, last = 3250, stride = 2;
  const size_t max_estimates = 20;
  PoseGraph g;

  // Initialize trajectory: add identity pose for the first step
  // that is linked to a later one through a tag observation.
  for (int i = first; i < last; i += stride) {
    for (int j = i + stride; j < last; j += stride) {
      for (const auto& [k, tag] : observations.at(j)) {
        if (observations.at(i).count(k) > 0) {
          cout << "Initializing pose graph at step " << i << endl;
          g.add({i, NodeType::POSE, SE3d::Identity()});
          goto initialized;
        }
      }
    }
  }
initialized:

  // Indices i,j are step/frame/pose IDs; k is for tags.
  // Strategy: at each step j, find previous poses Xi that
  // have observed the same tag(s) and compute odometry estimates
  //    Xj = Xi * Xij
  // where Xij^k is pose j in i's frame estimated using tag k:
  //    Xij^k = Xi^k * inv(Xj^k)
  // This generates multiple Xj estimates. They get averaged
  // over i and k for robustness.
  for (int j = first; j < last; j += stride) {
    if (observations.count(j) == 0) continue;

    // Collect estimates of Xj
    std::vector<SE3d> Xj_estimates;
    for (const auto& [k, tag] : observations[j]) {
      for (const auto& i : poses_by_tag[k]) {
        if (g.contains(i)) {
          SE3d Xij = observations[i][k].transform *
                     observations[j][k].transform.inverse();
          Xj_estimates.push_back(g.nodes[i].transform * Xij);
          if (Xj_estimates.size() >= max_estimates) {
            goto done_collecting_estimates;
          }
        }
      }
    }

  done_collecting_estimates:

    // Compute mean over estimates and add to graph
    if (Xj_estimates.size() > 0) {
      // SE3d pose = mean(Xj_estimates);
      SE3d pose = inlier_mean(Xj_estimates);
      g.add({j, NodeType::POSE, pose});
      add_tags(observations, j, &g);
    }
  }

  save_g2o(g, "test.g2o");

  return 0;
}

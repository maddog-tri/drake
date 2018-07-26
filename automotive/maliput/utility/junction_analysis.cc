#include "drake/automotive/maliput/utility/junction_analysis.h"

#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace drake {
namespace maliput {
namespace utility {

namespace detail {

std::unordered_set<const api::Segment*>
ExploreSegment(const api::Segment* segment) {
  std::cerr << "Explore " << segment->id().string() << "\n";
  // Make an empty work-queue.
  std::queue<const api::Segment*> workqueue;
  // Make an empty visited set.
  std::unordered_set<const api::Segment*> visited;

  // Add seed segment to the workqueue.
  workqueue.push(segment);

  // Work the queue.
  while (!workqueue.empty()) {
    const api::Segment* working_segment = workqueue.front();
    workqueue.pop();
    std::cerr << "   working on " << working_segment->id().string() << "\n";

    // Loop over each Lane in the Segment
    for (int li = 0; li < working_segment->num_lanes(); ++li) {
      const api::Lane* const lane = working_segment->lane(li);
      // Loop over each End of the Lane
      for (api::LaneEnd::Which end :
        {api::LaneEnd::kStart, api::LaneEnd::kFinish}) {
        const api::LaneEndSet* const confluent_set =
            lane->GetConfluentBranches(end);
        // Loop over the set of confluent lanes.
        for (int i = 0; i < confluent_set->size(); ++i) {
          // Get confluent segment.
          const api::Segment* confluent_segment =
              confluent_set->get(i).lane->segment();
          // Is confluent segment in visited set?
          if (visited.find(confluent_segment) == visited.end()) {
            // Not yet:  then push onto workqueue, and mark as visited.
            workqueue.push(confluent_segment);
            visited.insert(confluent_segment);
          }
        }
      }
    }
  }
  return visited;
}

}  // namespace detail


std::vector<std::unordered_set<const api::Segment*>>
AnalyzeConfluentSegments(const api::RoadGeometry* road_geometry) {
  // Load all segments (of all junctions) into a workset.
  std::unordered_set<const api::Segment*> workset;
  for (int ji = 0; ji < road_geometry->num_junctions(); ++ji) {
    const api::Junction* const junction = road_geometry->junction(ji);
    for (int si = 0; si < junction->num_segments(); ++si) {
      const api::Segment* const segment = junction->segment(si);
      workset.insert(segment);
    }
  }
  // Process segments in workset.
  std::vector<std::unordered_set<const api::Segment*>> components;
  while (!workset.empty()) {
    // Grab a segment from workset and explore its connected component.
    const api::Segment* const segment = *(workset.begin());
    components.emplace_back(detail::ExploreSegment(segment));
    // Remove all segments that were just visited from the workset.
    for (const api::Segment* visited_segment : components.back()) {
      workset.erase(visited_segment);
    }
  }
  // Return the list of connected components.
  return components;
}


}  // namespace utility
}  // namespace maliput
}  // namespace drake

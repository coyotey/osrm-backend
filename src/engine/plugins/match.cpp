#include "engine/plugins/match.hpp"
#include "engine/map_matching/sub_matching.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/integer_range.hpp"

#include <cstdlib>

#include <algorithm>
#include <functional>
#include <iterator>
#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{
namespace detail
{

// Filters PhantomNodes to obtain a set of viable candiates
void filterCandidates(const std::vector<util::Coordinate> &coordinates,
                      routing_algorithms::CandidateLists &candidates_lists)
{
    for (const auto current_coordinate : util::irange<std::size_t>(0, coordinates.size()))
    {
        bool allow_uturn = false;

        if (coordinates.size() - 1 > current_coordinate && 0 < current_coordinate)
        {
            double turn_angle =
                util::coordinate_calculation::computeAngle(coordinates[current_coordinate - 1],
                                                           coordinates[current_coordinate],
                                                           coordinates[current_coordinate + 1]);

            // sharp turns indicate a possible uturn
            if (turn_angle <= 90.0 || turn_angle >= 270.0)
            {
                allow_uturn = true;
            }
        }

        auto &candidates = candidates_lists[current_coordinate];
        if (candidates.empty())
        {
            continue;
        }

        // sort by forward id, then by reverse id and then by distance
        std::sort(candidates.begin(),
                  candidates.end(),
                  [](const PhantomNodeWithDistance &lhs, const PhantomNodeWithDistance &rhs) {
                      return lhs.phantom_node.forward_segment_id.id <
                                 rhs.phantom_node.forward_segment_id.id ||
                             (lhs.phantom_node.forward_segment_id.id ==
                                  rhs.phantom_node.forward_segment_id.id &&
                              (lhs.phantom_node.reverse_segment_id.id <
                                   rhs.phantom_node.reverse_segment_id.id ||
                               (lhs.phantom_node.reverse_segment_id.id ==
                                    rhs.phantom_node.reverse_segment_id.id &&
                                lhs.distance < rhs.distance)));
                  });

        auto new_end =
            std::unique(candidates.begin(),
                        candidates.end(),
                        [](const PhantomNodeWithDistance &lhs, const PhantomNodeWithDistance &rhs) {
                            return lhs.phantom_node.forward_segment_id.id ==
                                       rhs.phantom_node.forward_segment_id.id &&
                                   lhs.phantom_node.reverse_segment_id.id ==
                                       rhs.phantom_node.reverse_segment_id.id;
                        });
        candidates.resize(new_end - candidates.begin());

        if (!allow_uturn)
        {
            const auto compact_size = candidates.size();
            for (const auto i : util::irange<std::size_t>(0, compact_size))
            {
                // Split edge if it is bidirectional and append reverse direction to end of list
                if (candidates[i].phantom_node.forward_segment_id.enabled &&
                    candidates[i].phantom_node.reverse_segment_id.enabled)
                {
                    PhantomNode reverse_node(candidates[i].phantom_node);
                    reverse_node.forward_segment_id.enabled = false;
                    candidates.push_back(
                        PhantomNodeWithDistance{reverse_node, candidates[i].distance});

                    candidates[i].phantom_node.reverse_segment_id.enabled = false;
                }
            }
        }

        // sort by distance to make pruning effective
        std::sort(candidates.begin(),
                  candidates.end(),
                  [](const PhantomNodeWithDistance &lhs, const PhantomNodeWithDistance &rhs) {
                      return lhs.distance < rhs.distance;
                  });
    }
}
}
}
}
}

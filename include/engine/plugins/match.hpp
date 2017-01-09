#ifndef MATCH_HPP
#define MATCH_HPP

#include "engine/api/match_api.hpp"
#include "engine/api/match_parameters.hpp"
#include "engine/plugins/plugin_base.hpp"

#include "engine/map_matching/bayes_classifier.hpp"
#include "engine/routing_algorithms/map_matching.hpp"
#include "engine/routing_algorithms/shortest_path.hpp"
#include "util/json_util.hpp"

#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{

namespace detail
{
void filterCandidates(const std::vector<util::Coordinate> &coordinates,
                      routing_algorithms::CandidateLists &candidates_lists);
}

template <typename AlgorithmT> class MatchPlugin final : public BasePlugin<AlgorithmT>
{
  public:
    using SuperT = BasePlugin<AlgorithmT>;
    using SubMatching = map_matching::SubMatching;
    using SubMatchingList = routing_algorithms::SubMatchingList;
    using CandidateLists = routing_algorithms::CandidateLists;
    static const constexpr double DEFAULT_GPS_PRECISION = 5;
    static const constexpr double RADIUS_MULTIPLIER = 3;

    MatchPlugin(const int max_locations_map_matching)
        : map_matching(heaps, DEFAULT_GPS_PRECISION), shortest_path(heaps),
          max_locations_map_matching(max_locations_map_matching)
    {
    }

    Status HandleRequest(
        const std::shared_ptr<const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>>
            facade,
        const api::MatchParameters &parameters,
        util::json::Object &json_result) const;

  private:
    mutable SearchEngineData heaps;
    mutable routing_algorithms::MapMatching<AlgorithmT> map_matching;
    mutable routing_algorithms::ShortestPathRouting<AlgorithmT> shortest_path;
    const int max_locations_map_matching;
};

template <typename AlgorithmT>
Status MatchPlugin<AlgorithmT>::HandleRequest(
    const std::shared_ptr<const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>> facade,
    const api::MatchParameters &parameters,
    util::json::Object &json_result) const
{
    BOOST_ASSERT(parameters.IsValid());

    // enforce maximum number of locations for performance reasons
    if (max_locations_map_matching > 0 &&
        static_cast<int>(parameters.coordinates.size()) > max_locations_map_matching)
    {
        return SuperT::Error("TooBig", "Too many trace coordinates", json_result);
    }

    if (!SuperT::CheckAllCoordinates(parameters.coordinates))
    {
        return SuperT::Error("InvalidValue", "Invalid coordinate value.", json_result);
    }

    // Check for same or increasing timestamps. Impl. note: Incontrast to `sort(first,
    // last, less_equal)` checking `greater` in reverse meets irreflexive requirements.
    const auto time_increases_monotonically = std::is_sorted(
        parameters.timestamps.rbegin(), parameters.timestamps.rend(), std::greater<>{});

    if (!time_increases_monotonically)
    {
        return SuperT::Error(
            "InvalidValue", "Timestamps need to be monotonically increasing.", json_result);
    }

    // assuming radius is the standard deviation of a normal distribution
    // that models GPS noise (in this model), x3 should give us the correct
    // search radius with > 99% confidence
    std::vector<double> search_radiuses;
    if (parameters.radiuses.empty())
    {
        search_radiuses.resize(parameters.coordinates.size(),
                               DEFAULT_GPS_PRECISION * RADIUS_MULTIPLIER);
    }
    else
    {
        search_radiuses.resize(parameters.coordinates.size());
        std::transform(parameters.radiuses.begin(),
                       parameters.radiuses.end(),
                       search_radiuses.begin(),
                       [](const boost::optional<double> &maybe_radius) {
                           if (maybe_radius)
                           {
                               return *maybe_radius * RADIUS_MULTIPLIER;
                           }
                           else
                           {
                               return DEFAULT_GPS_PRECISION * RADIUS_MULTIPLIER;
                           }

                       });
    }

    auto candidates_lists = SuperT::GetPhantomNodesInRange(*facade, parameters, search_radiuses);

    detail::filterCandidates(parameters.coordinates, candidates_lists);
    if (std::all_of(candidates_lists.begin(),
                    candidates_lists.end(),
                    [](const std::vector<PhantomNodeWithDistance> &candidates) {
                        return candidates.empty();
                    }))
    {
        return SuperT::Error("NoSegment",
                             std::string("Could not find a matching segment for any coordinate."),
                             json_result);
    }

    // call the actual map matching
    SubMatchingList sub_matchings = map_matching(*facade,
                                                 candidates_lists,
                                                 parameters.coordinates,
                                                 parameters.timestamps,
                                                 parameters.radiuses);

    if (sub_matchings.size() == 0)
    {
        return SuperT::Error("NoMatch", "Could not match the trace.", json_result);
    }

    std::vector<InternalRouteResult> sub_routes(sub_matchings.size());
    for (auto index : util::irange<std::size_t>(0UL, sub_matchings.size()))
    {
        BOOST_ASSERT(sub_matchings[index].nodes.size() > 1);

        // FIXME we only run this to obtain the geometry
        // The clean way would be to get this directly from the map matching plugin
        PhantomNodes current_phantom_node_pair;
        for (unsigned i = 0; i < sub_matchings[index].nodes.size() - 1; ++i)
        {
            current_phantom_node_pair.source_phantom = sub_matchings[index].nodes[i];
            current_phantom_node_pair.target_phantom = sub_matchings[index].nodes[i + 1];
            BOOST_ASSERT(current_phantom_node_pair.source_phantom.IsValid());
            BOOST_ASSERT(current_phantom_node_pair.target_phantom.IsValid());
            sub_routes[index].segment_end_coordinates.emplace_back(current_phantom_node_pair);
        }
        // force uturns to be on, since we split the phantom nodes anyway and only have
        // bi-directional
        // phantom nodes for possible uturns
        shortest_path(
            *facade, sub_routes[index].segment_end_coordinates, {false}, sub_routes[index]);
        BOOST_ASSERT(sub_routes[index].shortest_path_length != INVALID_EDGE_WEIGHT);
    }

    api::MatchAPI match_api{*facade, parameters};
    match_api.MakeResponse(sub_matchings, sub_routes, json_result);

    return Status::Ok;
}
}
}
}

#endif // MATCH_HPP

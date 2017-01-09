#ifndef TRIP_HPP
#define TRIP_HPP

#include "engine/plugins/plugin_base.hpp"

#include "engine/api/trip_api.hpp"
#include "engine/api/trip_parameters.hpp"
#include "engine/routing_algorithms/many_to_many.hpp"
#include "engine/routing_algorithms/shortest_path.hpp"
#include "engine/trip/trip_brute_force.hpp"
#include "engine/trip/trip_farthest_insertion.hpp"
#include "util/dist_table_wrapper.hpp"
#include "util/json_container.hpp"

#include <boost/assert.hpp>

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{
namespace detail
{
bool IsStronglyConnectedComponent(const util::DistTableWrapper<EdgeWeight> &result_table);

bool IsSupportedParameterCombination(const bool fixed_start,
                                     const bool fixed_end,
                                     const bool roundtrip);

void ManipulateTableForFSE(const std::size_t source_id,
                           const std::size_t destination_id,
                           util::DistTableWrapper<EdgeWeight> &result_table);
}

template <typename AlgorithmT> class TripPlugin final : public BasePlugin<AlgorithmT>
{
  private:
    using SuperT = BasePlugin<AlgorithmT>;

    mutable SearchEngineData heaps;
    mutable routing_algorithms::ShortestPathRouting<AlgorithmT> shortest_path;
    mutable routing_algorithms::ManyToManyRouting<AlgorithmT> duration_table;
    const int max_locations_trip;

    InternalRouteResult
    ComputeRoute(const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT> &facade,
                 const std::vector<PhantomNode> &phantom_node_list,
                 const std::vector<NodeID> &trip,
                 const bool roundtrip) const;

  public:
    explicit TripPlugin(const int max_locations_trip_)
        : shortest_path(heaps), duration_table(heaps), max_locations_trip(max_locations_trip_)
    {
    }

    Status HandleRequest(
        const std::shared_ptr<const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>>
            facade,
        const api::TripParameters &parameters,
        util::json::Object &json_result) const;
};

template <typename AlgorithmT>
InternalRouteResult TripPlugin<AlgorithmT>::ComputeRoute(
    const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT> &facade,
    const std::vector<PhantomNode> &snapped_phantoms,
    const std::vector<NodeID> &trip,
    const bool roundtrip) const
{
    InternalRouteResult min_route;
    // given the final trip, compute total duration and return the route and location permutation
    PhantomNodes viapoint;

    // computes a roundtrip from the nodes in trip
    for (auto node = trip.begin(); node < trip.end() - 1; ++node)
    {
        const auto from_node = *node;
        const auto to_node = *std::next(node);

        viapoint = PhantomNodes{snapped_phantoms[from_node], snapped_phantoms[to_node]};
        min_route.segment_end_coordinates.emplace_back(viapoint);
    }

    // return back to the first node if it is a round trip
    if (roundtrip)
    {
        viapoint = PhantomNodes{snapped_phantoms[trip.back()], snapped_phantoms[trip.front()]};
        min_route.segment_end_coordinates.emplace_back(viapoint);
        // trip comes out to be something like 0 1 4 3 2 0
        BOOST_ASSERT(min_route.segment_end_coordinates.size() == trip.size());
    }
    else
    {
        // trip comes out to be something like 0 1 4 3 2, so the sizes don't match
        BOOST_ASSERT(min_route.segment_end_coordinates.size() == trip.size() - 1);
    }

    shortest_path(facade, min_route.segment_end_coordinates, {false}, min_route);
    BOOST_ASSERT_MSG(min_route.shortest_path_length < INVALID_EDGE_WEIGHT, "unroutable route");
    return min_route;
}

template <typename AlgorithmT>
Status TripPlugin<AlgorithmT>::HandleRequest(
    const std::shared_ptr<const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>> facade,
    const api::TripParameters &parameters,
    util::json::Object &json_result) const
{
    BOOST_ASSERT(parameters.IsValid());
    const auto number_of_locations = parameters.coordinates.size();

    std::size_t source_id = INVALID_INDEX;
    std::size_t destination_id = INVALID_INDEX;
    if (parameters.source == api::TripParameters::SourceType::First)
    {
        source_id = 0;
    }
    if (parameters.destination == api::TripParameters::DestinationType::Last)
    {
        BOOST_ASSERT(number_of_locations > 0);
        destination_id = number_of_locations - 1;
    }
    bool fixed_start = (source_id == 0);
    bool fixed_end = (destination_id == number_of_locations - 1);
    if (!detail::IsSupportedParameterCombination(fixed_start, fixed_end, parameters.roundtrip))
    {
        return SuperT::Error("NotImplemented", "This request is not supported", json_result);
    }

    // enforce maximum number of locations for performance reasons
    if (max_locations_trip > 0 && static_cast<int>(number_of_locations) > max_locations_trip)
    {
        return SuperT::Error("TooBig", "Too many trip coordinates", json_result);
    }

    if (!SuperT::CheckAllCoordinates(parameters.coordinates))
    {
        return SuperT::Error("InvalidValue", "Invalid coordinate value.", json_result);
    }

    auto phantom_node_pairs = GetPhantomNodes(*facade, parameters);
    if (phantom_node_pairs.size() != number_of_locations)
    {
        return SuperT::Error("NoSegment",
                             std::string("Could not find a matching segment for coordinate ") +
                                 std::to_string(phantom_node_pairs.size()),
                             json_result);
    }
    BOOST_ASSERT(phantom_node_pairs.size() == number_of_locations);

    if (fixed_start && fixed_end && (source_id >= parameters.coordinates.size() ||
                                     destination_id >= parameters.coordinates.size()))
    {
        return SuperT::Error("InvalidValue", "Invalid source or destination value.", json_result);
    }

    auto snapped_phantoms = SnapPhantomNodes(phantom_node_pairs);

    BOOST_ASSERT(snapped_phantoms.size() == number_of_locations);

    // compute the duration table of all phantom nodes
    auto result_table = util::DistTableWrapper<EdgeWeight>(
        duration_table(facade, snapped_phantoms, {}, {}), number_of_locations);

    if (result_table.size() == 0)
    {
        return Status::Error;
    }

    const constexpr std::size_t BF_MAX_FEASABLE = 10;
    BOOST_ASSERT_MSG(result_table.size() == number_of_locations * number_of_locations,
                     "Distance Table has wrong size");

    if (!detail::IsStronglyConnectedComponent(result_table))
    {
        return SuperT::Error("NoTrips", "No trip visiting all destinations possible.", json_result);
    }

    if (fixed_start && fixed_end)
    {
        detail::ManipulateTableForFSE(source_id, destination_id, result_table);
    }

    std::vector<NodeID> trip;
    trip.reserve(number_of_locations);
    // get an optimized order in which the destinations should be visited
    if (number_of_locations < BF_MAX_FEASABLE)
    {
        trip = trip::BruteForceTrip(number_of_locations, result_table);
    }
    else
    {
        trip = trip::FarthestInsertionTrip(number_of_locations, result_table);
    }

    // rotate result such that roundtrip starts at node with index 0
    // thist first if covers scenarios: !fixed_end || fixed_start || (fixed_start && fixed_end)
    if (!fixed_end || fixed_start)
    {
        auto desired_start_index = std::find(std::begin(trip), std::end(trip), 0);
        BOOST_ASSERT(desired_start_index != std::end(trip));
        std::rotate(std::begin(trip), desired_start_index, std::end(trip));
    }
    else if (fixed_end && !fixed_start && parameters.roundtrip)
    {
        auto desired_start_index = std::find(std::begin(trip), std::end(trip), destination_id);
        BOOST_ASSERT(desired_start_index != std::end(trip));
        std::rotate(std::begin(trip), desired_start_index, std::end(trip));
    }

    // get the route when visiting all destinations in optimized order
    InternalRouteResult route = ComputeRoute(facade, snapped_phantoms, trip, parameters.roundtrip);

    // get api response
    const std::vector<std::vector<NodeID>> trips = {trip};
    const std::vector<InternalRouteResult> routes = {route};
    api::TripAPI trip_api{*facade, parameters};
    trip_api.MakeResponse(trips, routes, snapped_phantoms, json_result);

    return Status::Ok;
}
}
}
}

#endif // TRIP_HPP

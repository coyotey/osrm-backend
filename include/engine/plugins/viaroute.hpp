#ifndef VIA_ROUTE_HPP
#define VIA_ROUTE_HPP

#include "engine/api/route_api.hpp"
#include "engine/datafacade/datafacade_base.hpp"
#include "engine/plugins/plugin_base.hpp"

#include "engine/routing_algorithms/alternative_path.hpp"
#include "engine/routing_algorithms/direct_shortest_path.hpp"
#include "engine/routing_algorithms/shortest_path.hpp"
#include "engine/search_engine_data.hpp"
#include "util/for_each_pair.hpp"
#include "util/json_container.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{

template <typename AlgorithmT> class ViaRoutePlugin final : public BasePlugin<AlgorithmT>
{
  private:
    using SuperT = BasePlugin<AlgorithmT>;

    mutable SearchEngineData heaps;
    mutable routing_algorithms::ShortestPathRouting<AlgorithmT> shortest_path;
    mutable routing_algorithms::AlternativeRouting<AlgorithmT> alternative_path;
    mutable routing_algorithms::DirectShortestPathRouting<AlgorithmT> direct_shortest_path;
    const int max_locations_viaroute;

  public:
    explicit ViaRoutePlugin(int max_locations_viaroute)
        : shortest_path(heaps), alternative_path(heaps), direct_shortest_path(heaps),
          max_locations_viaroute(max_locations_viaroute)
    {
    }

    Status HandleRequest(
        const std::shared_ptr<const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>>
            facade,
        const api::RouteParameters &route_parameters,
        util::json::Object &json_result) const;
};

template <typename AlgorithmT>
Status ViaRoutePlugin<AlgorithmT>::HandleRequest(
    const std::shared_ptr<const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>> facade,
    const api::RouteParameters &route_parameters,
    util::json::Object &json_result) const
{
    BOOST_ASSERT(route_parameters.IsValid());

    if (max_locations_viaroute > 0 &&
        (static_cast<int>(route_parameters.coordinates.size()) > max_locations_viaroute))
    {
        return SuperT::Error(
            "TooBig",
            "Number of entries " + std::to_string(route_parameters.coordinates.size()) +
                " is higher than current maximum (" + std::to_string(max_locations_viaroute) + ")",
            json_result);
    }

    if (!SuperT::CheckAllCoordinates(route_parameters.coordinates))
    {
        return SuperT::Error("InvalidValue", "Invalid coordinate value.", json_result);
    }

    auto phantom_node_pairs = SuperT::GetPhantomNodes(*facade, route_parameters);
    if (phantom_node_pairs.size() != route_parameters.coordinates.size())
    {
        return SuperT::Error("NoSegment",
                             std::string("Could not find a matching segment for coordinate ") +
                                 std::to_string(phantom_node_pairs.size()),
                             json_result);
    }
    BOOST_ASSERT(phantom_node_pairs.size() == route_parameters.coordinates.size());

    auto snapped_phantoms = SuperT::SnapPhantomNodes(phantom_node_pairs);

    const bool continue_straight_at_waypoint = route_parameters.continue_straight
                                                   ? *route_parameters.continue_straight
                                                   : facade->GetContinueStraightDefault();

    InternalRouteResult raw_route;
    auto build_phantom_pairs = [&raw_route, continue_straight_at_waypoint](
        const PhantomNode &first_node, const PhantomNode &second_node) {
        raw_route.segment_end_coordinates.push_back(PhantomNodes{first_node, second_node});
        auto &last_inserted = raw_route.segment_end_coordinates.back();
        // enable forward direction if possible
        if (last_inserted.source_phantom.forward_segment_id.id != SPECIAL_SEGMENTID)
        {
            last_inserted.source_phantom.forward_segment_id.enabled |=
                !continue_straight_at_waypoint;
        }
        // enable reverse direction if possible
        if (last_inserted.source_phantom.reverse_segment_id.id != SPECIAL_SEGMENTID)
        {
            last_inserted.source_phantom.reverse_segment_id.enabled |=
                !continue_straight_at_waypoint;
        }
    };
    util::for_each_pair(snapped_phantoms, build_phantom_pairs);

    if (1 == raw_route.segment_end_coordinates.size())
    {
        if (route_parameters.alternatives && facade->GetCoreSize() == 0)
        {
            alternative_path(*facade, raw_route.segment_end_coordinates.front(), raw_route);
        }
        else
        {
            direct_shortest_path(*facade, raw_route.segment_end_coordinates, raw_route);
        }
    }
    else
    {
        shortest_path(*facade,
                      raw_route.segment_end_coordinates,
                      route_parameters.continue_straight,
                      raw_route);
    }

    // we can only know this after the fact, different SCC ids still
    // allow for connection in one direction.
    if (raw_route.is_valid())
    {
        api::RouteAPI route_api{*facade, route_parameters};
        route_api.MakeResponse(raw_route, json_result);
    }
    else
    {
        auto first_component_id = snapped_phantoms.front().component.id;
        auto not_in_same_component = std::any_of(snapped_phantoms.begin(),
                                                 snapped_phantoms.end(),
                                                 [first_component_id](const PhantomNode &node) {
                                                     return node.component.id != first_component_id;
                                                 });

        if (not_in_same_component)
        {
            return SuperT::Error("NoRoute", "Impossible route between points", json_result);
        }
        else
        {
            return SuperT::Error("NoRoute", "No route found between points", json_result);
        }
    }

    return Status::Ok;
}
}
}
}

#endif // VIA_ROUTE_HPP

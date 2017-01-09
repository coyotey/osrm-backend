#ifndef TILEPLUGIN_HPP
#define TILEPLUGIN_HPP

#include "engine/api/tile_parameters.hpp"
#include "engine/plugins/plugin_base.hpp"

#include <utility>
#include <vector>

/*
 * This plugin generates Mapbox Vector tiles that show the internal
 * routing geometry and speed values on all road segments.
 * You can use this along with a vector-tile viewer, like Mapbox GL,
 * to display maps that show the exact road network that
 * OSRM is routing.  This is very useful for debugging routing
 * errors
 */
namespace osrm
{
namespace engine
{
namespace plugins
{

template <typename AlgorithmT> class TilePlugin final : public BasePlugin<AlgorithmT>
{
  public:
    Status HandleRequest(
        const std::shared_ptr<const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>>
            facade,
        const api::TileParameters &parameters,
        std::string &pbf_buffer) const;
};

namespace detail
{
constexpr const static int MIN_ZOOM_FOR_TURNS = 15;

// Used to accumulate all the information we want in the tile about
// a turn.
struct TurnData final
{
    const util::Coordinate coordinate;
    const int in_angle;
    const int turn_angle;
    const int weight;
};

using RTreeLeaf = datafacade::BaseDataFacade::RTreeLeaf;

std::vector<TurnData>
getTurnData(const datafacade::ContiguousInternalMemoryDataFacade<algorithm::CH> &facade,
            const std::vector<RTreeLeaf> &edges,
            const std::vector<std::size_t> &sorted_edge_indexes);
std::vector<std::size_t> getEdgeIndex(const std::vector<RTreeLeaf> &edges);
std::vector<RTreeLeaf> getEdges(const datafacade::ContiguousInternalMemoryDataFacadeBase &facade,
                                unsigned x,
                                unsigned y,
                                unsigned z);
void encodeVectorTile(const datafacade::ContiguousInternalMemoryDataFacadeBase &facade,
                      unsigned x,
                      unsigned y,
                      unsigned z,
                      const std::vector<RTreeLeaf> &edges,
                      const std::vector<std::size_t> &sorted_edge_indexes,
                      const std::vector<TurnData> &all_turn_data,
                      std::string &pbf_buffer);
}

template <typename AlgorithmT>
Status TilePlugin<AlgorithmT>::HandleRequest(
    const std::shared_ptr<const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>> facade,
    const api::TileParameters &parameters,
    std::string &pbf_buffer) const
{
    BOOST_ASSERT(parameters.IsValid());

    auto edges = detail::getEdges(*facade, parameters.x, parameters.y, parameters.y);

    auto edge_index = detail::getEdgeIndex(edges);

    std::vector<detail::TurnData> turns;

    // If we're zooming into 16 or higher, include turn data.  Why?  Because turns make the map
    // really cramped, so we don't bother including the data for tiles that span a large area.
    if (parameters.z >= detail::MIN_ZOOM_FOR_TURNS)
    {
        turns = detail::getTurnData(*facade, edges, edge_index);
    }

    detail::encodeVectorTile(
        *facade, parameters.x, parameters.y, parameters.y, edges, edge_index, turns, pbf_buffer);

    return Status::Ok;
}
}
}
}

#endif /* TILEPLUGIN_HPP */

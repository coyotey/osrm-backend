#ifndef TABLE_HPP
#define TABLE_HPP

#include "engine/plugins/plugin_base.hpp"

#include "engine/api/table_api.hpp"
#include "engine/api/table_parameters.hpp"
#include "engine/routing_algorithms/many_to_many.hpp"
#include "engine/search_engine_data.hpp"
#include "util/json_container.hpp"

namespace osrm
{
namespace engine
{
namespace plugins
{

template <typename AlgorithmT> class TablePlugin final : public BasePlugin<AlgorithmT>
{
    using SuperT = BasePlugin<AlgorithmT>;

  public:
    explicit TablePlugin(const int max_locations_distance_table)
        : distance_table(heaps), max_locations_distance_table(max_locations_distance_table)
    {
    }

    Status HandleRequest(
        const std::shared_ptr<const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>>
            facade,
        const api::TableParameters &params,
        util::json::Object &result) const;

  private:
    mutable SearchEngineData heaps;
    mutable routing_algorithms::ManyToManyRouting<AlgorithmT> distance_table;
    const int max_locations_distance_table;
};

template <typename AlgorithmT>
Status TablePlugin<AlgorithmT>::HandleRequest(
    const std::shared_ptr<const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>> facade,
    const api::TableParameters &params,
    util::json::Object &result) const
{
    BOOST_ASSERT(params.IsValid());

    if (!SuperT::CheckAllCoordinates(params.coordinates))
    {
        return SuperT::Error("InvalidOptions", "Coordinates are invalid", result);
    }

    if (params.bearings.size() > 0 && params.coordinates.size() != params.bearings.size())
    {
        return SuperT::Error(
            "InvalidOptions", "Number of bearings does not match number of coordinates", result);
    }

    // Empty sources or destinations means the user wants all of them included, respectively
    // The ManyToMany routing algorithm we dispatch to below already handles this perfectly.
    const auto num_sources =
        params.sources.empty() ? params.coordinates.size() : params.sources.size();
    const auto num_destinations =
        params.destinations.empty() ? params.coordinates.size() : params.destinations.size();

    if (max_locations_distance_table > 0 &&
        ((num_sources * num_destinations) >
         static_cast<std::size_t>(max_locations_distance_table * max_locations_distance_table)))
    {
        return SuperT::Error("TooBig", "Too many table coordinates", result);
    }

    auto snapped_phantoms = SuperT::SnapPhantomNodes(SuperT::GetPhantomNodes(*facade, params));
    auto result_table =
        distance_table(*facade, snapped_phantoms, params.sources, params.destinations);

    if (result_table.empty())
    {
        return SuperT::Error("NoTable", "No table found", result);
    }

    api::TableAPI table_api{*facade, params};
    table_api.MakeResponse(result_table, snapped_phantoms, result);

    return Status::Ok;
}
}
}
}

#endif // TABLE_HPP

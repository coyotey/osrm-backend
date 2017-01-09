#ifndef NEAREST_HPP
#define NEAREST_HPP

#include "engine/api/nearest_api.hpp"
#include "engine/api/nearest_parameters.hpp"
#include "engine/plugins/plugin_base.hpp"
#include "osrm/json_container.hpp"

namespace osrm
{
namespace engine
{
namespace plugins
{

template <typename AlgorithmT> class NearestPlugin final : public BasePlugin<AlgorithmT>
{
    using SuperT = BasePlugin<AlgorithmT>;

  public:
    explicit NearestPlugin(const int max_results_) : max_results{max_results_} {}

    Status HandleRequest(
        const std::shared_ptr<const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>>
            facade,
        const api::NearestParameters &params,
        util::json::Object &result) const;

  private:
    const int max_results;
};

template <typename AlgorithmT>
Status NearestPlugin<AlgorithmT>::HandleRequest(
    const std::shared_ptr<const datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>> facade,
    const api::NearestParameters &params,
    util::json::Object &json_result) const
{
    BOOST_ASSERT(params.IsValid());

    if (max_results > 0 &&
        (boost::numeric_cast<std::int64_t>(params.number_of_results) > max_results))
    {
        return SuperT::Error("TooBig",
                             "Number of results " + std::to_string(params.number_of_results) +
                                 " is higher than current maximum (" + std::to_string(max_results) +
                                 ")",
                             json_result);
    }

    if (!SuperT::CheckAllCoordinates(params.coordinates))
        return SuperT::Error("InvalidOptions", "Coordinates are invalid", json_result);

    if (params.coordinates.size() != 1)
    {
        return SuperT::Error(
            "InvalidOptions", "Only one input coordinate is supported", json_result);
    }

    auto phantom_nodes = SuperT::GetPhantomNodes(*facade, params, params.number_of_results);

    if (phantom_nodes.front().size() == 0)
    {
        return SuperT::Error(
            "NoSegment", "Could not find a matching segments for coordinate", json_result);
    }
    BOOST_ASSERT(phantom_nodes.front().size() > 0);

    api::NearestAPI nearest_api(*facade, params);
    nearest_api.MakeResponse(phantom_nodes, json_result);

    return Status::Ok;
}
}
}
}

#endif /* NEAREST_HPP */

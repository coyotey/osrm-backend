#include "customizer/customizer.hpp"
#include "customizer/cell_customizer.hpp"
#include "partition/edge_based_graph_reader.hpp"

#include "util/cell_storage.hpp"
#include "util/log.hpp"
#include "util/multi_level_partition.hpp"
#include "util/timing_util.hpp"

namespace osrm
{
namespace customize
{

int Customizer::Run(const CustomizationConfig &config)
{
    TIMER_START(loading_data);
    auto edge_based_graph = partition::LoadEdgeBasedGraph(config.edge_based_graph_path.string());
    util::Log() << "Loaded edge based graph for mapping partition ids: "
                << edge_based_graph->GetNumberOfEdges() << " edges, "
                << edge_based_graph->GetNumberOfNodes() << " nodes";

    osrm::util::PackedMultiLevelPartition mlp(config.mld_partition_path);
    osrm::util::CellStorage storage(mlp, *edge_based_graph);
    TIMER_STOP(loading_data);
    util::Log() << "Loading partition data took " << TIMER_SEC(loading_data) << " seconds";

    TIMER_START(cell_customize);
    osrm::customizer::CellCustomizer customizer(mlp);
    customizer.Customize(*edge_based_graph, storage);
    TIMER_STOP(cell_customize);
    util::Log() << "Cells customization took " << TIMER_SEC(cell_customize) << " seconds";

    TIMER_START(writing_mld_data);
    storage.Write(config.mld_storage_path);
    TIMER_STOP(writing_mld_data);
    util::Log() << "MLD customization writing took " << TIMER_SEC(writing_mld_data) << " seconds";

    return 0;
}

} // namespace partition
} // namespace osrm

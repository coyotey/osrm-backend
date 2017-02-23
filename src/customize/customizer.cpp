#include "customizer/customizer.hpp"
#include "customizer/cell_customizer.hpp"
#include "partition/edge_based_graph_reader.hpp"
#include "partition/node_based_graph_to_edge_based_graph_mapping_reader.hpp"

#include "util/cell_storage.hpp"
#include "util/log.hpp"
#include "util/multi_level_partition.hpp"
#include "util/timing_util.hpp"

#include <boost/container/small_vector.hpp>

#include <iomanip>
#include <iostream>

namespace osrm
{
namespace customize
{

template <typename Graph, typename Partition, typename CellStorage>
void CellStorageStatistics(const Graph &graph,
                           const Partition &partition,
                           const CellStorage &storage)
{
    util::Log() << "Cells statistics per level [source/destination/all]";

    for (std::size_t level = 1; level < partition.GetNumberOfLevels(); ++level)
    {
        std::unordered_map<util::CellID, std::size_t> cell_nodes;
        for (auto node : util::irange(0u, graph.GetNumberOfNodes()))
        {
            ++cell_nodes[partition.GetCell(level, node)];
        }

        std::stringstream sstr;
        std::size_t source = 0, destination = 0, total = 0;
        for (util::CellID cell_id = 0; cell_id < partition.GetNumberOfCells(level); ++cell_id)
        {
            BOOST_ASSERT(cell_nodes.count(cell_id) > 0);

            auto cell = storage.GetCell(level, cell_id);
            sstr << " " << cell_id << " " << cell.GetSourceNodes().size() << "/"
                 << cell.GetDestinationNodes().size() << "/" << cell_nodes[cell_id];
            source += cell.GetSourceNodes().size();
            destination += cell.GetSourceNodes().size();
            total += cell_nodes[cell_id];
        }

        util::Log() << "Level " << level << " #cells " << cell_nodes.size()
                    //<< sstr.str()
                    << " average source nodes " << source << " of " << total << " ("
                    << (100. * source / total) << "%)"
                    << " average destination nodes " << destination << " of " << total << " ("
                    << (100. * destination / total) << "%)";
    }
}

void PartionerOutputCustomizationData(const CustomizationConfig &config)
{
    auto mapping = partition::LoadNodeBasedGraphToEdgeBasedGraphMapping(config.base_path.string() +
                                                                        ".osrm.nbg_to_ebg");
    util::Log() << "Loaded node based graph to edge based graph mapping";

    auto edge_based_graph = partition::LoadEdgeBasedGraph(config.edge_based_graph_path.string());
    util::Log() << "Loaded edge based graph for mapping partition ids: "
                << edge_based_graph->GetNumberOfEdges() << " edges, "
                << edge_based_graph->GetNumberOfNodes() << " nodes";

    // Partition ids, keyed by node based graph nodes
    std::vector<BisectionID> node_based_partition_ids;
    const auto fingerprint = storage::io::FileReader::VerifyFingerprint;
    storage::io::FileReader reader{config.base_path.string() + ".nbp_ids", fingerprint};
    auto scc_depth = reader.ReadOne<std::uint32_t>();
    reader.DeserializeVector(node_based_partition_ids);

    // Partition ids, keyed by edge based graph nodes
    std::vector<NodeID> edge_based_partition_ids(edge_based_graph->GetNumberOfNodes());

    // Extract edge based border nodes, based on node based partition and mapping.
    for (const auto node : util::irange(0u, edge_based_graph->GetNumberOfNodes()))
    {
        const auto node_based_nodes = mapping.Lookup(node);

        const auto u = node_based_nodes.u;
        const auto v = node_based_nodes.v;

        if (node_based_partition_ids[u] == node_based_partition_ids[v])
        {
            // Can use partition_ids[u/v] as partition for edge based graph `node_id`
            edge_based_partition_ids[node] = node_based_partition_ids[u];

            auto edges = edge_based_graph->GetAdjacentEdgeRange(node);
            if (edges.size() == 1)
            { // check edge case with one adjacent edge-based edge
                auto edge = edges.front();
                auto other = edge_based_graph->GetTarget(edge);
                auto &data = edge_based_graph->GetEdgeData(edge);
                auto other_node_based_nodes = mapping.Lookup(other);
                if (data.backward &&
                    node_based_partition_ids[other_node_based_nodes.u] !=
                        node_based_partition_ids[u])
                { // use id of other u if [other_u, other_v] -> [u,v] is in other partition as u
                    std::cout << "=== case 1 for node " << node << "\n";
                    edge_based_partition_ids[node] =
                        node_based_partition_ids[other_node_based_nodes.u];
                }
                // if (data.forward && node_based_partition_ids[v] !=
                // node_based_partition_ids[other_node_based_nodes.v])
                // { // use id of other v if [u,v] -> [other_u, other_v] is in other partition as v
                //     std::cout << "=== case 2 for node " << node << "\n";
                //     edge_based_partition_ids[node] =
                //     node_based_partition_ids[other_node_based_nodes.v];
                // }
            }
        }
        else
        {
            // Border nodes u,v - need to be resolved.
            // FIXME: just pick one side for now. See #3205.

            bool use_u = false;
            // bool use_v = false;
            for (auto edge : edge_based_graph->GetAdjacentEdgeRange(node))
            {
                auto other = edge_based_graph->GetTarget(edge);
                auto &data = edge_based_graph->GetEdgeData(edge);
                auto other_node_based_nodes = mapping.Lookup(other);

                if (data.backward)
                { // can use id of u if [other_u, other_v] -> [u,v] is in the same partition as u
                    BOOST_ASSERT(u == other_node_based_nodes.v);
                    use_u |= node_based_partition_ids[u] ==
                             node_based_partition_ids[other_node_based_nodes.u];
                }

                // if (data.forward)
                // {  // can use id of v if [u,v] -> [other_u, other_v] is in the same partition as
                // v
                //     BOOST_ASSERT(v == other_node_based_nodes.u);
                //     use_v |= node_based_partition_ids[v] ==
                //     node_based_partition_ids[other_node_based_nodes.v];
                // }

                // if (node == 7529)
                // {
                //     std::cout << node << " " << u << "[" << node_based_partition_ids[u] << "], "
                //               << v << "[" << node_based_partition_ids[v] <<"]"
                //               << " -> "
                //               << other << " " << other_node_based_nodes.u << "[" <<
                //               node_based_partition_ids[other_node_based_nodes.u] << "], "
                //               << other_node_based_nodes.v << "[" <<
                //               node_based_partition_ids[other_node_based_nodes.v] <<"]"
                //               << "     " << data.forward << " " << data.backward << " " <<  "\n";
                // }
            }

            // TODO: better to use partition that introduce less cross cell connections
            edge_based_partition_ids[node] = node_based_partition_ids[use_u ? u : v];
        }
    }

    BOOST_ASSERT(edge_based_partition_ids.size() == edge_based_graph->GetNumberOfNodes());

    // find bit size of bisection ids
    int first_nonzero_position = sizeof(BisectionID) * CHAR_BIT;
    for (auto id : edge_based_partition_ids)
    {
        first_nonzero_position =
            id == 0 ? first_nonzero_position : std::min(first_nonzero_position, __builtin_ctz(id));
    }
    BOOST_ASSERT(first_nonzero_position != sizeof(BisectionID) * CHAR_BIT);

    // split bisection id bits into groups starting from SCC and stop at level 1
    int mask_from = sizeof(BisectionID) * CHAR_BIT - scc_depth;
    boost::container::small_vector<BisectionID, 8> level_masks;
    for (int mask_to = sizeof(BisectionID) * CHAR_BIT; mask_to > first_nonzero_position;
         mask_to = mask_from, mask_from -= 3) // TODO: find better grouping
    {
        auto bit = std::max(first_nonzero_position, mask_from);
        level_masks.push_back(((1u << (sizeof(BisectionID) * CHAR_BIT - bit)) - 1) << bit);
    }

    util::Log() << "Bisection IDs split for SCC depth " << scc_depth
                << " and first non-zero bit position " << first_nonzero_position
                << " number of levels is " << level_masks.size();
    for (auto x : level_masks)
        std::cout << std::setw(8) << std::hex << x << std::dec << "\n";

    // collect cell ids as masked bisection ids
    std::vector<std::vector<osrm::util::CellID>> partitions(
        level_masks.size(), std::vector<osrm::util::CellID>(edge_based_partition_ids.size()));
    std::vector<std::unordered_set<osrm::util::CellID>> partition_sets(level_masks.size());
    for (std::size_t index = 0; index < edge_based_partition_ids.size(); ++index)
    {
        auto bisection_id = edge_based_partition_ids[index];
        for (std::size_t level = 0; level < level_masks.size(); ++level)
        {
            osrm::util::CellID cell_id = bisection_id & level_masks[level_masks.size() - 1 - level];
            partitions[level][index] = cell_id;
            partition_sets[level].insert(cell_id);
        }
    }

    std::vector<std::size_t> level_to_num_cells;
    std::transform(partition_sets.begin(),
                   partition_sets.end(),
                   std::back_inserter(level_to_num_cells),
                   [](const std::unordered_set<osrm::util::CellID> &partition_set) {
                       return partition_set.size();
                   });
    // std::cout << "# of cell on levels\n";
    // for (std::size_t level = 0; level < partition_sets.size(); ++level)
    // {
    //     std::cout << level_to_num_cells[level] << ": ";
    //     for (auto x : partition_sets[level])
    //         std::cout << " " << x;
    //     std::cout << "\n";
    // }

    TIMER_START(packed_mlp);
    osrm::util::PackedMultiLevelPartition<false> mlp{partitions, level_to_num_cells};
    TIMER_STOP(packed_mlp);
    util::Log() << "PackedMultiLevelPartition constructed in " << TIMER_SEC(packed_mlp)
                << " seconds";

    TIMER_START(cell_storage);
    osrm::util::CellStorage<false> storage(mlp, *edge_based_graph);
    TIMER_STOP(cell_storage);
    util::Log() << "CellStorage constructed in " << TIMER_SEC(cell_storage) << " seconds";

    TIMER_START(writing_mld_data);
    mlp.Write(config.mld_partition_path);
    storage.Write(config.mld_storage_path);
    TIMER_STOP(writing_mld_data);
    util::Log() << "MLD data writing took " << TIMER_SEC(writing_mld_data) << " seconds";
}

int Customizer::Run(const CustomizationConfig &config)
{
    PartionerOutputCustomizationData(config); // TODO: move to partitioner output_customization_data

    TIMER_START(loading_data);
    auto edge_based_graph = partition::LoadEdgeBasedGraph(config.edge_based_graph_path.string());
    util::Log() << "Loaded edge based graph for mapping partition ids: "
                << edge_based_graph->GetNumberOfEdges() << " edges, "
                << edge_based_graph->GetNumberOfNodes() << " nodes";

    util::PackedMultiLevelPartition<false> mlp;
    mlp.Read(config.mld_partition_path);
    // here start with fresh storage, TODO check caching
    util::CellStorage<false> storage(mlp, *edge_based_graph);
    TIMER_STOP(loading_data);
    util::Log() << "Loading partition data took " << TIMER_SEC(loading_data) << " seconds";

    CellStorageStatistics(*edge_based_graph, mlp, storage);

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

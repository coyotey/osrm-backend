#include "engine/plugins/trip.hpp"

#include "engine/api/trip_api.hpp"
#include "engine/api/trip_parameters.hpp"
#include "engine/trip/trip_brute_force.hpp"
#include "engine/trip/trip_farthest_insertion.hpp"
#include "engine/trip/trip_nearest_neighbour.hpp"
#include "util/dist_table_wrapper.hpp" // to access the dist table more easily
#include "util/json_container.hpp"
#include "util/matrix_graph_wrapper.hpp" // wrapper to use tarjan scc on dist table

#include <boost/assert.hpp>

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{

bool IsStronglyConnectedComponent(const util::DistTableWrapper<EdgeWeight> &result_table)
{
    return std::find(std::begin(result_table), std::end(result_table), INVALID_EDGE_WEIGHT) ==
           std::end(result_table);
}

bool IsSupportedParameterCombination(const bool fixed_start,
                                     const bool fixed_end,
                                     const bool roundtrip)
{
    if (fixed_start && fixed_end && !roundtrip)
    {
        return true;
    }
    else if (roundtrip)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void ManipulateTableForFSE(const std::size_t source_id,
                           const std::size_t destination_id,
                           util::DistTableWrapper<EdgeWeight> &result_table)
{
    // ****************** Change Table *************************
    // The following code manipulates the table and produces the new table for
    // Trip with Fixed Start and End (TFSE). In the example the source is a
    // and destination is c. The new table forces the roundtrip to start at
    // source and end at destination by virtually squashing them together.
    // This way the brute force and the farthest insertion algorithms don't
    // have to be modified, and instead we can just pass a modified table to
    // return a non-roundtrip "optimal" route from a start node to an end node.

    // Original Table           // New Table
    //   a  b  c  d  e          //   a        b         c        d         e
    // a 0  15 36 34 30         // a 0        15        10000    34        30
    // b 15 0  25 30 34         // b 10000    0         25       30        34
    // c 36 25 0  18 32         // c 0        10000     0        10000     10000
    // d 34 30 18 0  15         // d 10000    30        18       0         15
    // e 30 34 32 15 0          // e 10000    34        32       15        0

    // change parameters.source column
    // set any node to source to impossibly high numbers so it will never
    // try to use any node->source in the middle of the "optimal path"
    for (std::size_t i = 0; i < result_table.GetNumberOfNodes(); i++)
    {
        if (i == source_id)
            continue;
        result_table.SetValue(i, source_id, INVALID_EDGE_WEIGHT);
    }

    // change parameters.destination row
    // set destination to anywhere else to impossibly high numbers so it will
    // never try to use destination->any node in the middle of the "optimal path"
    for (std::size_t i = 0; i < result_table.GetNumberOfNodes(); i++)
    {
        if (i == destination_id)
            continue;
        result_table.SetValue(destination_id, i, INVALID_EDGE_WEIGHT);
    }

    // set destination->source to zero so rountrip treats source and
    // destination as one location
    result_table.SetValue(destination_id, source_id, 0);

    // set source->destination as very high number so algorithm is forced
    // to find another path to get to destination
    result_table.SetValue(source_id, destination_id, INVALID_EDGE_WEIGHT);

    //*********  End of changes to table  *************************************
}
}
}
}

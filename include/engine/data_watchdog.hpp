#ifndef OSRM_ENGINE_DATA_WATCHDOG_HPP
#define OSRM_ENGINE_DATA_WATCHDOG_HPP

#include "engine/datafacade/contiguous_internalmem_datafacade.hpp"
#include "engine/datafacade/shared_memory_allocator.hpp"

#include "storage/shared_barrier.hpp"
#include "storage/shared_datatype.hpp"
#include "storage/shared_memory.hpp"

#include <boost/interprocess/sync/named_upgradable_mutex.hpp>
#include <boost/thread/lock_types.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <memory>
#include <thread>

namespace osrm
{
namespace engine
{

// This class monitors the shared memory region that contains the pointers to
// the data and layout regions that should be used. This region is updated
// once a new dataset arrives.
template <typename AlgorithmT> class DataWatchdog final
{
    using FacadeT = datafacade::ContiguousInternalMemoryDataFacade<AlgorithmT>;

  public:
    DataWatchdog() : barrier(boost::interprocess::open_only), active(true), timestamp(0)
    {
        // create the initial facade before launching the watchdog thread
        {
            boost::interprocess::scoped_lock<storage::SharedBarrier::mutex_type>
                current_region_lock(barrier.GetMutex());

            facade = std::make_shared<const FacadeT>(
                std::make_unique<datafacade::SharedMemoryAllocator>(barrier.GetRegion()));
            timestamp = barrier.GetTimestamp();
        }

        watcher = std::thread(&DataWatchdog::Run, this);
    }

    virtual ~DataWatchdog()
    {
        active = false;
        barrier.NotifyAll();
        watcher.join();
    }

    std::shared_ptr<const FacadeT> Get() const { return facade; }

  private:
    void Run()
    {
        while (active)
        {
            boost::interprocess::scoped_lock<storage::SharedBarrier::mutex_type>
                current_region_lock(barrier.GetMutex());

            while (active && timestamp == barrier.GetTimestamp())
            {
                barrier.Wait(current_region_lock);
            }

            if (timestamp != barrier.GetTimestamp())
            {
                facade = std::make_shared<const FacadeT>(
                    std::make_unique<datafacade::SharedMemoryAllocator>(barrier.GetRegion()));
                timestamp = barrier.GetTimestamp();
                util::Log() << "updated facade to region "
                            << storage::regionToString(barrier.GetRegion()) << " with timestamp "
                            << timestamp;
            }
        }

        util::Log() << "DataWatchdog thread stopped";
    }

    storage::SharedBarrier barrier;
    std::thread watcher;
    bool active;
    unsigned timestamp;
    std::shared_ptr<const FacadeT> facade;
};
}
}

#endif

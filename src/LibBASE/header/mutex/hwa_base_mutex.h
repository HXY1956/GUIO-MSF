#ifndef hwa_base_mutex_h
#define hwa_base_mutex_h
#include <thread>
#include <mutex>

namespace hwa_base
{
    /** @brief class for base_mutex. */
    class base_mutex
    {
    public:
        /** @brief default constructor. */
        base_mutex();

        /** @brief copy constructor. */
        base_mutex(const base_mutex& Other);

        /** @brief default destructor. */
        ~base_mutex();

        /** @brief override operator =. */
        base_mutex operator=(const base_mutex& Other);

        /** @brief lock. */
        void lock();

        /** @brief unlock. */
        void unlock();

        bool isLock = false;

    protected:
#ifdef USE_OPENMP
        omp_lock_t _mutex;
#else
        std::mutex _mutex;

#endif // USE_OPENMP
    };
} // namespace

#endif
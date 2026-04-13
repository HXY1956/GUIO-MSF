#include <iostream>
#include <iomanip>
#include "hwa_base_mutex.h"

namespace hwa_base
{
    base_mutex::base_mutex()
    {
        isLock = false;
    }

    base_mutex::base_mutex(const base_mutex& Other)
    {
        isLock = Other.isLock;
    }

    base_mutex::~base_mutex()
    {
    }

    base_mutex base_mutex::operator=(const base_mutex& Other)
    {
        return base_mutex();
    }

    // ----------
    void base_mutex::lock()
    {
        _mutex.lock();
    }

    // ----------
    void base_mutex::unlock()
    {
        _mutex.unlock();
    }

}

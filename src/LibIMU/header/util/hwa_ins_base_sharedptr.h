#ifndef hwa_ins_sharedptr_h
#define hwa_ins_sharedptr_h
#include <memory>
#include <mutex>
#include <atomic>
#include <utility>
#include <functional>
#include "hwa_ins_proc.h"

namespace hwa_ins {
    template <class T>
    class SharedHandle {
    public:
        using Ptr = std::shared_ptr<T>;
        explicit SharedHandle(Ptr init = std::make_shared<T>())
            : cur_(std::move(init)) {}
        Ptr get() const {
            return std::atomic_load_explicit(&cur_, std::memory_order_acquire);
        }
        Ptr swap_in(Ptr newp) {
            Ptr old = std::atomic_load_explicit(&cur_, std::memory_order_acquire);
            while (!std::atomic_compare_exchange_weak_explicit(
                &cur_, &old, newp, std::memory_order_release, std::memory_order_acquire)) {
            }
            return old;
        }
        std::shared_ptr<T> adopt(std::unique_ptr<T> up) {
            return swap_in(Ptr(up.release()));
        }
        Ptr extract() {
            return swap_in(nullptr);
        }
        template <class F>
        auto with_current(F&& f) -> decltype(f(std::declval<T&>())) {
            auto sp = get();
            if (!sp) throw std::runtime_error("SharedHandle: no current object");
            return f(*sp);
        }

    private:
        mutable std::shared_ptr<T> cur_;
    };

    using SharedIns = SharedHandle<ins_obj>;
};


#endif
#pragma once

#include "czmq.h"
#include <memory>
#include <mutex>

namespace RayFEM {
    template <typename T, void (*Destroyer)(T**)>
    class ZmqPtr {
    public:
        ZmqPtr(T* ptr = nullptr) : ptr_(ptr) {}
        ~ZmqPtr() { if (ptr_) Destroyer(&ptr_); }

        ZmqPtr(const ZmqPtr&) = delete;
        ZmqPtr& operator=(const ZmqPtr&) = delete;

        ZmqPtr(ZmqPtr&& other) noexcept : ptr_(other.ptr_) { other.ptr_ = nullptr; }
        ZmqPtr& operator=(ZmqPtr&& other) noexcept {
            if (this != std::addressof(other)) {
                if (ptr_) Destroyer(&ptr_);
                ptr_ = other.ptr_;
                other.ptr_ = nullptr;
            }
            return *this;
        }

        T* release() {
            T* temp = ptr_;
            ptr_ = nullptr;
            return temp;
        }

        T* get() const { return ptr_; }
        operator T*() const { return ptr_; }
        T** operator&() { return &ptr_; }

    private:
        T* ptr_ = nullptr;
    };

    // RayFEM's client side (init, stats, shutdown and StreamClient) runs
    // inside other programs, so it must not take over their Ctrl-C. CZMQ
    // installs SIGINT and SIGTERM handlers that only set zsys_interrupted
    // when its first socket is created; this keeps (or restores) the
    // program's own. The viewer's Server does not call it: its main loop
    // watches zsys_interrupted.
    inline void KeepHostSignals() {
        static std::once_flag once;
        std::call_once(once, [] { zsys_handler_set(nullptr); });
    }

    using SocketPtr = ZmqPtr<zsock_t, zsock_destroy>;
    using MsgPtr    = ZmqPtr<zmsg_t, zmsg_destroy>;
    using FramePtr  = ZmqPtr<zframe_t, zframe_destroy>;
}
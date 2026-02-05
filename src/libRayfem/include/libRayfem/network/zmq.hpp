#pragma once

#include "czmq.h"

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
            if (this != &other) {
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

    using SocketPtr = ZmqPtr<zsock_t, zsock_destroy>;
    using MsgPtr    = ZmqPtr<zmsg_t, zmsg_destroy>;
    using FramePtr  = ZmqPtr<zframe_t, zframe_destroy>;
}
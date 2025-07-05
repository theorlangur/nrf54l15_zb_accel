#ifndef LIB_MSGQ_TYPED_HPP_
#define LIB_MSGQ_TYPED_HPP_

#include <zephyr/kernel.h>
#include <cstddef>

namespace msgq
{
    namespace impl
    {
        template<auto*, typename Entry, size_t N, size_t align = alignof(Entry)>
        char __noinit __aligned(align) g_MessageQueueFifo[sizeof(Entry) * N];

        template<typename E>
        struct QueueConfig
        {
            size_t align = alignof(E);
            k_timeout_t timeout = K_FOREVER;
        };

        template<auto *q>
        struct MsgQFifo;
    }

    template<typename Entry, size_t N, impl::QueueConfig<Entry> opts>
    struct Queue;

    namespace impl
    {
        template<typename Entry, size_t N, QueueConfig<Entry> opts, Queue<Entry, N, opts> *q>
        struct MsgQFifo<q>
        {
            static constexpr auto& fifo() { return g_MessageQueueFifo<q, Entry, N, opts.align>; }
            static constexpr size_t size() { return sizeof(g_MessageQueueFifo<q, Entry, N, opts.align>); }
        };
    }

    template<typename Entry, size_t N, impl::QueueConfig<Entry> opts = {}>
    struct Queue
    {
        template<typename FifoType>
        constexpr Queue(FifoType):
            q{
                .wait_q = Z_WAIT_Q_INIT(&q.wait_q),
                    .msg_size = sizeof(Entry), 
                    .max_msgs = N, 
                    .buffer_start = FifoType::fifo(), 
                    .buffer_end = FifoType::fifo() + FifoType::size(),
                    .read_ptr = FifoType::fifo(), 
                    .write_ptr = FifoType::fifo(), 
                    .used_msgs = 0, 
                    Z_POLL_EVENT_OBJ_INIT(q) 
            }
        {
        }
        Queue(const Queue&) = delete;
        Queue(Queue&&) = delete;

        int operator<<(Entry e)
        {
            return k_msgq_put(&q, &e, opts.timeout);
        }

        int operator>>(Entry &e)
        {
            return k_msgq_get(&q, &e, opts.timeout);
        }

        int Send(Entry e, k_timeout_t timeout)
        {
            return k_msgq_put(&q, &e, timeout);
        }

        int Recv(Entry &e, k_timeout_t timeout)
        {
            return k_msgq_get(&q, &e, timeout);
        }

        explicit constexpr operator k_msgq*() { return &q; }

        k_msgq q;
    };

#define K_MSGQ_DEFINE_TYPED(type, q_name) \
constinit TYPE_SECTION_ITERABLE(type, q_name, k_msgq, q_name) = {::msgq::impl::MsgQFifo<&q_name>{}}

}

#endif

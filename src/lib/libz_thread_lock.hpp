#ifndef LIBZ_THREAD_LOCK_HPP_
#define LIBZ_THREAD_LOCK_HPP_

#include <zephyr/spinlock.h>

namespace thread{
    class SpinLock
    {
    public:
        auto lock() { return k_spin_lock(&m_Lock); }
        void unlock(k_spinlock_key_t k) { k_spin_unlock(&m_Lock, k); }

    private:
        k_spinlock m_Lock{};
    };

    struct DummyLock
    {
        k_spinlock_key_t lock() { return {}; }
        void unlock(k_spinlock_key_t k) {}
    };

    class ILockable
    {
    public:
        virtual ~ILockable() = default; 
        virtual void lock() = 0;
        virtual void unlock() = 0;
    };

    class NoLock: public ILockable
    {
    public:
        virtual void lock() override {}
        virtual void unlock() override {}
    };

    template<class L>
    struct LockGuard
    {
        LockGuard(L *pL):m_pLock(pL) { if (pL) m_Key = pL->lock(); }
        LockGuard(L &l):LockGuard(&l){}
        ~LockGuard() { if (m_pLock) m_pLock->unlock(m_Key); }
    private:
        L *m_pLock;
        k_spinlock_key_t m_Key;
    };
    template<class L>
    LockGuard(L *)->LockGuard<L>;
    template<class L>
    LockGuard(L &)->LockGuard<L>;
}
#endif

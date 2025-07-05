#ifndef ZBH_ALARM_HPP_
#define ZBH_ALARM_HPP_

#include <optional>
#include <zephyr/sys/reboot.h>
#include <cstdint>
#include "lib_function.hpp"
#include "lib_misc_helpers.hpp"
#include "../lib/libz_thread_lock.hpp"
#include "zboss_api_core.h"

#ifndef ALARM_LIST_LOCK_TYPE
#define ALARM_LIST_LOCK_TYPE thread::SpinLock
#endif

namespace zb
{
    struct ZbAlarm
    {
        static constexpr uint8_t kCounterOfDeathInactive = 0xff;
        static constexpr uint8_t kCounterOfDeathValue = 6;
        static constexpr uint8_t kLowOnHandlesThreshold = 2;

        static bool g_RunningOutOfHandles;
        static uint8_t g_CounterOfDeath;

        struct TimerList
        {
            static constexpr uint8_t kFull = 9;
            static constexpr uint8_t kMaxSize = kFull - 1;

            using handle_t = uint8_t;
            static constexpr handle_t kInvalidHandle = kFull;

            struct TimerEntry
            {
                ZbAlarm *pAlarm;
                void* cb;
                void *param;
            };

            struct HandleEntry
            {
                TimerEntry &entry;
                handle_t h;
            };

            std::optional<HandleEntry> Allocate()
            {
                thread::LockGuard l(&g_AlarmLock);
                if (head != kFull)
                {
                    auto r = head;
                    head = entries[head].nextFree;
                    return HandleEntry{entries[r].entry, r};
                }
                return std::nullopt;
            }

            void Free(handle_t h)
            {
                thread::LockGuard l(&g_AlarmLock);
                entries[h].nextFree = head;
                head = h;
            }

            auto& GetEntry(handle_t h) { return entries[h].entry; }

            static constexpr TimerList Create()
            {
                TimerList r;
                r.head = 0;
                for(size_t i = 0; i < kMaxSize; ++i)
                {
                    auto &e = r.entries[i];
                    e.nextFree = i + 1;
                }
                r.entries[kMaxSize - 1].nextFree = kFull;
                return r;
            }

        private:

            union ListEntry
            {
                uint8_t nextFree;
                TimerEntry entry;
            };
            handle_t head;
            ListEntry entries[kMaxSize];

            static ALARM_LIST_LOCK_TYPE g_AlarmLock;
        };
        constinit static TimerList g_TimerList;

        using callback_t = void(*)(void*);

        static void on_alarm(TimerList::TimerEntry const& e)
        {
            ((callback_t)e.cb)(e.param);
        }

        template<auto CB = ZbAlarm::on_alarm>
        static void on_scheduled_alarm(uint8_t param)
        {
            auto e = g_TimerList.GetEntry(param);
            g_TimerList.Free(e.pAlarm->h);
            e.pAlarm->h = TimerList::kInvalidHandle;
            CB(e);
        }

        const char *pDescr = nullptr;
        TimerList::handle_t h = TimerList::kInvalidHandle;

        ~ZbAlarm() { Cancel(); }

        bool IsRunning() const { return h != TimerList::kInvalidHandle; }
        void Cancel() { Cancel<on_alarm>(); }
        zb_ret_t Setup(callback_t cb, void *param, uint32_t time) { return Setup<callback_t, on_alarm>(cb, param, time); }

    protected:

        template<auto CB>
        void Cancel()
        {
            if (IsRunning())
            {
                //REMOVE_ME:a workaround for esp zigbee sdk memory leak:
                zb_schedule_alarm_cancel(on_scheduled_alarm<CB>, h, nullptr);
                g_TimerList.Free(h);
                h = TimerList::kInvalidHandle;
            }
        }

        template<class callback_t, auto CB>
        zb_ret_t Setup(callback_t cb, void *param, uint32_t time)
        {
            Cancel<CB>();
            if (auto r = g_TimerList.Allocate())
            {
                auto &v = *r;
                h = v.h;
                static_assert(sizeof(void*) == sizeof(callback_t));
                v.entry.cb = (void*)cb;
                v.entry.param = param;
                v.entry.pAlarm = this;

                if (h >= kLowOnHandlesThreshold)
                {
                    FMT_PRINT("Got back handle {:x} >= threshold of {:x}. Prepare to die\n", h, kLowOnHandlesThreshold);
                    //soon we'll be out of handles - let's restart at a convenient moment
                    g_RunningOutOfHandles = true;
                }
                return zb_schedule_app_alarm(on_scheduled_alarm<CB>, h, ZB_MILLISECONDS_TO_BEACON_INTERVAL(time));
            }else
                return RET_NO_MEMORY;
        }

    public:
        static void deactivate_counter_of_death()
        {
            FMT_PRINT("Low on handles: Counter of death deactivated\n");
            g_CounterOfDeath = kCounterOfDeathInactive;
        }

        static void check_counter_of_death()
        {
            if (g_RunningOutOfHandles)
            {
                g_CounterOfDeath = kCounterOfDeathValue;
                FMT_PRINT("Low on handles: Counter of death activated: {} iterations left\n", g_CounterOfDeath);
            }
        }

        static void check_death_count()
        {
            if (g_RunningOutOfHandles && g_CounterOfDeath != kCounterOfDeathInactive)
            {
                if (!(--g_CounterOfDeath))
                {
                    //boom
                    FMT_PRINT("Low on handles: time to die and reborn\n");

                    sys_reboot(SYS_REBOOT_WARM);
                    return;
                }
                FMT_PRINT("Low on handles: tick-tock: {} iterations left\n", ZbAlarm::g_CounterOfDeath);
            }
        }
    };

    inline bool ZbAlarm::g_RunningOutOfHandles = false;
    inline uint8_t ZbAlarm::g_CounterOfDeath = kCounterOfDeathInactive;
    inline ALARM_LIST_LOCK_TYPE ZbAlarm::TimerList::g_AlarmLock;
    inline constinit ZbAlarm::TimerList ZbAlarm::g_TimerList = ZbAlarm::TimerList::Create();

    struct ZbTimer: ZbAlarm
    {
        //return 'true' if timer should go on, false - if timer should stop
        using callback_t = bool (*)(void* param);

        uint32_t m_Interval = 0;

        ~ZbTimer() { Cancel(); }

        static void on_timer(TimerList::TimerEntry const& e)
        {
            if (callback_t(e.cb)(e.param))
            {
                ZbTimer *pT = static_cast<ZbTimer*>(e.pAlarm);
                //re-schedule
                auto res = pT->Setup(callback_t(e.cb), e.param, pT->m_Interval);
                if (res != RET_OK)
                {
                    FMT_PRINT("Could not re-register timer with error {:x}\n", res);
                }
            }
        }

        void Cancel() { ZbAlarm::Cancel<on_timer>(); }

        zb_ret_t Setup(callback_t cb, void *param, uint32_t time)
        {
            m_Interval = time;
            return ZbAlarm::Setup<callback_t, on_timer>(cb, param, time);
        }
    };

    template<size_t FuncSZ = 16>
    struct ZbTimerExt: ZbTimer
    {
        using generic_callback_t = FixedFunction<FuncSZ, bool()>;
        generic_callback_t m_Callback;

        static bool on_timer_ext(void *param) { return (*(generic_callback_t*)param)(); }

        template<class callback_t>
        zb_ret_t Setup(callback_t &&cb, uint32_t time)
        {
            m_Callback = std::forward<callback_t>(cb);
            return ZbTimer::Setup(on_timer_ext, &m_Callback, time);
        }
    };
    using ZbTimerExt16 = ZbTimerExt<16>;

    template<size_t FuncSZ = 16>
    struct ZbAlarmExt: ZbAlarm
    {
        using generic_callback_t = FixedFunction<FuncSZ, void()>;
        generic_callback_t m_Callback;

        static void on_timer_ext(void *param) { (*(generic_callback_t*)param)(); }

        template<class callback_t>
        zb_ret_t Setup(callback_t &&cb, uint32_t time)
        {
            m_Callback = std::forward<callback_t>(cb);
            return ZbAlarm::Setup(on_timer_ext, &m_Callback, time);
        }
    };
    using ZbAlarmExt16 = ZbAlarmExt<16>;
}
#endif

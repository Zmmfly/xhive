target("src_utest")
    set_kind("object")
    on_load(function(target) 
        local conf     = target:data("kconfig")
        local sdir     = os.scriptdir()
        local perf_dir = path.join(sdir, "perf")
        local smp_dir  = path.join(sdir, "smp")

        local add_to = function(target, name, lst_dir, lst_tab, conf, public)
            local list = {}
            public = public or false
            for k, v in pairs(lst_tab) do
                if conf[k] then
                    if type(v) == "table" then
                        for _, src in ipairs(v) do
                            table.insert(list, path.join(lst_dir, src))
                        end
                    else
                        table.insert(list, path.join(lst_dir, v))
                    end
                end
            end
            if public then
                target:add(name, list, {public = true})
            else
                target:add(name, list)
            end
        end

        local src_maps = {
            RT_UTEST_OBJECT        = "object_tc.c",
            RT_UTEST_MEMHEAP       = "memheap_tc.c",
            RT_UTEST_SMALL_MEM     = "mem_tc.c",
            RT_UTEST_SLAB          = "slab_tc.c",
            RT_UTEST_IRQ           = "irq_tc.c",
            RT_UTEST_SEMAPHORE     = "semaphore_tc.c",
            RT_UTEST_EVENT         = "event_tc.c",
            RT_UTEST_TIMER         = "timer_tc.c",
            RT_UTEST_MESSAGEQUEUE  = "messagequeue_tc.c",
            RT_UTEST_SIGNAL        = "signal_tc.c",
            RT_UTEST_MUTEX         = {"mutex_tc.c", "mutex_pi_tc.c"},
            RT_UTEST_MAILBOX       = "mailbox_tc.c",
            RT_UTEST_THREAD        = {"thread_tc.c", "thread_overflow_tc.c", "thread_suspend_tc.c"},
            RT_UTEST_ATOMIC        = "atomic_tc.c",
            RT_UTEST_HOOKLIST      = "hooklist_tc.c",
            RT_UTEST_MTSAFE_KPRINT = "mtsafe_kprint_tc.c",
            RT_UTEST_MEMPOOL       = "mempool_tc.c",
            RT_UTEST_SCHEDULER     = {"sched_timed_sem_tc.c", "sched_timed_mtx_tc.c", "sched_mtx_tc.c", "sched_sem_tc.c", "sched_thread_tc.c"}
        }

        local smp_maps = {
            RT_UTEST_SMP_SPINLOCK           = "smp_spinlock_tc.c",
            RT_UTEST_SMP_ASSIGNED_IDLE_CORE = "smp_assigned_idle_cores_tc.c",
            RT_UTEST_SMP_INTERRUPT_PRI      = "smp_interrupt_pri_tc.c",
            RT_UTEST_SMP_THREAD_PREEMPTION  = "smp_thread_preemption_tc.c",
            RT_UTEST_SMP_AFFFINITY          = {"smp_bind_affinity_tc", "smp_affinity_pri1_tc.c", "smp_affinity_pri2_tc.c"}
        }

        if conf.RT_USING_UTESTCASES then
            target:add("includedirs", sdir)
            add_to(target, "files", sdir, src_maps, conf)
            add_to(target, "files", smp_dir, smp_maps, conf)
        end

        if conf.RT_UTEST_SYS_PERF then
            target:add("files", path.join(perf_dir, "*.c"))
            target:add("includedirs", perf_dir)
        end
    end)

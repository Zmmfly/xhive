# Third-Party Embedded Libraries

**NOTE: Update the table below when libraries are added or updated**

| Library Name | Version  |                              License                               |                            URL                             |
| :----------: | :------: | :----------------------------------------------------------------: | :--------------------------------------------------------: |
|   freertos   |   TBD    |                                TBD                                 |                            TBD                             |
|  RT-Thread   | `v5.2.2` |                             Apache 2.0                             |                            TBD                             |
|  SEGGER_RTT  | `8.56a`  | [license](https://github.com/SEGGERMicro/RTT/blob/main/LICENSE.md) | [repository](https://github.com/SEGGERMicro/RTT/tree/main) |


## RT-Thread Nano

Usage Instructions:
1. Implement `rt_systick_config`
2. Call `rt_os_tick_callback` in the SysTick handler

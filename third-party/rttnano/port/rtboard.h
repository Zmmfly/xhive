#ifndef __RT_BOARD_H__
#define __RT_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void rt_os_tick_callback(void);
void* rt_bss_end();
void* rt_ram_end();

#ifdef __cplusplus
}
#endif // __cplusplus
#endif /* __RT_BOARD_H__ */

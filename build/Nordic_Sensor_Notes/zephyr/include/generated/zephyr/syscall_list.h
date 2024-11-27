/* auto-generated by gen_syscalls.py, don't edit */

#ifndef ZEPHYR_SYSCALL_LIST_H
#define ZEPHYR_SYSCALL_LIST_H

#define K_SYSCALL_DEVICE_GET_BINDING 0
#define K_SYSCALL_DEVICE_GET_BY_DT_NODELABEL 1
#define K_SYSCALL_DEVICE_INIT 2
#define K_SYSCALL_DEVICE_IS_READY 3
#define K_SYSCALL_GPIO_GET_PENDING_INT 4
#define K_SYSCALL_GPIO_PIN_CONFIGURE 5
#define K_SYSCALL_GPIO_PIN_GET_CONFIG 6
#define K_SYSCALL_GPIO_PIN_INTERRUPT_CONFIGURE 7
#define K_SYSCALL_GPIO_PORT_CLEAR_BITS_RAW 8
#define K_SYSCALL_GPIO_PORT_GET_DIRECTION 9
#define K_SYSCALL_GPIO_PORT_GET_RAW 10
#define K_SYSCALL_GPIO_PORT_SET_BITS_RAW 11
#define K_SYSCALL_GPIO_PORT_SET_MASKED_RAW 12
#define K_SYSCALL_GPIO_PORT_TOGGLE_BITS 13
#define K_SYSCALL_I2C_CONFIGURE 14
#define K_SYSCALL_I2C_GET_CONFIG 15
#define K_SYSCALL_I2C_RECOVER_BUS 16
#define K_SYSCALL_I2C_TARGET_DRIVER_REGISTER 17
#define K_SYSCALL_I2C_TARGET_DRIVER_UNREGISTER 18
#define K_SYSCALL_I2C_TRANSFER 19
#define K_SYSCALL_K_BUSY_WAIT 20
#define K_SYSCALL_K_CONDVAR_BROADCAST 21
#define K_SYSCALL_K_CONDVAR_INIT 22
#define K_SYSCALL_K_CONDVAR_SIGNAL 23
#define K_SYSCALL_K_CONDVAR_WAIT 24
#define K_SYSCALL_K_EVENT_CLEAR 25
#define K_SYSCALL_K_EVENT_INIT 26
#define K_SYSCALL_K_EVENT_POST 27
#define K_SYSCALL_K_EVENT_SET 28
#define K_SYSCALL_K_EVENT_SET_MASKED 29
#define K_SYSCALL_K_EVENT_WAIT 30
#define K_SYSCALL_K_EVENT_WAIT_ALL 31
#define K_SYSCALL_K_FLOAT_DISABLE 32
#define K_SYSCALL_K_FLOAT_ENABLE 33
#define K_SYSCALL_K_FUTEX_WAIT 34
#define K_SYSCALL_K_FUTEX_WAKE 35
#define K_SYSCALL_K_IS_PREEMPT_THREAD 36
#define K_SYSCALL_K_MSGQ_ALLOC_INIT 37
#define K_SYSCALL_K_MSGQ_GET 38
#define K_SYSCALL_K_MSGQ_GET_ATTRS 39
#define K_SYSCALL_K_MSGQ_NUM_FREE_GET 40
#define K_SYSCALL_K_MSGQ_NUM_USED_GET 41
#define K_SYSCALL_K_MSGQ_PEEK 42
#define K_SYSCALL_K_MSGQ_PEEK_AT 43
#define K_SYSCALL_K_MSGQ_PURGE 44
#define K_SYSCALL_K_MSGQ_PUT 45
#define K_SYSCALL_K_MUTEX_INIT 46
#define K_SYSCALL_K_MUTEX_LOCK 47
#define K_SYSCALL_K_MUTEX_UNLOCK 48
#define K_SYSCALL_K_OBJECT_ACCESS_GRANT 49
#define K_SYSCALL_K_OBJECT_ALLOC 50
#define K_SYSCALL_K_OBJECT_ALLOC_SIZE 51
#define K_SYSCALL_K_OBJECT_RELEASE 52
#define K_SYSCALL_K_PIPE_ALLOC_INIT 53
#define K_SYSCALL_K_PIPE_BUFFER_FLUSH 54
#define K_SYSCALL_K_PIPE_FLUSH 55
#define K_SYSCALL_K_PIPE_GET 56
#define K_SYSCALL_K_PIPE_PUT 57
#define K_SYSCALL_K_PIPE_READ_AVAIL 58
#define K_SYSCALL_K_PIPE_WRITE_AVAIL 59
#define K_SYSCALL_K_POLL 60
#define K_SYSCALL_K_POLL_SIGNAL_CHECK 61
#define K_SYSCALL_K_POLL_SIGNAL_INIT 62
#define K_SYSCALL_K_POLL_SIGNAL_RAISE 63
#define K_SYSCALL_K_POLL_SIGNAL_RESET 64
#define K_SYSCALL_K_QUEUE_ALLOC_APPEND 65
#define K_SYSCALL_K_QUEUE_ALLOC_PREPEND 66
#define K_SYSCALL_K_QUEUE_CANCEL_WAIT 67
#define K_SYSCALL_K_QUEUE_GET 68
#define K_SYSCALL_K_QUEUE_INIT 69
#define K_SYSCALL_K_QUEUE_IS_EMPTY 70
#define K_SYSCALL_K_QUEUE_PEEK_HEAD 71
#define K_SYSCALL_K_QUEUE_PEEK_TAIL 72
#define K_SYSCALL_K_SCHED_CURRENT_THREAD_QUERY 73
#define K_SYSCALL_K_SEM_COUNT_GET 74
#define K_SYSCALL_K_SEM_GIVE 75
#define K_SYSCALL_K_SEM_INIT 76
#define K_SYSCALL_K_SEM_RESET 77
#define K_SYSCALL_K_SEM_TAKE 78
#define K_SYSCALL_K_SLEEP 79
#define K_SYSCALL_K_STACK_ALLOC_INIT 80
#define K_SYSCALL_K_STACK_POP 81
#define K_SYSCALL_K_STACK_PUSH 82
#define K_SYSCALL_K_STR_OUT 83
#define K_SYSCALL_K_THREAD_ABORT 84
#define K_SYSCALL_K_THREAD_CREATE 85
#define K_SYSCALL_K_THREAD_CUSTOM_DATA_GET 86
#define K_SYSCALL_K_THREAD_CUSTOM_DATA_SET 87
#define K_SYSCALL_K_THREAD_DEADLINE_SET 88
#define K_SYSCALL_K_THREAD_JOIN 89
#define K_SYSCALL_K_THREAD_NAME_COPY 90
#define K_SYSCALL_K_THREAD_NAME_SET 91
#define K_SYSCALL_K_THREAD_PRIORITY_GET 92
#define K_SYSCALL_K_THREAD_PRIORITY_SET 93
#define K_SYSCALL_K_THREAD_RESUME 94
#define K_SYSCALL_K_THREAD_STACK_ALLOC 95
#define K_SYSCALL_K_THREAD_STACK_FREE 96
#define K_SYSCALL_K_THREAD_STACK_SPACE_GET 97
#define K_SYSCALL_K_THREAD_START 98
#define K_SYSCALL_K_THREAD_SUSPEND 99
#define K_SYSCALL_K_THREAD_TIMEOUT_EXPIRES_TICKS 100
#define K_SYSCALL_K_THREAD_TIMEOUT_REMAINING_TICKS 101
#define K_SYSCALL_K_TIMER_EXPIRES_TICKS 102
#define K_SYSCALL_K_TIMER_REMAINING_TICKS 103
#define K_SYSCALL_K_TIMER_START 104
#define K_SYSCALL_K_TIMER_STATUS_GET 105
#define K_SYSCALL_K_TIMER_STATUS_SYNC 106
#define K_SYSCALL_K_TIMER_STOP 107
#define K_SYSCALL_K_TIMER_USER_DATA_GET 108
#define K_SYSCALL_K_TIMER_USER_DATA_SET 109
#define K_SYSCALL_K_UPTIME_TICKS 110
#define K_SYSCALL_K_USLEEP 111
#define K_SYSCALL_K_WAKEUP 112
#define K_SYSCALL_K_YIELD 113
#define K_SYSCALL_LOG_BUFFERED_CNT 114
#define K_SYSCALL_LOG_FILTER_SET 115
#define K_SYSCALL_LOG_FRONTEND_FILTER_SET 116
#define K_SYSCALL_LOG_PANIC 117
#define K_SYSCALL_LOG_PROCESS 118
#define K_SYSCALL_SENSOR_ATTR_GET 119
#define K_SYSCALL_SENSOR_ATTR_SET 120
#define K_SYSCALL_SENSOR_CHANNEL_GET 121
#define K_SYSCALL_SENSOR_GET_DECODER 122
#define K_SYSCALL_SENSOR_RECONFIGURE_READ_IODEV 123
#define K_SYSCALL_SENSOR_SAMPLE_FETCH 124
#define K_SYSCALL_SENSOR_SAMPLE_FETCH_CHAN 125
#define K_SYSCALL_SYS_CLOCK_HW_CYCLES_PER_SEC_RUNTIME_GET 126
#define K_SYSCALL_UART_CONFIGURE 127
#define K_SYSCALL_UART_CONFIG_GET 128
#define K_SYSCALL_UART_DRV_CMD 129
#define K_SYSCALL_UART_ERR_CHECK 130
#define K_SYSCALL_UART_IRQ_ERR_DISABLE 131
#define K_SYSCALL_UART_IRQ_ERR_ENABLE 132
#define K_SYSCALL_UART_IRQ_IS_PENDING 133
#define K_SYSCALL_UART_IRQ_RX_DISABLE 134
#define K_SYSCALL_UART_IRQ_RX_ENABLE 135
#define K_SYSCALL_UART_IRQ_TX_DISABLE 136
#define K_SYSCALL_UART_IRQ_TX_ENABLE 137
#define K_SYSCALL_UART_IRQ_UPDATE 138
#define K_SYSCALL_UART_LINE_CTRL_GET 139
#define K_SYSCALL_UART_LINE_CTRL_SET 140
#define K_SYSCALL_UART_POLL_IN 141
#define K_SYSCALL_UART_POLL_IN_U16 142
#define K_SYSCALL_UART_POLL_OUT 143
#define K_SYSCALL_UART_POLL_OUT_U16 144
#define K_SYSCALL_UART_RX_DISABLE 145
#define K_SYSCALL_UART_RX_ENABLE 146
#define K_SYSCALL_UART_RX_ENABLE_U16 147
#define K_SYSCALL_UART_TX 148
#define K_SYSCALL_UART_TX_ABORT 149
#define K_SYSCALL_UART_TX_U16 150
#define K_SYSCALL_ZEPHYR_FPUTC 151
#define K_SYSCALL_ZEPHYR_FWRITE 152
#define K_SYSCALL_ZEPHYR_READ_STDIN 153
#define K_SYSCALL_ZEPHYR_WRITE_STDOUT 154
#define K_SYSCALL_Z_LOG_MSG_SIMPLE_CREATE_0 155
#define K_SYSCALL_Z_LOG_MSG_SIMPLE_CREATE_1 156
#define K_SYSCALL_Z_LOG_MSG_SIMPLE_CREATE_2 157
#define K_SYSCALL_Z_LOG_MSG_STATIC_CREATE 158
#define K_SYSCALL_Z_SYS_MUTEX_KERNEL_LOCK 159
#define K_SYSCALL_Z_SYS_MUTEX_KERNEL_UNLOCK 160
#define K_SYSCALL_BAD 161
#define K_SYSCALL_LIMIT 162


/* Following syscalls are not used in image */
#define K_SYSCALL_ADC_CHANNEL_SETUP 163
#define K_SYSCALL_ADC_READ 164
#define K_SYSCALL_ADC_READ_ASYNC 165
#define K_SYSCALL_ATOMIC_ADD 166
#define K_SYSCALL_ATOMIC_AND 167
#define K_SYSCALL_ATOMIC_CAS 168
#define K_SYSCALL_ATOMIC_NAND 169
#define K_SYSCALL_ATOMIC_OR 170
#define K_SYSCALL_ATOMIC_PTR_CAS 171
#define K_SYSCALL_ATOMIC_PTR_SET 172
#define K_SYSCALL_ATOMIC_SET 173
#define K_SYSCALL_ATOMIC_SUB 174
#define K_SYSCALL_ATOMIC_XOR 175
#define K_SYSCALL_AUXDISPLAY_BACKLIGHT_GET 176
#define K_SYSCALL_AUXDISPLAY_BACKLIGHT_SET 177
#define K_SYSCALL_AUXDISPLAY_BRIGHTNESS_GET 178
#define K_SYSCALL_AUXDISPLAY_BRIGHTNESS_SET 179
#define K_SYSCALL_AUXDISPLAY_CAPABILITIES_GET 180
#define K_SYSCALL_AUXDISPLAY_CLEAR 181
#define K_SYSCALL_AUXDISPLAY_CURSOR_POSITION_GET 182
#define K_SYSCALL_AUXDISPLAY_CURSOR_POSITION_SET 183
#define K_SYSCALL_AUXDISPLAY_CURSOR_SET_ENABLED 184
#define K_SYSCALL_AUXDISPLAY_CURSOR_SHIFT_SET 185
#define K_SYSCALL_AUXDISPLAY_CUSTOM_CHARACTER_SET 186
#define K_SYSCALL_AUXDISPLAY_CUSTOM_COMMAND 187
#define K_SYSCALL_AUXDISPLAY_DISPLAY_OFF 188
#define K_SYSCALL_AUXDISPLAY_DISPLAY_ON 189
#define K_SYSCALL_AUXDISPLAY_DISPLAY_POSITION_GET 190
#define K_SYSCALL_AUXDISPLAY_DISPLAY_POSITION_SET 191
#define K_SYSCALL_AUXDISPLAY_IS_BUSY 192
#define K_SYSCALL_AUXDISPLAY_POSITION_BLINKING_SET_ENABLED 193
#define K_SYSCALL_AUXDISPLAY_WRITE 194
#define K_SYSCALL_BBRAM_CHECK_INVALID 195
#define K_SYSCALL_BBRAM_CHECK_POWER 196
#define K_SYSCALL_BBRAM_CHECK_STANDBY_POWER 197
#define K_SYSCALL_BBRAM_GET_SIZE 198
#define K_SYSCALL_BBRAM_READ 199
#define K_SYSCALL_BBRAM_WRITE 200
#define K_SYSCALL_BC12_SET_RESULT_CB 201
#define K_SYSCALL_BC12_SET_ROLE 202
#define K_SYSCALL_CAN_ADD_RX_FILTER_MSGQ 203
#define K_SYSCALL_CAN_CALC_TIMING 204
#define K_SYSCALL_CAN_CALC_TIMING_DATA 205
#define K_SYSCALL_CAN_GET_BITRATE_MAX 206
#define K_SYSCALL_CAN_GET_BITRATE_MIN 207
#define K_SYSCALL_CAN_GET_CAPABILITIES 208
#define K_SYSCALL_CAN_GET_CORE_CLOCK 209
#define K_SYSCALL_CAN_GET_MAX_FILTERS 210
#define K_SYSCALL_CAN_GET_MODE 211
#define K_SYSCALL_CAN_GET_STATE 212
#define K_SYSCALL_CAN_GET_TIMING_DATA_MAX 213
#define K_SYSCALL_CAN_GET_TIMING_DATA_MIN 214
#define K_SYSCALL_CAN_GET_TIMING_MAX 215
#define K_SYSCALL_CAN_GET_TIMING_MIN 216
#define K_SYSCALL_CAN_GET_TRANSCEIVER 217
#define K_SYSCALL_CAN_RECOVER 218
#define K_SYSCALL_CAN_REMOVE_RX_FILTER 219
#define K_SYSCALL_CAN_SEND 220
#define K_SYSCALL_CAN_SET_BITRATE 221
#define K_SYSCALL_CAN_SET_BITRATE_DATA 222
#define K_SYSCALL_CAN_SET_MODE 223
#define K_SYSCALL_CAN_SET_TIMING 224
#define K_SYSCALL_CAN_SET_TIMING_DATA 225
#define K_SYSCALL_CAN_START 226
#define K_SYSCALL_CAN_STATS_GET_ACK_ERRORS 227
#define K_SYSCALL_CAN_STATS_GET_BIT0_ERRORS 228
#define K_SYSCALL_CAN_STATS_GET_BIT1_ERRORS 229
#define K_SYSCALL_CAN_STATS_GET_BIT_ERRORS 230
#define K_SYSCALL_CAN_STATS_GET_CRC_ERRORS 231
#define K_SYSCALL_CAN_STATS_GET_FORM_ERRORS 232
#define K_SYSCALL_CAN_STATS_GET_RX_OVERRUNS 233
#define K_SYSCALL_CAN_STATS_GET_STUFF_ERRORS 234
#define K_SYSCALL_CAN_STOP 235
#define K_SYSCALL_CHARGER_CHARGE_ENABLE 236
#define K_SYSCALL_CHARGER_GET_PROP 237
#define K_SYSCALL_CHARGER_SET_PROP 238
#define K_SYSCALL_COMPARATOR_GET_OUTPUT 239
#define K_SYSCALL_COMPARATOR_SET_TRIGGER 240
#define K_SYSCALL_COMPARATOR_TRIGGER_IS_PENDING 241
#define K_SYSCALL_COUNTER_CANCEL_CHANNEL_ALARM 242
#define K_SYSCALL_COUNTER_GET_FREQUENCY 243
#define K_SYSCALL_COUNTER_GET_GUARD_PERIOD 244
#define K_SYSCALL_COUNTER_GET_MAX_TOP_VALUE 245
#define K_SYSCALL_COUNTER_GET_NUM_OF_CHANNELS 246
#define K_SYSCALL_COUNTER_GET_PENDING_INT 247
#define K_SYSCALL_COUNTER_GET_TOP_VALUE 248
#define K_SYSCALL_COUNTER_GET_VALUE 249
#define K_SYSCALL_COUNTER_GET_VALUE_64 250
#define K_SYSCALL_COUNTER_IS_COUNTING_UP 251
#define K_SYSCALL_COUNTER_SET_CHANNEL_ALARM 252
#define K_SYSCALL_COUNTER_SET_GUARD_PERIOD 253
#define K_SYSCALL_COUNTER_SET_TOP_VALUE 254
#define K_SYSCALL_COUNTER_START 255
#define K_SYSCALL_COUNTER_STOP 256
#define K_SYSCALL_COUNTER_TICKS_TO_US 257
#define K_SYSCALL_COUNTER_US_TO_TICKS 258
#define K_SYSCALL_DAC_CHANNEL_SETUP 259
#define K_SYSCALL_DAC_WRITE_VALUE 260
#define K_SYSCALL_DEVMUX_SELECT_GET 261
#define K_SYSCALL_DEVMUX_SELECT_SET 262
#define K_SYSCALL_DMA_CHAN_FILTER 263
#define K_SYSCALL_DMA_RELEASE_CHANNEL 264
#define K_SYSCALL_DMA_REQUEST_CHANNEL 265
#define K_SYSCALL_DMA_RESUME 266
#define K_SYSCALL_DMA_START 267
#define K_SYSCALL_DMA_STOP 268
#define K_SYSCALL_DMA_SUSPEND 269
#define K_SYSCALL_EEPROM_GET_SIZE 270
#define K_SYSCALL_EEPROM_READ 271
#define K_SYSCALL_EEPROM_WRITE 272
#define K_SYSCALL_EMUL_FUEL_GAUGE_IS_BATTERY_CUTOFF 273
#define K_SYSCALL_EMUL_FUEL_GAUGE_SET_BATTERY_CHARGING 274
#define K_SYSCALL_ENTROPY_GET_ENTROPY 275
#define K_SYSCALL_ESPI_CONFIG 276
#define K_SYSCALL_ESPI_FLASH_ERASE 277
#define K_SYSCALL_ESPI_GET_CHANNEL_STATUS 278
#define K_SYSCALL_ESPI_READ_FLASH 279
#define K_SYSCALL_ESPI_READ_LPC_REQUEST 280
#define K_SYSCALL_ESPI_READ_REQUEST 281
#define K_SYSCALL_ESPI_RECEIVE_OOB 282
#define K_SYSCALL_ESPI_RECEIVE_VWIRE 283
#define K_SYSCALL_ESPI_SAF_ACTIVATE 284
#define K_SYSCALL_ESPI_SAF_CONFIG 285
#define K_SYSCALL_ESPI_SAF_FLASH_ERASE 286
#define K_SYSCALL_ESPI_SAF_FLASH_READ 287
#define K_SYSCALL_ESPI_SAF_FLASH_UNSUCCESS 288
#define K_SYSCALL_ESPI_SAF_FLASH_WRITE 289
#define K_SYSCALL_ESPI_SAF_GET_CHANNEL_STATUS 290
#define K_SYSCALL_ESPI_SAF_SET_PROTECTION_REGIONS 291
#define K_SYSCALL_ESPI_SEND_OOB 292
#define K_SYSCALL_ESPI_SEND_VWIRE 293
#define K_SYSCALL_ESPI_WRITE_FLASH 294
#define K_SYSCALL_ESPI_WRITE_LPC_REQUEST 295
#define K_SYSCALL_ESPI_WRITE_REQUEST 296
#define K_SYSCALL_FLASH_ERASE 297
#define K_SYSCALL_FLASH_EX_OP 298
#define K_SYSCALL_FLASH_FILL 299
#define K_SYSCALL_FLASH_FLATTEN 300
#define K_SYSCALL_FLASH_GET_PAGE_COUNT 301
#define K_SYSCALL_FLASH_GET_PAGE_INFO_BY_IDX 302
#define K_SYSCALL_FLASH_GET_PAGE_INFO_BY_OFFS 303
#define K_SYSCALL_FLASH_GET_PARAMETERS 304
#define K_SYSCALL_FLASH_GET_WRITE_BLOCK_SIZE 305
#define K_SYSCALL_FLASH_READ 306
#define K_SYSCALL_FLASH_READ_JEDEC_ID 307
#define K_SYSCALL_FLASH_SFDP_READ 308
#define K_SYSCALL_FLASH_SIMULATOR_GET_MEMORY 309
#define K_SYSCALL_FLASH_WRITE 310
#define K_SYSCALL_FUEL_GAUGE_BATTERY_CUTOFF 311
#define K_SYSCALL_FUEL_GAUGE_GET_BUFFER_PROP 312
#define K_SYSCALL_FUEL_GAUGE_GET_PROP 313
#define K_SYSCALL_FUEL_GAUGE_GET_PROPS 314
#define K_SYSCALL_FUEL_GAUGE_SET_PROP 315
#define K_SYSCALL_FUEL_GAUGE_SET_PROPS 316
#define K_SYSCALL_GNSS_GET_ENABLED_SYSTEMS 317
#define K_SYSCALL_GNSS_GET_FIX_RATE 318
#define K_SYSCALL_GNSS_GET_NAVIGATION_MODE 319
#define K_SYSCALL_GNSS_GET_SUPPORTED_SYSTEMS 320
#define K_SYSCALL_GNSS_SET_ENABLED_SYSTEMS 321
#define K_SYSCALL_GNSS_SET_FIX_RATE 322
#define K_SYSCALL_GNSS_SET_NAVIGATION_MODE 323
#define K_SYSCALL_HAPTICS_START_OUTPUT 324
#define K_SYSCALL_HAPTICS_STOP_OUTPUT 325
#define K_SYSCALL_HWINFO_CLEAR_RESET_CAUSE 326
#define K_SYSCALL_HWINFO_GET_DEVICE_EUI64 327
#define K_SYSCALL_HWINFO_GET_DEVICE_ID 328
#define K_SYSCALL_HWINFO_GET_RESET_CAUSE 329
#define K_SYSCALL_HWINFO_GET_SUPPORTED_RESET_CAUSE 330
#define K_SYSCALL_HWSPINLOCK_GET_MAX_ID 331
#define K_SYSCALL_HWSPINLOCK_LOCK 332
#define K_SYSCALL_HWSPINLOCK_TRYLOCK 333
#define K_SYSCALL_HWSPINLOCK_UNLOCK 334
#define K_SYSCALL_I2S_BUF_READ 335
#define K_SYSCALL_I2S_BUF_WRITE 336
#define K_SYSCALL_I2S_CONFIGURE 337
#define K_SYSCALL_I2S_TRIGGER 338
#define K_SYSCALL_I3C_DO_CCC 339
#define K_SYSCALL_I3C_TRANSFER 340
#define K_SYSCALL_IPM_COMPLETE 341
#define K_SYSCALL_IPM_MAX_DATA_SIZE_GET 342
#define K_SYSCALL_IPM_MAX_ID_VAL_GET 343
#define K_SYSCALL_IPM_SEND 344
#define K_SYSCALL_IPM_SET_ENABLED 345
#define K_SYSCALL_IVSHMEM_ENABLE_INTERRUPTS 346
#define K_SYSCALL_IVSHMEM_GET_ID 347
#define K_SYSCALL_IVSHMEM_GET_MAX_PEERS 348
#define K_SYSCALL_IVSHMEM_GET_MEM 349
#define K_SYSCALL_IVSHMEM_GET_OUTPUT_MEM_SECTION 350
#define K_SYSCALL_IVSHMEM_GET_PROTOCOL 351
#define K_SYSCALL_IVSHMEM_GET_RW_MEM_SECTION 352
#define K_SYSCALL_IVSHMEM_GET_STATE 353
#define K_SYSCALL_IVSHMEM_GET_VECTORS 354
#define K_SYSCALL_IVSHMEM_INT_PEER 355
#define K_SYSCALL_IVSHMEM_REGISTER_HANDLER 356
#define K_SYSCALL_IVSHMEM_SET_STATE 357
#define K_SYSCALL_KSCAN_CONFIG 358
#define K_SYSCALL_KSCAN_DISABLE_CALLBACK 359
#define K_SYSCALL_KSCAN_ENABLE_CALLBACK 360
#define K_SYSCALL_K_MEM_PAGING_HISTOGRAM_BACKING_STORE_PAGE_IN_GET 361
#define K_SYSCALL_K_MEM_PAGING_HISTOGRAM_BACKING_STORE_PAGE_OUT_GET 362
#define K_SYSCALL_K_MEM_PAGING_HISTOGRAM_EVICTION_GET 363
#define K_SYSCALL_K_MEM_PAGING_STATS_GET 364
#define K_SYSCALL_K_MEM_PAGING_THREAD_STATS_GET 365
#define K_SYSCALL_LED_BLINK 366
#define K_SYSCALL_LED_GET_INFO 367
#define K_SYSCALL_LED_OFF 368
#define K_SYSCALL_LED_ON 369
#define K_SYSCALL_LED_SET_BRIGHTNESS 370
#define K_SYSCALL_LED_SET_CHANNEL 371
#define K_SYSCALL_LED_SET_COLOR 372
#define K_SYSCALL_LED_WRITE_CHANNELS 373
#define K_SYSCALL_LLEXT_GET_FN_TABLE 374
#define K_SYSCALL_MAXIM_DS3231_GET_SYNCPOINT 375
#define K_SYSCALL_MAXIM_DS3231_REQ_SYNCPOINT 376
#define K_SYSCALL_MBOX_MAX_CHANNELS_GET 377
#define K_SYSCALL_MBOX_MTU_GET 378
#define K_SYSCALL_MBOX_SEND 379
#define K_SYSCALL_MBOX_SET_ENABLED 380
#define K_SYSCALL_MDIO_BUS_DISABLE 381
#define K_SYSCALL_MDIO_BUS_ENABLE 382
#define K_SYSCALL_MDIO_READ 383
#define K_SYSCALL_MDIO_READ_C45 384
#define K_SYSCALL_MDIO_WRITE 385
#define K_SYSCALL_MDIO_WRITE_C45 386
#define K_SYSCALL_MSPI_CONFIG 387
#define K_SYSCALL_MSPI_DEV_CONFIG 388
#define K_SYSCALL_MSPI_GET_CHANNEL_STATUS 389
#define K_SYSCALL_MSPI_SCRAMBLE_CONFIG 390
#define K_SYSCALL_MSPI_TIMING_CONFIG 391
#define K_SYSCALL_MSPI_TRANSCEIVE 392
#define K_SYSCALL_MSPI_XIP_CONFIG 393
#define K_SYSCALL_NET_ADDR_NTOP 394
#define K_SYSCALL_NET_ADDR_PTON 395
#define K_SYSCALL_NET_ETH_GET_PTP_CLOCK_BY_INDEX 396
#define K_SYSCALL_NET_IF_GET_BY_INDEX 397
#define K_SYSCALL_NET_IF_IPV4_ADDR_ADD_BY_INDEX 398
#define K_SYSCALL_NET_IF_IPV4_ADDR_LOOKUP_BY_INDEX 399
#define K_SYSCALL_NET_IF_IPV4_ADDR_RM_BY_INDEX 400
#define K_SYSCALL_NET_IF_IPV4_SET_GW_BY_INDEX 401
#define K_SYSCALL_NET_IF_IPV4_SET_NETMASK_BY_ADDR_BY_INDEX 402
#define K_SYSCALL_NET_IF_IPV4_SET_NETMASK_BY_INDEX 403
#define K_SYSCALL_NET_IF_IPV6_ADDR_ADD_BY_INDEX 404
#define K_SYSCALL_NET_IF_IPV6_ADDR_LOOKUP_BY_INDEX 405
#define K_SYSCALL_NET_IF_IPV6_ADDR_RM_BY_INDEX 406
#define K_SYSCALL_NET_SOCKET_SERVICE_REGISTER 407
#define K_SYSCALL_NRF_QSPI_NOR_XIP_ENABLE 408
#define K_SYSCALL_PECI_CONFIG 409
#define K_SYSCALL_PECI_DISABLE 410
#define K_SYSCALL_PECI_ENABLE 411
#define K_SYSCALL_PECI_TRANSFER 412
#define K_SYSCALL_PS2_CONFIG 413
#define K_SYSCALL_PS2_DISABLE_CALLBACK 414
#define K_SYSCALL_PS2_ENABLE_CALLBACK 415
#define K_SYSCALL_PS2_READ 416
#define K_SYSCALL_PS2_WRITE 417
#define K_SYSCALL_PTP_CLOCK_GET 418
#define K_SYSCALL_PWM_CAPTURE_CYCLES 419
#define K_SYSCALL_PWM_DISABLE_CAPTURE 420
#define K_SYSCALL_PWM_ENABLE_CAPTURE 421
#define K_SYSCALL_PWM_GET_CYCLES_PER_SEC 422
#define K_SYSCALL_PWM_SET_CYCLES 423
#define K_SYSCALL_RESET_LINE_ASSERT 424
#define K_SYSCALL_RESET_LINE_DEASSERT 425
#define K_SYSCALL_RESET_LINE_TOGGLE 426
#define K_SYSCALL_RESET_STATUS 427
#define K_SYSCALL_RETAINED_MEM_CLEAR 428
#define K_SYSCALL_RETAINED_MEM_READ 429
#define K_SYSCALL_RETAINED_MEM_SIZE 430
#define K_SYSCALL_RETAINED_MEM_WRITE 431
#define K_SYSCALL_RTC_ALARM_GET_SUPPORTED_FIELDS 432
#define K_SYSCALL_RTC_ALARM_GET_TIME 433
#define K_SYSCALL_RTC_ALARM_IS_PENDING 434
#define K_SYSCALL_RTC_ALARM_SET_CALLBACK 435
#define K_SYSCALL_RTC_ALARM_SET_TIME 436
#define K_SYSCALL_RTC_GET_CALIBRATION 437
#define K_SYSCALL_RTC_GET_TIME 438
#define K_SYSCALL_RTC_SET_CALIBRATION 439
#define K_SYSCALL_RTC_SET_TIME 440
#define K_SYSCALL_RTC_UPDATE_SET_CALLBACK 441
#define K_SYSCALL_RTIO_CQE_COPY_OUT 442
#define K_SYSCALL_RTIO_CQE_GET_MEMPOOL_BUFFER 443
#define K_SYSCALL_RTIO_RELEASE_BUFFER 444
#define K_SYSCALL_RTIO_SQE_CANCEL 445
#define K_SYSCALL_RTIO_SQE_COPY_IN_GET_HANDLES 446
#define K_SYSCALL_RTIO_SUBMIT 447
#define K_SYSCALL_SDHC_CARD_BUSY 448
#define K_SYSCALL_SDHC_CARD_PRESENT 449
#define K_SYSCALL_SDHC_DISABLE_INTERRUPT 450
#define K_SYSCALL_SDHC_ENABLE_INTERRUPT 451
#define K_SYSCALL_SDHC_EXECUTE_TUNING 452
#define K_SYSCALL_SDHC_GET_HOST_PROPS 453
#define K_SYSCALL_SDHC_HW_RESET 454
#define K_SYSCALL_SDHC_REQUEST 455
#define K_SYSCALL_SDHC_SET_IO 456
#define K_SYSCALL_SIP_SUPERVISORY_CALL 457
#define K_SYSCALL_SIP_SVC_PLAT_ASYNC_RES_REQ 458
#define K_SYSCALL_SIP_SVC_PLAT_ASYNC_RES_RES 459
#define K_SYSCALL_SIP_SVC_PLAT_FORMAT_TRANS_ID 460
#define K_SYSCALL_SIP_SVC_PLAT_FREE_ASYNC_MEMORY 461
#define K_SYSCALL_SIP_SVC_PLAT_FUNC_ID_VALID 462
#define K_SYSCALL_SIP_SVC_PLAT_GET_ERROR_CODE 463
#define K_SYSCALL_SIP_SVC_PLAT_GET_TRANS_IDX 464
#define K_SYSCALL_SIP_SVC_PLAT_UPDATE_TRANS_ID 465
#define K_SYSCALL_SMBUS_BLOCK_PCALL 466
#define K_SYSCALL_SMBUS_BLOCK_READ 467
#define K_SYSCALL_SMBUS_BLOCK_WRITE 468
#define K_SYSCALL_SMBUS_BYTE_DATA_READ 469
#define K_SYSCALL_SMBUS_BYTE_DATA_WRITE 470
#define K_SYSCALL_SMBUS_BYTE_READ 471
#define K_SYSCALL_SMBUS_BYTE_WRITE 472
#define K_SYSCALL_SMBUS_CONFIGURE 473
#define K_SYSCALL_SMBUS_GET_CONFIG 474
#define K_SYSCALL_SMBUS_HOST_NOTIFY_REMOVE_CB 475
#define K_SYSCALL_SMBUS_PCALL 476
#define K_SYSCALL_SMBUS_QUICK 477
#define K_SYSCALL_SMBUS_SMBALERT_REMOVE_CB 478
#define K_SYSCALL_SMBUS_WORD_DATA_READ 479
#define K_SYSCALL_SMBUS_WORD_DATA_WRITE 480
#define K_SYSCALL_SPI_RELEASE 481
#define K_SYSCALL_SPI_TRANSCEIVE 482
#define K_SYSCALL_STEPPER_ENABLE 483
#define K_SYSCALL_STEPPER_ENABLE_CONSTANT_VELOCITY_MODE 484
#define K_SYSCALL_STEPPER_GET_ACTUAL_POSITION 485
#define K_SYSCALL_STEPPER_GET_MICRO_STEP_RES 486
#define K_SYSCALL_STEPPER_IS_MOVING 487
#define K_SYSCALL_STEPPER_MOVE 488
#define K_SYSCALL_STEPPER_SET_ACTUAL_POSITION 489
#define K_SYSCALL_STEPPER_SET_MAX_VELOCITY 490
#define K_SYSCALL_STEPPER_SET_MICRO_STEP_RES 491
#define K_SYSCALL_STEPPER_SET_TARGET_POSITION 492
#define K_SYSCALL_SYSCON_GET_BASE 493
#define K_SYSCALL_SYSCON_GET_SIZE 494
#define K_SYSCALL_SYSCON_READ_REG 495
#define K_SYSCALL_SYSCON_WRITE_REG 496
#define K_SYSCALL_SYS_CACHE_DATA_FLUSH_AND_INVD_RANGE 497
#define K_SYSCALL_SYS_CACHE_DATA_FLUSH_RANGE 498
#define K_SYSCALL_SYS_CACHE_DATA_INVD_RANGE 499
#define K_SYSCALL_SYS_CSRAND_GET 500
#define K_SYSCALL_SYS_RAND_GET 501
#define K_SYSCALL_TEE_CANCEL 502
#define K_SYSCALL_TEE_CLOSE_SESSION 503
#define K_SYSCALL_TEE_GET_VERSION 504
#define K_SYSCALL_TEE_INVOKE_FUNC 505
#define K_SYSCALL_TEE_OPEN_SESSION 506
#define K_SYSCALL_TEE_SHM_ALLOC 507
#define K_SYSCALL_TEE_SHM_FREE 508
#define K_SYSCALL_TEE_SHM_REGISTER 509
#define K_SYSCALL_TEE_SHM_UNREGISTER 510
#define K_SYSCALL_TEE_SUPPL_RECV 511
#define K_SYSCALL_TEE_SUPPL_SEND 512
#define K_SYSCALL_TGPIO_PIN_CONFIG_EXT_TIMESTAMP 513
#define K_SYSCALL_TGPIO_PIN_DISABLE 514
#define K_SYSCALL_TGPIO_PIN_PERIODIC_OUTPUT 515
#define K_SYSCALL_TGPIO_PIN_READ_TS_EC 516
#define K_SYSCALL_TGPIO_PORT_GET_CYCLES_PER_SECOND 517
#define K_SYSCALL_TGPIO_PORT_GET_TIME 518
#define K_SYSCALL_UPDATEHUB_AUTOHANDLER 519
#define K_SYSCALL_UPDATEHUB_CONFIRM 520
#define K_SYSCALL_UPDATEHUB_PROBE 521
#define K_SYSCALL_UPDATEHUB_REBOOT 522
#define K_SYSCALL_UPDATEHUB_UPDATE 523
#define K_SYSCALL_USER_FAULT 524
#define K_SYSCALL_W1_CHANGE_BUS_LOCK 525
#define K_SYSCALL_W1_CONFIGURE 526
#define K_SYSCALL_W1_GET_SLAVE_COUNT 527
#define K_SYSCALL_W1_READ_BIT 528
#define K_SYSCALL_W1_READ_BLOCK 529
#define K_SYSCALL_W1_READ_BYTE 530
#define K_SYSCALL_W1_RESET_BUS 531
#define K_SYSCALL_W1_SEARCH_BUS 532
#define K_SYSCALL_W1_WRITE_BIT 533
#define K_SYSCALL_W1_WRITE_BLOCK 534
#define K_SYSCALL_W1_WRITE_BYTE 535
#define K_SYSCALL_WDT_DISABLE 536
#define K_SYSCALL_WDT_FEED 537
#define K_SYSCALL_WDT_SETUP 538
#define K_SYSCALL_XTENSA_USER_FAULT 539
#define K_SYSCALL_ZSOCK_ACCEPT 540
#define K_SYSCALL_ZSOCK_BIND 541
#define K_SYSCALL_ZSOCK_CLOSE 542
#define K_SYSCALL_ZSOCK_CONNECT 543
#define K_SYSCALL_ZSOCK_FCNTL_IMPL 544
#define K_SYSCALL_ZSOCK_GETHOSTNAME 545
#define K_SYSCALL_ZSOCK_GETPEERNAME 546
#define K_SYSCALL_ZSOCK_GETSOCKNAME 547
#define K_SYSCALL_ZSOCK_GETSOCKOPT 548
#define K_SYSCALL_ZSOCK_GET_CONTEXT_OBJECT 549
#define K_SYSCALL_ZSOCK_INET_PTON 550
#define K_SYSCALL_ZSOCK_IOCTL_IMPL 551
#define K_SYSCALL_ZSOCK_LISTEN 552
#define K_SYSCALL_ZSOCK_POLL 553
#define K_SYSCALL_ZSOCK_RECVFROM 554
#define K_SYSCALL_ZSOCK_RECVMSG 555
#define K_SYSCALL_ZSOCK_SELECT 556
#define K_SYSCALL_ZSOCK_SENDMSG 557
#define K_SYSCALL_ZSOCK_SENDTO 558
#define K_SYSCALL_ZSOCK_SETSOCKOPT 559
#define K_SYSCALL_ZSOCK_SHUTDOWN 560
#define K_SYSCALL_ZSOCK_SOCKET 561
#define K_SYSCALL_ZSOCK_SOCKETPAIR 562
#define K_SYSCALL_Z_ERRNO 563
#define K_SYSCALL_Z_ZSOCK_GETADDRINFO_INTERNAL 564


#ifndef _ASMLANGUAGE

#include <stdarg.h>
#include <stdint.h>

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_SYSCALL_LIST_H */

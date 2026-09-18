set(BLE_DIR ble)
file(GLOB_RECURSE BLE_C_SOURCES ${BLE_DIR}/*.c)

set(USB_DIR usb)
file(GLOB_RECURSE USB_C_SOURCES ${USB_DIR}/*.c)

set(CHAIN_BUS_DIR chain_bus)
file(GLOB_RECURSE CHAIN_BUS_C_SOURCES ${CHAIN_BUS_DIR}/*.cpp)

set(WEB_DIR web)
file(GLOB_RECURSE WEB_IMG_C_SOURCES ${WEB_DIR}/*.jpg ${WEB_DIR}/*.png)

idf_component_register(SRCS "main.cpp" "btn_progress.c" "settings.c" "dual_button.c" "adc_detect.c" "diag_power.c" "web/web_assets.c" ${BLE_C_SOURCES} ${USB_C_SOURCES} ${CHAIN_BUS_C_SOURCES}
                       INCLUDE_DIRS "include" "web" ${RM_DIR} ${BLE_DIR} ${USB_DIR} ${CHAIN_BUS_DIR}
                       EMBED_FILES web/index.html web/styles.css web/script.js web/favicon.ico ${WEB_IMG_C_SOURCES})

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)

idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
cmake_policy(SET CMP0079 NEW)
target_link_libraries(${tusb_lib} PRIVATE ${COMPONENT_LIB})

# ── 【2026-09-16 本轮核心修复】别让 --gc-sections 把 USJ 连接监视器删掉 ──────────
#
#   症状：BLE 档（light sleep 生效）静置后按键延迟明显；而"插着 USB"时完全正常。
#
#   根因链（全部有 map 实证，见 REFERENCE §N）：
#     ① IDF 的 `CONFIG_USJ_NO_AUTO_LS_ON_CONNECTION=y` 靠
#        `usb_serial_jtag_connection_monitor.c` 里的 tick hook 监视 USJ 是否连着主机，
#        连着就持一把 ESP_PM_NO_LIGHT_SLEEP 锁 —— 这正是"插线时延迟正常"的原因。
#     ② 该 obj 在 IDF 里的唯一入口是一个 **不定义任何符号** 的空钩子函数
#        `usb_serial_jtag_connection_monitor_include()`，它靠 CMakeLists 里的
#        `-u usb_serial_jtag_connection_monitor_include` 强制拉活（NDEBUG 下
#        `ESP_SYSTEM_INIT_FN` 会把注册结构标成 `unused`，没有 -u 就会被 GC）。
#     ③ **`-u` 只对"命令行上直接列出的库"生效**，而本工程把该库放在了
#        `INTERFACE` 链接里 ⇒ 它出现在命令行上时会在 `--start-group` 之后、且被
#        `--whole-archive` 处理过 ⇒ `-u` 的未定义符号需求没能在"该库被扫描之前"
#        进入未决符号表 ⇒ 整个 obj（含 .text/.bss/.esp_system_init_fn.*）被 GC。
#        map 的 "Discarded input sections" 里明写着：
#          .text  0x0 0x0 ...libesp_driver_usb_serial_jtag.a(usb_serial_jtag_connection_monitor.c.obj)
#          .bss   0x0 0x0 ...(同上)          <- s_usb_serial_jtag_conn_status / pm_lock 都没了
#     ④ 结果：那把 NO_LIGHT_SLEEP 锁**从来不存在** ⇒ 运行期实际进不进 light sleep
#        与"有没有插线"无关地由 BLE 活动决定 —— 用户观察到的"插线就不卡"其实是
#        ADB/USB 总线活动带来的副作用，而不是那把锁在起作用。
#
#   修法：用 `--whole-archive` 把该静态库整个拉进链接（只会把该库 3 个 obj 都编进去，
#   体积代价约 1KB 级，换来 tick hook 与 PM 锁恢复）。这比"在应用里调用
#   usb_serial_jtag_is_connected() 当锚点"更稳 —— 后者依赖"USJ 驱动已安装"，
#   而本工程从不调用 usb_serial_jtag_driver_install()，行为不可控。
#
#   ⚠️ 该库在 `idf_component_register(...)` 之后才被 IDF 组件管理器对主组件可见，
#   所以获取组件属性必须放在注册语句之后（同上 `espressif__tinyusb` 的用法）。
#
#   ⚠️ 2026-09-18 更正：这里必须传**组件名** `esp_driver_usb_serial_jtag`，不能传
#   链接目标名 `__idf_esp_driver_usb_serial_jtag`。IDF 的 `__component_get_target()`
#   只认两种写法：① 别名（`idf::<name>`，内部拼成 3 个下划线）；② 与
#   `COMPONENT_NAME` 属性逐字相等的字符串（即裸组件名）。传目标名会让两种匹配
#   全部落空 ⇒ `Failed to resolve component '...'` + CMake 配置期直接 FATAL。
idf_component_get_property(usj_lib esp_driver_usb_serial_jtag COMPONENT_LIB)
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,--whole-archive" "${usj_lib}" "-Wl,--no-whole-archive")

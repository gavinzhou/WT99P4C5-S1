# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

> **重要**：本仓库的父目录有顶层 `../CLAUDE.md`（HyperFi 项目通用规则——通信语言、命名规范、ESP32-C5 lessons learned、设备端口）。**先读父目录 CLAUDE.md** 再读本文。本文件只补充 firmware 仓库特有的内容、不重复父规则。

## What this repo is

`WT99P4C5-S1/` 是 **HyperFi product board 的 ESP-IDF 工程**——从 `wireless-tag-com/WT99P4C5-S1` fork 出来、加 HyperFi 算法 + 上行链路。本工程的目标硬件：

- **Host**: ESP32-P4 (双核 RISC-V、PSRAM 32MB) — 跑 pipeline / fall detection / MQTT / HTTPS uploader
- **Slave**: ESP32-C5 (WiFi 6 / 5GHz CSI capable) — 通过 SDIO 跟 P4 通信、esp-hosted-mcu vendored 在 `../hyperfi/firmware/esp-hosted-mcu-vendored/`
- **C5 firmware** 不在本工程编译：是预编译 `.bin` 嵌入到 P4 镜像（`main/c5_fw.bin`、`EMBED_FILES` in `main/CMakeLists.txt`），P4 boot 时通过 UART 烧给 C5（`c5_flasher.c`）

⚠️ **不要把这个 fork 与 `../hyperfi/firmware/c5_*` 搞混**：那边是早期 M1/M2 单板 dev firmware（深圳测试用），这里是 M3+ product board firmware（MFR pilot 用）。

## Common commands

ESP-IDF v5.5+（项目用 `v5.5-beta1-204` patched，详见父 CLAUDE.md 的 ESP32-C5 lessons）。所有命令在仓库根跑。

### Build / Flash / Monitor

```bash
# 全量构建
idf.py build

# 烧录 + 监控（端口名可能是 /dev/cu.usbmodem101 或 1101——看 ls /dev/cu.usbmodem* 实际值）
idf.py -p /dev/cu.usbmodem1101 flash monitor

# 只烧不监控（快）
idf.py -p /dev/cu.usbmodem1101 flash

# 退出 monitor: Ctrl+]
```

### NVS 擦除（WiFi protocol 改了之后必做、详见父 CLAUDE.md）

```bash
# C5 不支持 erase_flash、必须 erase_region
esptool.py --chip esp32p4 -p /dev/cu.usbmodem1101 erase_region 0x9000 0x6000
```

### Firmware self-test 模式

每次 boot 时自动跑：`mXX_test.c` 在 `app_main` 早期调用、PASS 才继续主循环。**不需要手动触发**——看 monitor 输出 `=== MX.X Self-Test: ALL PASS` 即可。Fixtures (`mXX_fixtures.h`) 由 `../hyperfi/tests/gen_*_fixtures.py` 生成。

### 改 protobuf

```bash
# nanopb 0.4.8 vendored runtime 在 main/（pb_common.c, pb_encode.c, pb_decode.c）
cd main
python3 ~/nanopb-vendor/generator/nanopb_generator.py proto/hyperfi/csi/telemetry.proto
# 同时也要在 ../hyperfi/ 重新生成 Python protobuf（详见那边的 CLAUDE.md）
```

## Architecture

### 启动顺序（main.cpp）

```
shutter_test → m32_test → m33_test → m34_test → m35_proto_test → m36_test
  ↓ self-test ALL PASS
shutter / collapse / pipeline / fall_detector init
  ↓
c5_flasher: P4 通过 UART 烧 C5 firmware（嵌入的 c5_fw.bin、~13s）
  ↓
esp_hosted_connect_to_slave: P4↔C5 SDIO handshake + WiFi stack init
  ↓
ETH static IP → mqtt_publisher → event_uploader → event_orchestrator
  ↓
WiFi STA: connect to AP (CSI source) → CSI 流 ON
  ↓
1Hz pipeline: ev_buf push → window emit → telemetry/alert publish
```

### Pipeline 数据流（一个 fall event 的完整链路）

```
CSI frame (~300 fps)
   ↓ pipeline_on_csi_frame
   ├→ event_buffer push (rolling 60s × 200fps)
   └→ shutter (currently bypassed) → mean amp
        ↓ 1Hz emit_window
        ├→ event_buffer push window
        ├→ collapse FSM → fall_detector → telemetry MQTT
        └→ if fall_event_rising_edge:
             event_orchestrator_handle()
               ↓ atomic CAS in_flight (single-event-in-flight)
               ↓ event_buffer_request_snapshot(±30s, on_ready)
               ↓ [wait 30s post-window]
               ↓ on_snapshot_ready: encode 1.6-2.0 MB EventRawContext
                  via proto_codec_encode_event_raw_context (pb_callback streaming)
               ↓ event_uploader_submit
                  ├→ MQTT pub raw/request → wait raw/upload_url (presigned)
                  └→ HTTPS PUT to mock S3 → MQTT pub raw/done
               ↓ on_upload_done: heap_caps_free PSRAM blob, IDLE
```

详细架构见 `../hyperfi/docs/adr/ADR-023-m3-e2e-pipeline.md`（**必读**、~700 行、含 errata 记录所有实测发现）。

### `main/` 关键文件分组

| 分组 | 文件 |
|---|---|
| 入口 | `main.cpp`, `CMakeLists.txt` |
| Pipeline 核心 | `pipeline.{c,h}`, `shutter.{c,h}`, `collapse.{c,h}`, `metrics.{c,h}`, `fall_detector.{c,h}` |
| Self-test (boot 时跑) | `m32_test.c`, `m33_test.c`, `m34_test.c`, `m35_proto_test.c`, `m36_test.c` + `mXX_fixtures.h` |
| Self-test fixtures | `m32_fixtures.h`, `m33_fixtures.h`, `m34_fixtures.h` (来自 `../hyperfi/tests/gen_*.py`) |
| Networking 上行 | `mqtt_publisher.{c,h}`, `event_uploader.{c,h}`, `event_orchestrator.{c,h}` |
| Event 数据流 | `event_buffer.{c,h}` (60s ring), `proto_codec.{c,h}` (streaming encode) |
| C5 boot/flash | `c5_flasher.{c,h}`, `c5_fw.bin` (embedded) |
| Protobuf | `proto/hyperfi/csi/telemetry.{proto,pb.c,pb.h,options}`, `pb_*.c` (nanopb runtime) |
| Load test | `load_test.{c,h}` (HYPERFI_LOAD_TEST_ENABLED=0 默认 disable、改 1 跑 1Mbps stress harness) |

### Components 

`components/wt99p4c5_s1_board/` 是板级 BSP（ETH init、PSRAM config——Waveshare 板配套）。`components/bsp_extra/` 板外设额外驱动。**这两个 component 是 fork 上游内容、不要随便改**。HyperFi 的所有改动应该都在 `main/` 里。

## Things to know

- **顶层 ESP-IDF 不在 git**：`~/.espressif/esp-idf-v55b1/` 不应该 commit、即使本工程是 IDF 工程
- **load_test 默认 disabled**（2026-04-28 起、`main.cpp` `HYPERFI_LOAD_TEST_ENABLED=0`）—跑 1 Mbps stress 时手动 toggle 1
- **collapse threshold 当前是 hack 值**（pipeline.c 显式覆盖 default 0.12 → 0.040）—— 这是 M3.6.4 E2E 测试用、TD-002 in `../hyperfi/docs/tech-debt.md` 追踪、**MFR pilot 前必须 restore**
- **shutter 默认 bypass**（`pipeline.h` `bypass_shutter=true`）—Spatial Shutter (P2 patent) 实现完成但 CFO correction 没做、当前跑 raw `|H|`
- **C5 PHY register patch**：5GHz CSI 需要 `~/.espressif/esp-idf-v55b1/components/esp_wifi/include/esp_wifi_he_types.h` 加 `lltf_bit_mode` 字段（详见父 CLAUDE.md "ESP32-C5 CSI 关键经验"）
- **Build 卡在 `Linking C static library libmain.a`**：第一次 link 因为新加 nanopb + event_buffer + uploader + orchestrator 一堆对象、要 30-60s。第二次开始很快
- **flash 失败 `Could not open /dev/cu.usbmodem101`**：端口名可能是 `1101`、用 `ls /dev/cu.usbmodem*` 看实际值
- **idf.py set-target 需要 `--preview` 给 C5**：但本工程目标是 P4，不需要这个 flag。详见父 CLAUDE.md
- **Branch 是 `hyperfi-m2`**：fork 在这个 branch 上演进、不在 `main`。Tag 用 `hyperfi-m<N.n>-<desc>` 前缀（不是 `v-m...`）

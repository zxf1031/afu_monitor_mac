# -*- coding: utf-8 -*-
"""AFU 穿戴设备 BLE 协议串口监控工具（macOS）

左窗格: 串口全量原始日志(每行带 PC 时间戳)
右窗格: 对 "tx raw / rx raw" 行做 AFU 公共帧解析, 翻译成中文 + 协议字段

这是 Windows 版 tools/afu_monitor.py 的独立副本，协议解析相同。
macOS 上串口是 /dev/cu.*，打开期间用 caffeinate 防止睡眠。

用法:
    python3 afu_monitor.py [--baud 2000000]
    python3 afu_monitor.py --port /dev/cu.usbserial-XXXX --baud 2000000
    python3 afu_monitor.py --file fw_log.txt     # 离线回放已保存的日志
"""
import argparse
import ctypes
import datetime
import os
import queue
import re
import subprocess
import sys
import tempfile
import threading
import time
import traceback
import zlib
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

# ---------------------------------------------------------------------------
# 协议常量表
# ---------------------------------------------------------------------------

DOMAINS = {
    0x01: "SYSTEM(系统)", 0x10: "OFFLINE(离线同步)", 0x20: "MEASUREMENT(主动测量)",
    0x30: "DEBUG_RAW_PPG(内部)", 0x40: "DEBUG(内部)", 0x50: "DEVICE_CONTROL(设备控制)",
    0x60: "FILE(内部)", 0x70: "EXERCISE(运动)", 0xF0: "FACTORY(厂测)",
}
# 第10/11/13节：已占用但暂不公布 opcode。消费者固件不得实现，capability 位必须为 0。
UNPUBLISHED_DOMAINS = {
    0x30: ("DEBUG_RAW_PPG", "第10节"),
    0x40: ("DEBUG", "第11节"),
    0x60: ("FILE", "第13节"),
}

OPCODES = {
    0x01: {0x01: "GET_CAPABILITIES", 0x02: "GET_DEVICE_SIGNATURE", 0x03: "GET_BATTERY",
           0x04: "TIME_SYNC", 0x05: "GET_DEVICE_INFO", 0x06: "GET_USER_PROFILE",
           0x07: "SET_USER_PROFILE", 0x10: "AUTH_GET_CHALLENGE", 0x11: "AUTH_SET_KEY",
           0x12: "AUTH_PROVE", 0x20: "USER_RESET", 0x21: "SOFTWARE_REBOOT",
           0x40: "BATTERY_STATE_EVENT", 0x41: "LOG_EVENT"},
    0x10: {0x01: "OFFLINE_OPEN", 0x02: "OFFLINE_PULL", 0x03: "OFFLINE_CLOSE",
           0x40: "OFFLINE_DATA"},
    0x20: {0x01: "START", 0x02: "STOP", 0x03: "GET_STATE",
           0x04: "GET_MEASUREMENT_PERIOD", 0x05: "SET_MEASUREMENT_PERIOD",
           0x40: "DATA"},
    0x50: {0x01: "GET_CASE_BATTERY", 0x02: "FIND_START", 0x03: "FIND_STOP",
           0x04: "GET_BLE_NAME", 0x05: "SET_BLE_NAME", 0x06: "GET_POWER_MODE",
           0x07: "SET_POWER_MODE", 0x08: "GET_WEAR_STATE", 0x09: "GET_IBEACON_CONFIG",
           0x0A: "SET_IBEACON_CONFIG", 0x0B: "GET_SLEEP_STATE",
           0x40: "CASE_BATTERY_EVENT", 0x41: "WEAR_STATE_EVENT",
           0x42: "FIND_END_EVENT", 0x43: "SLEEP_STATE_EVENT"},
    0x70: {0x01: "GET_CAPABILITIES", 0x02: "GET_STATE", 0x03: "GET_RECOGNITION_CONFIG",
           0x04: "SET_RECOGNITION_CONFIG", 0x05: "CONFIG_REALTIME_EXTENSION",
           0x06: "GET_REALTIME_EXTENSION_CAPABILITIES", 0x10: "START", 0x11: "PAUSE",
           0x12: "RESUME", 0x13: "STOP", 0x14: "UPDATE_APP_DISTANCE",
           0x15: "GET_APP_DISTANCE_STATE", 0x16: "CONFIRM_STATISTICS_START",
           0x40: "STATE_CHANGED", 0x41: "REALTIME_DATA", 0x42: "AUTO_EXERCISE_SAVED",
           0x43: "REALTIME_EXTENSION"},
}

# 协议第 5.2 节：值 → 名称 / 使用场景（原文，不自行改写）
STATUS = {
    0: "OK", 1: "INVALID_FRAME", 2: "INVALID_PARAMETER", 3: "NOT_SUPPORTED",
    4: "NOT_READY", 5: "BUSY", 6: "INVALID_CURSOR", 7: "GAP",
    8: "MTU_TOO_SMALL", 9: "FORBIDDEN", 10: "NO_DATA", 11: "INTERNAL_ERROR",
    12: "DURATION_TOO_LONG", 13: "DISABLED", 14: "NOT_WORN",
    15: "CHECKSUM_MISMATCH", 16: "NO_SPACE", 17: "IDEMPOTENCY_CAPACITY_FULL",
}
STATUS_USAGE = {
    0: "成功/已受理",
    1: "帧头、flags 或长度错误",
    2: "参数非法",
    3: "固件未实现该能力",
    4: "能力存在，依赖/状态未就绪",
    5: "临时资源竞争",
    6: "cursor/session/offset 无效",
    7: "历史数据已缺失，同时给出恢复 cursor",
    8: "MTU 不足",
    9: "安全、权限或当前状态禁止",
    10: "当前没有数据",
    11: "内部失败",
    12: "时长超限",
    13: "功能在当前构建中关闭",
    14: "主动测量启动时确认未佩戴",
    15: "数据校验不匹配",
    16: "存储或固定资源不足",
    17: "首次操作因幂等结果容量不足而未受理；未执行业务副作用",
}

FLAG_BITS = [(0x01, "RESPONSE"), (0x02, "EVENT"), (0x04, "FRAGMENT"),
             (0x08, "MORE"), (0x10, "ERROR")]

MEASURE_TYPES = {1: "HR(心率)", 2: "HRV(心率变异性)", 3: "SpO2(血氧)",
                 4: "TEMP(体温)", 5: "BP(血压)"}
MEASURE_VALUE_LEN = {1: 3, 2: 6, 3: 8, 4: 3, 5: 9}  # DATA.value 固定长度
MEASURE_DURATION_MAX_S = 330
MEASURE_STATES = {0: "IDLE(空闲)", 1: "SCANNING(扫描)", 2: "MEASURING(测量中)",
                  3: "POSTPROCESSING(后处理)", 4: "COMPLETED(已完成)"}
REPORT_MODES = {1: "ONE_SHOT(单次)", 2: "CONTINUOUS(连续)"}
RESULT_ORIGINS = {0: "NONE", 1: "PROTOCOL(协议)", 2: "ADMISSION(准入)", 3: "PROVIDER",
                  4: "ALGORITHM(算法)", 5: "SENSOR(传感器)"}
AUTH_RESULTS = {0: "SUCCESS(成功)", 1: "KEY_NOT_PROVISIONED(未置密钥)",
                2: "PROOF_FAILED(证明失败)", 3: "LOCKED(锁定)",
                4: "PROVISIONING_NOT_AVAILABLE(非置备窗口)", 5: "KEY_ALREADY_PROVISIONED(已有密钥)"}
BIOLOGICAL_SEX = {0: "未知", 1: "男", 2: "女"}
PROFILE_MASK_BITS = [(0x0001, "出生日期"), (0x0002, "身高"), (0x0004, "体重"),
                     (0x0008, "生理性别")]
CHARGE_STATUS = {0: "NOT_CHARGING(未充电)", 1: "CHARGING(充电中)", 2: "FULL(已充满)"}
WEAR_STATES = {0: "UNKNOWN(未知)", 1: "NOT_WORN(未佩戴)", 2: "WORN(已佩戴)"}
SLEEP_STATES = {0: "UNKNOWN(未知)", 1: "AWAKE(清醒)", 2: "ASLEEP(睡眠)"}
SPORT_TYPES = {0x0001: "WALK(步行)", 0x0002: "RUN(跑步)", 0x0003: "CYCLING(骑行)",
               0x0004: "POOL_SWIM(泳池游泳)", 0x00FF: "OTHER(其他)", 0xFFFF: "INVALID"}
EXERCISE_STATES = {0: "IDLE(空闲)", 1: "STARTING", 2: "RUNNING(进行中)", 3: "PAUSED(暂停)",
                   4: "STOPPING", 5: "COMPLETED(已完成)", 6: "DISCARDED(已丢弃)",
                   7: "ABORTED(异常中止)", 8: "WAITING_FOR_CONFIRMATION(等待确认)"}
CONTROL_MODES = {0: "NONE", 1: "APP_CONTROLLED(App控制)", 2: "AUTO_CONTROLLED(自动控制)",
                 3: "AUTONOMOUS_AFTER_DISCONNECT(断连自主)"}
START_SOURCES = {0: "UNKNOWN", 1: "APP_ACTIVE(App主动)", 2: "AUTO_RECOGNITION(自动识别)"}
DISPOSITIONS = {0: "NONE", 1: "SAVED(已保存)", 2: "DISCARDED_TOO_SHORT(过短丢弃)",
                3: "DISCARDED_INVALID(无效丢弃)", 4: "ABORTED_WITH_RECORD(中止但保留)"}
END_REASONS = {0: "NONE", 1: "USER_STOP(用户结束)", 2: "AUTO_LOW_ACTIVITY(低活动)",
               3: "AUTO_NOT_WORN(未佩戴)", 4: "AUTO_CONTINUOUS_INACTIVE(持续静止)",
               5: "CHARGING_STARTED(进入充电)", 6: "LOW_BATTERY(低电)",
               7: "DEVICE_RESTART(设备重启)", 8: "SENSOR_FAILURE(传感器故障)",
               9: "STORAGE_LIMIT(存储上限)", 10: "MAX_DURATION(时长上限)"}
DATASETS = {1: "HEALTH_HISTORY(健康)", 2: "ALGORITHM_INTERMEDIATE(算法中间)",
            3: "DIAGNOSTIC(诊断)", 4: "EXERCISE_HISTORY(运动记录)"}
FACT_TYPES = {
    0x01: "STEPS(步数)", 0x02: "CALORIES(卡路里)",
    0x10: "SLEEP_SUMMARY", 0x11: "SLEEP_STAGE", 0x12: "SLEEP_ONSET", 0x13: "SLEEP_WAKEUP",
    0x20: "HEART_RATE(心率)", 0x21: "RESTING_HR(静息心率)", 0x22: "HRV",
    0x23: "BOXYGEN(血氧)", 0x24: "STRESS(压力)", 0x25: "BREATH_RATE(呼吸)",
    0x30: "ACTIVE_HOURS", 0x31: "BODY_TEMP(体温)",
}
POWER_MODES = {0: "NORMAL", 1: "POWER_SAVE(省电)", 2: "FLIGHT(飞行)", 3: "SHIP(运输)"}
FIND_END_REASONS = {0: "STOPPED(手动停止)", 1: "TIMEOUT(超时结束)", 2: "INTERNAL_ERROR(内部错误)"}
DEVICE_CLASSES = {0x01: "RING(戒指)", 0x02: "SCREENLESS_BAND(无屏手环)"}
CAPS_MAX_FRAME = 250
CAPS_MAX_PAGE = 4096
CAPS_MIN_OFFLINE_MTU = 128
# bit, 协议名, 中文, 消费者是否允许声明
CAPS_DOMAIN_BITS = [
    (0, "SYSTEM", "系统", True),
    (1, "OFFLINE", "离线同步", True),
    (2, "MEASUREMENT", "主动测量", True),
    (3, "DEBUG_RAW_PPG", "内部PPG", False),
    (4, "DEBUG", "内部调试", False),
    (5, "DEVICE_CONTROL", "设备控制", True),
    (6, "FILE", "内部文件", False),
    (7, "EXERCISE", "运动", True),
]
CAPS_MEAS_BITS = [
    (0, "HR", "心率"),
    (1, "HRV", "心率变异性"),
    (2, "SpO2", "血氧"),
    (3, "TEMP", "体温"),
    (4, "BP", "血压"),
]
CAPS_SEC_BITS = [
    (0, "LESC_REQUIRED", "必须先完成LESC加密"),
    (1, "APP_AUTH_REQUIRED", "必须完成AFU应用层认证"),
    (2, "PROVISIONING_REQUIRES_EXTERNAL_POWER", "业务置备窗口须外部供电（为0只表示协议不强制）"),
    (4, "BLE_PRIVACY_REQUIRED", "必须使用BLE隐私地址"),
]
IBEACON_WINDOW_SEC = 60
IBEACON_DEFAULT_PERIOD_MIN = 6
IBEACON_MIN_PERIOD_MIN = 2
IBEACON_MAX_PERIOD_MIN = 255
RECOG_CONFIG_VERSION = 1
EXT_CAPABILITY_VERSION = 1
EXT_MAX_ITEMS_MIN = 1
EXT_MAX_ITEMS_MAX = 64
EXT_ITEM_BYTES = 12
EXT_PREFIX_BYTES = 9  # 含 status
EXT_GROUPS = {
    0x0001: {  # GAIT
        "name": "GAIT(步态)",
        "version": 1,
        "payload_bytes": 32,
        "sports": {0x0001, 0x0002},  # WALK, RUN
        "mask_allowed": 0x00000003,
        "bits": [(0, "cadence步频"), (1, "stride步幅")],
    },
    0x0002: {  # SPEED
        "name": "SPEED(速度)",
        "version": 1,
        "payload_bytes": 28,
        "sports": {0x0001, 0x0002, 0x0003},  # WALK, RUN, CYCLING
        "mask_allowed": 0x00000001,
        "bits": [(0, "current_speed速度")],
    },
    0x0003: {  # SWIM
        "name": "SWIM(游泳)",
        "version": 1,
        "payload_bytes": 28,
        "sports": {0x0004},  # POOL_SWIM
        "mask_allowed": 0x00000003,
        "bits": [(0, "stroke_rate划频"), (1, "average_swolf")],
    },
}

# ---------------------------------------------------------------------------
# 基础工具
# ---------------------------------------------------------------------------

def u16(b, o):
    return b[o] | (b[o + 1] << 8)


def i16(b, o):
    v = u16(b, o)
    return v - 0x10000 if v >= 0x8000 else v


def u32(b, o):
    return int.from_bytes(b[o:o + 4], "little")


def i32(b, o):
    v = u32(b, o)
    return v - 0x100000000 if v >= 0x80000000 else v


def u64(b, o):
    return int.from_bytes(b[o:o + 8], "little")


def hexs(b):
    return " ".join(f"{x:02X}" for x in b)


def build_afu_frame(domain, opcode, flags, rid, payload=b""):
    if len(payload) > 255:
        raise ValueError("payload 超过 255")
    return bytes([0xAF, 0x01, domain & 0xFF, opcode & 0xFF, flags & 0xFF,
                  rid & 0xFF, (rid >> 8) & 0xFF, len(payload)]) + payload


FACT_CN = {
    0x01: "步数", 0x02: "卡路里",
    0x10: "睡眠汇总", 0x11: "睡眠阶段", 0x12: "入睡", 0x13: "出睡",
    0x20: "心率", 0x21: "静息心率", 0x22: "心率变异性",
    0x23: "血氧", 0x24: "压力", 0x25: "呼吸",
    0x30: "活跃小时", 0x31: "体温",
}
SLEEP_STAGE_CN = {0: "清醒", 1: "REM", 2: "浅睡", 3: "深睡"}
SLEEP_TYPE_CN = {0: "未知", 1: "科学睡眠", 2: "小睡"}


def clock_hm(s):
    if not s:
        return "无时间"
    try:
        dt = datetime.datetime.utcfromtimestamp(s) + datetime.timedelta(hours=8)
        return dt.strftime("%m-%d %H:%M")
    except (OverflowError, OSError, ValueError):
        return str(s)


def time_span(st, et):
    if st and et and et != st:
        return f"{clock_hm(st)}–{clock_hm(et)}"
    return clock_hm(st or et)


def _utc8(s):
    return datetime.datetime.utcfromtimestamp(s) + datetime.timedelta(hours=8)


def period_span(st, et):
    """周期记录的起止时间。两个时间都在时标出间隔秒数。"""
    if st and et:
        try:
            a, b = _utc8(st), _utc8(et)
        except (OverflowError, OSError, ValueError):
            return f"{st}–{et}"
        if a.date() == b.date():
            span = f"{a.strftime('%Y-%m-%d %H:%M:%S')}–{b.strftime('%H:%M:%S')}(UTC+8)"
        else:
            span = (f"{a.strftime('%Y-%m-%d %H:%M:%S')}–"
                    f"{b.strftime('%Y-%m-%d %H:%M:%S')}(UTC+8)")
        if et >= st:
            return f"{span}  周期{et - st}秒"
        return span
    return s_to_str(st or et) if (st or et) else "无时间"


def health_value(ftype, body):
    """一条健康记录 → (指标名, 实际上传的值)。没有数值就返回 None。"""
    name = FACT_CN.get(ftype, name_of(FACT_TYPES, ftype))
    if ftype in (0x01, 0x02) and len(body) >= 10:
        st, et = u32(body, 0), u32(body, 4)
        if ftype == 0x01 and len(body) >= 12:
            steps = u32(body, 8)
            val = "无值" if steps == 0xFFFFFFFF else f"{steps}步"
        else:
            kcal = u16(body, 8)
            val = "无值" if kcal == 0xFFFF else f"{kcal}kcal"
        return name, f"{time_span(st, et)}  {val}"
    if ftype == 0x10 and len(body) >= 31:
        bed, asleep = u32(body, 0), u32(body, 4)
        wake, off = u32(body, 8), u32(body, 12)

        def mins(o):
            v = u16(body, o)
            return "无" if v == 0xFFFF else f"{v}分钟"

        score = body[28]
        score_s = "无" if score == 0xFF else str(score)
        wakes = body[30]
        wakes_s = "无" if wakes == 0xFF else f"{wakes}次"
        return name, (
            f"上床{clock_hm(bed)} 入睡{clock_hm(asleep)} 出睡{clock_hm(wake)} 离床{clock_hm(off)}  "
            f"总睡{mins(16)} 深睡{mins(18)} REM{mins(20)} 浅睡{mins(22)} 清醒{mins(24)}  "
            f"评分{score_s} {SLEEP_TYPE_CN.get(body[29], body[29])} 醒来{wakes_s}"
        )
    if ftype == 0x11 and len(body) >= 7:
        n = u16(body, 5)
        packed = body[7:]
        counts = {0: 0, 1: 0, 2: 0, 3: 0}
        for i in range(n):
            if i // 4 >= len(packed):
                break
            stage = (packed[i // 4] >> ((i % 4) * 2)) & 0x03
            counts[stage] = counts.get(stage, 0) + 1
        parts = [f"{SLEEP_STAGE_CN.get(k, k)}{v}" for k, v in counts.items() if v]
        return name, f"{clock_hm(u32(body, 1))}起 {n}段（30秒/段） " + " ".join(parts)
    if ftype == 0x12 and len(body) >= 4:
        return name, clock_hm(u32(body, 0))
    if ftype == 0x13 and len(body) >= 4:
        return name, clock_hm(u32(body, 0))
    if ftype in (0x20, 0x21, 0x23, 0x24, 0x25, 0x30) and len(body) >= 9:
        st, et, raw = u32(body, 0), u32(body, 4), body[8]
        if ftype in (0x20, 0x21):
            val = "无值" if raw == 0xFF else f"{raw}bpm"
        elif ftype == 0x23:
            val = "无值" if raw == 0xFF else f"{raw}%"
        elif ftype == 0x24:
            val = "无值" if raw == 0xFF else str(raw)
        elif ftype == 0x25:
            val = "无值" if raw == 0xFF else f"{raw}次/分"
        else:
            val = "活跃" if raw == 1 else ("不活跃" if raw == 0 else f"非法{raw}")
        span = period_span(st, et) if ftype == 0x20 else time_span(st, et)
        return name, f"{span}  {val}"
    if ftype == 0x22 and len(body) >= 10:
        hrv = u16(body, 8)
        val = "无值" if hrv == 0xFFFF else f"{hrv}ms"
        return name, f"{time_span(u32(body, 0), u32(body, 4))}  {val}"
    if ftype == 0x31 and len(body) >= 10:
        temp = i16(body, 8)
        val = "无值" if temp == -32768 else f"{temp / 10:.1f}℃"
        return name, f"{time_span(u32(body, 0), u32(body, 4))}  {val}"
    return name, "未能读出数值"


EX_CHUNK_DATA_MAX = 4060


def exercise_chunk_info(raw):
    """EXERCISE_HISTORY 每条 seq 前 30 字节 chunk 头。"""
    if len(raw) < 30:
        return None
    dlen = u16(raw, 24)
    return {
        "flags": u16(raw, 2),
        "eid": int.from_bytes(raw[4:12], "little"),
        "index": u16(raw, 12),
        "count": u16(raw, 14),
        "body_len": u32(raw, 16),
        "offset": u32(raw, 20),
        "dlen": dlen,
        "crc": u32(raw, 26),
        "data": bytes(raw[30:30 + dlen]),
        "schema": raw[0],
        "record_kind": raw[1],
    }


def exercise_value(raw):
    """一条运动历史记录 → 实际上传的结果，不展开字段表。"""
    if len(raw) < 30:
        return "运动记录", "记录过短"
    info = exercise_chunk_info(raw)
    flags = info["flags"]
    idx, cnt = info["index"], info["count"]
    data = info["data"]
    if not ((flags & 0x03) == 0x03 and cnt == 1):
        return "运动记录", f"第{idx + 1}/{cnt or '?'}块，收齐后才有完整数值"
    return exercise_body_text(data)


def exercise_body_text(data):
    """完整逻辑 body（单块或分片拼完）→ 与单条运动记录相同的一行。"""
    if len(data) < 112:
        return "运动记录", "记录不完整"
    mask = int.from_bytes(data[8:16], "little")
    parts = [
        cn_only(name_of(SPORT_TYPES, u16(data, 0))),
        time_span(u32(data, 16), u32(data, 20)),
        f"活动{u32(data, 24)}秒",
    ]
    sport = u16(data, 0)
    pace_unit = "秒/100米" if sport == 0x0004 else "秒/公里"
    cur_speed_unit = {0x0003: "km/h", 0x0004: "秒/100米"}.get(sport, "秒/公里")

    def add(bit, text):
        if mask & (1 << bit):
            parts.append(text)

    add(0, f"距离{u32(data, 32) / 100:.2f}米")
    add(1, f"卡路里{u32(data, 36) / 100:.2f}kcal")
    add(2, f"当时心率{data[91]}bpm")
    add(3, f"当时速度{u32(data, 40) / 100:.2f}{cur_speed_unit}")
    add(4, f"步数{u32(data, 60)}")
    add(5, f"平均心率{data[92]}bpm")
    add(6, f"最低心率{data[93]}bpm")
    add(7, f"最高心率{data[94]}bpm")
    add(8, f"平均配速{u32(data, 44) / 100:.2f}{pace_unit}")
    add(9, f"最快配速{u32(data, 48) / 100:.2f}{pace_unit}")
    add(10, f"平均速度{u32(data, 52) / 100:.2f}km/h")
    add(11, f"最快速度{u32(data, 56) / 100:.2f}km/h")
    add(12, f"平均步频{u32(data, 64) / 100:.2f}步/分钟")
    add(13, f"平均步幅{u32(data, 68)}mm")
    add(14, f"累计爬升{u32(data, 72) / 100:.2f}米")
    add(15, f"泳池{u16(data, 76)}米")
    add(16, f"趟数{u16(data, 78)}")
    add(17, f"划水{u32(data, 80)}次")
    add(18, f"泳姿{SWIM_STROKES.get(data[5], data[5])}")
    add(19, f"平均SWOLF{u16(data, 84) / 100:.2f}")
    add(20, f"平均划频{u16(data, 86) / 100:.2f}次/分钟")
    add(21, f"最大摄氧量{u16(data, 88) / 100:.2f}")
    add(22, f"运动年龄{data[90]}")
    add(23, f"训练效果{u16(data, 96) / 100:.2f}")
    add(24, f"恢复{u16(data, 98)}分钟")
    add(25, f"训练负荷{u32(data, 100) / 100:.2f}")
    slots = u16(data, 110) if mask & (1 << 26) else 0
    if slots and len(data) >= 112 + slots:
        bpm = [b for b in data[112:112 + slots] if 1 <= b <= 250]
        shown = " ".join(str(b) for b in bpm[:24])
        more = f" …共{len(bpm)}个" if len(bpm) > 24 else ""
        parts.append(f"心率点 {shown}{more}" if bpm else "心率点 无有效值")
    return "运动", "  ".join(parts)


def _exercise_join(group):
    """同一 exercise_id 的两块都在时拼成一条；未齐返回 None。"""
    if len(group) < 2:
        return None
    by_idx = {}
    for item in group:
        by_idx[item["chunk"]["index"]] = item
    if 0 not in by_idx or 1 not in by_idx:
        return None
    a = by_idx[0]["chunk"]
    b = by_idx[1]["chunk"]
    seq0 = by_idx[0].get("seq")
    seq1 = by_idx[1].get("seq")

    def fail(msg):
        return {"seq": seq0, "kind": "value", "metric": "运动记录", "text": msg}

    if not (
        a["eid"] == b["eid"] and a["count"] == 2 and b["count"] == 2
        and a["body_len"] == b["body_len"] and a["crc"] == b["crc"]
        and a["schema"] == 0x01 and b["schema"] == 0x01
        and a["record_kind"] == 0x01 and b["record_kind"] == 0x01
    ):
        return fail("两块对不上，未汇总")
    if not (a["flags"] & 0x01) or not (b["flags"] & 0x02):
        return fail("分片标志不对，未汇总")
    if seq0 is not None and seq1 is not None and seq1 != (seq0 + 1) & 0xFFFFFFFF:
        return fail("两块 seq 不连续，未汇总")
    body_len = a["body_len"]
    expect0 = min(EX_CHUNK_DATA_MAX, body_len)
    expect1 = body_len - EX_CHUNK_DATA_MAX
    if (
        expect1 <= 0 or a["offset"] != 0 or b["offset"] != EX_CHUNK_DATA_MAX
        or a["dlen"] != expect0 or b["dlen"] != expect1
        or len(a["data"]) != a["dlen"] or len(b["data"]) != b["dlen"]
    ):
        return fail("分片长度不对，未汇总")
    body = a["data"] + b["data"]
    if len(body) != body_len:
        return fail("分片长度不对，未汇总")
    metric, text = exercise_body_text(body)
    if (zlib.crc32(body) & 0xFFFFFFFF) != a["crc"]:
        text += "  整段校验不符"
    return {"seq": seq0, "kind": "value", "metric": metric, "text": text}


def absorb_exercise_chunk(pending, item):
    """跨页攒长运动分片。返回 (记录或 None, 是否仍在等待下一块)。"""
    info = item["chunk"]
    eid = info["eid"]
    group = []
    for old in pending.get(eid, []):
        c = old["chunk"]
        if c["index"] == info["index"]:
            continue
        if (c["crc"] != info["crc"] or c["body_len"] != info["body_len"]
                or c["count"] != info["count"]):
            continue
        group.append(old)
    group.append(item)
    joined = _exercise_join(group)
    if joined is None:
        pending[eid] = group
        return None, True
    pending.pop(eid, None)
    return joined, False


def ascii_preview(data, limit=120):
    """分片级可读预览：控制符收成 /，其余可打印 ASCII。"""
    chars = []
    for x in data:
        if x in (10, 13):
            chars.append("/")
        elif x == 9 or 32 <= x < 127:
            chars.append(chr(x))
        elif x >= 128:
            continue
        else:
            chars.append(".")
    text = re.sub(r"[./]+", " ", "".join(chars)).strip()
    if len(text) > limit:
        return text[:limit] + "…"
    return text


def diagnostic_payload_text(raw: bytes) -> str:
    """DIAGNOSTIC opaque：常见 4 字节头 01 01 00 01，后面是固件日志 ASCII/UTF-8。"""
    p = bytes(raw)
    if len(p) >= 4 and p[:2] == b"\x01\x01":
        p = p[4:]
    text = p.decode("utf-8", "replace").replace("\x00", "")
    text = "".join(
        ch if ch in "\r\n\t" or ord(ch) >= 32 else ""
        for ch in text
    )
    return text.replace("\r\n", "\n").replace("\r", "\n")


def _resync_diag_envelope(page: bytes, start: int):
    """串口截断后，按 seq(u32) + payload_len=0x0104 重新对齐诊断记录。"""
    n = len(page)
    for i in range(start, n - 6):
        if page[i + 4] != 0x04 or page[i + 5] != 0x01:
            continue
        seq = u32(page, i)
        if 1 <= seq <= 0x100000:
            return i
    return None


def decode_offline_records(dataset, page: bytes):
    """把拼好的逻辑页拆成记录。dataset=1 健康 / 3 诊断 / 其它当不透明。"""
    recs = []
    off = 0
    n = len(page)
    while off + 6 <= n:
        seq = u32(page, off)
        plen = u16(page, off + 4)
        bad = plen == 0 or plen > 4090 or off + 6 + plen > n
        if dataset == 3 and not bad and not (1 <= seq <= 0x100000):
            bad = True
        if bad:
            nxt = _resync_diag_envelope(page, off + 1) if dataset == 3 else None
            if nxt is None:
                leftover = n - off
                if leftover > 0:
                    recs.append({
                        "seq": None,
                        "plen": leftover,
                        "note": f"剩余 {leftover} 字节无法对齐成记录",
                        "text": ascii_preview(page[off:], 80),
                    })
                break
            off = nxt
            continue
        raw = bytes(page[off + 6:off + 6 + plen])
        off += 6 + plen
        item = {"seq": seq, "plen": plen, "raw": raw}
        if dataset == 1:
            schema = raw[0] if raw else 0
            ftype = raw[1] if len(raw) > 1 else 0
            blen = u16(raw, 2) if len(raw) >= 4 else 0
            body = raw[4:4 + blen] if len(raw) >= 4 else b""
            metric, text = health_value(ftype, body)
            item["kind"] = "value"
            item["metric"] = metric
            item["text"] = text
            item["schema"] = schema
        elif dataset == 3:
            item["kind"] = "diag"
            item["text"] = diagnostic_payload_text(raw)
        elif dataset == 4:
            info = exercise_chunk_info(raw)
            if info and info["count"] == 2:
                item["kind"] = "ex_chunk"
                item["chunk"] = info
            else:
                metric, text = exercise_value(raw)
                item["kind"] = "value"
                item["metric"] = metric
                item["text"] = text
        else:
            item["kind"] = "opaque"
            item["text"] = hexs(raw[:48]) + (" …" if plen > 48 else "")
        recs.append(item)
    return recs


def format_offline_page_lines(dataset, page_id, page, holes, meta, ex_pending=None):
    ds = cn_only(name_of(DATASETS, dataset))
    if holes:
        hs = "，".join(f"偏移{o}缺{n}字节" for o, n in holes[:4])
        lines = [
            f"======== {ds} 本页未收齐 ========",
            f"串口缺片: {hs}",
            "正文不作为上传结果",
            "======== 本页结束 ========",
        ]
        return lines, []
    recs = decode_offline_records(dataset, page)
    held = 0
    if dataset == 4:
        folded = []
        for r in recs:
            if r.get("kind") != "ex_chunk":
                folded.append(r)
                continue
            if ex_pending is None:
                c = r["chunk"]
                folded.append({
                    "seq": r.get("seq"),
                    "kind": "value",
                    "metric": "运动记录",
                    "text": f"第{c['index'] + 1}/{c['count'] or '?'}块，收齐后才有完整数值",
                })
                continue
            joined, waiting = absorb_exercise_chunk(ex_pending, r)
            if waiting:
                held += 1
            elif joined is not None:
                folded.append(joined)
        recs = folded
    n_ok = sum(1 for r in recs if r.get("seq") is not None)
    n_val = sum(1 for r in recs if r.get("kind") == "value")
    span = ""
    if meta and meta.get("first") is not None and meta.get("last") is not None:
        span = f"（seq {meta['first']}–{meta['last']}）"
    if held and not n_val and not any(r.get("note") for r in recs):
        return [
            f"======== {ds} 长运动分片待汇总{span} ========",
            "下一块到齐后按一条运动记录打印",
            "======== 本页结束 ========",
        ], recs
    lines = [f"======== {ds} 实际上传 {n_val or n_ok} 条{span} ========"]
    if dataset == 3:
        body = "".join(r.get("text") or "" for r in recs if r.get("kind") == "diag")
        lines.append("-------- 诊断日志正文 --------")
        if body.strip():
            for ln in body.split("\n"):
                lines.append(ln)
        else:
            lines.append("(无可用文本)")
    elif dataset in (1, 4):
        groups = {}
        order = []
        for r in recs:
            if r.get("kind") != "value":
                if r.get("note"):
                    lines.append(r["note"])
                continue
            metric = r.get("metric") or "其他"
            if metric not in groups:
                groups[metric] = []
                order.append(metric)
            groups[metric].append(r.get("text") or "无值")
        if not order:
            lines.append("本页没有读出数值")
        for metric in order:
            vals = groups[metric]
            lines.append(f"{metric} {len(vals)}条")
            for v in vals:
                lines.append(f"  {v}")
        if held:
            lines.append("另有长运动分片未齐，下一块到齐后汇总")
    else:
        lines.append("-------- 记录 --------")
        for r in recs:
            if r.get("seq") is None:
                lines.append(r.get("note", ""))
            else:
                lines.append(f"seq={r['seq']} ({r['plen']}字节) {r.get('text', '')}")
    lines.append("======== 本页结束 ========")
    return lines, recs


class OfflineSession:
    """把 OFFLINE_DATA 分片按 offset 拼成逻辑页，CLOSE 时给出本轮同步摘要。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.dataset = None
        self.epoch = None
        self.snapshot_id = None
        self.oldest = None
        self.snapshot_end = None
        self.first_seq = None
        self.claimed_count = None
        self.pulls = []
        self.gaps = []
        self.pull_ask = {}
        self.expect = {}
        self.bufs = {}
        self.completed = []
        self._dumped = {}
        self._last_ctrl = None
        self.ex_pending = {}

    def on_afu(self, data):
        if len(data) < 8 or data[0] != 0xAF or data[1] != 0x01 or data[2] != 0x10:
            return []
        opcode, flags = data[3], data[4]
        rid = u16(data, 5)
        payload = data[8:]
        if opcode != 0x40:
            ident = (opcode, rid, flags, bytes(payload[:24]))
            if ident == self._last_ctrl:
                return []
            self._last_ctrl = ident
        if opcode == 0x01 and flags & 0x01 and payload and payload[0] == 0 and len(payload) >= 26:
            extra = self._flush_ex_pending()
            self.reset()
            self.dataset = payload[1]
            self.epoch = u16(payload, 2)
            self.snapshot_id = u32(payload, 4)
            self.oldest = u32(payload, 8)
            self.snapshot_end = u32(payload, 12)
            self.first_seq = u32(payload, 16)
            self.claimed_count = u32(payload, 20)
            return extra
        if opcode == 0x02 and not (flags & 0x01) and len(payload) >= 9:
            self.pull_ask[rid] = u32(payload, 5)
            return []
        if opcode == 0x02 and flags & 0x01 and payload:
            if payload[0] == 0 and len(payload) >= 31:
                info = {
                    "rid": rid,
                    "dataset": payload[1],
                    "page_id": u16(payload, 6),
                    "first": u32(payload, 8),
                    "last": u32(payload, 12),
                    "next_seq": u32(payload, 16),
                    "nrec": u16(payload, 20),
                    "page_bytes": u16(payload, 22),
                    "frags": payload[24],
                }
                self.pulls.append(info)
                self.expect[rid] = info
                if self.dataset is None:
                    self.dataset = payload[1]
            elif payload[0] == 7 and len(payload) >= 12:
                self.gaps.append({
                    "rid": rid,
                    "dataset": payload[1],
                    "next_seq": u32(payload, 6),
                    "asked": self.pull_ask.get(rid),
                })
            return []
        if opcode == 0x40:
            return self._on_data(rid, flags, payload)
        if opcode == 0x03 and flags & 0x01:
            extra = []
            for key, buf in list(self.bufs.items()):
                extra.extend(self._try_dump(key, buf, force=True))
            extra.extend(self._flush_ex_pending())
            extra.extend(self.summary_lines())
            return extra
        return []

    def _on_data(self, rid, flags, payload):
        if len(payload) < 9:
            return []
        dataset = payload[0]
        page_id = u16(payload, 1)
        idx = payload[3]
        count = payload[4]
        offset = u16(payload, 5)
        flen = u16(payload, 7)
        chunk = payload[9:]
        more = bool(flags & 0x08)
        if offset > 4095:
            return []
        if not (1 <= flen <= 230):
            flen = len(chunk)
        flen = min(flen, len(chunk))
        if more:
            flen = min(flen, 227)
        chunk = chunk[:flen]
        if not chunk:
            return []
        exp = self.expect.get(rid) or {}
        if count == 0:
            count = exp.get("frags") or 0
        key = (rid, page_id if page_id else exp.get("page_id", 0))
        buf = self.bufs.setdefault(key, {
            "dataset": dataset,
            "page_id": key[1],
            "rid": rid,
            "parts": {},
            "count": count,
            "last": not more,
        })
        buf["dataset"] = dataset
        old = buf["parts"].get(offset)
        if old is None or len(chunk) > len(old):
            buf["parts"][offset] = chunk
        if count:
            buf["count"] = count
        if not more:
            buf["last"] = True
        if exp.get("page_bytes"):
            buf["page_bytes"] = exp["page_bytes"]
            buf["first"] = exp.get("first")
            buf["last_seq"] = exp.get("last")
            buf["nrec"] = exp.get("nrec")
        force = buf.get("last") and not more
        return self._try_dump(key, buf, force=force)

    def _assemble(self, buf):
        parts = buf.get("parts") or {}
        if not parts:
            return b"", [], 0
        expected = buf.get("page_bytes") or 0
        size = max(off + len(c) for off, c in parts.items())
        if expected:
            size = max(size, expected)
        size = min(size, 4096)
        mem = bytearray(size)
        filled = bytearray(size)
        for off, chunk in parts.items():
            if off >= size:
                continue
            end = min(size, off + len(chunk))
            mem[off:end] = chunk[:end - off]
            filled[off:end] = b"\x01" * (end - off)
        holes = []
        i = 0
        while i < size:
            if filled[i] == 0:
                j = i
                while j < size and filled[j] == 0:
                    j += 1
                holes.append((i, j - i))
                i = j
            else:
                i += 1
        if expected:
            size = min(size, expected)
            mem = mem[:size]
            trimmed = []
            for o, n in holes:
                if o >= size:
                    continue
                n = min(n, size - o)
                if n:
                    trimmed.append((o, n))
            holes = trimmed
        filled_n = size - sum(n for _, n in holes)
        return bytes(mem), holes, filled_n

    def _try_dump(self, key, buf, force=False):
        page, holes, filled_n = self._assemble(buf)
        if not page:
            return []
        expected = buf.get("page_bytes") or 0
        nfrag = buf.get("count") or 0
        have = len(buf.get("parts") or {})
        complete = (expected and filled_n >= expected and not holes) or (
            nfrag and have >= nfrag and buf.get("last"))
        if not complete and not force and not buf.get("last"):
            return []
        if not complete and not force and holes and filled_n < max(64, expected // 4 if expected else 64):
            return []
        hole_n = sum(n for _, n in holes)
        prev = self._dumped.get(key)
        if prev is not None and hole_n >= prev[1] and prev[0] >= filled_n:
            return []
        self._dumped[key] = (filled_n, hole_n)
        meta = {
            "first": buf.get("first"),
            "last": buf.get("last_seq"),
        }
        lines, recs = format_offline_page_lines(
            buf.get("dataset") or self.dataset or 0,
            buf.get("page_id") or 0,
            page, holes, meta, self.ex_pending)
        info = {
            "page_id": buf.get("page_id"),
            "dataset": buf.get("dataset"),
            "first": buf.get("first"),
            "last": buf.get("last_seq"),
            "claimed_nrec": buf.get("nrec"),
            "nrec": sum(1 for r in recs if r.get("seq") is not None),
            "page_bytes": len(page),
            "holes": holes,
            "recs": recs,
        }
        existed = [i for i, x in enumerate(self.completed) if x.get("page_id") == info["page_id"]]
        if existed:
            self.completed[existed[-1]] = info
        else:
            self.completed.append(info)
        return lines

    def summary_lines(self):
        ds = name_of(DATASETS, self.dataset) if self.dataset is not None else "未知"
        lines = ["======== 本轮离线同步结束 ========"]
        if self.claimed_count is not None:
            lines.append(
                f"数据集: {ds}  epoch={self.epoch}  snapshot_id={self.snapshot_id}")
            lines.append(
                f"设备 OPEN 声称: seq {self.first_seq}..{self.snapshot_end}，共 {self.claimed_count} 条")
        else:
            lines.append(f"数据集: {ds}")
        if self.pulls:
            lines.append("已拉取页:")
            for p in self.pulls:
                lines.append(
                    f"  page_id={p['page_id']}  seq {p['first']}–{p['last']}  "
                    f"设备声明 {p['nrec']} 条  {p['page_bytes']} 字节  {p['frags']} 分片")
        else:
            lines.append("已拉取页: 无")
        got = 0
        diag_n = 0
        incomplete = 0
        tally = {}
        if self.completed:
            lines.append("实际上传:")
        for info in self.completed:
            if info.get("holes"):
                incomplete += 1
                continue
            got += info.get("nrec") or 0
            for r in info.get("recs") or []:
                if r.get("kind") == "diag":
                    diag_n += 1
                    continue
                if r.get("kind") != "value":
                    continue
                metric = r.get("metric") or "其他"
                tally[metric] = tally.get(metric, 0) + 1
        if tally:
            lines.append("  " + "，".join(f"{k} {v}条" for k, v in tally.items()))
        if diag_n:
            lines.append(f"  诊断日志 {diag_n}条")
        elif self.completed and not tally and not incomplete:
            lines.append("  没有读出数值")
        if incomplete:
            lines.append(f"  未收齐 {incomplete} 页，缺片正文未计入")
        if self.gaps:
            for g in self.gaps:
                nxt = g["next_seq"]
                asked = g.get("asked")
                extra = ""
                if asked is not None and nxt is not None and nxt > asked:
                    extra = f"（请求 seq={asked}，{asked}–{nxt - 1} 已缺失，从 {nxt} 继续）"
                elif asked is not None:
                    extra = f"（请求 seq={asked}，设备给出 {nxt}）"
                lines.append(f"缺口 GAP: next_seq={nxt}{extra}")
        lines.append(f"合计 {got} 条，具体数值见上方。")
        lines.append("======== 同步摘要结束 ========")
        return lines

    def _flush_ex_pending(self):
        """同步结束时，没收齐的长运动分片不当成一条完整记录。"""
        lines = []
        for group in self.ex_pending.values():
            bits = []
            for item in sorted(group, key=lambda r: r["chunk"]["index"]):
                c = item["chunk"]
                bits.append(f"第{c['index'] + 1}/{c['count']}块")
            lines.append("长运动分片未收齐，未汇总：" + "，".join(bits))
        self.ex_pending.clear()
        return lines


def ms_to_str(ms):
    if ms == 0:
        return "未提供(0)"
    try:
        dt = datetime.datetime.utcfromtimestamp(ms / 1000) + datetime.timedelta(hours=8)
        return dt.strftime("%Y-%m-%d %H:%M:%S") + f".{ms % 1000:03d}(UTC+8)"
    except (OverflowError, OSError, ValueError):
        return f"{ms}ms"


def s_to_str(s):
    try:
        dt = datetime.datetime.utcfromtimestamp(s) + datetime.timedelta(hours=8)
        return dt.strftime("%Y-%m-%d %H:%M:%S") + "(UTC+8)"
    except (OverflowError, OSError, ValueError):
        return f"{s}s"


def name_of(table, value):
    return table.get(value, f"未知(0x{value:02X})")


def flags_str(flags):
    names = [n for bit, n in FLAG_BITS if flags & bit]
    return "|".join(names) if names else "无(请求)"


def format_status(st):
    """协议 5.2：名称 + 值 + 使用场景。"""
    name = STATUS.get(st)
    usage = STATUS_USAGE.get(st)
    if name is None:
        return f"status=未知  值={st}(0x{st:02X})  不在协议第5.2节（0..17）"
    return f"status={name}  值={st}(0x{st:02X})  使用场景：{usage}"


def status_summary(st):
    name = STATUS.get(st, f"错误码{st}")
    usage = STATUS_USAGE.get(st)
    if usage:
        return f"{name}（{usage}）"
    return name


# ---------------------------------------------------------------------------
# 各命令 payload 解析 (追加到 out 列表)
# ---------------------------------------------------------------------------

def caps_check(p):
    """解析 GET_CAPABILITIES 成功响应。返回 (info, issues)。payload 含 status。"""
    info = {
        "domains": [], "meas": [], "user_reset": False,
        "lesc": False, "auth": False, "extpwr": False, "privacy": False,
        "max_frame": 0, "max_page": 0, "min_mtu": 0,
    }
    issues = []
    if len(p) < 12:
        issues.append(f"成功响应须固定 12 字节，实际 {len(p)}")
        return info, issues
    dm, mm = u16(p, 1), u16(p, 3)
    maint, sf = p[5], p[6]
    max_frame, max_page, min_mtu, reserved = p[7], u16(p, 8), p[10], p[11]
    info.update(max_frame=max_frame, max_page=max_page, min_mtu=min_mtu)
    public_mask = 0
    internal_mask = 0
    for bit, _en, cn, allowed in CAPS_DOMAIN_BITS:
        if allowed:
            public_mask |= 1 << bit
        else:
            internal_mask |= 1 << bit
        if dm & (1 << bit) and allowed:
            info["domains"].append(cn)
    leftover_dom = dm & ~public_mask & ~internal_mask
    if leftover_dom:
        issues.append(f"domain_mask 含未定义位 0x{leftover_dom:04X}，须为 0")
    if dm & internal_mask:
        issues.append(
            f"内部域位必须为 0（DEBUG_RAW_PPG/DEBUG/FILE），实际 domain_mask=0x{dm:04X}")
    leftover_meas = mm & ~0x001F
    if leftover_meas:
        issues.append(f"measurement_mask 含保留位 0x{leftover_meas:04X}，须为 0")
    for bit, _en, cn in CAPS_MEAS_BITS:
        if mm & (1 << bit):
            info["meas"].append(cn)
    if maint & ~0x01:
        issues.append(f"maintenance_mask 仅 bit0=USER_RESET，其余须 0，实际 0x{maint:02X}")
    info["user_reset"] = bool(maint & 0x01)
    info["lesc"] = bool(sf & 0x01)
    info["auth"] = bool(sf & 0x02)
    info["extpwr"] = bool(sf & 0x04)
    info["privacy"] = bool(sf & 0x10)
    if sf & 0x08:
        issues.append("security_flags.bit3 保留，必须为 0")
    if sf & 0xE0:
        issues.append(f"security_flags.bit5..7 保留，必须为 0，实际 0x{sf:02X}")
    if max_frame != CAPS_MAX_FRAME:
        issues.append(f"max_frame_bytes 协议固定 {CAPS_MAX_FRAME}，实际 {max_frame}")
    if max_page != CAPS_MAX_PAGE:
        issues.append(f"max_offline_page 协议固定 {CAPS_MAX_PAGE}，实际 {max_page}")
    if min_mtu != CAPS_MIN_OFFLINE_MTU:
        issues.append(f"min_offline_mtu 协议固定 {CAPS_MIN_OFFLINE_MTU}，实际 {min_mtu}")
    if reserved != 0:
        issues.append(f"reserved 须为 0，实际 {reserved}")
    if len(p) != 12:
        issues.append(f"多余 {len(p) - 12} 字节")
    return info, issues


def s_caps_resp(p):
    info, issues = caps_check(p)
    if len(p) < 12:
        return "能力响应过短"
    parts = [
        "域：" + ("、".join(info["domains"]) if info["domains"] else "无"),
        "测量：" + ("、".join(info["meas"]) if info["meas"] else "无"),
    ]
    parts.append("可恢复出厂" if info["user_reset"] else "不支持恢复出厂")
    need = []
    if info["lesc"]:
        need.append("LESC")
    if info["auth"]:
        need.append("AFU认证")
    if info["extpwr"]:
        need.append("置备须供电")
    if info["privacy"]:
        need.append("BLE隐私")
    if need:
        parts.append("须" + "+".join(need))
    else:
        parts.append("未声明安全要求")
    parts.append(f"帧{info['max_frame']}/页{info['max_page']}/离线MTU{info['min_mtu']}")
    text = "；".join(parts)
    if issues:
        return text + "  !! " + "；".join(issues[:2])
    return text + "  校验通过"


def p_caps_resp(p, out):
    info, issues = caps_check(p)
    if len(p) < 12:
        for it in issues:
            out.append(f"!! {it}")
        return
    dm, mm = u16(p, 1), u16(p, 3)
    maint, sf = p[5], p[6]
    out.append(f"domain_mask=0x{dm:04X}  已声明公开域: "
               + ("、".join(info["domains"]) if info["domains"] else "无"))
    for bit, en, cn, allowed in CAPS_DOMAIN_BITS:
        on = bool(dm & (1 << bit))
        if allowed:
            extra = ""
            if bit == 1 and on:
                extra = "（至少一种离线数据集，不是全部）"
            state = "已声明" if on else "未声明"
        else:
            state = "已声明（非法）" if on else "未声明（消费者固件必须为0）"
        out.append(f"  bit{bit} {en} {cn} = {int(on)} {state}{extra if allowed else ''}")
    leftover_dom = dm & ~0x00FF
    if leftover_dom:
        out.append(f"  bit8..15 保留 = 0x{leftover_dom:04X}（必须为0）")
    out.append(f"measurement_mask=0x{mm:04X}  已声明测量: "
               + ("、".join(info["meas"]) if info["meas"] else "无"))
    for bit, en, cn in CAPS_MEAS_BITS:
        on = bool(mm & (1 << bit))
        out.append(f"  bit{bit} {en} {cn} = {int(on)} "
                   + ("已声明" if on else "未接入（必须为0）"))
    leftover_meas = mm & ~0x001F
    if leftover_meas:
        out.append(f"  保留位 = 0x{leftover_meas:04X}（必须为0）")
    out.append(f"maintenance_mask=0x{maint:02X}  USER_RESET="
               + ("支持恢复出厂设置" if info["user_reset"] else "不支持")
               + ("" if (maint & ~0x01) == 0 else f"  其余位0x{maint & ~0x01:02X}非法"))
    out.append(f"security_flags=0x{sf:02X}")
    for bit, en, cn in CAPS_SEC_BITS:
        on = bool(sf & (1 << bit))
        out.append(f"  bit{bit} {en} = {int(on)}  {cn if on else '未要求'}")
    out.append(f"  bit3/5/6/7 保留 = 0x{(sf & 0xE8):02X}（必须为0）")
    out.append(f"max_frame_bytes={p[7]}（协议固定 {CAPS_MAX_FRAME}）  "
               f"max_offline_page={u16(p, 8)}（协议固定 {CAPS_MAX_PAGE}）")
    out.append(f"min_offline_mtu={p[10]}（离线同步最低MTU，协议固定 {CAPS_MIN_OFFLINE_MTU}）  "
               f"reserved={p[11]}（必须为0）")
    if issues:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
    else:
        out.append("校验: 通过（12字节；内部域位为0；未接入测量位为0；固定上限250/4096/128；reserved=0）")


def p_device_info_resp(p, out):
    if len(p) < 41:
        out.append("!! 长度不足41字节固定前缀"); return
    mac = ":".join(f"{x:02X}" for x in p[1:7])
    out.append(f"device_id={hexs(p[1:17])}  (出厂MAC={mac})")
    out.append(f"device_class={name_of(DEVICE_CLASSES, p[17])}  vendor_id=0x{p[18]:02X}  model_id=0x{p[19]:02X}")
    fw = bytes(p[20:30]).decode("ascii", "replace")
    hw = bytes(p[30:40]).decode("ascii", "replace")
    out.append(f"固件版本=\"{fw}\"  硬件版本=\"{hw}\"")
    o = 40
    for label in ("platform_vendor_id", "ota_target_id", "serial(SN)"):
        if o >= len(p):
            return
        n = p[o]; o += 1
        s = bytes(p[o:o + n]).decode("ascii", "replace"); o += n
        out.append(f"{label}({n}字节)=\"{s}\"")


def p_battery_snapshot(p, out, base=0):
    if len(p) < base + 24:
        out.append("!! 电量快照不足24字节"); return
    o = base
    out.append(f"电量={p[o]}%  充电状态={name_of(CHARGE_STATUS, p[o + 1])}")
    fl = u16(p, o + 2)
    fls = [n for bit, n in [(0, "外部供电"), (1, "电压有效"), (2, "电流有效"),
                            (3, "温度有效"), (4, "循环次数有效"), (5, "健康度有效")] if fl & (1 << bit)]
    out.append(f"battery_flags=0x{fl:04X}: {'/'.join(fls) if fls else '无'}")
    out.append(f"电压={u32(p, o + 4)}mV  电流={i32(p, o + 8)}mA  温度={i32(p, o + 12) / 10}℃")
    out.append(f"循环次数={i32(p, o + 16)}  健康度={i32(p, o + 20)}%")


def p_time_sync_req(p, out):
    if len(p) < 15:
        out.append("!! TIME_SYNC请求不足15字节"); return
    out.append(f"手机时间={ms_to_str(u64(p, 0))}")
    tz = i16(p, 8)
    out.append(f"时区={tz}分钟(UTC{'+' if tz >= 0 else '-'}{abs(tz) // 60}:{abs(tz) % 60:02d})  "
               f"source={'APP系统' if p[10] == 1 else '网络' if p[10] == 2 else p[10]}")
    out.append(f"operation_id=0x{u32(p, 11):08X}")


def p_time_sync_resp(p, out):
    if len(p) < 18:
        out.append("!! TIME_SYNC响应不足18字节"); return
    out.append(f"operation_id=0x{u32(p, 1):08X}")
    out.append(f"设备应用时间={ms_to_str(u64(p, 5))}")
    fl = p[17]
    out.append(f"timebase_generation={u32(p, 13)}  flags=0x{fl:02X}"
               f"({'之前有有效时间' if fl & 1 else ''}{' 事件已持久化' if fl & 2 else ''})")


def profile_mask_str(mask):
    names = [n for bit, n in PROFILE_MASK_BITS if mask & bit]
    return "/".join(names) if names else "无"


def _profile_age(year, month, day):
    try:
        born = datetime.date(year, month, day)
        today = datetime.date.today()
        return today.year - born.year - ((today.month, today.day) < (born.month, born.day))
    except ValueError:
        return None


def decode_profile_fields(p, o, mask):
    """从 birth_year 起 10 字节；返回 (摘要短语列表, 详情行)."""
    if len(p) < o + 10:
        return [], ["!! 用户资料字段不足10字节"]
    year, month, day = u16(p, o), p[o + 2], p[o + 3]
    height, weight, sex = u16(p, o + 4), u16(p, o + 6), p[o + 8]
    parts, lines = [], []
    if mask & 0x0001:
        age = _profile_age(year, month, day)
        age_s = f"，约{age}岁" if age is not None else ""
        lines.append(f"出生日期={year:04d}-{month:02d}-{day:02d}{age_s}")
        parts.append(f"{year:04d}-{month:02d}-{day:02d}")
    else:
        lines.append("出生日期=未设置")
    if mask & 0x0002:
        lines.append(f"身高={height} cm")
        parts.append(f"{height}cm")
    else:
        lines.append("身高=未设置")
    if mask & 0x0004:
        kg = weight / 10
        kg_s = f"{kg:g}" if kg == int(kg) else f"{kg:.1f}"
        lines.append(f"体重={kg_s} kg（{weight}×0.1kg）")
        parts.append(f"{kg_s}kg")
    else:
        lines.append("体重=未设置")
    if mask & 0x0008:
        sex_s = BIOLOGICAL_SEX.get(sex, f"未知({sex})")
        lines.append(f"生理性别={sex_s}")
        parts.append(sex_s)
    else:
        lines.append("生理性别=未设置")
    return parts, lines


def p_user_profile_snapshot(p, out, base):
    """GET 响应 base=1；SET 响应 base=5。"""
    if len(p) < base + 16:
        out.append("!! 用户资料快照不足16字节")
        return
    mask = u16(p, base + 4)
    out.append(f"profile_revision={u32(p, base)}  valid_mask=0x{mask:04X}({profile_mask_str(mask)})")
    _, lines = decode_profile_fields(p, base + 6, mask)
    out.extend(lines)


def p_get_user_profile_resp(p, out):
    p_user_profile_snapshot(p, out, 1)


def p_set_user_profile_req(p, out):
    if len(p) < 18:
        out.append("!! SET_USER_PROFILE请求不足18字节")
        return
    upd, mask = u16(p, 4), u16(p, 6)
    out.append(f"operation_id=0x{u32(p, 0):08X}")
    out.append(f"update_mask=0x{upd:04X}({profile_mask_str(upd)})  "
               f"valid_mask=0x{mask:04X}({profile_mask_str(mask)})")
    _, lines = decode_profile_fields(p, 8, upd)
    out.extend(lines)


def p_set_user_profile_resp(p, out):
    if len(p) < 21:
        out.append("!! SET_USER_PROFILE响应不足21字节")
        return
    out.append(f"operation_id=0x{u32(p, 1):08X}")
    p_user_profile_snapshot(p, out, 5)


def s_user_profile(p, base):
    if len(p) < base + 16:
        return ""
    mask = u16(p, base + 4)
    parts, _ = decode_profile_fields(p, base + 6, mask)
    if not parts:
        return "用户资料均为空"
    return "，".join(parts)


def s_set_user_profile_req(p):
    if len(p) < 18:
        return ""
    upd = u16(p, 4)
    parts, _ = decode_profile_fields(p, 8, upd)
    action = profile_mask_str(upd)
    return f"更新{action}" + (f"：{'，'.join(parts)}" if parts else "")


def _printable(bs):
    return all(0x20 <= x < 0x7F for x in bs)


def p_signature_resp(p, out):
    if len(p) < 115:
        out.append("!! 签名响应不足115字节"); return
    sign = bytes(p[1:65])
    out.append(f"sign[64]={hexs(sign[:16])}...(共64字节, secp256r1 r||s)")
    if sign == bytes(range(1, 65)):
        out.append("!! 签名为 01..40 顺序填充, 疑似固件桩数据(未真正实现)")
    out.append(f"format_version={p[65]}  source_len={p[66]}")
    src = bytes(p[67:67 + p[66]])
    if _printable(src):
        out.append(f"source=\"{src.decode('ascii')}\"")
    else:
        out.append(f"source(二进制, 非文本)={hexs(src)}")


def p_auth_resp(p, out):
    if len(p) < 2:
        out.append("!! AUTH响应过短"); return
    out.append(f"auth_result={name_of(AUTH_RESULTS, p[1])}")
    if len(p) >= 17:
        out.append(f"nonce[15]={hexs(p[2:17])}")


def p_wear_state(p, out, base=0):
    if len(p) < base + 4:
        out.append("!! 佩戴状态过短"); return
    conf = p[base + 1]
    out.append(f"wear_state={name_of(WEAR_STATES, p[base])}  "
               f"confidence={'未提供' if conf == 0xFF else str(conf)}  "
               f"sample_age={u16(p, base + 2)}s")


def p_sleep_state(p, out, base=0):
    if len(p) < base + 3:
        out.append("!! 睡眠状态过短"); return
    conf = p[base + 1]
    out.append(f"sleep_state={name_of(SLEEP_STATES, p[base])}  "
               f"confidence={'未提供' if conf == 0xFF else str(conf)}  flags=0x{p[base + 2]:02X}")
    if p[base + 2] & 1 and len(p) >= base + 11:
        out.append(f"时间戳={ms_to_str(u64(p, base + 3))}")


def ibeacon_period_ok(enabled, period_min):
    if enabled == 0:
        return period_min == 0
    if enabled == 1:
        return period_min == 0 or IBEACON_MIN_PERIOD_MIN <= period_min <= IBEACON_MAX_PERIOD_MIN
    return False


def ibeacon_check(enabled, period_min, window_sec):
    issues = []
    if enabled not in (0, 1):
        issues.append(f"enabled={enabled} 只能是 0 关 / 1 开")
    if enabled == 0 and period_min != 0:
        issues.append(f"已关闭时 period_min 必须为 0，实际 {period_min}")
    if enabled == 1 and not (period_min == 0 or IBEACON_MIN_PERIOD_MIN <= period_min <= IBEACON_MAX_PERIOD_MIN):
        issues.append(
            f"已开启时 period_min 须为 0(默认{IBEACON_DEFAULT_PERIOD_MIN}分钟) 或 "
            f"{IBEACON_MIN_PERIOD_MIN}..{IBEACON_MAX_PERIOD_MIN}，实际 {period_min}")
    if enabled == 1 and window_sec != IBEACON_WINDOW_SEC:
        issues.append(f"已开启时 window_sec 必须为 {IBEACON_WINDOW_SEC}，实际 {window_sec}")
    if enabled == 0 and window_sec not in (0, IBEACON_WINDOW_SEC):
        issues.append(f"关闭时 window_sec 应为 0 或 {IBEACON_WINDOW_SEC}，实际 {window_sec}")
    return issues


def s_ibeacon_cfg(enabled, period_min, window_sec=None):
    on = "开启" if enabled == 1 else ("关闭" if enabled == 0 else f"enabled={enabled}")
    if period_min == 0 and enabled == 1:
        period = f"周期默认{IBEACON_DEFAULT_PERIOD_MIN}分钟"
    else:
        period = f"周期{period_min}分钟"
    if window_sec is None:
        text = f"{on}，{period}"
        issues = []
        if enabled not in (0, 1) or not ibeacon_period_ok(enabled, period_min):
            issues = ibeacon_check(enabled, period_min, IBEACON_WINDOW_SEC)
    else:
        text = f"{on}，{period}，窗口{window_sec}秒"
        issues = ibeacon_check(enabled, period_min, window_sec)
    if issues:
        return text + "  !! " + "；".join(issues)
    return text + "  校验通过"


def p_get_ibeacon_resp(p, out):
    if len(p) < 4:
        out.append(f"!! GET_IBEACON_CONFIG 成功响应须 4 字节，实际 {len(p)}")
        return
    enabled, period_min, window_sec = p[1], p[2], p[3]
    out.append(f"enabled={enabled}({'开' if enabled == 1 else '关' if enabled == 0 else '非法'})")
    out.append(f"period_min={period_min}"
               + ("（默认6分钟）" if enabled == 1 and period_min == 0 else ""))
    out.append(f"window_sec={window_sec}")
    issues = ibeacon_check(enabled, period_min, window_sec)
    if len(p) != 4:
        issues.append(f"多余 {len(p) - 4} 字节")
    if issues:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
    else:
        out.append("校验: 通过（开=1且周期2..255或0=默认6；窗=60秒；关则周期必须0）")


def p_set_ibeacon_req(p, out):
    if len(p) < 6:
        out.append(f"!! SET_IBEACON_CONFIG 请求须 6 字节，实际 {len(p)}")
        return
    enabled, period_min = p[4], p[5]
    out.append(f"operation_id=0x{u32(p, 0):08X}")
    out.append(f"enabled={enabled}  period_min={period_min}")
    issues = []
    if enabled not in (0, 1) or not ibeacon_period_ok(enabled, period_min):
        issues = ibeacon_check(enabled, period_min, IBEACON_WINDOW_SEC if enabled else 0)
    if len(p) != 6:
        issues.append(f"多余 {len(p) - 6} 字节")
    if issues:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
    else:
        out.append("校验: 通过")


def p_set_ibeacon_resp(p, out):
    if len(p) < 8:
        out.append(f"!! SET_IBEACON_CONFIG 成功响应须 8 字节，实际 {len(p)}")
        return
    enabled, period_min, window_sec = p[5], p[6], p[7]
    out.append(f"operation_id=0x{u32(p, 1):08X}")
    out.append(f"enabled={enabled}  effective_period_min={period_min}  window_sec={window_sec}")
    issues = ibeacon_check(enabled, period_min, window_sec)
    if len(p) != 8:
        issues.append(f"多余 {len(p) - 8} 字节")
    if issues:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
    else:
        out.append("校验: 通过")


def recog_check(version, enabled, reserved):
    issues = []
    if version != RECOG_CONFIG_VERSION:
        issues.append(f"config_version 须为 {RECOG_CONFIG_VERSION}，实际 {version}")
    if enabled not in (0, 1):
        issues.append(f"auto_recognition_enabled={enabled} 只能是 0关 / 1开")
    if reserved != 0:
        issues.append(f"reserved 须为 0，实际 {reserved}")
    return issues


def s_recog_cfg(version, enabled, reserved):
    on = "开启" if enabled == 1 else ("关闭" if enabled == 0 else f"enabled={enabled}")
    text = f"自动识别{on}，config_version={version}"
    issues = recog_check(version, enabled, reserved)
    if issues:
        return text + "  !! " + "；".join(issues)
    return text + "  校验通过"


def p_get_recognition_resp(p, out):
    if len(p) < 5:
        out.append(f"!! GET_RECOGNITION_CONFIG 成功响应须 5 字节，实际 {len(p)}")
        return
    version, enabled, reserved = u16(p, 1), p[3], p[4]
    out.append(f"config_version={version}（首版须为 {RECOG_CONFIG_VERSION}）")
    out.append(f"auto_recognition_enabled={enabled}"
               f"({'开' if enabled == 1 else '关' if enabled == 0 else '非法'})")
    out.append(f"reserved={reserved}")
    issues = recog_check(version, enabled, reserved)
    if len(p) != 5:
        issues.append(f"多余 {len(p) - 5} 字节")
    if issues:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
    else:
        out.append("校验: 通过（version=1，开关0/1，reserved=0，恰好5字节）")


def p_set_recognition_req(p, out):
    if len(p) < 6:
        out.append(f"!! SET_RECOGNITION_CONFIG 请求须 6 字节，实际 {len(p)}")
        return
    enabled, reserved = p[4], p[5]
    out.append(f"operation_id=0x{u32(p, 0):08X}")
    out.append(f"auto_recognition_enabled={enabled}  reserved={reserved}")
    issues = []
    if enabled not in (0, 1):
        issues.append(f"auto_recognition_enabled={enabled} 只能是 0关 / 1开")
    if reserved != 0:
        issues.append(f"reserved 须为 0，实际 {reserved}")
    if len(p) != 6:
        issues.append(f"多余 {len(p) - 6} 字节")
    if issues:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
    else:
        out.append("校验: 通过")


def p_set_recognition_resp(p, out):
    if len(p) < 9:
        out.append(f"!! SET_RECOGNITION_CONFIG 成功响应须 9 字节，实际 {len(p)}")
        return
    version, enabled, reserved = u16(p, 5), p[7], p[8]
    out.append(f"operation_id=0x{u32(p, 1):08X}")
    out.append(f"config_version={version}  auto_recognition_enabled={enabled}  reserved={reserved}")
    issues = recog_check(version, enabled, reserved)
    if len(p) != 9:
        issues.append(f"多余 {len(p) - 9} 字节")
    if issues:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
    else:
        out.append("校验: 通过")


def ext_mask_bits(mask, bits):
    names = [n for bit, n in bits if mask & (1 << bit)]
    return "/".join(names) if names else "无字段"


def ext_item_issues(sport, data_type, fmt_ver, reserved0, payload_bytes, reserved1, mask):
    """已知组必须核对；未知组返回 None 表示跳过字段校验。"""
    spec = EXT_GROUPS.get(data_type)
    if spec is None:
        return None
    issues = []
    if fmt_ver != spec["version"]:
        issues.append(f"format_version 须为 {spec['version']}，实际 {fmt_ver}")
    if reserved0 != 0:
        issues.append(f"reserved0 须为 0，实际 {reserved0}")
    if reserved1 != 0:
        issues.append(f"reserved1 须为 0，实际 {reserved1}")
    if payload_bytes != spec["payload_bytes"]:
        issues.append(f"payload_bytes 须为 {spec['payload_bytes']}（含24字节公共头），实际 {payload_bytes}")
    if sport not in spec["sports"]:
        sports = "、".join(cn_only(name_of(SPORT_TYPES, s)) for s in sorted(spec["sports"]))
        issues.append(f"{spec['name']} 不适用于 {cn_only(name_of(SPORT_TYPES, sport))}（仅 {sports}）")
    allowed = spec["mask_allowed"]
    if mask & ~allowed:
        issues.append(f"supported_field_mask=0x{mask:08X} 含保留位（允许 0x{allowed:08X}）")
    if (mask & allowed) == 0:
        issues.append("supported_field_mask 至少一位为 1")
    return issues


def s_ext_caps_req(p):
    if len(p) < 5:
        return "请求过短"
    sport, start, max_items = u16(p, 0), u16(p, 2), p[4]
    text = f"{cn_only(name_of(SPORT_TYPES, sport))}，start={start}，max_items={max_items}"
    issues = []
    if max_items < EXT_MAX_ITEMS_MIN or max_items > EXT_MAX_ITEMS_MAX:
        issues.append(f"max_items 须为 {EXT_MAX_ITEMS_MIN}..{EXT_MAX_ITEMS_MAX}")
    if len(p) != 5:
        issues.append(f"须恰好 5 字节")
    if issues:
        return text + "  !! " + "；".join(issues)
    return text + "  校验通过"


def s_ext_caps_resp(p):
    if len(p) < EXT_PREFIX_BYTES:
        return "响应过短"
    sport, total, nxt, cnt = u16(p, 2), u16(p, 4), u16(p, 6), p[8]
    names = []
    o = EXT_PREFIX_BYTES
    for _ in range(cnt):
        if o + EXT_ITEM_BYTES > len(p):
            break
        dt = u16(p, o)
        spec = EXT_GROUPS.get(dt)
        names.append(spec["name"].split("(")[0] if spec else f"0x{dt:04X}")
        o += EXT_ITEM_BYTES
    page = "、".join(names) if names else "无"
    extra = "" if nxt == 0xFFFF else f"，下一页={nxt}"
    return f"{cn_only(name_of(SPORT_TYPES, sport))} 共{total}组本页{cnt}项：{page}{extra}"


def p_get_ext_caps_req(p, out):
    if len(p) < 5:
        out.append(f"!! GET_REALTIME_EXTENSION_CAPABILITIES 请求须 5 字节，实际 {len(p)}")
        return
    sport, start, max_items = u16(p, 0), u16(p, 2), p[4]
    out.append(f"sport_type={name_of(SPORT_TYPES, sport)}")
    out.append(f"start_index={start}  max_items={max_items}（协议 1..64；MTU128 时每页最多 9 项）")
    issues = []
    if max_items < EXT_MAX_ITEMS_MIN or max_items > EXT_MAX_ITEMS_MAX:
        issues.append(
            f"max_items 须为 {EXT_MAX_ITEMS_MIN}..{EXT_MAX_ITEMS_MAX}，实际 {max_items}")
    if len(p) != 5:
        issues.append(f"多余 {len(p) - 5} 字节")
    if issues:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
    else:
        out.append("校验: 通过")


def p_get_ext_caps_resp(p, out):
    if len(p) < EXT_PREFIX_BYTES:
        out.append(f"!! 成功前缀须 {EXT_PREFIX_BYTES} 字节（含 status），实际 {len(p)}")
        return
    cap_ver = p[1]
    sport, total, nxt, cnt = u16(p, 2), u16(p, 4), u16(p, 6), p[8]
    out.append(f"capability_version={cap_ver}（须为 {EXT_CAPABILITY_VERSION}）")
    out.append(f"sport_type={name_of(SPORT_TYPES, sport)}")
    out.append(f"total_items={total}  next_index={'结束(0xFFFF)' if nxt == 0xFFFF else nxt}  item_count={cnt}")
    issues = []
    if cap_ver != EXT_CAPABILITY_VERSION:
        issues.append(f"capability_version 须为 {EXT_CAPABILITY_VERSION}，实际 {cap_ver}")
    if cnt * EXT_ITEM_BYTES + EXT_PREFIX_BYTES != len(p):
        issues.append(
            f"长度须为 {EXT_PREFIX_BYTES}+{cnt}×{EXT_ITEM_BYTES}="
            f"{EXT_PREFIX_BYTES + cnt * EXT_ITEM_BYTES}，实际 {len(p)}")
    if cnt > total:
        issues.append(f"item_count={cnt} 大于 total_items={total}")
    if nxt != 0xFFFF and nxt >= total:
        issues.append(f"next_index={nxt} 须 < total_items={total}，末页才是 0xFFFF")
    if nxt == 0xFFFF and total == 0 and cnt != 0:
        issues.append("total_items=0 时本页不应有条目")
    need = EXT_PREFIX_BYTES + cnt * EXT_ITEM_BYTES
    if len(p) < need:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
        out.append(f"!! 条目字节不足: 需要{need}, 实际{len(p)}")
        return
    seen = []
    for i in range(cnt):
        o = EXT_PREFIX_BYTES + i * EXT_ITEM_BYTES
        dt, fmt_ver = u16(p, o), p[o + 2]
        reserved0, payload_bytes = p[o + 3], u16(p, o + 4)
        reserved1, mask = u16(p, o + 6), u32(p, o + 8)
        spec = EXT_GROUPS.get(dt)
        label = spec["name"] if spec else f"未知(0x{dt:04X})"
        bits = spec["bits"] if spec else []
        out.append(
            f"  [{i}] {label}  format_version={fmt_ver}  reserved0={reserved0}  "
            f"payload_bytes={payload_bytes}  reserved1={reserved1}  "
            f"mask=0x{mask:08X}({ext_mask_bits(mask, bits) if spec else '未注册'})")
        seen.append((dt, fmt_ver))
        item_issues = ext_item_issues(sport, dt, fmt_ver, reserved0, payload_bytes, reserved1, mask)
        if item_issues is None:
            out.append(f"      （未知数据组，按协议跳过字段校验）")
        else:
            for it in item_issues:
                issues.append(f"[{i}] {it}")
    for i in range(1, len(seen)):
        if seen[i] == seen[i - 1]:
            issues.append(f"(data_type,format_version)={seen[i]} 重复")
        elif seen[i] < seen[i - 1]:
            issues.append("条目未按 (data_type,format_version) 升序")
            break
    if issues:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
    else:
        out.append("校验: 通过（version=1，长度=前缀+12×项，组/版本/payload/mask/适用运动/排序均符合）")


def p_config_ext_req(p, out):
    if len(p) < 6:
        out.append(f"!! CONFIG_REALTIME_EXTENSION 请求须 6 字节，实际 {len(p)}")
        return
    sport, dt, fmt_ver, enabled = u16(p, 0), u16(p, 2), p[4], p[5]
    spec = EXT_GROUPS.get(dt)
    out.append(f"sport_type={name_of(SPORT_TYPES, sport)}  "
               f"data_type={spec['name'] if spec else f'0x{dt:04X}'}")
    out.append(f"format_version={fmt_ver}  enabled={enabled}")
    issues = []
    if enabled not in (0, 1):
        issues.append(f"enabled={enabled} 只能是 0/1")
    if spec is None:
        issues.append(f"未知 data_type=0x{dt:04X}")
    elif fmt_ver != spec["version"]:
        issues.append(f"format_version 须为 {spec['version']}")
    elif sport not in spec["sports"]:
        issues.append(f"{spec['name']} 不适用于该运动")
    if len(p) != 6:
        issues.append(f"多余 {len(p) - 6} 字节")
    if issues:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
    else:
        out.append("校验: 通过")


def p_config_ext_resp(p, out):
    if len(p) < 7:
        out.append(f"!! CONFIG_REALTIME_EXTENSION 成功响应须 7 字节，实际 {len(p)}")
        return
    sport, dt, fmt_ver, enabled = u16(p, 1), u16(p, 3), p[5], p[6]
    spec = EXT_GROUPS.get(dt)
    out.append(f"sport_type={name_of(SPORT_TYPES, sport)}  "
               f"data_type={spec['name'] if spec else f'0x{dt:04X}'}")
    out.append(f"format_version={fmt_ver}  enabled={enabled}")
    issues = []
    if enabled not in (0, 1):
        issues.append(f"enabled={enabled} 只能是 0/1")
    if spec is None:
        issues.append(f"未知 data_type=0x{dt:04X}")
    elif fmt_ver != spec["version"]:
        issues.append(f"format_version 须为 {spec['version']}")
    if len(p) != 7:
        issues.append(f"多余 {len(p) - 7} 字节")
    if issues:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
    else:
        out.append("校验: 通过")


def p_case_battery(p, out, base=0):
    if len(p) < base + 8:
        out.append("!! 充电仓快照不足8字节"); return
    fl = p[base]
    fls = [n for bit, n in [(0, "仓在位"), (1, "电量有效"), (2, "仓外部供电"),
                            (3, "电源状态有效"), (4, "温度有效"), (5, "电流有效")] if fl & (1 << bit)]
    out.append(f"case_flags=0x{fl:02X}: {'/'.join(fls) if fls else '无'}")
    out.append(f"charge_status={name_of(CHARGE_STATUS, p[base + 1]) if p[base + 1] != 0xFF else '未知(0xFF)'}  "
               f"温度={p[base + 2] if p[base + 2] < 0x80 else p[base + 2] - 0x100}℃  "
               f"电流={i16(p, base + 3)}mA")
    out.append(f"电量={'未知' if p[base + 5] == 0xFF else str(p[base + 5]) + '%'}  sample_age={u16(p, base + 6)}s")


def p_find_start_req(p, out):
    if len(p) < 3:
        out.append("!! FIND_START请求过短"); return
    sigs = [n for bit, n in [(0, "LED"), (1, "振动"), (2, "蜂鸣器")] if p[0] & (1 << bit)]
    out.append(f"signal_mask=0x{p[0]:02X}({'/'.join(sigs) if sigs else '设备默认'})  时长={u16(p, 1)}s")


def p_find_start_resp(p, out):
    if len(p) < 6:
        out.append("!! FIND_START响应过短"); return
    sigs = [n for bit, n in [(0, "LED"), (1, "振动"), (2, "蜂鸣器")] if p[3] & (1 << bit)]
    out.append(f"find_id={u16(p, 1)}  accepted_mask=0x{p[3]:02X}({'/'.join(sigs)})  接受时长={u16(p, 4)}s")


def measure_cn(t):
    return cn_only(name_of(MEASURE_TYPES, t))


def measure_result_hint():
    return ("说明: 心率/心率变异性/血氧/体温/血压共用同一套流程——"
            "START 只开会话，STOP/GET_STATE 只报状态，"
            "主值只在 MEASUREMENT/DATA(0x40) 事件；VALID=1 才是可用结果")


def p_measure_start_req(p, out):
    if len(p) < 4:
        out.append("!! START请求不足4字节"); return
    mtype, mode, dur = p[0], p[1], u16(p, 2)
    out.append(f"type={name_of(MEASURE_TYPES, mtype)}  模式={name_of(REPORT_MODES, mode)}  "
               f"时长={dur}s{'(设备默认预算)' if dur == 0 else ''}")
    issues = []
    if mtype not in MEASURE_TYPES:
        issues.append(f"type={mtype} 不在 1..5（HR/HRV/SpO2/TEMP/BP）")
    if mode not in REPORT_MODES:
        issues.append(f"report_mode={mode} 只能是 1单次 / 2连续")
    if mtype == 2 and mode != 1:
        issues.append("HRV v1.0 只支持 ONE_SHOT")
    if dur != 0 and not (1 <= dur <= MEASURE_DURATION_MAX_S):
        issues.append(f"duration_s 须为 0(默认) 或 1..{MEASURE_DURATION_MAX_S}，实际 {dur}")
    if len(p) != 4:
        issues.append(f"请求须恰好 4 字节，实际 {len(p)}")
    if issues:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
    else:
        out.append("校验: 通过（本帧仍无测量值）")
        out.append(measure_result_hint())


def p_measure_start_resp(p, out, is_err):
    if is_err:
        if len(p) >= 3:
            out.append(f"result_origin={name_of(RESULT_ORIGINS, p[1])}  result_code=0x{p[2]:02X}")
            out.append("说明: START 失败不分配 measurement_id，也不会有 DATA")
        elif len(p) == 1:
            out.append("!! 仅1字节status, 不符合9.1节固定3字节失败布局(status+origin+code)")
        return
    if len(p) < 7:
        out.append(f"!! START成功响应须 7 字节，实际 {len(p)}"); return
    dur = u16(p, 5)
    out.append(f"measurement_id={u16(p, 1)}  type={name_of(MEASURE_TYPES, p[3])}  "
               f"模式={name_of(REPORT_MODES, p[4])}  接受时长={dur}s（本轮权威预算）")
    issues = []
    if p[3] not in MEASURE_TYPES:
        issues.append(f"type={p[3]} 非法")
    if p[4] not in REPORT_MODES:
        issues.append(f"report_mode={p[4]} 非法")
    if not (1 <= dur <= MEASURE_DURATION_MAX_S):
        issues.append(f"accepted_duration_s 须为 1..{MEASURE_DURATION_MAX_S}，实际 {dur}")
    if len(p) != 7:
        issues.append(f"多余 {len(p) - 7} 字节")
    if issues:
        for it in issues:
            out.append(f"!! 校验失败: {it}")
    else:
        out.append("校验: 通过")
    out.append(measure_result_hint())


def p_measure_stop_resp(p, out):
    if len(p) < 4:
        out.append(f"!! STOP成功响应须 4 字节，实际 {len(p)}")
        return
    st = p[3]
    out.append(f"measurement_id={u16(p, 1)}  final_state={name_of(MEASURE_STATES, st)}")
    if st not in MEASURE_STATES:
        out.append(f"!! final_state={st} 不在 0..4")
    if len(p) != 4:
        out.append(f"!! 多余 {len(p) - 4} 字节")
    out.append(measure_result_hint())


def p_measure_get_state_req(p, out):
    if len(p) < 2:
        out.append("!! GET_STATE 请求须 2 字节"); return
    mid = u16(p, 0)
    out.append(f"measurement_id={mid}")
    if mid == 0:
        out.append("!! 主动测量没有 GET_STATE(0) 查当前会话；0 不是合法 measurement_id，"
                   "设备回 INVALID_CURSOR 符合预期（运动域 GET_STATE(0) 是另一套）")
    if len(p) != 2:
        out.append(f"!! 多余 {len(p) - 2} 字节")


def p_measure_get_state_resp(p, out):
    if len(p) < 10:
        out.append(f"!! GET_STATE 成功响应须 10 字节，实际 {len(p)}"); return
    st = p[5]
    out.append(f"measurement_id={u16(p, 1)}  type={name_of(MEASURE_TYPES, p[3])}  "
               f"模式={name_of(REPORT_MODES, p[4])}")
    out.append(f"state={name_of(MEASURE_STATES, st)}  last_sample_seq={u32(p, 6)}"
               f"{'（尚未产生 DATA）' if u32(p, 6) == 0 else ''}")
    if len(p) != 10:
        out.append(f"!! 多余 {len(p) - 10} 字节")
    out.append(measure_result_hint())


def p_measure_period_resp(p, out):
    if len(p) < 10:
        out.append("!! 周期响应不足10字节"); return
    out.append(f"type={name_of(MEASURE_TYPES, p[1])}  当前周期={u16(p, 2)}min  "
               f"范围={u16(p, 4)}~{u16(p, 6)}min  步进={u16(p, 8)}min")


def _conf(v):
    return "未提供" if v == 0xFF else str(v)


def decode_measure_value(mtype, val):
    """返回人可读的测量主值；val 为 value 字段原始字节。"""
    if mtype == 1 and len(val) >= 3:
        return f"心率={u16(val, 0)} bpm  confidence={_conf(val[2])}"
    if mtype == 2 and len(val) >= 6:
        rmssd = u32(val, 2)
        return (f"HRV窗口={u16(val, 0)}s  "
                f"RMSSD(相邻心跳间隔波动)={rmssd / 100:.2f}ms"
                f"(rmssd_ms_x100={rmssd})")
    if mtype == 3 and len(val) >= 8:
        flags = val[0]
        spo2 = val[1]
        parts = [f"血氧={spo2}%", f"confidence={_conf(val[2])}"]
        if flags & 1:
            parts.append(f"脉率={u16(val, 3)}  pulse_conf={_conf(val[5])}")
        else:
            parts.append("脉率=未提供")
        if flags & 2:
            parts.append(f"灌注={val[6]}")
        if val[7]:
            parts.append(f"failure_reason=0x{val[7]:02X}")
        return "  ".join(parts)
    if mtype == 4 and len(val) >= 3:
        return f"皮温={i16(val, 0) / 100:.2f}℃  confidence={_conf(val[2])}"
    if mtype == 5 and len(val) >= 9:
        return (f"收缩压={u16(val, 0)}  舒张压={u16(val, 2)}  "
                f"平均压={u16(val, 4)}  脉率={u16(val, 6)}  "
                f"confidence={_conf(val[8])}")
    return f"value={hexs(val)}" if val else "value=(空)"


def p_measure_data(p, out):
    if len(p) < 19:
        out.append("!! DATA事件过短"); return
    mtype = p[6]
    out.append(f"measurement_id={u16(p, 0)}  sample_seq={u32(p, 2)}  "
               f"type={name_of(MEASURE_TYPES, mtype)}  "
               f"state={name_of(MEASURE_STATES, p[7])}")
    sb = u16(p, 8)
    quals = [n for bit, n in [(0, "VALID"), (1, "CONTACT_OK"), (2, "MOTION"), (3, "LOW_SIGNAL")]
             if sb & (1 << bit)]
    origin, code = (sb >> 4) & 0x0F, (sb >> 8) & 0xFF
    out.append(f"status_bits=0x{sb:04X}: {'/'.join(quals) if quals else '无质量位'}  "
               f"origin={name_of(RESULT_ORIGINS, origin)} code=0x{code:02X}  "
               f"{'主值可用' if sb & 1 else '主值不可用(VALID=0)'}")
    vlen = p[18]
    val = p[19:19 + vlen]
    out.append(f"时间={ms_to_str(u64(p, 10))}  value_len={vlen}(实际{len(val)})")
    expect = MEASURE_VALUE_LEN.get(mtype)
    if expect is not None and vlen != expect:
        out.append(f"!! value_len 须为 {expect} 字节（{measure_cn(mtype)} 固定布局），实际 {vlen}")
    if vlen != len(val):
        out.append("!! value 长度不足")
    out.append(decode_measure_value(mtype, val))
    if not (sb & 1):
        out.append("结论: 主值不可用（VALID=0）。未出结果时仍发固定长度 DATA，"
                   f"{measure_cn(mtype)} 的 0 是占位不是测到 0。")
    else:
        out.append(f"结论: {measure_cn(mtype)}主值可用")
    out.append(f"value原始={hexs(val)}")


def p_exercise_state(p, out, base=0, has_status=True):
    o = base
    if len(p) < o + 48:
        out.append("!! 运动状态不足48字节"); return
    out.append(f"exercise_id=0x{u64(p, o):016X}  sport={name_of(SPORT_TYPES, u16(p, o + 8))}")
    out.append(f"来源={name_of(START_SOURCES, p[o + 10])}  状态={name_of(EXERCISE_STATES, p[o + 11])}  "
               f"控制={name_of(CONTROL_MODES, p[o + 12])}")
    sf = u16(p, o + 16)
    out.append(f"state_flags=0x{sf:04X}({'结束时间有效' if sf & 1 else '结束时间未置位'})  "
               f"timebase_generation={u32(p, o + 20)}")
    out.append(f"处置={name_of(DISPOSITIONS, p[o + 13])}  结束原因={name_of(END_REASONS, p[o + 14])}  "
               f"泳池长度={u16(p, o + 18)}m")
    st, et = u32(p, o + 24), u32(p, o + 28)
    out.append(f"开始={'无' if st == 0 else s_to_str(st)}  结束={'无' if et == 0 else s_to_str(et)}")
    out.append(f"活动时长={u32(p, o + 32)}s  暂停时长={u32(p, o + 36)}s  "
               f"断连累计={u32(p, o + 40)}s  last_realtime_seq={u32(p, o + 44)}")


def p_exercise_start_req(p, out):
    if len(p) < 12:
        out.append("!! START请求不足12字节"); return
    st = u32(p, 8)
    out.append(f"operation_id=0x{u32(p, 0):08X}  sport={name_of(SPORT_TYPES, u16(p, 4))}  "
               f"泳池长度={u16(p, 6)}m")
    out.append(f"app_start_time={'无' if st == 0 else s_to_str(st)}(仅诊断用)")


def p_exercise_start_resp(p, out):
    if len(p) == 11:
        out.append(f"冲突运动 exercise_id=0x{u64(p, 1):016X}  "
                   f"来源={name_of(START_SOURCES, p[9])}  状态={name_of(EXERCISE_STATES, p[10])}")
        return
    if len(p) < 25:
        out.append("!! START响应不足25字节"); return
    out.append(f"operation_id=0x{u32(p, 1):08X}  exercise_id=0x{u64(p, 5):016X}")
    out.append(f"sport={name_of(SPORT_TYPES, u16(p, 13))}  状态={name_of(EXERCISE_STATES, p[15])}  "
               f"控制={name_of(CONTROL_MODES, p[16])}")
    out.append(f"权威开始时间={s_to_str(u32(p, 17))}  timebase_generation={u32(p, 21)}")


def p_exercise_stop_req(p, out):
    if len(p) < 16:
        out.append("!! STOP请求不足16字节"); return
    out.append(f"operation_id=0x{u32(p, 0):08X}  exercise_id=0x{u64(p, 4):016X}  "
               f"reason={name_of(END_REASONS, p[12])}")


def p_exercise_stop_resp(p, out):
    if len(p) < 29:
        out.append("!! STOP响应不足29字节"); return
    out.append(f"operation_id=0x{u32(p, 1):08X}  exercise_id=0x{u64(p, 5):016X}")
    out.append(f"final_state={name_of(EXERCISE_STATES, p[13])}  处置={name_of(DISPOSITIONS, p[14])}  "
               f"结束原因={name_of(END_REASONS, p[15])}")
    out.append(f"结束时间={s_to_str(u32(p, 17))}  活动={u32(p, 21)}s  暂停={u32(p, 25)}s")


def p_exercise_op_id_req(p, out):
    if len(p) < 12:
        out.append("!! 请求不足12字节"); return
    out.append(f"operation_id=0x{u32(p, 0):08X}  exercise_id=0x{u64(p, 4):016X}")


def p_exercise_pause_resp(p, out):
    if len(p) < 26:
        out.append(f"!! PAUSE/RESUME成功响应须 26 字节，实际 {len(p)}"); return
    out.append(f"operation_id=0x{u32(p, 1):08X}  exercise_id=0x{u64(p, 5):016X}")
    out.append(f"状态={name_of(EXERCISE_STATES, p[13])}  变更时间={s_to_str(u32(p, 14))}")
    out.append(f"活动={u32(p, 18)}s  暂停={u32(p, 22)}s")


def p_confirm_start_resp(p, out):
    if len(p) < 25:
        out.append(f"!! CONFIRM_STATISTICS_START成功响应须 25 字节，实际 {len(p)}"); return
    out.append(f"operation_id=0x{u32(p, 1):08X}  exercise_id=0x{u64(p, 5):016X}")
    out.append(f"正式起点={s_to_str(u32(p, 13))}  timebase_generation={u32(p, 17)}")
    out.append(f"状态={name_of(EXERCISE_STATES, p[21])}  控制={name_of(CONTROL_MODES, p[22])}  "
               f"reserved={u16(p, 23)}")


def p_app_distance_req(p, out):
    if len(p) < 16:
        out.append(f"!! UPDATE_APP_DISTANCE 请求须 16 字节，实际 {len(p)}"); return
    out.append(f"exercise_id=0x{u64(p, 0):016X}  update_seq={u32(p, 8)}  "
               f"累计距离={_x100(u32(p, 12))}m")


def p_app_distance_get_req(p, out):
    if len(p) < 8:
        out.append(f"!! GET_APP_DISTANCE_STATE 请求须 8 字节，实际 {len(p)}"); return
    out.append(f"exercise_id=0x{u64(p, 0):016X}")


def p_app_distance_resp(p, out):
    if len(p) < 17:
        out.append(f"!! 距离状态成功响应须 17 字节，实际 {len(p)}"); return
    out.append(f"exercise_id=0x{u64(p, 1):016X}  update_seq={u32(p, 9)}  "
               f"累计距离={_x100(u32(p, 13))}m")


def p_auto_exercise_saved(p, out):
    if len(p) < 18:
        out.append(f"!! AUTO_EXERCISE_SAVED 须 18 字节，实际 {len(p)}"); return
    out.append(f"exercise_id=0x{u64(p, 0):016X}  sport={name_of(SPORT_TYPES, u16(p, 8))}")
    out.append(f"开始={s_to_str(u32(p, 10))}  结束={s_to_str(u32(p, 14))}")


def p_realtime_extension(p, out):
    if len(p) < 24:
        out.append(f"!! REALTIME_EXTENSION 公共头须 24 字节，实际 {len(p)}"); return
    dt, ver = u16(p, 0), p[2]
    spec = EXT_GROUPS.get(dt)
    mask = u32(p, 20)
    label = spec["name"] if spec else f"未知(0x{dt:04X})"
    out.append(f"data_type={label}  format_version={ver}  reserved={p[3]}")
    out.append(f"exercise_id=0x{u64(p, 4):016X}  extension_seq={u32(p, 12)}  "
               f"时间={s_to_str(u32(p, 16))}")
    out.append(f"field_validity_mask=0x{mask:08X}")
    if spec and len(p) != spec["payload_bytes"]:
        out.append(f"!! payload 须为 {spec['payload_bytes']} 字节，实际 {len(p)}")
    body = p[24:]
    parts = []
    if dt == 0x0001 and len(body) >= 8:
        if mask & 0x01:
            parts.append(f"实时步频={_x100(u32(body, 0))}步/分钟")
        if mask & 0x02:
            parts.append(f"实时步幅={u32(body, 4)}mm")
    elif dt == 0x0002 and len(body) >= 4:
        if mask & 0x01:
            parts.append(f"实时速度={_x100(u32(body, 0))}km/h")
    elif dt == 0x0003 and len(body) >= 4:
        if mask & 0x01:
            parts.append(f"实时划频={_x100(u16(body, 0))}次/分钟")
        if mask & 0x02:
            parts.append(f"平均SWOLF={_x100(u16(body, 2))}")
    else:
        parts.append(f"body={hexs(body)}")
    out.append("有效指标: " + (", ".join(parts) if parts else "无"))


def p_set_period_resp(p, out):
    if len(p) < 8:
        out.append(f"!! SET_MEASUREMENT_PERIOD 成功响应须 8 字节，实际 {len(p)}"); return
    out.append(f"operation_id=0x{u32(p, 1):08X}  type={name_of(MEASURE_TYPES, p[5])}  "
               f"生效周期={u16(p, 6)}min")


def p_set_ble_name_resp(p, out):
    if len(p) < 6:
        out.append("!! SET_BLE_NAME 响应过短"); return
    n = p[5]
    out.append(f"operation_id=0x{u32(p, 1):08X}  "
               f"生效名称({n}字节)=\"{bytes(p[6:6 + n]).decode('utf-8', 'replace')}\"")


def p_log_event(p, out):
    if len(p) < 16:
        out.append(f"!! LOG_EVENT 头须 16 字节，实际 {len(p)}"); return
    slen, mlen = p[13], u16(p, 14)
    src = bytes(p[16:16 + slen])
    msg = bytes(p[16 + slen:16 + slen + mlen])
    ts = u64(p, 4)
    out.append(f"log_seq={u32(p, 0)}  时间={'无有效UTC' if ts == 0 else ms_to_str(ts)}  "
               f"level={LOG_LEVELS.get(p[12], p[12])}")
    out.append(f"source=\"{src.decode('ascii', 'replace')}\"")
    out.append(f"message=\"{msg.decode('utf-8', 'replace')}\"")
    if 16 + slen + mlen != len(p):
        out.append(f"!! 长度应为 16+{slen}+{mlen}={16 + slen + mlen}，实际 {len(p)}")


SWIM_STROKES = {0: "未知", 1: "自由泳", 2: "蛙泳", 3: "混合"}
LOG_LEVELS = {1: "ERROR", 2: "WARN", 3: "INFO", 4: "DEBUG"}


def _x100(v):
    return f"{v / 100:.2f}"


def realtime_metric_parts(p):
    """102 字节 REALTIME_DATA：只输出 field_validity_mask 置位的指标。"""
    vm = u32(p, 20)
    sport = u16(p, 16)
    pace_unit = "秒/100米" if sport == 0x0004 else "秒/公里"
    parts = []

    def add(bit, text):
        if vm & (1 << bit):
            parts.append(text)

    add(0, f"心率={p[36]}bpm")
    add(1, f"最高心率={p[37]}bpm")
    add(2, f"卡路里={_x100(u32(p, 40))}kcal")
    add(3, f"步数={u32(p, 44)}")
    add(4, f"游泳趟数={u16(p, 52)}")
    add(5, f"划水次数={u16(p, 54)}")
    add(6, f"主泳姿={SWIM_STROKES.get(p[56], p[56])}")
    add(7, f"距离={_x100(u32(p, 48))}m")
    add(8, f"当前配速={_x100(u32(p, 60))}{pace_unit}")
    add(9, f"平均配速={_x100(u32(p, 64))}{pace_unit}")
    add(10, f"最快配速={_x100(u32(p, 68))}{pace_unit}")
    add(11, f"平均步幅={u32(p, 72)}mm")
    add(12, f"平均步频={_x100(u32(p, 76))}步/分钟")
    add(13, f"最大摄氧量={_x100(u16(p, 80))}ml/kg/min")
    add(14, f"平均速度={_x100(u32(p, 82))}km/h")
    add(15, f"累计爬升={_x100(u32(p, 86))}m")
    add(16, f"平均划频={_x100(u16(p, 90))}次/分钟")
    add(17, f"平均心率={p[92]}bpm")
    add(18, f"最低心率={p[93]}bpm")
    add(19, f"训练效果={_x100(u16(p, 94))}")
    add(20, f"恢复时间={u16(p, 96)}分钟")
    add(21, f"训练负荷={_x100(u32(p, 98))}")
    if vm & ~0x003FFFFF:
        parts.append(f"保留有效位=0x{vm & ~0x003FFFFF:08X}")
    return parts


def p_realtime_data(p, out):
    if len(p) != 102:
        out.append(f"!! REALTIME_DATA应为102字节, 实际{len(p)}"); return
    vm = u32(p, 20)
    sf = u16(p, 24)
    sfs = [n for bit, n in [(0, "佩戴状态有效"), (1, "已佩戴"), (2, "有丢样"),
                            (3, "时间不确定")] if sf & (1 << bit)]
    out.append(f"exercise_id=0x{u64(p, 0):016X}  sample_seq={u32(p, 8)}  "
               f"时间={s_to_str(u32(p, 12))}")
    out.append(f"sport={name_of(SPORT_TYPES, u16(p, 16))}  状态={name_of(EXERCISE_STATES, p[18])}  "
               f"validity_mask=0x{vm:08X}")
    out.append(f"活动={u32(p, 28)}s  暂停={u32(p, 32)}s  "
               f"status_flags=0x{sf:04X}({'/'.join(sfs) if sfs else '无'})")
    parts = realtime_metric_parts(p)
    out.append("有效指标: " + (", ".join(parts) if parts else "无"))


def p_exercise_caps_req(p, out):
    if len(p) < 3:
        out.append("!! 请求不足3字节"); return
    out.append(f"start_index={u16(p, 0)}  max_items={p[2]}")


def p_exercise_caps_resp(p, out):
    if len(p) < 18:
        out.append("!! 能力响应前缀不足18字节"); return
    out.append(f"capability_version={u16(p, 1)}  max_stored={u16(p, 3)}  "
               f"max_duration={u32(p, 5)}s  disconnect_grace={u32(p, 9)}s")
    total, nxt, cnt = u16(p, 13), u16(p, 15), p[17]
    out.append(f"total_items={total}  next_index={'结束(0xFFFF)' if nxt == 0xFFFF else nxt}  item_count={cnt}")
    need = 18 + cnt * 16
    if len(p) < need:
        out.append(f"!! 条目字节不足: 需要{need}, 实际{len(p)}"); return
    for i in range(cnt):
        o = 18 + i * 16
        ff = u16(p, o + 2)
        ffs = [n for bit, n in [(0, "APP_START"), (1, "AUTO_RECOGNITION"), (2, "PAUSE_RESUME"),
                                (3, "DEVICE_DISTANCE"), (4, "POOL_LENGTH"), (5, "APP_DISTANCE_INPUT"),
                                (6, "REALTIME_EXT"), (7, "AUTO_START_CONFIRM")] if ff & (1 << bit)]
        out.append(f"  [{i}] {name_of(SPORT_TYPES, u16(p, o))}  feature=0x{ff:04X}({'/'.join(ffs)})  "
                   f"profile_mask=0x{u16(p, o + 4):04X}  realtime_mask=0x{u32(p, o + 8):08X}")


def p_offline_open_req(p, out):
    if len(p) < 7:
        out.append("!! OPEN请求过短"); return
    out.append(f"dataset={name_of(DATASETS, p[0])}  app_epoch={u16(p, 1)}  "
               f"expected_seq={u32(p, 3)}{'(全量)' if u32(p, 3) == 0 else ''}")


def p_offline_open_resp(p, out):
    if len(p) < 26:
        out.append("!! OPEN响应不足26字节"); return
    fl = u16(p, 24)
    out.append(f"dataset={name_of(DATASETS, p[1])}  epoch={u16(p, 2)}  snapshot_id={u32(p, 4)}")
    out.append(f"oldest_seq={u32(p, 8)}  snapshot_end={u32(p, 12)}  first_seq={u32(p, 16)}  "
               f"候选记录数={u32(p, 20)}")
    out.append(f"flags=0x{fl:04X}({'仍有数据' if fl & 1 else ''}"
               f"{' 起点前有GAP' if fl & 2 else ''}{' 已到快照尾' if fl & 4 else ''})")


def p_offline_pull_req(p, out):
    if len(p) < 9:
        out.append("!! PULL请求过短"); return
    out.append(f"dataset={name_of(DATASETS, p[0])}  snapshot_id={u32(p, 1)}  start_seq={u32(p, 5)}")


def p_offline_pull_resp(p, out):
    if len(p) >= 31:
        fl = u16(p, 25)
        out.append(f"dataset={name_of(DATASETS, p[1])}  snapshot_id={u32(p, 2)}  page_id={u16(p, 6)}")
        out.append(f"first_seq={u32(p, 8)}  last_seq={u32(p, 12)}  next_seq={u32(p, 16)}")
        out.append(f"记录数={u16(p, 20)}  页字节={u16(p, 22)}  分片数={p[24]}  "
                   f"flags=0x{fl:04X}  page_crc32=0x{u32(p, 27):08X}")
    elif len(p) >= 12:
        out.append(f"GAP恢复: dataset={name_of(DATASETS, p[1])}  next_seq={u32(p, 6)}")


def p_offline_data(p, out):
    if len(p) < 9:
        out.append("!! DATA事件过短"); return
    # 分片还不是一条完整记录。数值在本页拼完后按指标列出，这里不展开字段。
    return


def p_ble_name_resp(p, out):
    if len(p) < 2:
        out.append("!! 名称响应过短"); return
    n = p[1]
    out.append(f"BLE名称({n}字节)=\"{bytes(p[2:2 + n]).decode('utf-8', 'replace')}\"")


def p_set_ble_name_req(p, out):
    if len(p) < 5:
        out.append("!! SET_BLE_NAME请求过短"); return
    n = p[4]
    out.append(f"operation_id=0x{u32(p, 0):08X}  新名称=\"{bytes(p[5:5 + n]).decode('utf-8', 'replace')}\""
               f"{'(恢复默认)' if n == 0 else ''}")


def p_generic_op_req(p, out):
    if len(p) >= 4:
        out.append(f"operation_id=0x{u32(p, 0):08X}")


def p_generic_op_resp(p, out):
    if len(p) >= 5:
        out.append(f"operation_id=0x{u32(p, 1):08X}")


# ---------------------------------------------------------------------------
# 大白话摘要 (给非专业人员看的"指令介绍")
# ---------------------------------------------------------------------------

CMD_CN = {
    (0x01, 0x01): "查询设备能力", (0x01, 0x02): "查询设备签名", (0x01, 0x03): "查询设备电量",
    (0x01, 0x04): "同步手机时间", (0x01, 0x05): "查询设备信息", (0x01, 0x06): "查询用户资料",
    (0x01, 0x07): "设置用户资料", (0x01, 0x10): "请求认证随机数", (0x01, 0x11): "设置认证密钥",
    (0x01, 0x12): "提交认证证明", (0x01, 0x20): "恢复出厂设置(清除用户数据)",
    (0x01, 0x21): "重启设备", (0x01, 0x40): "电量变化", (0x01, 0x41): "日志",
    (0x10, 0x01): "打开离线数据同步", (0x10, 0x02): "拉取离线数据页",
    (0x10, 0x03): "关闭离线数据同步", (0x10, 0x40): "离线数据分片",
    (0x20, 0x01): "开始测量", (0x20, 0x02): "停止测量", (0x20, 0x03): "查询测量状态",
    (0x20, 0x04): "查询测量周期", (0x20, 0x05): "设置测量周期", (0x20, 0x40): "测量数据",
    (0x50, 0x01): "查询充电仓电量", (0x50, 0x02): "开始找设备", (0x50, 0x03): "停止找设备",
    (0x50, 0x04): "查询蓝牙名称", (0x50, 0x05): "设置蓝牙名称",
    (0x50, 0x06): "查询电源模式", (0x50, 0x07): "设置电源模式",
    (0x50, 0x08): "查询佩戴状态", (0x50, 0x09): "查询iBeacon配置",
    (0x50, 0x0A): "设置iBeacon配置", (0x50, 0x0B): "查询睡眠状态",
    (0x50, 0x40): "充电仓电量变化", (0x50, 0x41): "佩戴状态变化",
    (0x50, 0x42): "找设备结束", (0x50, 0x43): "睡眠状态变化",
    (0x70, 0x01): "查询运动能力", (0x70, 0x02): "查询运动状态",
    (0x70, 0x03): "查询运动自动识别配置", (0x70, 0x04): "设置运动自动识别配置",
    (0x70, 0x05): "配置实时数据扩展", (0x70, 0x06): "查询实时扩展能力",
    (0x70, 0x10): "开始运动", (0x70, 0x11): "暂停运动", (0x70, 0x12): "恢复运动",
    (0x70, 0x13): "结束运动", (0x70, 0x14): "更新App距离", (0x70, 0x15): "查询App距离状态",
    (0x70, 0x16): "确认统计开始", (0x70, 0x40): "运动状态变化", (0x70, 0x41): "运动实时数据",
    (0x70, 0x42): "自动识别运动已保存", (0x70, 0x43): "实时扩展数据",
}


def opcode_name(domain, opcode):
    named = OPCODES.get(domain, {}).get(opcode)
    if named:
        return named
    if domain in UNPUBLISHED_DOMAINS:
        return f"opcode=0x{opcode:02X}（未公开）"
    return f"opcode=0x{opcode:02X}"


def cmd_label(domain, opcode):
    """测试可见名称：公开指令用中文；未公开域只用 域名(0xDD)/opcode=0xOO。"""
    if (domain, opcode) in CMD_CN:
        return CMD_CN[(domain, opcode)]
    if domain in UNPUBLISHED_DOMAINS:
        name, _chap = UNPUBLISHED_DOMAINS[domain]
        return f"{name}(0x{domain:02X})/opcode=0x{opcode:02X} 未公开"
    return opcode_name(domain, opcode)


def unpublished_note(domain, opcode):
    name, chap = UNPUBLISHED_DOMAINS.get(domain, (f"0x{domain:02X}", "手册"))
    return (f"域ID={name}(0x{domain:02X})  opcode=0x{opcode:02X}  "
            f"{chap}未公布指令名；消费者固件不得实现，回 NOT_SUPPORTED 为预期")


def p_unpublished_req(p, out):
    """未公开域请求：只原样展示字节，不杜撰字段名。"""
    if p and p[0] == len(p) - 1:
        body = bytes(p[1:])
        if body and all(32 <= b < 127 for b in body):
            out.append(f"原始payload可读文本({p[0]}字节): {body.decode('ascii')}")
            out.append("（不是公开字段名，仅按字节显示）")
            return
    if p:
        out.append(f"payload={hexs(p)}")


def cn_only(text):
    """从 'HR(心率)' 中取括号内中文; 无括号则原样返回"""
    m = re.search(r"\(([^)]*)\)", text)
    return m.group(1) if m else text


def s_battery(p, base):
    if len(p) < base + 24:
        return ""
    parts = [f"电量 {p[base]}%", cn_only(name_of(CHARGE_STATUS, p[base + 1]))]
    fl = u16(p, base + 2)
    if fl & 0x02:
        parts.append(f"电压 {u32(p, base + 4)}mV")
    if fl & 0x08:
        parts.append(f"温度 {i32(p, base + 12) / 10}℃")
    return "，".join(parts)


def s_case(p, base):
    if len(p) < base + 8:
        return ""
    pct = p[base + 5]
    return f"仓电量 {'未知' if pct == 0xFF else str(pct) + '%'}"


def s_device_info(p):
    if len(p) < 41:
        return ""
    mac = ":".join(f"{x:02X}" for x in p[1:7])
    fw = bytes(p[20:30]).decode("ascii", "replace").strip("\x00 ").strip()
    o, vals = 40, []
    for _ in range(3):
        if o >= len(p):
            break
        n = p[o]; o += 1
        vals.append(bytes(p[o:o + n]).decode("ascii", "replace")); o += n
    sn = vals[2] if len(vals) > 2 else ""
    return f"固件 \"{fw}\"，SN=\"{sn}\"，MAC={mac}"


def s_ex_state(p, base):
    if len(p) < base + 48:
        return ""
    st = p[base + 11]
    if st == 0:
        return "当前空闲，无运动"
    return (f"{cn_only(name_of(SPORT_TYPES, u16(p, base + 8)))}，"
            f"{cn_only(name_of(EXERCISE_STATES, st))}，已运动 {u32(p, base + 32)} 秒")


def s_realtime(p):
    if len(p) != 102:
        return ""
    parts = [x.replace("=", " ") for x in realtime_metric_parts(p)]
    dur = f"已运动 {u32(p, 28)} 秒"
    paused = u32(p, 32)
    if paused:
        dur += f"，暂停 {paused} 秒"
    parts.append(dur)
    return "，".join(parts)


def s_offline_data(p):
    if len(p) < 5:
        return "离线数据分片"
    idx, cnt = p[3] + 1, p[4]
    last = "，本页最后一片" if cnt and idx == cnt else ""
    return f"{cn_only(name_of(DATASETS, p[0]))} 分片 {idx}/{cnt}{last}"


def s_find_req(p):
    sigs = [n for bit, n in [(0, "LED"), (1, "振动"), (2, "蜂鸣器")] if p[0] & (1 << bit)]
    return f"{'/'.join(sigs) if sigs else '默认方式'}，{u16(p, 1)} 秒"


def s_measure_start_resp(p):
    if p[0] == 0 and len(p) >= 7:
        return (f"{measure_cn(p[3])}已开始（ID={u16(p, 1)}，预算{u16(p, 5)}秒）；"
                "数值在 DATA 事件，本帧没有")
    if len(p) >= 3:
        return f"原因：{cn_only(name_of(RESULT_ORIGINS, p[1]))}检查未通过"
    return ""


def s_measure_stop_resp(p):
    if not p or p[0] != 0 or len(p) < 4:
        return ""
    return (f"测量ID={u16(p, 1)} 已结束，{name_of(MEASURE_STATES, p[3])}；"
            "本帧无心率/HRV/血氧值")


def s_measure_get_state_resp(p):
    if not p or p[0] != 0 or len(p) < 10:
        return ""
    seq = u32(p, 6)
    extra = "，尚未有 DATA" if seq == 0 else f"，已出样本 seq={seq}"
    return (f"{measure_cn(p[3])} ID={u16(p, 1)} "
            f"{name_of(MEASURE_STATES, p[5])}{extra}；本帧无测量值")


def s_measure_data(p):
    """测量 DATA 第一行：有效就报数值，无效就说清为什么不能当结果用。"""
    if len(p) < 19:
        return ""
    mtype = p[6]
    state = p[7]
    sb = u16(p, 8)
    valid = bool(sb & 0x01)
    vlen = p[18]
    val = bytes(p[19:19 + vlen])
    reasons = []
    if sb & 0x08:
        reasons.append("信号弱")
    if sb & 0x04:
        reasons.append("有体动")
    if not (sb & 0x02):
        reasons.append("接触未确认")
    origin = (sb >> 4) & 0x0F
    if origin == 4:
        reasons.append("算法未交出可用结果")
    elif origin == 5:
        reasons.append("传感器失败")
    elif origin == 3:
        reasons.append("测量模块未就绪")
    elif origin and origin != 0:
        reasons.append(cn_only(name_of(RESULT_ORIGINS, origin)))
    why = "，".join(reasons) if reasons else "VALID=0"
    ended = "测量结束，" if state == 4 else ""

    if mtype == 1:
        name = "心率"
        if valid and len(val) >= 2:
            return f"{name} {u16(val, 0)} bpm"
        return f"{ended}{name}无有效值（{why}；不是心率为0）"
    if mtype == 2:
        name = "心率变异性"
        if valid and len(val) >= 6:
            return (f"{name} {u32(val, 2) / 100:.2f} ms"
                    f"（RMSSD=相邻心跳间隔波动，窗口{u16(val, 0)}秒）")
        return (f"{ended}{name}无有效值（{why}；"
                f"RMSSD=相邻心跳间隔波动，本包未算出，0是占位不是测到0）")
    if mtype == 3:
        name = "血氧"
        if valid and len(val) >= 2:
            return f"{name} {val[1]}%"
        return f"{ended}{name}无有效值（{why}；不是血氧为0）"
    if mtype == 4:
        name = "体温"
        if valid and len(val) >= 2:
            return f"{name} {i16(val, 0) / 100:.2f}℃"
        return f"{ended}{name}无有效值（{why}）"
    if mtype == 5:
        name = "血压"
        if valid and len(val) >= 4:
            return f"{name} {u16(val, 0)}/{u16(val, 2)} mmHg"
        return f"{ended}{name}无有效值（{why}）"
    return cn_only(name_of(MEASURE_TYPES, mtype))


# (domain, opcode, kind) -> 摘要细节函数(payload) -> str; kind: req/resp/event
SUMMARIES = {
    (0x01, 0x01, "resp"): s_caps_resp,
    (0x01, 0x02, "resp"): lambda p: (f"来源 \"{bytes(p[67:67 + p[66]]).decode('ascii')}\""
                                      if len(p) >= 67 and _printable(bytes(p[67:67 + p[66]]))
                                      else f"来源(二进制)={hexs(p[67:67 + p[66]])}" if len(p) >= 67 else ""),
    (0x01, 0x03, "resp"): lambda p: s_battery(p, 1),
    (0x01, 0x04, "req"): lambda p: ms_to_str(u64(p, 0)),
    (0x01, 0x04, "resp"): lambda p: f"设备时间已更新为 {ms_to_str(u64(p, 5))}",
    (0x01, 0x05, "resp"): s_device_info,
    (0x01, 0x06, "resp"): lambda p: s_user_profile(p, 1),
    (0x01, 0x07, "req"): s_set_user_profile_req,
    (0x01, 0x07, "resp"): lambda p: s_user_profile(p, 5),
    (0x01, 0x10, "resp"): lambda p: f"认证结果：{cn_only(name_of(AUTH_RESULTS, p[1]))}",
    (0x01, 0x11, "resp"): lambda p: f"密钥设置：{cn_only(name_of(AUTH_RESULTS, p[1]))}",
    (0x01, 0x12, "resp"): lambda p: f"认证结果：{cn_only(name_of(AUTH_RESULTS, p[1]))}",
    (0x01, 0x40, "event"): lambda p: s_battery(p, 0),
    (0x10, 0x01, "req"): lambda p: f"{cn_only(name_of(DATASETS, p[0]))}数据，{'全量' if u32(p, 3) == 0 else '增量'}同步",
    (0x10, 0x01, "resp"): lambda p: f"待传 {u32(p, 20)} 条记录",
    (0x10, 0x02, "req"): lambda p: f"起始序号 {u32(p, 5)}",
    (0x10, 0x02, "resp"): lambda p: (
        f"本页 {u16(p, 20)} 条记录" if len(p) >= 31 and p[0] == 0
        else (f"从序号 {u32(p, 6)} 继续" if len(p) >= 12 else "")),
    (0x10, 0x40, "event"): s_offline_data,
    (0x20, 0x01, "req"): lambda p: (
        f"{measure_cn(p[0])}，{cn_only(name_of(REPORT_MODES, p[1]))}模式，"
        + ("设备默认预算" if u16(p, 2) == 0 else f"{u16(p, 2)} 秒")
    ) if len(p) >= 4 else "请求过短",
    (0x20, 0x01, "resp"): s_measure_start_resp,
    (0x20, 0x02, "req"): lambda p: f"测量ID={u16(p, 0)}",
    (0x20, 0x02, "resp"): s_measure_stop_resp,
    (0x20, 0x03, "req"): lambda p: (
        "measurement_id=0（主动测量没有用 0 查当前）" if len(p) >= 2 and u16(p, 0) == 0
        else (f"测量ID={u16(p, 0)}" if len(p) >= 2 else "请求过短")
    ),
    (0x20, 0x03, "resp"): s_measure_get_state_resp,
    (0x20, 0x04, "req"): lambda p: cn_only(name_of(MEASURE_TYPES, p[0])),
    (0x20, 0x04, "resp"): lambda p: f"{cn_only(name_of(MEASURE_TYPES, p[1]))}：当前 {u16(p, 2)} 分钟（范围 {u16(p, 4)}~{u16(p, 6)}）",
    (0x20, 0x05, "req"): lambda p: f"{cn_only(name_of(MEASURE_TYPES, p[4]))}周期设为 {u16(p, 5)} 分钟",
    (0x20, 0x05, "resp"): lambda p: (
        f"{cn_only(name_of(MEASURE_TYPES, p[5]))}周期 {u16(p, 6)} 分钟" if len(p) >= 8 else ""),
    (0x01, 0x41, "event"): lambda p: (
        bytes(p[16 + p[13]:16 + p[13] + u16(p, 14)]).decode("utf-8", "replace")[:40]
        if len(p) >= 16 else ""),
    (0x20, 0x40, "event"): s_measure_data,
    (0x50, 0x01, "resp"): lambda p: s_case(p, 1),
    (0x50, 0x02, "req"): s_find_req,
    (0x50, 0x02, "resp"): lambda p: f"已开始（find_id={u16(p, 1)}，{u16(p, 4)} 秒）",
    (0x50, 0x03, "req"): lambda p: f"find_id={u16(p, 0)}",
    (0x50, 0x03, "resp"): lambda p: f"已停止（{cn_only(name_of(FIND_END_REASONS, p[3]))}）",
    (0x50, 0x04, "resp"): lambda p: f"名称 \"{bytes(p[2:2 + p[1]]).decode('utf-8', 'replace')}\"",
    (0x50, 0x05, "req"): lambda p: f"新名称 \"{bytes(p[5:5 + p[4]]).decode('utf-8', 'replace')}\"",
    (0x50, 0x05, "resp"): lambda p: (
        f"名称 \"{bytes(p[6:6 + p[5]]).decode('utf-8', 'replace')}\"" if len(p) >= 6 else ""),
    (0x50, 0x06, "resp"): lambda p: cn_only(name_of(POWER_MODES, p[1])),
    (0x50, 0x07, "req"): lambda p: f"目标：{cn_only(name_of(POWER_MODES, p[4]))}",
    (0x50, 0x07, "resp"): lambda p: f"已切换：{cn_only(name_of(POWER_MODES, p[5]))}",
    (0x50, 0x08, "resp"): lambda p: cn_only(name_of(WEAR_STATES, p[1])),
    (0x50, 0x09, "resp"): lambda p: s_ibeacon_cfg(p[1], p[2], p[3]) if len(p) >= 4 else "响应过短",
    (0x50, 0x0A, "req"): lambda p: s_ibeacon_cfg(p[4], p[5]) if len(p) >= 6 else "请求过短",
    (0x50, 0x0A, "resp"): lambda p: s_ibeacon_cfg(p[5], p[6], p[7]) if len(p) >= 8 else "响应过短",
    (0x50, 0x0B, "resp"): lambda p: cn_only(name_of(SLEEP_STATES, p[1])),
    (0x50, 0x40, "event"): lambda p: s_case(p, 0),
    (0x50, 0x41, "event"): lambda p: f"变为「{cn_only(name_of(WEAR_STATES, p[0]))}」",
    (0x50, 0x42, "event"): lambda p: cn_only(name_of(FIND_END_REASONS, p[2])),
    (0x50, 0x43, "event"): lambda p: f"变为「{cn_only(name_of(SLEEP_STATES, p[0]))}」",
    (0x70, 0x01, "resp"): lambda p: f"共 {u16(p, 13)} 项运动能力，本页 {p[17]} 项" if len(p) >= 18 else "",
    (0x70, 0x03, "resp"): lambda p: s_recog_cfg(u16(p, 1), p[3], p[4]) if len(p) >= 5 else "响应过短",
    (0x70, 0x04, "req"): lambda p: (
        f"自动识别{'开启' if p[4] == 1 else '关闭' if p[4] == 0 else f'enabled={p[4]}'}"
        + ("  校验通过" if p[4] in (0, 1) and p[5] == 0 and len(p) == 6 else "  !! 参数非法")
    ) if len(p) >= 6 else "请求过短",
    (0x70, 0x04, "resp"): lambda p: s_recog_cfg(u16(p, 5), p[7], p[8]) if len(p) >= 9 else "响应过短",
    (0x70, 0x05, "req"): lambda p: (
        f"{cn_only(name_of(SPORT_TYPES, u16(p, 0)))}/"
        f"{(EXT_GROUPS.get(u16(p, 2)) or {}).get('name', f'0x{u16(p,2):04X}')}"
        f"{'开启' if p[5] == 1 else '关闭'}"
    ) if len(p) >= 6 else "请求过短",
    (0x70, 0x05, "resp"): lambda p: (
        f"{cn_only(name_of(SPORT_TYPES, u16(p, 1)))}/"
        f"{(EXT_GROUPS.get(u16(p, 3)) or {}).get('name', f'0x{u16(p,3):04X}')}"
        f"{'已开启' if p[6] == 1 else '已关闭'}"
    ) if len(p) >= 7 else "响应过短",
    (0x70, 0x06, "req"): s_ext_caps_req,
    (0x70, 0x06, "resp"): s_ext_caps_resp,
    (0x70, 0x02, "req"): lambda p: "查询当前状态" if u64(p, 0) == 0 else "精确查询指定运动",
    (0x70, 0x02, "resp"): lambda p: s_ex_state(p, 1),
    (0x70, 0x10, "req"): lambda p: cn_only(name_of(SPORT_TYPES, u16(p, 4))),
    (0x70, 0x10, "resp"): lambda p: f"运动ID=0x{u64(p, 5):016X}",
    (0x70, 0x11, "req"): lambda p: f"运动ID=0x{u64(p, 4):016X}",
    (0x70, 0x12, "req"): lambda p: f"运动ID=0x{u64(p, 4):016X}",
    (0x70, 0x11, "resp"): lambda p: (
        f"{cn_only(name_of(EXERCISE_STATES, p[13]))}，活动 {u32(p, 18)} 秒" if len(p) >= 26 else ""),
    (0x70, 0x12, "resp"): lambda p: (
        f"{cn_only(name_of(EXERCISE_STATES, p[13]))}，活动 {u32(p, 18)} 秒" if len(p) >= 26 else ""),
    (0x70, 0x13, "req"): lambda p: f"原因：{cn_only(name_of(END_REASONS, p[12]))}",
    (0x70, 0x13, "resp"): lambda p: f"{cn_only(name_of(EXERCISE_STATES, p[13]))}，活动 {u32(p, 21)} 秒",
    (0x70, 0x14, "req"): lambda p: (
        f"序号 {u32(p, 8)}，累计 {_x100(u32(p, 12))}m" if len(p) >= 16 else ""),
    (0x70, 0x14, "resp"): lambda p: (
        f"已受理序号 {u32(p, 9)}，累计 {_x100(u32(p, 13))}m" if len(p) >= 17 else ""),
    (0x70, 0x15, "resp"): lambda p: (
        f"最近序号 {u32(p, 9)}，累计 {_x100(u32(p, 13))}m" if len(p) >= 17 else ""),
    (0x70, 0x16, "resp"): lambda p: (
        f"正式起点 {s_to_str(u32(p, 13))}" if len(p) >= 25 else ""),
    (0x70, 0x42, "event"): lambda p: (
        f"{cn_only(name_of(SPORT_TYPES, u16(p, 8)))}，{s_to_str(u32(p, 10))}–{s_to_str(u32(p, 14))}"
        if len(p) >= 18 else ""),
    (0x70, 0x43, "event"): lambda p: (
        (EXT_GROUPS.get(u16(p, 0)) or {}).get("name", f"0x{u16(p, 0):04X}") if len(p) >= 2 else ""),
    (0x70, 0x40, "event"): lambda p: s_ex_state(p, 0),
    (0x70, 0x41, "event"): s_realtime,
}


def offline_open_no_data(direction, domain, opcode, flags, payload):
    """健康/运动等离线 OPEN 返回 NO_DATA：没有新数据，不是同步失败。"""
    if direction == "rx" or flags & 0x02:
        return False
    return domain == 0x10 and opcode == 0x01 and bool(payload) and payload[0] == 10


def offline_no_data_summary(payload):
    ds = payload[1] if len(payload) > 1 else 0
    name = {1: "健康历史", 2: "算法中间数据", 3: "诊断记录", 4: "运动历史"}.get(ds, "离线数据")
    return f"▲ 设备回复：{name}当前没有需要同步的数据，本次同步正常结束"


def plain_summary(direction, domain, opcode, flags, payload):
    """生成一行大白话: 'APP下发：查询设备电量' / '设备回复：电量 47%，充电中'"""
    is_resp = bool(flags & 0x01)
    is_event = bool(flags & 0x02)
    is_err = bool(flags & 0x10)
    if offline_open_no_data(direction, domain, opcode, flags, payload):
        return offline_no_data_summary(payload)
    base = cmd_label(domain, opcode)
    if direction == "rx":
        kind, head = "req", "▼ APP下发"
    elif is_event:
        kind, head = "event", "▲ 设备上报"
    elif is_err:
        kind, head = "resp", "▲ 设备回复(失败)"
    else:
        kind, head = "resp", "▲ 设备回复"
    detail = ""
    fn = SUMMARIES.get((domain, opcode, kind))
    if fn and payload:
        try:
            detail = fn(payload) or ""
        except Exception:
            detail = ""
    if direction == "rx":
        text = f"{head}：{base}"
        if detail:
            text += f"（{detail}）"
    elif not is_event and payload and payload[0] != 0:
        text = f"{head}：{base}——{status_summary(payload[0])}"
        if detail:
            text += f"（{detail}）"
    else:
        text = f"{head}：{detail}" if detail else f"{head}：{base}"
    return text


def view_category(summary):
    """右边筛选用。对不上这四类的内容始终显示。"""
    if "固件日志" in summary or "固件实时日志" in summary:
        return "cat_fw"
    if "APP下发" in summary:
        return "cat_app"
    if "设备上报" in summary:
        return "cat_event"
    if "设备回复" in summary:
        return "cat_reply"
    return None


def view_tags(level, summary):
    cat = view_category(summary)
    if not cat:
        return level
    return (level, cat)


# (domain, opcode, kind) -> 解析函数; kind: req/resp/any
PARSERS = {
    (0x01, 0x01, "resp"): p_caps_resp,
    (0x01, 0x02, "resp"): p_signature_resp,
    (0x01, 0x03, "resp"): lambda p, out: p_battery_snapshot(p, out, 1),
    (0x01, 0x04, "req"): p_time_sync_req,
    (0x01, 0x04, "resp"): p_time_sync_resp,
    (0x01, 0x05, "resp"): p_device_info_resp,
    (0x01, 0x06, "resp"): p_get_user_profile_resp,
    (0x01, 0x07, "req"): p_set_user_profile_req,
    (0x01, 0x07, "resp"): p_set_user_profile_resp,
    (0x01, 0x10, "resp"): p_auth_resp,
    (0x01, 0x11, "req"): lambda p, out: out.append(f"auth_key[16]={hexs(p[0:16])}"),
    (0x01, 0x11, "resp"): p_auth_resp,
    (0x01, 0x12, "req"): lambda p, out: out.append(f"encrypted_nonce[16]={hexs(p[0:16])}"),
    (0x01, 0x12, "resp"): p_auth_resp,
    (0x01, 0x20, "req"): p_generic_op_req,
    (0x01, 0x20, "resp"): p_generic_op_resp,
    (0x01, 0x21, "req"): p_generic_op_req,
    (0x01, 0x21, "resp"): p_generic_op_resp,
    (0x01, 0x40, "any"): lambda p, out: p_battery_snapshot(p, out, 0),
    (0x01, 0x41, "any"): p_log_event,
    (0x10, 0x01, "req"): p_offline_open_req,
    (0x10, 0x01, "resp"): p_offline_open_resp,
    (0x10, 0x02, "req"): p_offline_pull_req,
    (0x10, 0x02, "resp"): p_offline_pull_resp,
    (0x10, 0x40, "any"): p_offline_data,
    (0x20, 0x01, "req"): p_measure_start_req,
    (0x20, 0x02, "req"): lambda p, out: out.append(f"measurement_id={u16(p, 0)}"),
    (0x20, 0x02, "resp"): p_measure_stop_resp,
    (0x20, 0x03, "req"): p_measure_get_state_req,
    (0x20, 0x03, "resp"): p_measure_get_state_resp,
    (0x20, 0x04, "req"): lambda p, out: out.append(f"type={name_of(MEASURE_TYPES, p[0])}"),
    (0x20, 0x04, "resp"): p_measure_period_resp,
    (0x20, 0x05, "req"): lambda p, out: (out.append(f"operation_id=0x{u32(p, 0):08X}"),
                                          out.append(f"type={name_of(MEASURE_TYPES, p[4])}  "
                                                     f"周期={u16(p, 5)}min")),
    (0x20, 0x05, "resp"): p_set_period_resp,
    (0x20, 0x40, "any"): p_measure_data,
    (0x50, 0x01, "resp"): lambda p, out: p_case_battery(p, out, 1),
    (0x50, 0x02, "req"): p_find_start_req,
    (0x50, 0x02, "resp"): p_find_start_resp,
    (0x50, 0x03, "req"): lambda p, out: out.append(f"find_id={u16(p, 0)}"),
    (0x50, 0x03, "resp"): lambda p, out: out.append(
        f"find_id={u16(p, 1)}  end_reason={name_of(FIND_END_REASONS, p[3])}"),
    (0x50, 0x04, "resp"): p_ble_name_resp,
    (0x50, 0x05, "req"): p_set_ble_name_req,
    (0x50, 0x05, "resp"): p_set_ble_name_resp,
    (0x50, 0x06, "resp"): lambda p, out: out.append(f"power_mode={name_of(POWER_MODES, p[1])}"),
    (0x50, 0x07, "req"): lambda p, out: (out.append(f"operation_id=0x{u32(p, 0):08X}"),
                                          out.append(f"目标模式={name_of(POWER_MODES, p[4])}")),
    (0x50, 0x07, "resp"): lambda p, out: out.append(
        f"operation_id=0x{u32(p, 1):08X}  accepted_mode={name_of(POWER_MODES, p[5])}"),
    (0x50, 0x08, "resp"): lambda p, out: p_wear_state(p, out, 1),
    (0x50, 0x09, "resp"): p_get_ibeacon_resp,
    (0x50, 0x0A, "req"): p_set_ibeacon_req,
    (0x50, 0x0A, "resp"): p_set_ibeacon_resp,
    (0x50, 0x0B, "resp"): lambda p, out: p_sleep_state(p, out, 1),
    (0x50, 0x40, "any"): lambda p, out: p_case_battery(p, out, 0),
    (0x50, 0x41, "any"): lambda p, out: p_wear_state(p, out, 0),
    (0x50, 0x42, "any"): lambda p, out: out.append(
        f"find_id={u16(p, 0)}  end_reason={name_of(FIND_END_REASONS, p[2])}"),
    (0x50, 0x43, "any"): lambda p, out: p_sleep_state(p, out, 0),
    (0x70, 0x01, "req"): p_exercise_caps_req,
    (0x70, 0x01, "resp"): p_exercise_caps_resp,
    (0x70, 0x02, "req"): lambda p, out: out.append(
        f"exercise_id=0x{u64(p, 0):016X}{'(查询当前状态)' if u64(p, 0) == 0 else '(精确查询)'}"),
    (0x70, 0x02, "resp"): lambda p, out: p_exercise_state(p, out, 1),
    (0x70, 0x03, "resp"): p_get_recognition_resp,
    (0x70, 0x04, "req"): p_set_recognition_req,
    (0x70, 0x04, "resp"): p_set_recognition_resp,
    (0x70, 0x05, "req"): p_config_ext_req,
    (0x70, 0x05, "resp"): p_config_ext_resp,
    (0x70, 0x06, "req"): p_get_ext_caps_req,
    (0x70, 0x06, "resp"): p_get_ext_caps_resp,
    (0x70, 0x10, "req"): p_exercise_start_req,
    (0x70, 0x10, "resp"): p_exercise_start_resp,
    (0x70, 0x11, "req"): p_exercise_op_id_req,
    (0x70, 0x11, "resp"): p_exercise_pause_resp,
    (0x70, 0x12, "req"): p_exercise_op_id_req,
    (0x70, 0x12, "resp"): p_exercise_pause_resp,
    (0x70, 0x13, "req"): p_exercise_stop_req,
    (0x70, 0x13, "resp"): p_exercise_stop_resp,
    (0x70, 0x14, "req"): p_app_distance_req,
    (0x70, 0x14, "resp"): p_app_distance_resp,
    (0x70, 0x15, "req"): p_app_distance_get_req,
    (0x70, 0x15, "resp"): p_app_distance_resp,
    (0x70, 0x16, "req"): p_exercise_op_id_req,
    (0x70, 0x16, "resp"): p_confirm_start_resp,
    (0x70, 0x40, "any"): lambda p, out: p_exercise_state(p, out, 0),
    (0x70, 0x41, "any"): p_realtime_data,
    (0x70, 0x42, "any"): p_auto_exercise_saved,
    (0x70, 0x43, "any"): p_realtime_extension,
}


def parse_frame(direction, data):
    """返回 (大白话摘要, 协议标题, 详情行列表, 级别) 级别: rx/tx/err/event"""
    if len(data) < 8:
        return None
    if data[0] != 0xAF or data[1] != 0x01:
        return None
    domain, opcode, flags = data[2], data[3], data[4]
    rid = u16(data, 5)
    plen = data[7]
    payload = data[8:]
    dname = DOMAINS.get(domain, f"未知域(0x{domain:02X})")
    oname = opcode_name(domain, opcode)
    is_resp = bool(flags & 0x01)
    is_event = bool(flags & 0x02)
    is_err = bool(flags & 0x10)
    kind = "resp" if is_resp else ("any" if is_event else "req")

    summary = plain_summary(direction, domain, opcode, flags, payload)
    proto = f"{dname}/{oname}  request_id=0x{rid:04X}"
    lines = [f"flags=0x{flags:02X}({flags_str(flags)})  payload_len={plen}(实际{len(payload)})"
             + ("  !! 长度不符" if plen != len(payload) else "")]
    if domain in UNPUBLISHED_DOMAINS:
        lines.append(unpublished_note(domain, opcode))
    if is_resp and payload:
        st = payload[0]
        lines.append(format_status(st))
        if is_err and len(payload) <= 1:
            parser = None  # 错误响应仅 status 时不再解析后续字段
        elif (domain, opcode) == (0x20, 0x01):
            p_measure_start_resp(payload, lines, is_err)
            parser = None
        else:
            parser = PARSERS.get((domain, opcode, "resp"))
        if parser is not None:
            try:
                parser(payload, lines)
            except Exception as exc:  # 解析失败不影响原始显示
                lines.append(f"!! 解析异常: {exc}")
        elif (domain, opcode) != (0x20, 0x01) and len(payload) > 1:
            lines.append(f"payload={hexs(payload[1:])}")
    elif payload:
        parser = PARSERS.get((domain, opcode, kind)) or PARSERS.get((domain, opcode, "any"))
        if parser:
            try:
                parser(payload, lines)
            except Exception as exc:
                lines.append(f"!! 解析异常: {exc}")
        elif domain in UNPUBLISHED_DOMAINS:
            p_unpublished_req(payload, lines)
        else:
            lines.append(f"payload={hexs(payload)}")
    lines.append(f"原始hex: {hexs(data)}")
    no_sync_data = offline_open_no_data(direction, domain, opcode, flags, payload)
    if no_sync_data:
        lines.append("说明: 固件当前没有新数据要上传，本次同步正常结束")
    level = (direction if direction in ("rx", "tx") else "ok") if no_sync_data else (
        "err" if is_err else ("event" if is_event else direction))
    return summary, proto, lines, level


# ---------------------------------------------------------------------------
# 串口行匹配: 兼容 "tx raw len=20 af 01 ..." 和 "rx raw len 8 af 01 ..."
# ---------------------------------------------------------------------------

RAW_RE = re.compile(r"(?i)\b(tx|rx)\s+raw\s+len\s*=?\s*(\d+)\s+(.*)")
# 固件常用 "%x" 把 0x02 打成 "2"；相邻两字节偶尔粘成 "11e3"
HEX_WORD_RE = re.compile(r"\b[0-9a-fA-F]+\b")


def parse_hex_bytes(text):
    """解析固件 hex 打印：1~2 位、以及粘连的偶数位长串。"""
    out = []
    nibble_fixed = 0
    glued = 0
    for t in HEX_WORD_RE.findall(text):
        if len(t) <= 2:
            if len(t) == 1:
                nibble_fixed += 1
            out.append(int(t, 16))
        elif len(t) % 2 == 0:
            glued += 1
            for i in range(0, len(t), 2):
                out.append(int(t[i:i + 2], 16))
        else:
            nibble_fixed += 1
            out.append(int(t[0], 16))
            for i in range(1, len(t), 2):
                out.append(int(t[i:i + 2], 16))
    return bytes(out), nibble_fixed, glued


def classify_line(text):
    """热路径：普通固件日志不做正则。返回 afu / fw / hci / ''。"""
    if " raw " in text or text.startswith("tx raw") or text.startswith("rx raw"):
        return "afu"
    if "EXERCISE_REALTIME" in text:
        return "fw"
    s = text.lstrip()
    if len(s) >= 3 and s[2] == ":" and s[0] in "TRtr" and s[1] in "Xx":
        return "hci"
    if s and s[0] in "0123456789abcdefABCDEF":
        return "hci"
    return ""


_FW_KV_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)=(0x[0-9A-Fa-f]+|-?\d+)")
_FW_FAIL_RE = re.compile(r"fail\s+(-?\d+)")


def fw_kv_to_realtime(kv):
    """把固件 EXERCISE_REALTIME 调试行按协议偏移填回 102 字节，复用同一套解析。"""
    p = bytearray(102)

    def num(key):
        return int(kv[key], 0)

    def put(off, key, n):
        if key not in kv:
            return
        p[off:off + n] = (num(key) & ((1 << (8 * n)) - 1)).to_bytes(n, "little")

    put(0, "exercise_id", 8)
    put(8, "sample_seq", 4)
    put(12, "timestamp_s", 4)
    put(16, "sport_type", 2)
    put(18, "state", 1)
    put(20, "field_validity_mask", 4)
    put(24, "status_flags", 2)
    put(28, "active_duration_s", 4)
    put(32, "paused_duration_s", 4)
    put(36, "heart_rate_bpm", 1)
    put(37, "max_heart_rate_bpm", 1)
    put(40, "active_calories_x100", 4)
    put(44, "step_count", 4)
    put(48, "distance_cm", 4)
    put(52, "swim_lap_count", 2)
    put(54, "swim_stroke_count", 2)
    put(56, "swim_main_stroke", 1)
    put(60, "current_pace_x100", 4)
    put(64, "average_pace_x100", 4)
    put(68, "fastest_pace_x100", 4)
    put(72, "average_stride_mm", 4)
    put(76, "average_cadence_x100", 4)
    put(80, "vo2_max_x100", 2)
    put(82, "average_speed_x100", 4)
    put(86, "ascent_cm", 4)
    put(90, "average_stroke_rate_x100", 2)
    put(92, "average_heart_rate_bpm", 1)
    put(93, "min_heart_rate_bpm", 1)
    put(94, "training_effect_x100", 2)
    put(96, "recovery_time_min", 2)
    put(98, "training_load_x100", 4)
    return bytes(p)


class FwRealtimeTrace:
    """固件把一条实时样本拆成多行 printf。收齐或看到 submit fail 再出一条解析。"""

    def __init__(self):
        self.kv = {}

    def reset(self):
        self.kv = {}

    def feed(self, line):
        if "submit fail" in line:
            m = _FW_FAIL_RE.search(line)
            return self._finish(m.group(1) if m else "?")
        found = dict(_FW_KV_RE.findall(line))
        if not found:
            return None
        if "exercise_id" in found and "active_duration_s" in self.kv:
            prev = self._finish(None)
            self.kv = found
            return prev
        self.kv.update(found)
        return None

    def _finish(self, fail):
        kv = self.kv
        self.kv = {}
        lines = []
        summary = "固件实时日志"
        if kv:
            payload = fw_kv_to_realtime(kv)
            summary = "固件日志：" + (s_realtime(payload) or "运动实时")
            p_realtime_data(payload, lines)
        if fail is not None:
            summary += f"，未发出AFU帧(submit fail {fail})"
            lines.append(f"提交失败: realtime submit fail {fail}。这秒没有协议帧，上面的数来自固件调试打印")
        if not lines:
            return None
        return summary, lines


def extract_frame(line):
    if "raw" not in line:
        return None
    m = RAW_RE.search(line)
    if not m:
        return None
    direction = m.group(1).lower()
    declared = int(m.group(2))
    data, nibble_fixed, glued = parse_hex_bytes(m.group(3))
    if len(data) < 8:
        return None
    return direction, declared, data, nibble_fixed, glued


HCI_START_RE = re.compile(r"^(TX|RX):((?:[0-9a-fA-F]{2}\s*)+)$", re.I)
HCI_CONT_RE = re.compile(r"^(?:[0-9a-fA-F]{2}\s+){2,}[0-9a-fA-F]{2}\s*$")
HCI_BYTE_RE = re.compile(r"[0-9a-fA-F]{2}")


class HciAfuStitcher:
    """把固件 TX:/RX: HCI ACL 折行拼回完整 ATT 载荷里的 AFU 帧。"""

    def __init__(self):
        self.acc = None
        self.hci_dir = None

    def feed(self, line):
        s = line.strip()
        out = []
        m = HCI_START_RE.match(s)
        if m:
            if self.acc:
                out.extend(self._drain(False))
            self.hci_dir = m.group(1).upper()
            self.acc = bytearray(int(x, 16) for x in HCI_BYTE_RE.findall(m.group(2)))
            out.extend(self._drain(False))
            return out
        if self.acc is not None and HCI_CONT_RE.match(s):
            self.acc.extend(int(x, 16) for x in HCI_BYTE_RE.findall(s))
            out.extend(self._drain(False))
            return out
        if self.acc is not None:
            out.extend(self._drain(True))
        return out

    def _drain(self, abandon):
        frames = []
        while self.acc is not None and len(self.acc) >= 5:
            if self.acc[0] != 0x02:
                self.acc = None
                return frames
            dlen = self.acc[3] | (self.acc[4] << 8)
            need = 5 + dlen
            if dlen == 0 or need > 2048:
                self.acc = None
                return frames
            if len(self.acc) < need:
                if abandon:
                    self.acc = None
                return frames
            pkt = bytes(self.acc[:need])
            del self.acc[:need]
            if not self.acc:
                self.acc = None
            afu = self._acl_to_afu(pkt)
            if afu:
                frames.append(afu)
        if abandon:
            self.acc = None
        return frames

    def _acl_to_afu(self, pkt):
        if len(pkt) < 9:
            return None
        cid = pkt[7] | (pkt[8] << 8)
        if cid != 4:
            return None
        att = pkt[9:]
        if len(att) < 4:
            return None
        if att[0] not in (0x1B, 0x1D, 0x12, 0x52):
            return None
        value = att[3:]
        if len(value) >= 8 and value[0] == 0xAF and value[1] == 0x01:
            return value
        return None


# ---------------------------------------------------------------------------
# 日志分段落盘
# ---------------------------------------------------------------------------

def app_dir():
    if getattr(sys, "frozen", False):
        exe = Path(sys.executable).resolve()
        # PyInstaller 的 .app 结构是 Name.app/Contents/MacOS/程序名。
        if (sys.platform == "darwin" and exe.parent.name == "MacOS"
                and exe.parent.parent.name == "Contents"):
            return exe.parents[3]
        return exe.parent
    return Path(__file__).resolve().parent


def default_log_dir():
    """macOS 从访达打开 .app 时，应用会被放到只读的随机目录。日志必须写到用户目录。"""
    if sys.platform == "darwin" and getattr(sys, "frozen", False):
        return Path.home() / "Library" / "Application Support" / "AFUMonitor" / "logs"
    return app_dir() / "logs"


def install_crash_log(log_dir: Path):
    """无控制台打包时，未捕获异常写到 logs/crash.log，避免锁屏后静默消失。"""
    log_dir.mkdir(parents=True, exist_ok=True)
    crash_path = log_dir / "crash.log"

    def _write(exc_type, exc, tb):
        try:
            with open(crash_path, "a", encoding="utf-8") as fp:
                fp.write(f"\n==== {datetime.datetime.now().isoformat(sep=' ', timespec='seconds')} ====\n")
                traceback.print_exception(exc_type, exc, tb, file=fp)
        except OSError:
            pass

    sys.excepthook = _write

    if hasattr(threading, "excepthook"):
        def _thread_hook(args):
            _write(args.exc_type, args.exc_value, args.exc_traceback)
        threading.excepthook = _thread_hook
    return _write


def workstation_is_locked():
    """Windows 锁屏时前台窗口为 0。macOS 不据此暂停界面。"""
    if sys.platform != "win32":
        return False
    try:
        return ctypes.windll.user32.GetForegroundWindow() == 0
    except Exception:
        return False


_ES_CONTINUOUS = 0x80000000
_ES_SYSTEM_REQUIRED = 0x00000001
_ES_AWAYMODE_REQUIRED = 0x00000040
_caffeinate_proc = None

# macOS 自带、不是 USB 转串口的 cu 节点
_MAC_PORT_SKIP = (
    "Bluetooth", "debug-console", "wlan-debug", "iPhone", "iPad", "AirPods",
)


def list_serial_ports():
    """枚举串口。macOS 只用 /dev/cu.*（call-out），避开会等待载波的 /dev/tty.*。"""
    found = []
    try:
        from serial.tools import list_ports
        found = list(list_ports.comports())
    except Exception:
        found = []
    if sys.platform != "darwin":
        return sorted(p.device for p in found)
    devices = [p.device for p in found if "/dev/cu." in p.device]
    if not devices:
        try:
            devices = [str(p) for p in Path("/dev").glob("cu.*")]
        except OSError:
            devices = []
    kept = []
    for device in devices:
        name = device.rsplit("/", 1)[-1]
        if any(skip in name for skip in _MAC_PORT_SKIP):
            continue
        kept.append(device)
    return sorted(set(kept))


def reveal_folder(path: Path):
    """在系统文件管理器中打开日志目录。"""
    if sys.platform == "darwin":
        subprocess.run(["open", str(path)], check=False)
        return
    if sys.platform == "win32":
        os.startfile(str(path))
        return
    subprocess.run(["xdg-open", str(path)], check=False)


def set_keep_awake(enable):
    """串口打开期间阻止系统睡眠。macOS 用 caffeinate，关闭串口时结束该进程。"""
    global _caffeinate_proc
    if sys.platform == "darwin":
        if enable:
            if _caffeinate_proc is not None and _caffeinate_proc.poll() is None:
                return
            try:
                _caffeinate_proc = subprocess.Popen(
                    ["caffeinate", "-i", "-m", "-w", str(os.getpid())],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except OSError:
                _caffeinate_proc = None
            return
        proc = _caffeinate_proc
        _caffeinate_proc = None
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                proc.kill()
        return
    if sys.platform != "win32":
        return
    try:
        fn = ctypes.windll.kernel32.SetThreadExecutionState
        fn.argtypes = [ctypes.c_uint]
        fn.restype = ctypes.c_uint
        if enable:
            fn(_ES_CONTINUOUS | _ES_SYSTEM_REQUIRED | _ES_AWAYMODE_REQUIRED)
        else:
            fn(_ES_CONTINUOUS)
    except Exception:
        pass


def fmt_size(n):
    if n < 1024:
        return f"{n} B"
    if n < 1024 * 1024:
        return f"{n / 1024:.1f} KB"
    return f"{n / (1024 * 1024):.1f} MB"


class SegmentedLog:
    MAX_BYTES = 32 * 1024 * 1024
    MAX_SECONDS = 3600

    def __init__(self, directory: Path):
        self.directory = directory
        self.directory.mkdir(parents=True, exist_ok=True)
        self.fp = None
        self.path = None
        self.started = 0.0
        self.written = 0
        self._pending = 0
        self._dirty_at = 0.0
        self._lock = threading.Lock()

    def set_segment(self, stamp: str):
        with self._lock:
            self._set_segment(stamp)

    def _set_segment(self, stamp: str):
        self._close()
        self.path = self.directory / f"{stamp}.txt"
        self.fp = open(self.path, "a", encoding="utf-8-sig", newline="\n")
        self.started = time.time()
        try:
            self.written = self.path.stat().st_size
        except OSError:
            self.written = 0
        self._pending = 0
        self._dirty_at = 0.0

    def write(self, line: str):
        with self._lock:
            if self.fp is None:
                return
            if not line.endswith("\n"):
                line += "\n"
            try:
                if self._pending == 0:
                    self._dirty_at = time.monotonic()
                self.fp.write(line)
                self.written += len(line.encode("utf-8"))
                self._pending += 1
                # 高速刷屏时按块合并；安静下来由读线程和界面定时把尾巴写出去。
                if self._pending >= 48:
                    self._flush()
            except OSError:
                pass

    def flush(self):
        with self._lock:
            self._flush()

    def flush_if_due(self, max_age=0.05):
        """还有没落盘的内容，并且已经过了这一小段时间，就写出去。"""
        with self._lock:
            if self._pending and (time.monotonic() - self._dirty_at) >= max_age:
                self._flush()

    def _flush(self):
        if self.fp:
            try:
                self.fp.flush()
            except OSError:
                pass
            self._pending = 0
            self._dirty_at = 0.0

    def close(self):
        with self._lock:
            self._close()

    def _close(self):
        if self.fp:
            try:
                self.fp.flush()
                self.fp.close()
            except OSError:
                pass
        self.fp = None

    def should_rotate(self):
        if self.fp is None:
            return True
        if self.written >= self.MAX_BYTES:
            return True
        if time.time() - self.started >= self.MAX_SECONDS:
            return True
        return False


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

# 工具栏对齐 band_tool 串口页：浅底、黑主按钮、白描边、安静按钮、红描边。
# 日志区保持深色。macOS 的 Aqua 按钮不认 tk.Button 的 bg，所以按钮用 clam 主题画出来。
C = {
    "bg": "#f3f3f1",
    "panel": "#ffffff",
    "bar": "#ffffff",
    "border": "#e4e4e4",
    "line": "#ececec",
    "text": "#161616",
    "muted": "#737373",
    "green": "#0c7a4e",
    "blue": "#1d4ed8",
    "red": "#9f1239",
    "purple": "#6d28d9",
    "orange": "#b45309",
    "input": "#ffffff",
    "log_bg": "#121212",
    "log_fg": "#e8e8e8",
    "log_dim": "#9a9a9a",
    "log_green": "#3fb950",
    "log_blue": "#79b8ff",
    "log_red": "#ff7b72",
    "log_purple": "#d2a8ff",
}

if sys.platform == "darwin":
    UI_FONT = ("PingFang SC", 13)
    TITLE_FONT = ("PingFang SC", 17, "bold")
    HEAD_FONT = ("PingFang SC", 13, "bold")
    LOG_FONT = ("Menlo", 12)
else:
    UI_FONT = ("Microsoft YaHei UI", 9)
    TITLE_FONT = ("Microsoft YaHei UI", 13, "bold")
    HEAD_FONT = ("Microsoft YaHei UI", 10, "bold")
    LOG_FONT = ("Consolas", 10)


class MonitorApp:
    # 界面只留最近一段；完整内容在 logs/。文字控件太大时，滚动和裁剪都会把 Tk 卡住。
    UI_MAX_LINES = 2000
    UI_KEEP_LINES = 1400
    # 每轮只画有限行，并限制耗时，剩下的下一轮再画。
    POLL_LINE_BUDGET = 40
    POLL_PARSED_BUDGET = 48
    POLL_BUDGET_S = 0.012
    SEE_INTERVAL_S = 0.12

    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.queue = queue.Queue()
        self.parse_q = queue.Queue()
        self.parsed_ui_q = queue.Queue()
        self._state_lock = threading.Lock()
        self._ui_gen = 0
        self._see_at = 0.0
        self._see_pending = False
        self._log_in_reader = False
        self._ui_dropped = 0
        self._parser = threading.Thread(target=self._parse_loop, name="afu-parse", daemon=True)
        self._parser.start()
        self.ser = None
        self.reader = None
        self.running = False
        self._hold_open = False
        self._session_paused = False
        self._closing = False
        self._reconnect_job = None
        self._lock_poll_job = None
        self._lock_hits = 0
        self._unlock_hits = 0
        self._paused_lines = 0
        self.frame_count = 0
        self._raw_lines = 0
        self._parsed_lines = 0
        self._count_dirty = False
        self._status_at = 0.0
        self.offline = OfflineSession()
        self.hci = HciAfuStitcher()
        self.fw_rt = FwRealtimeTrace()
        self.segment_stamp = None
        self.log_root = Path(args.log_dir) if args.log_dir else default_log_dir()
        self.serial_log = SegmentedLog(self.log_root / "serial")
        self.parsed_log = SegmentedLog(self.log_root / "parsed")

        root.title("AFU 协议监控 (macOS)")
        root.geometry("1580x860")
        root.minsize(1180, 640)
        root.configure(bg=C["bg"])
        try:
            root.lift()
        except tk.TclError:
            pass
        self._init_style()

        self._build_header()
        self._build_toolbar()
        self._build_panes()
        self._build_statusbar()
        self._refresh_ports()

        self.root.after(20, self._poll)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.root.report_callback_exception = self._on_tk_exception
        self.root.after(300, self._install_session_watch)

        if args.file:
            self._open_file(args.file)
        else:
            self.toggle()

    def _init_style(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure(".", background=C["bg"], foreground=C["text"], font=UI_FONT)
        specs = (
            ("Primary.TButton", "#161616", "white", "#2a2a2a", "#161616"),
            ("Ghost.TButton", "white", "#161616", "#f6f6f4", "#e4e4e4"),
            ("Quiet.TButton", "#f3f3f1", "#525252", "#ececec", "#f3f3f1"),
            ("Danger.TButton", "white", "#9f1239", "#fdf2f4", "#f3d5dc"),
        )
        for name, bg, fg, active, border in specs:
            style.configure(
                name, background=bg, foreground=fg, bordercolor=border,
                lightcolor=border, darkcolor=border, borderwidth=1,
                focusthickness=0, focuscolor=bg, padding=(12, 6),
                font=UI_FONT, relief="flat")
            style.map(
                name,
                background=[("active", active), ("pressed", active), ("disabled", C["bg"])],
                foreground=[("disabled", "#c8c8c8"), ("active", fg)],
                bordercolor=[("active", border), ("disabled", C["line"])])
        style.configure(
            "Tool.TCombobox", fieldbackground="white", background="white",
            foreground=C["text"], arrowcolor=C["muted"], bordercolor=C["border"],
            lightcolor=C["border"], darkcolor=C["border"], padding=4, arrowsize=13)
        style.map(
            "Tool.TCombobox",
            fieldbackground=[("readonly", "white"), ("disabled", "#f6f6f4")],
            foreground=[("readonly", C["text"]), ("disabled", "#c8c8c8")],
            background=[("active", "#f6f6f4"), ("readonly", "white")])
        for name, bg in (("Tool.TCheckbutton", C["bg"]), ("Pane.TCheckbutton", "white")):
            style.configure(
                name, background=bg, foreground=C["text"], focuscolor=bg,
                font=UI_FONT, indicatormargin=4)
            style.map(
                name,
                background=[("active", bg), ("selected", bg)],
                foreground=[("active", C["text"]), ("disabled", "#c8c8c8")])

    def _mk_btn(self, parent, text, command, kind="ghost"):
        styles = {
            "primary": "Primary.TButton",
            "ghost": "Ghost.TButton",
            "quiet": "Quiet.TButton",
            "danger": "Danger.TButton",
        }
        return ttk.Button(parent, text=text, command=command, style=styles[kind], cursor="hand2")

    def _set_port_button(self, text):
        kind = "Ghost.TButton" if text == "关闭" else "Primary.TButton"
        self._safe_config(self.btn, text=text, style=kind)

    def _mk_label(self, parent, text, **kw):
        opts = dict(bg=parent["bg"], fg=C["muted"], font=UI_FONT)
        opts.update(kw)
        return tk.Label(parent, text=text, **opts)

    def _build_header(self):
        header = tk.Frame(self.root, bg=C["bar"], height=56)
        header.pack(side=tk.TOP, fill=tk.X)
        header.pack_propagate(False)
        tk.Frame(header, bg=C["blue"], width=4).pack(side=tk.LEFT, fill=tk.Y)
        tk.Label(header, text="AFU 协议监控", bg=C["bar"], fg=C["text"],
                 font=TITLE_FONT).pack(side=tk.LEFT, padx=14)
        self._mk_label(header, "串口全量  ·  协议解析", bg=C["bar"],
                       fg=C["muted"]).pack(side=tk.LEFT, padx=(0, 8))
        self.led = tk.Canvas(header, width=12, height=12, bg=C["bar"],
                             highlightthickness=0)
        self.led.pack(side=tk.RIGHT, padx=(0, 16))
        self._led_id = self.led.create_oval(1, 1, 11, 11, fill="#c4c4c4", outline="")
        tk.Frame(self.root, bg=C["line"], height=1).pack(side=tk.TOP, fill=tk.X)
        self.status = tk.Label(header, text="未连接", bg=C["bar"], fg=C["muted"],
                               font=UI_FONT)
        self.status.pack(side=tk.RIGHT, padx=8)
        self.count_label = tk.Label(header, text="已解析 0 帧", bg=C["bar"],
                                    fg=C["blue"], font=UI_FONT)
        self.count_label.pack(side=tk.RIGHT, padx=16)

    def _build_toolbar(self):
        bar = tk.Frame(self.root, bg=C["bg"])
        bar.pack(side=tk.TOP, fill=tk.X)
        inner = tk.Frame(bar, bg=C["bg"])
        inner.pack(fill=tk.X, padx=14, pady=10)

        self._mk_label(inner, "端口", bg=C["bg"]).pack(side=tk.LEFT)
        self.port_var = tk.StringVar(value=self.args.port)
        self.port_combo = ttk.Combobox(inner, textvariable=self.port_var,
                                       width=36, style="Tool.TCombobox",
                                       font=UI_FONT)
        self.port_combo.pack(side=tk.LEFT, padx=(8, 4))
        self._mk_btn(inner, "刷新", self._refresh_ports, "ghost").pack(side=tk.LEFT, padx=2)

        self._mk_label(inner, "波特率", bg=C["bg"]).pack(side=tk.LEFT, padx=(14, 0))
        self.baud_var = tk.StringVar(value=str(self.args.baud))
        baud = tk.Entry(inner, textvariable=self.baud_var, width=10,
                        bg="white", fg=C["text"], insertbackground=C["text"],
                        relief=tk.FLAT, font=LOG_FONT, highlightthickness=1,
                        highlightbackground=C["border"], highlightcolor=C["text"])
        baud.pack(side=tk.LEFT, padx=6, ipady=5)
        self._mk_label(inner, "8N1", bg=C["bg"], fg="#a3a3a3").pack(side=tk.LEFT, padx=(2, 8))

        self.btn = self._mk_btn(inner, "打开", self.toggle, "primary")
        self.btn.pack(side=tk.LEFT, padx=(8, 4))
        self._mk_btn(inner, "清空", self.clear, "quiet").pack(side=tk.LEFT, padx=2)
        self._mk_btn(inner, "日志目录", self._open_log_dir, "quiet").pack(side=tk.LEFT, padx=2)
        self._mk_btn(inner, "清除缓存", self._clear_log_cache, "danger").pack(side=tk.LEFT, padx=2)

        self.autoscroll = tk.BooleanVar(value=True)
        chk = ttk.Checkbutton(inner, text="跟随滚动", variable=self.autoscroll,
                              style="Tool.TCheckbutton")
        chk.pack(side=tk.RIGHT, padx=4)
        tk.Frame(self.root, bg=C["line"], height=1).pack(side=tk.TOP, fill=tk.X)

    def _build_panes(self):
        wrap = tk.Frame(self.root, bg=C["bg"])
        wrap.pack(fill=tk.BOTH, expand=True, padx=10, pady=(4, 0))
        paned = tk.PanedWindow(wrap, orient=tk.HORIZONTAL, bg=C["bg"],
                               sashwidth=6, sashrelief=tk.FLAT, bd=0,
                               sashpad=3)
        paned.pack(fill=tk.BOTH, expand=True)
        self.raw = self._make_pane(paned, "串口全日志", C["blue"])
        self.parsed = self._make_pane(paned, "协议解析", C["green"])
        paned.add(self.raw[0], stretch="always", minsize=360)
        paned.add(self.parsed[0], stretch="always", minsize=420)
        self.raw_text = self.raw[1]
        self.parsed_text = self.parsed[1]
        for widget in (self.raw_text, self.parsed_text):
            widget.tag_config("rx", foreground=C["log_blue"])
            widget.tag_config("tx", foreground=C["log_green"])
            widget.tag_config("err", foreground=C["log_red"])
            widget.tag_config("event", foreground=C["log_purple"])
            widget.tag_config("dim", foreground=C["log_dim"])
            widget.tag_config("ok", foreground=C["log_green"])
        for tag in ("cat_app", "cat_reply", "cat_event", "cat_fw"):
            self.parsed_text.tag_config(tag, elide=False)
        self._build_parse_filters(self.parsed[2])

    def _make_pane(self, parent, title, accent):
        outer = tk.Frame(parent, bg=C["border"])
        inner = tk.Frame(outer, bg=C["panel"])
        inner.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)
        header = tk.Frame(inner, bg=C["panel"])
        header.pack(fill=tk.X)
        tk.Frame(header, bg=accent, width=3).pack(side=tk.LEFT, fill=tk.Y)
        tk.Label(header, text=title, bg=C["panel"], fg=C["text"],
                 font=HEAD_FONT).pack(side=tk.LEFT, padx=10, pady=7)
        body = tk.Frame(inner, bg=C["log_bg"])
        body.pack(fill=tk.BOTH, expand=True)
        text = tk.Text(
            body, wrap=tk.NONE, font=LOG_FONT, state=tk.DISABLED,
            undo=False, autoseparators=False, maxundo=0, exportselection=False,
            bg=C["log_bg"], fg=C["log_fg"], insertbackground=C["log_fg"],
            selectbackground="#264f78", selectforeground="#ffffff",
            relief=tk.FLAT, bd=0, highlightthickness=0, padx=8, pady=6)
        vs = tk.Scrollbar(body, orient=tk.VERTICAL, command=text.yview,
                          bg="#2a2a2a", troughcolor=C["log_bg"],
                          activebackground="#3a3a3a", highlightthickness=0, bd=0)
        hs = tk.Scrollbar(body, orient=tk.HORIZONTAL, command=text.xview,
                          bg="#2a2a2a", troughcolor=C["log_bg"],
                          activebackground="#3a3a3a", highlightthickness=0, bd=0)
        text.configure(yscrollcommand=vs.set, xscrollcommand=hs.set)
        text._vs = vs
        text._hs = hs
        vs.pack(side=tk.RIGHT, fill=tk.Y)
        hs.pack(side=tk.BOTTOM, fill=tk.X)
        text.pack(fill=tk.BOTH, expand=True)
        return outer, text, header

    def _build_parse_filters(self, header):
        """勾选才在右边显示该类。取消勾选只是藏起来，解析日志文件仍全量保存。"""
        self.show_fw = tk.BooleanVar(value=True)
        self.show_event = tk.BooleanVar(value=True)
        self.show_reply = tk.BooleanVar(value=True)
        self.show_app = tk.BooleanVar(value=True)
        box = tk.Frame(header, bg="white")
        box.pack(side=tk.RIGHT, padx=(0, 8))
        for text, var in (
            ("固件日志", self.show_fw),
            ("设备上报", self.show_event),
            ("设备回复", self.show_reply),
            ("APP下发", self.show_app),
        ):
            ttk.Checkbutton(
                box, text=text, variable=var, command=self._apply_parse_filter,
                style="Pane.TCheckbutton",
            ).pack(side=tk.RIGHT, padx=4)

    def _apply_parse_filter(self):
        shown = (
            (self.show_app, "cat_app"),
            (self.show_reply, "cat_reply"),
            (self.show_event, "cat_event"),
            (self.show_fw, "cat_fw"),
        )
        try:
            for var, tag in shown:
                self.parsed_text.tag_config(tag, elide=not var.get())
        except tk.TclError:
            return

    def _build_statusbar(self):
        bar = tk.Frame(self.root, bg=C["bar"], height=28)
        bar.pack(side=tk.BOTTOM, fill=tk.X)
        bar.pack_propagate(False)
        self.footer = tk.Label(bar, text="", bg=C["bar"], fg=C["muted"],
                               font=UI_FONT, anchor=tk.W)
        self.footer.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=14)
        self._update_footer()

    def _set_status(self, text, color=None):
        self._safe_config(self.status, text=text, fg=color or C["muted"])
        led = C["green"] if "已连接" in text else (
            C["red"] if "失败" in text or "错误" in text or "异常" in text else (
                C["orange"] if "锁屏" in text or "释放" in text or "休眠" in text or "重连" in text
                else "#c4c4c4"))
        try:
            self.led.itemconfig(self._led_id, fill=led)
        except tk.TclError:
            pass

    def _update_footer(self):
        stamp = self.segment_stamp or "-"
        s = self.serial_log.written if self.serial_log.fp else 0
        p = self.parsed_log.written if self.parsed_log.fp else 0
        rec = "REC" if self.serial_log.fp else "—"
        lag = self.parsed_ui_q.qsize()
        self._safe_config(
            self.footer,
            text=f"{rec}   日志目录  {self.log_root}    当前段  {stamp}    "
                 f"串口 {fmt_size(s)}    解析 {fmt_size(p)}    "
                 f"分段规则: 满 1 小时或 32MB 自动切新文件"
                 + (f"    界面略过调试日志 {self._ui_dropped} 行（已落盘）" if self._ui_dropped else "")
                 + (f"    解析结果正在刷到界面" if lag > 8 else ""))

    def _on_tk_exception(self, exc_type, exc, tb):
        try:
            sys.excepthook(exc_type, exc, tb)
        except Exception:
            pass

    def _safe_config(self, widget, **kwargs):
        try:
            widget.config(**kwargs)
        except tk.TclError:
            pass

    def _log_system(self, text, tag="event"):
        ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"{ts} | [monitor] {text}"
        self.serial_log.write(line)
        self.serial_log.flush()
        if not self._session_paused:
            self._append(self.raw_text, line, tag)

    def _install_session_watch(self):
        self._poll_lock_state()

    def _teardown_session_watch(self):
        job = self._lock_poll_job
        self._lock_poll_job = None
        if job is None:
            return
        try:
            self.root.after_cancel(job)
        except Exception:
            pass

    def _poll_lock_state(self):
        locked = workstation_is_locked()
        if locked:
            self._lock_hits += 1
            self._unlock_hits = 0
            if self._lock_hits >= 1 and not self._session_paused:
                self._on_session_pause()
        else:
            self._unlock_hits += 1
            self._lock_hits = 0
            if self._unlock_hits >= 2 and self._session_paused:
                self._on_session_resume()
        try:
            if self._lock_poll_job is not None:
                self.root.after_cancel(self._lock_poll_job)
        except Exception:
            pass
        try:
            self._lock_poll_job = self.root.after(250, self._poll_lock_state)
        except tk.TclError:
            self._lock_poll_job = None

    def _on_session_pause(self, reason=None):
        if self._session_paused:
            return
        self._session_paused = True
        self._paused_lines = 0

    def _on_session_resume(self, reason=None):
        self._session_paused = False
        self._update_footer()

    def _cancel_reconnect(self):
        job = self._reconnect_job
        self._reconnect_job = None
        if job is None:
            return
        try:
            self.root.after_cancel(job)
        except Exception:
            pass

    def _schedule_reconnect(self, delay_ms, reason):
        self._cancel_reconnect()

        def _go():
            self._reconnect_job = None
            if not self._hold_open or self.running:
                return
            self._open_serial(silent=True, reason=reason)

        self._reconnect_job = self.root.after(delay_ms, _go)

    # ---------------- 串口 ----------------

    def _refresh_ports(self):
        ports = list_serial_ports()
        self.port_combo.config(values=ports)
        if ports and self.port_var.get().strip() not in ports:
            self.port_var.set(ports[0])

    def toggle(self):
        if self.running or self._hold_open:
            self._hold_open = False
            self._session_paused = False
            self._cancel_reconnect()
            self._close_serial()
        else:
            self._open_serial()

    def _open_serial(self, silent=False, reason="打开"):
        if self.running:
            return
        try:
            import serial
            self.ser = serial.Serial(
                self.port_var.get().strip(),
                baudrate=int(self.baud_var.get().strip()),
                timeout=0.05)
        except Exception as exc:
            self._set_status(f"打开失败: {exc}", C["red"])
            self._log_system(f"{reason}失败: {exc}", "err")
            if not silent:
                messagebox.showerror(
                    "串口打开失败",
                    f"{self.port_var.get()} 打开失败：\n{exc}\n\n"
                    "请从下拉框选择 /dev/cu. 开头的端口，不要选 /dev/tty.。\n"
                    "Resource busy：该串口已被其他程序占用，先关掉占用程序再打开。\n"
                    "Permission denied：到「系统设置 → 隐私与安全性」允许本程序访问串口，\n"
                    "或在终端执行：sudo chmod a+rw 端口路径。\n"
                    "列表为空：检查 USB 转串口线。CH340 在 Apple 芯片上通常要先装沁恒驱动。\n"
                    "2 Mbps 打不开时，把波特率改成该芯片实际支持的速率后再试。")
            elif self._hold_open:
                self._schedule_reconnect(3000, "自动重试")
            return
        self._closing = False
        self._hold_open = True
        self.running = True
        set_keep_awake(True)
        if not self._session_paused:
            self._set_port_button("关闭")
            self._set_status(f"已连接 {self.port_var.get()} @ {self.baud_var.get()}", C["green"])
        self._rotate_segment("open" if reason == "打开" else "reconnect")
        if reason != "打开":
            self._log_system(f"{reason}：串口已重新打开")
        self._log_in_reader = True
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()

    def _close_serial(self, keep_hold=False, touch_ui=True):
        self._closing = True
        self.running = False
        self._log_in_reader = False
        if not keep_hold:
            self._hold_open = False
        ser = self.ser
        self.ser = None
        if ser:
            try:
                ser.close()
            except Exception:
                pass
        if not keep_hold:
            set_keep_awake(False)
        if touch_ui and not self._session_paused:
            self._set_port_button("关闭" if keep_hold else "打开")
            if not keep_hold:
                self._set_status("未连接", C["muted"])
        self.serial_log.flush()
        self.parsed_log.flush()

    def _read_loop(self):
        buf = b""
        try:
            while self.running:
                ser = self.ser
                if ser is None:
                    break
                try:
                    data = ser.read(8192)
                except Exception as exc:
                    if not self._closing:
                        self.queue.put(("__error__", str(exc)))
                    break
                if not data:
                    continue
                buf += data
                wrote = False
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    text = line.decode("utf-8", "replace").rstrip("\r")
                    self._ingest_line(ts, text)
                    wrote = True
                if wrote:
                    self.serial_log.flush()
        finally:
            if buf:
                ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
                text = buf.decode("utf-8", "replace").rstrip("\r")
                if text:
                    self._ingest_line(ts, text)
            self.serial_log.flush()
            self.parsed_log.flush()

    def _open_file(self, path):
        self._set_status(f"离线回放: {path}", C["blue"])
        self.btn.config(state=tk.DISABLED)
        self._rotate_segment("replay")
        self._log_in_reader = True

        def feed():
            try:
                with open(path, "r", encoding="utf-8", errors="replace") as f:
                    for raw_line in f:
                        raw_line = raw_line.rstrip("\r\n")
                        if " | " in raw_line[:16]:
                            ts, text = raw_line.split(" | ", 1)
                        else:
                            ts, text = "", raw_line
                        self._ingest_line(ts, text)
            except Exception as exc:
                self.queue.put(("__error__", str(exc)))
            self.queue.put(("__eof__", ""))

        threading.Thread(target=feed, daemon=True).start()

    # ---------------- 日志 ----------------

    def _rotate_segment(self, reason="auto"):
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.serial_log.set_segment(stamp)
        self.parsed_log.set_segment(stamp)
        self.segment_stamp = stamp
        header = (f"# AFU log segment={stamp}  reason={reason}  "
                  f"port={self.port_var.get()}  baud={self.baud_var.get()}")
        self.serial_log.write(header)
        self.parsed_log.write(header)
        self.serial_log.flush()
        self.parsed_log.flush()
        if not self._session_paused:
            self._update_footer()

    def _ensure_segment(self):
        if self.serial_log.should_rotate() or self.parsed_log.should_rotate():
            self._rotate_segment("auto")

    def _open_log_dir(self):
        self.log_root.mkdir(parents=True, exist_ok=True)
        (self.log_root / "serial").mkdir(parents=True, exist_ok=True)
        (self.log_root / "parsed").mkdir(parents=True, exist_ok=True)
        try:
            reveal_folder(self.log_root)
        except OSError as exc:
            messagebox.showerror("无法打开目录", str(exc))

    def _clear_log_cache(self):
        ok = messagebox.askyesno(
            "清除日志缓存",
            f"将删除以下目录中的全部日志文件，且不可恢复：\n\n"
            f"{self.log_root / 'serial'}\n"
            f"{self.log_root / 'parsed'}\n\n确定清除？")
        if not ok:
            return
        self.serial_log.close()
        self.parsed_log.close()
        deleted = 0
        for folder in (self.log_root / "serial", self.log_root / "parsed"):
            if not folder.exists():
                continue
            for f in folder.glob("*.txt"):
                try:
                    f.unlink()
                    deleted += 1
                except OSError:
                    pass
        self._rotate_segment("cleared")
        messagebox.showinfo("清除完成", f"已删除 {deleted} 个日志文件，已开启新分段。")

    # ---------------- 显示 ----------------

    def _flush_lines(self, widget, rows, counter_name):
        """同一轮多行合成少量 insert。批量插入时先摘掉滚动条回调，避免每行都重排。"""
        if not rows:
            return
        vs = getattr(widget, "_vs", None)
        hs = getattr(widget, "_hs", None)
        try:
            widget.config(state=tk.NORMAL)
            if vs is not None:
                widget.configure(yscrollcommand="", xscrollcommand="")
            buf = []
            tag = rows[0][1]

            def dump():
                if not buf:
                    return
                chunk = "".join(buf)
                buf.clear()
                if tag is None:
                    widget.insert(tk.END, chunk)
                else:
                    widget.insert(tk.END, chunk, tag)

            for text, row_tag in rows:
                if row_tag != tag:
                    dump()
                    tag = row_tag
                buf.append(text if text.endswith("\n") else text + "\n")
            dump()
            n = getattr(self, counter_name) + len(rows)
            trimmed = False
            if n > self.UI_MAX_LINES:
                drop = n - self.UI_KEEP_LINES
                widget.delete("1.0", f"{drop + 1}.0")
                n = self.UI_KEEP_LINES
                trimmed = True
            setattr(self, counter_name, n)
            if vs is not None:
                widget.configure(yscrollcommand=vs.set, xscrollcommand=hs.set)
                vs.set(*widget.yview())
                hs.set(*widget.xview())
            if self.autoscroll.get():
                now = time.monotonic()
                if trimmed or now - self._see_at >= self.SEE_INTERVAL_S:
                    self._see_at = now
                    self._see_pending = False
                    widget.see(tk.END)
                else:
                    self._see_pending = True
            widget.config(state=tk.DISABLED)
        except tk.TclError:
            try:
                if vs is not None:
                    widget.configure(yscrollcommand=vs.set, xscrollcommand=hs.set)
                widget.config(state=tk.DISABLED)
            except tk.TclError:
                pass

    def _append(self, widget, text, tag=None):
        name = "_raw_lines" if widget is self.raw_text else "_parsed_lines"
        self._flush_lines(widget, [(text, tag)], name)

    def _emit_offline_dump(self, ts, extra_lines, parsed_batch):
        if not extra_lines:
            return
        block = [f"[{ts}] {extra_lines[0]}"]
        block.extend(f"    {ln}" for ln in extra_lines[1:])
        block.append("")
        if parsed_batch is not None:
            parsed_batch.append((block[0], "ok"))
            for ln in extra_lines[1:]:
                parsed_batch.append((f"    {ln}", "ok"))
            parsed_batch.append(("", "ok"))
        self.parsed_log.write("\n".join(block))
        self.parsed_log.flush()

    def _poll(self):
        try:
            self._poll_once()
        except Exception:
            try:
                sys.excepthook(*sys.exc_info())
            except Exception:
                pass
        try:
            self.serial_log.flush_if_due()
            self.parsed_log.flush_if_due()
            ui_pending = (not self.queue.empty()) or (not self.parsed_ui_q.empty())
            if self._see_pending and not ui_pending:
                self._catch_up_scroll()
            busy = self.running or ui_pending or self.args.file or (not self.parse_q.empty())
            self.root.after(8 if ui_pending else (20 if busy else 200), self._poll)
        except tk.TclError:
            try:
                self.root.after(500, self._poll)
            except tk.TclError:
                pass

    def _catch_up_scroll(self):
        if not self.autoscroll.get():
            self._see_pending = False
            return
        self._see_pending = False
        self._see_at = time.monotonic()
        for widget in (self.raw_text, self.parsed_text):
            try:
                widget.see(tk.END)
            except tk.TclError:
                pass

    def _parse_loop(self):
        """协议解析离开 Tk 线程。每隔几帧让出一次，界面才能插进去刷新。"""
        n = 0
        tick = time.monotonic()
        while True:
            item = self.parse_q.get()
            if item is None:
                return
            gen, ts, text, kind = item
            batch = None
            try:
                with self._state_lock:
                    # 清空界面只丢掉还没画上去的结果，已经进队列的帧仍写入解析日志。
                    show = gen == self._ui_gen and not self._session_paused
                    batch = [] if show else None
                    self._dispatch_kind(ts, text, kind, batch, gen == self._ui_gen)
            except Exception:
                traceback.print_exc()
                continue
            if batch and gen == self._ui_gen:
                step = self.POLL_PARSED_BUDGET
                for i in range(0, len(batch), step):
                    self.parsed_ui_q.put((gen, batch[i:i + step]))
            n += 1
            now = time.monotonic()
            if n >= 8 or now - tick >= 0.005:
                n = 0
                tick = now
                time.sleep(0)

    def _ingest_line(self, ts, text):
        """读线程：先把原始行写入串口日志，再决定界面要不要显示。"""
        prefix = f"{ts} | " if ts else ""
        self.serial_log.write(prefix + text)
        kind = classify_line(text)
        if kind:
            self.parse_q.put((self._ui_gen, ts, text, kind))
        if kind or self.queue.qsize() < 800:
            self.queue.put((ts, text))
        else:
            self._ui_dropped += 1

    def _emit_parsed(self, ts, summary, proto, lines, level, parsed_batch, track=True):
        if track:
            self.frame_count += 1
            self._count_dirty = True
        block = [f"[{ts}] {summary}", f"    协议: {proto}"]
        block.extend(f"    {ln}" for ln in lines)
        block.append("")
        if parsed_batch is not None:
            head_tags = view_tags(level, summary)
            dim_tags = view_tags("dim", summary)
            parsed_batch.append((block[0], head_tags))
            parsed_batch.append((block[1], dim_tags))
            for ln in lines:
                parsed_batch.append((f"    {ln}", head_tags))
            parsed_batch.append(("", head_tags))
        self.parsed_log.write("\n".join(block))
        self.parsed_log.flush()

    def _dispatch_kind(self, ts, text, kind, parsed_batch, track=True):
        if kind == "fw":
            got = self.fw_rt.feed(text)
            if got:
                summary, lines = got
                self._emit_parsed(ts, summary, "固件调试 EXERCISE_REALTIME（不是AFU帧）",
                                  lines, "event", parsed_batch, track)
            return
        if kind == "hci":
            extra = []
            for afu in self.hci.feed(text):
                extra.extend(self.offline.on_afu(afu))
            if extra:
                self._emit_offline_dump(ts, extra, parsed_batch)
            return
        if kind != "afu":
            return
        frame = extract_frame(text)
        if not frame:
            return
        direction, _declared, data, nibble_fixed, glued = frame
        result = parse_frame(direction, data)
        extra = self.offline.on_afu(data)
        if result:
            summary, proto, lines, level = result
            notes = []
            if nibble_fixed:
                notes.append(f"{nibble_fixed} 处未补零(如 2 表示 02)")
            if glued:
                notes.append(f"{glued} 处粘连已拆开(如 11e3 表示 11 E3)")
            if notes:
                lines.insert(0, "!! 固件hex有 " + "，".join(notes))
            self._emit_parsed(ts, summary, proto, lines, level, parsed_batch, track)
        if extra:
            self._emit_offline_dump(ts, extra, parsed_batch)

    def _poll_once(self):
        ui = not self._session_paused
        wrote = False
        raw_batch = [] if ui else None
        parsed_batch = [] if ui else None
        try:
            if self._log_in_reader:
                self._ensure_segment()
            deadline = time.monotonic() + self.POLL_BUDGET_S
            while time.monotonic() < deadline:
                took = False
                if raw_batch is None or len(raw_batch) < self.POLL_LINE_BUDGET:
                    try:
                        ts, text = self.queue.get_nowait()
                    except queue.Empty:
                        ts = None
                    if ts is not None:
                        took = True
                        if ts == "__error__":
                            self._log_system(f"串口异常: {text}", "err")
                            if ui:
                                self._set_status(f"串口异常: {text}", C["red"])
                            self._close_serial(keep_hold=True, touch_ui=ui)
                            if self._hold_open:
                                self._schedule_reconnect(2000, "串口异常后重连")
                            continue
                        if ts == "__eof__":
                            self._log_in_reader = False
                            if ui:
                                self._set_status("回放完成", C["green"])
                            continue
                        if not self._log_in_reader:
                            self._ensure_segment()
                            prefix = f"{ts} | " if ts else ""
                            self.serial_log.write(prefix + text)
                            kind = classify_line(text)
                            if kind:
                                with self._state_lock:
                                    self._dispatch_kind(ts, text, kind, parsed_batch)
                        if raw_batch is not None:
                            raw_batch.append((f"{ts} | {text}" if ts else text, None))
                        else:
                            self._paused_lines += 1
                        wrote = True
                if parsed_batch is not None and len(parsed_batch) < self.POLL_PARSED_BUDGET:
                    try:
                        gen, rows = self.parsed_ui_q.get_nowait()
                    except queue.Empty:
                        rows = None
                    if rows is not None:
                        took = True
                        if gen == self._ui_gen:
                            parsed_batch.extend(rows)
                            wrote = True
                elif not ui:
                    try:
                        self.parsed_ui_q.get_nowait()
                        took = True
                    except queue.Empty:
                        pass
                if not took:
                    break
        finally:
            self._paint_batches(raw_batch, parsed_batch, wrote)

    def _paint_batches(self, raw_batch, parsed_batch, wrote):
        if raw_batch:
            self._flush_lines(self.raw_text, raw_batch, "_raw_lines")
        if parsed_batch:
            self._flush_lines(self.parsed_text, parsed_batch, "_parsed_lines")
        if wrote and self._session_paused:
            self.serial_log.flush()
        elif not self._session_paused and (wrote or self._count_dirty):
            now = time.monotonic()
            if now - self._status_at >= 0.25:
                self._status_at = now
                if wrote:
                    self._update_footer()
                if self._count_dirty:
                    self._safe_config(self.count_label, text=f"已解析 {self.frame_count} 帧")
                    self._count_dirty = False

    def clear(self):
        for widget in (self.raw_text, self.parsed_text):
            try:
                widget.config(state=tk.NORMAL)
                widget.delete("1.0", tk.END)
                widget.config(state=tk.DISABLED)
            except tk.TclError:
                pass
        self._raw_lines = 0
        self._parsed_lines = 0
        self._see_pending = False
        with self._state_lock:
            self._ui_gen += 1
            self.frame_count = 0
            self._count_dirty = False
            self.offline.reset()
            self.hci = HciAfuStitcher()
            self.fw_rt.reset()
        self._safe_config(self.count_label, text="已解析 0 帧")
        while True:
            try:
                self.parsed_ui_q.get_nowait()
            except queue.Empty:
                break
        self._ui_dropped = 0

    def _on_close(self):
        self._hold_open = False
        self._cancel_reconnect()
        self._teardown_session_watch()
        self._close_serial()
        reader = self.reader
        if reader is not None and reader.is_alive() and reader is not threading.current_thread():
            reader.join(timeout=2.0)
        try:
            self.parse_q.put(None)
        except Exception:
            pass
        parser = getattr(self, "_parser", None)
        if parser is not None and parser.is_alive() and parser is not threading.current_thread():
            parser.join(timeout=30.0)
        self.serial_log.close()
        self.parsed_log.close()
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def check_offline_nodata_wording():
    """离线 OPEN 的 NO_DATA 不能显示成同步失败。"""
    def frame(dataset, status, flags=0x11):
        payload = bytes([status, dataset]) + b"\x00" * 24
        return bytes([0xAF, 0x01, 0x10, 0x01, flags, 0x01, 0x00, len(payload)]) + payload

    summary, _proto, lines, level = parse_frame("tx", frame(1, 10))
    if "失败" in summary or "健康历史" not in summary or "没有需要同步" not in summary:
        raise RuntimeError(summary)
    if level == "err" or not any("没有新数据" in ln for ln in lines):
        raise RuntimeError(f"{level} {lines}")
    summary, _proto, _lines, level = parse_frame("tx", frame(4, 10))
    if "失败" in summary or "运动历史" not in summary or level == "err":
        raise RuntimeError(f"{level} {summary}")
    summary, _proto, _lines, level = parse_frame("tx", frame(1, 3))
    if "失败" not in summary or level != "err":
        raise RuntimeError(f"real offline error changed: {level} {summary}")
    body = (0x6AB4937A).to_bytes(4, "little") + (0x6AB494A6).to_bytes(4, "little") + bytes([0x51])
    metric, text = health_value(0x20, body)
    if metric != "心率" or "11:05:30" not in text or "11:10:30" not in text or "周期300秒" not in text or "81bpm" not in text:
        raise RuntimeError(f"period hr: {metric} {text}")


def check_exercise_chunks():
    """单条运动记录保持原样；超过 4060 字节的长运动两块拼成同一行。"""
    def chunk_page(seq, body, index, count, eid, crc):
        if count == 1:
            data = body
            flags = 0x0003
            offset = 0
        else:
            offset = 0 if index == 0 else EX_CHUNK_DATA_MAX
            data = body[offset:offset + (EX_CHUNK_DATA_MAX if index == 0 else len(body) - offset)]
            flags = 0x0001 if index == 0 else 0x0002
        raw = bytearray(30 + len(data))
        raw[0] = 0x01
        raw[1] = 0x01
        raw[2:4] = flags.to_bytes(2, "little")
        raw[4:12] = int(eid).to_bytes(8, "little")
        raw[12:14] = index.to_bytes(2, "little")
        raw[14:16] = count.to_bytes(2, "little")
        raw[16:20] = len(body).to_bytes(4, "little")
        raw[20:24] = offset.to_bytes(4, "little")
        raw[24:26] = len(data).to_bytes(2, "little")
        raw[26:30] = (crc & 0xFFFFFFFF).to_bytes(4, "little")
        raw[30:] = data
        page = bytearray(6 + len(raw))
        page[0:4] = seq.to_bytes(4, "little")
        page[4:6] = len(raw).to_bytes(2, "little")
        page[6:] = raw
        return bytes(page), bytes(raw)

    small = bytearray(112)
    small[0:2] = (1).to_bytes(2, "little")
    small[24:28] = (60).to_bytes(4, "little")
    small_crc = zlib.crc32(bytes(small)) & 0xFFFFFFFF
    page, raw = chunk_page(3, bytes(small), 0, 1, 0x15, small_crc)
    metric, text = exercise_value(raw)
    if metric != "运动" or "活动60秒" not in text or "步行" not in text:
        raise RuntimeError(f"single exercise changed: {metric} {text}")
    lines, _ = format_offline_page_lines(4, 1, page, [], {"first": 3, "last": 3}, {})
    blob = "\n".join(lines)
    if text not in blob or "运动 1条" not in blob:
        raise RuntimeError(f"single page mismatch: {blob}")

    body = bytearray(112 + 4000)
    body[0:2] = (2).to_bytes(2, "little")
    mask = 1 | (1 << 26)
    body[8:16] = mask.to_bytes(8, "little")
    body[24:28] = (1375).to_bytes(4, "little")
    body[32:36] = (12345).to_bytes(4, "little")
    body[110:112] = (4000).to_bytes(2, "little")
    body[4060] = 123
    body = bytes(body)
    crc = zlib.crc32(body) & 0xFFFFFFFF
    eid = 0x6AB2222D00000015
    page0, _ = chunk_page(10, body, 0, 2, eid, crc)
    page1, _ = chunk_page(11, body, 1, 2, eid, crc)
    pending = {}
    lines0, _ = format_offline_page_lines(4, 1, page0, [], {"first": 10, "last": 10}, pending)
    blob0 = "\n".join(lines0)
    if "活动1375秒" in blob0 or "待汇总" not in blob0 or not pending:
        raise RuntimeError(f"first chunk should wait: {blob0}")
    lines1, recs1 = format_offline_page_lines(4, 2, page1, [], {"first": 11, "last": 11}, pending)
    blob1 = "\n".join(lines1)
    if ("活动1375秒" not in blob1 or "距离123.45米" not in blob1
            or "心率点 123" not in blob1 or "运动 1条" not in blob1
            or "第1/2" in blob1 or pending):
        raise RuntimeError(f"joined exercise mismatch: {blob1}")
    if recs1[0].get("metric") != "运动":
        raise RuntimeError("joined metric changed")

    bad0, _ = chunk_page(10, body, 0, 2, eid, 0)
    bad1, _ = chunk_page(11, body, 1, 2, eid, 0)
    bad_pending = {}
    format_offline_page_lines(4, 1, bad0, [], {"first": 10, "last": 10}, bad_pending)
    bad_lines, _ = format_offline_page_lines(4, 2, bad1, [], {"first": 11, "last": 11}, bad_pending)
    bad_blob = "\n".join(bad_lines)
    if "活动1375秒" not in bad_blob or "整段校验不符" not in bad_blob:
        raise RuntimeError(f"crc note missing: {bad_blob}")

    sess = OfflineSession()
    sess.dataset = 4
    sess._try_dump((1, 1), {
        "dataset": 4, "page_id": 1, "parts": {0: page0}, "count": 1, "last": True,
        "page_bytes": len(page0), "first": 10, "last_seq": 10,
    }, force=True)
    joined_lines = sess._try_dump((2, 2), {
        "dataset": 4, "page_id": 2, "parts": {0: page1}, "count": 1, "last": True,
        "page_bytes": len(page1), "first": 11, "last_seq": 11,
    }, force=True)
    if not any("活动1375秒" in ln and "心率点 123" in ln for ln in joined_lines):
        raise RuntimeError("session did not print joined exercise")
    lone = OfflineSession()
    lone.dataset = 4
    lone._try_dump((1, 1), {
        "dataset": 4, "page_id": 1, "parts": {0: page0}, "count": 1, "last": True,
        "page_bytes": len(page0), "first": 10, "last_seq": 10,
    }, force=True)
    left = lone._flush_ex_pending()
    if not left or "未收齐" not in left[0] or "第1/2块" not in left[0]:
        raise RuntimeError(f"incomplete flush: {left}")


def run_selftest():
    """不真锁屏：走暂停/恢复同一套路径，确认不再改写 Tk 窗口过程。"""
    check_offline_nodata_wording()
    check_exercise_chunks()
    tmp = Path(tempfile.mkdtemp(prefix="afu_selftest_"))
    empty = tmp / "empty.txt"
    empty.write_text("", encoding="utf-8")
    args = argparse.Namespace(
        port="COM3", baud=2_000_000, file=str(empty), log_dir=str(tmp / "logs"))
    root = tk.Tk()
    app = MonitorApp(root, args)
    root.update()
    if hasattr(app, "_old_wndproc") or hasattr(app, "_wndproc_ref"):
        raise RuntimeError("must not subclass Tk window proc")
    app._on_session_pause("锁屏")
    if not app._session_paused:
        raise RuntimeError("pause flag not set")
    for i in range(40):
        app.queue.put((
            datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3],
            f"selftest line {i}"))
        app._poll_once()
        root.update()
    if app._paused_lines != 40:
        raise RuntimeError(f"paused ingest lost lines: {app._paused_lines}")
    log_text = app.serial_log.path.read_text(encoding="utf-8")
    if "selftest line 39" not in log_text:
        raise RuntimeError("paused serial log missing data")
    app._on_session_resume("解锁")
    if app._session_paused:
        raise RuntimeError("pause flag not cleared")
    root.update()
    this = sys.modules[__name__]
    orig = this.workstation_is_locked
    this.workstation_is_locked = lambda: True
    app._session_paused = False
    app._lock_hits = 0
    app._poll_lock_state()
    if not app._session_paused:
        raise RuntimeError("poll did not pause")
    this.workstation_is_locked = lambda: False
    app._unlock_hits = 0
    app._poll_lock_state()
    app._poll_lock_state()
    if app._session_paused:
        raise RuntimeError("poll did not resume")
    this.workstation_is_locked = orig
    app._log_in_reader = True
    sample = "rx raw len=8 af 01 01 01 00 01 00 00"
    for _ in range(400):
        app._ingest_line("12:00:00.000", sample)
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and app.frame_count < 400:
        time.sleep(0.01)
    if app.frame_count < 400:
        raise RuntimeError(f"parse worker stalled at {app.frame_count}")
    worst = 0.0
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and (not app.queue.empty() or not app.parsed_ui_q.empty()):
        started = time.perf_counter()
        app._poll_once()
        worst = max(worst, time.perf_counter() - started)
    if worst > 0.08:
        raise RuntimeError(f"ui poll blocked for {worst:.3f}s")
    shown = app.parsed_text.get("1.0", tk.END)
    if "GET_CAPABILITIES" not in shown:
        raise RuntimeError("parsed pane missing frames")
    serial_path = app.serial_log.path
    parsed_path = app.parsed_log.path
    for _ in range(800):
        app.queue.put(("12:00:00.000", "noise-line"))
    marker = "FULLSAVE_DEBUG_MARKER"
    app._ingest_line("12:00:02.000", marker)
    for _ in range(20):
        app._ingest_line("12:00:03.000", sample)
    app._on_close()
    serial_text = serial_path.read_text(encoding="utf-8")
    parsed_text = parsed_path.read_text(encoding="utf-8")
    if marker not in serial_text:
        raise RuntimeError("line skipped on screen was missing from serial log")
    if serial_text.count(sample) < 420:
        raise RuntimeError("serial log is not complete")
    if parsed_text.count("GET_CAPABILITIES") < 420:
        raise RuntimeError("parsed log is not complete")
    print("SELFTEST_OK")
    return 0


def main():
    ap = argparse.ArgumentParser(description="AFU 穿戴协议串口监控")
    ap.add_argument("--port", default="", help="macOS 串口，例如 /dev/cu.usbserial-XXXX；留空则选第一个 /dev/cu.*")
    ap.add_argument("--baud", type=int, default=2_000_000)
    ap.add_argument("--file", default=None, help="离线回放已保存的日志文件")
    ap.add_argument("--log-dir", default=None, help="日志根目录，默认 exe 同级 logs/")
    ap.add_argument("--selftest", action="store_true", help="锁屏路径自检后退出")
    args = ap.parse_args()
    if args.selftest:
        sys.exit(run_selftest())
    log_dir = Path(args.log_dir) if args.log_dir else default_log_dir()
    crash_hook = install_crash_log(log_dir)
    try:
        root = tk.Tk()
        MonitorApp(root, args)
        root.mainloop()
    except Exception:
        crash_hook(*sys.exc_info())
        raise


if __name__ == "__main__":
    main()

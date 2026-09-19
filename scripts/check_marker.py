"""SDK marker 识别自检（独立小程序，不跑主流程）。

为什么单独做：主程序跑一次要几分钟，而"SDK 到底认不认这批牌子"其实几秒钟
就能问出来。2026-09-18 连续四次实车 run 都是 `callbacks=2 /
observed_candidates=0`，分不清是"车不支持"、"标识没进视野"、"反光/光晕"还是
"这批牌子不是官方 marker"。这个小工具把这几件事一次问清。

用法（**必须已连上车的热点**）：

    python scripts\\check_marker.py                 # 默认订阅 red，跑 30 秒
    python scripts\\check_marker.py --seconds 60
    python scripts\\check_marker.py --color green
    python scripts\\check_marker.py --color ""      # 不设颜色过滤器

看什么：
  * "车支持的识别功能" 里有没有 marker —— 没有就是**车/固件不支持**，
    这条路彻底走不通；
  * "回调次数" 是不是 > 2 且不再全是空 —— 是就说明识别器在跑；
  * 把一张标识牌放到镜头前 20~40 cm、正对、别让灯反光，看是否突然出现
    `marker=...` 那一行 —— 出现就说明**能认**，之前只是距离/角度/反光问题。

本体不碰底盘、不发运动指令，只订阅识别 + 读回调。
"""
import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description="SDK marker 识别自检")
    parser.add_argument("--seconds", type=float, default=30.0,
                        help="观察多久（默认 30 秒）")
    parser.add_argument("--color", default="red",
                        help='颜色过滤器 red/green/blue，留空 "" = 不设')
    parser.add_argument("--robot", action="store_true",
                        help="同时也订阅机器人识别（看它认不认小车）")
    args = parser.parse_args()

    try:
        import robomaster
        from robomaster import robot
        from robomaster import vision as sdk_vision
    except Exception as error:
        print("导入 robomaster SDK 失败：%s" % error)
        return 2

    print("=" * 64)
    print("SDK marker 识别自检")
    print("=" * 64)

    ep_robot = robot.Robot()
    print("[1/4] 正在连接机器人（AP / UDP）...", flush=True)
    try:
        ep_robot.initialize(conn_type="ap", proto_type="udp")
    except Exception as error:
        print("连不上车：%s: %s" % (type(error).__name__, error))
        print("提示：SDK 连不上时会抛 `TypeError: exceptions must derive from")
        print("      BaseException`（client.py:95 用字符串 raise），别被它误导。")
        print("      先跑 scripts/check_robot_link.py 确认链路。")
        return 1
    print("      已连接", flush=True)

    try:
        print("[2/4] 打开视频流（识别需要画面）...", flush=True)
        ep_robot.camera.start_video_stream(display=False)
        time.sleep(1.0)

        print("[3/4] 询问车支持哪些识别功能 ...", flush=True)
        try:
            mask = ep_robot.vision._get_sdk_function()
        except Exception as error:
            mask = None
            print("      问不出来：%s" % error)
        bits = ((1 << 1, "person"), (1 << 2, "gesture"), (1 << 3, "line"),
                (1 << 4, "marker"), (1 << 5, "robot"))
        if mask is None:
            print("      车自报功能：**问不出来**（掩码 None）")
        else:
            names = [name for bit, name in bits if mask & bit]
            print("      车自报功能：mask=%s → %s" % (mask, "、".join(names) or "（无）"))
            if "marker" not in names:
                print("      ⚠️ **车里没有 marker 识别功能** —— SDK 这条路走不通，")
                print("         必须改用相机帧自己识别这批牌子。")

        received = []
        raw_count = {"n": 0}

        def on_marker(info):
            raw_count["n"] += 1
            received.append((time.monotonic(), info))
            print("      [marker 回调 #%d] %r" % (raw_count["n"], info), flush=True)

        def on_robot(info):
            print("      [robot 回调] %r" % (info,), flush=True)

        print("[4/4] 订阅 marker（color=%r），观察 %.0f 秒 ..."
              % (args.color or "(无过滤器)", args.seconds), flush=True)
        if args.color:
            result = ep_robot.vision.sub_detect_info(
                name=sdk_vision.MARKER, color=args.color, callback=on_marker)
        else:
            result = ep_robot.vision.sub_detect_info(
                name=sdk_vision.MARKER, callback=on_marker)
        print("      订阅返回：%r" % (result,), flush=True)
        if args.robot:
            ep_robot.vision.sub_detect_info(
                name=sdk_vision.ROBOT, callback=on_robot)
            print("      已同时订阅 robot 识别", flush=True)

        print()
        print("      >>> 现在把一张标识牌放到镜头前 20~40cm、正对、避免灯直射反光 <<<")
        print()

        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            time.sleep(0.5)

        print()
        print("=" * 64)
        print("结果")
        print("=" * 64)
        print("  回调总次数 :", raw_count["n"])
        non_empty = [item for _t, item in received if item]
        print("  非空回调   :", len(non_empty))
        if non_empty:
            print("  ✅ **识别器能认** —— 之前是距离/角度/反光问题。")
            for _t, item in non_empty[:5]:
                print("     %r" % (item,))
        elif raw_count["n"] == 0:
            print("  ❌ 一次回调都没有 —— 订阅没生效，或识别器没跑起来。")
        else:
            print("  ⚠️ 有回调但全是空的 —— 识别器在跑但**没认出任何标识**。")
            print("     若是正对镜头 20~40cm 仍然如此，基本可以判定：")
            print("     **这批牌子不是 SDK 能识别的 marker**（反光/图案不符）。")
            print("     先试另外两种颜色，再试换一张官方 marker 牌对照。")
        return 0
    finally:
        try:
            ep_robot.vision.unsub_detect_info(sdk_vision.MARKER)
        except Exception:
            pass
        try:
            ep_robot.camera.stop_video_stream()
        except Exception:
            pass
        try:
            ep_robot.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())

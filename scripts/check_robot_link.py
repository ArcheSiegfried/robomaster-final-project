"""RoboMaster 链路自检（独立于 SDK 的初始化流程）。

为什么需要它：SDK 连不上时会抛 `TypeError: exceptions must derive from
BaseException`（client.py:95 用字符串 raise），把真原因盖住。这个脚本直接
用 UDP 问一句，明确回答"车到底在不在"。

用法：
    python scripts\\check_robot_link.py
"""
import socket
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

ROBOT_ADDRESS = "192.168.2.1"
ROBOT_PORT = 20020


def local_ap_address():
    """本机在车热点里的地址（取不到返回 None）。"""
    for target in ((ROBOT_ADDRESS, ROBOT_PORT), ("192.168.2.1", 80)):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                probe.settimeout(0.5)
                probe.connect(target)
                address = probe.getsockname()[0]
                if address.startswith("192.168.2."):
                    return address
        except Exception:
            continue
    return None


def probe_udp(local, timeout=2.0):
    """复刻 SDK 的握手：把本机绑到 local、发 ProtoGetVersion、等回复。"""
    if local is None:
        return False, "本机没有 192.168.2.x 地址"
    try:
        from robomaster import conn, protocol
    except Exception as error:
        return False, "导入 rocmaster SDK 失败：%s" % error

    connection = None
    try:
        connection = conn.Connection((local, 10100), (ROBOT_ADDRESS, ROBOT_PORT),
                                     protocol="udp")
        connection.create()
        message = protocol.Msg(0x0B, 0x0B, protocol.ProtoGetVersion())
        connection.send(message.pack())
        connection._sock.settimeout(timeout)
        reply = connection.recv()
        return (reply is not None), ("收到应答" if reply is not None else "超时，没有应答")
    except Exception as error:
        return False, "%s: %s" % (type(error).__name__, error)
    finally:
        try:
            if connection is not None:
                connection.close()
        except Exception:
            pass


def main():
    print("=" * 62)
    print("RoboMaster 链路自检")
    print("=" * 62)

    local = local_ap_address()
    print("1) 本机热点地址 :", local or "（没有 —— Windows 连上了不等于在车热点里）")

    if local is None:
        print()
        print("结论：本机不在 192.168.2.x 网段 —— 车收不到你的包。")
        print("      先连上车的热点（SSID 形如 RMEP-XXXX），再跑本脚本。")
        return 2

    ok, detail = probe_udp(local)
    print("2) 机器人应答   :", detail)

    print()
    if ok:
        print("结论：链路通。可以用 main.py 启动了。")
        return 0
    print("结论：热点在，但 %s:%d 不回话。" % (ROBOT_ADDRESS, ROBOT_PORT))
    print("      可能：车没开机/固件没起、或车的控制服务卡住。")
    print("      先试：重启车；再试 ping 192.168.2.1 看是否通。")
    return 1


if __name__ == "__main__":
    sys.exit(main())

# 直角断口：云台侧视实验版

此方案只由 `route_only_main.py` 启用。`main.py` 默认仍注册原来的 `route.RouteTask`，不改变完整任务流程。启动实验仍会连接实车；代码助手不得运行。

## 动作顺序

1. 保留原来的旧线物理末端判定、短距离空白跨越和新线真实断口筛选。找不到合格断口就停车，不做扇形扫线。
2. 停车至少 0.22 秒，取得新鲜的云台相对底盘角度遥测，再将机器人设为 `FREE`，云台根据新线断口后直线的有向切线一次转向约 75–100°（相对断口处车头）。SDK `moveto` 的 yaw 直接使用相对底盘角，不能再叠加上电基准角；出口拒绝超过 ±105° 的请求。这仍只是图像斜率的近似角度，尚未校准成可靠的地面角度。
3. 云台保持侧视，底盘只沿原车头方向以最高请求 0.12 m/s 前进，不横移、不转向。侧视时新线可能斜着进入画面、物理端点也可能在画面外，所以改用可见线段的拟合线轴，而不要求近似竖直的内部端点。线尚未进入侧视画面时最多低速等待 2.4 秒；已看见后若丢失则停车，0.55 秒仍未恢复则失败。线轴在画面近处连续 3 个新帧居中才停车并开始底盘转向。侧视阶段总限时 3.8 秒，超时停车；**这些时间只是安全上限，不是成功触发条件**。
4. 底盘原地转到云台指向的角度。转角目前由已下发的角速度按时间估计，并非陀螺仪实测；转完立即切回 `CHASSIS_LEAD` 和低头视角。
5. 普通绿色巡线检测再次确认方向与居中后才交还控制权。否则停车并报告失败，不自动重试。

未确认新线、朝错误方向前进、达到阶段/任务超时、视频失效或人为暂停，都会停车。任务返回后，实验云台出口会恢复 `CHASSIS_LEAD`，避免正常巡线时镜头不跟随车头。

## 接口和限制

- `route_gimbal.py` 只产生 `TaskUpdate`、`MotionCommand`、`GimbalCommand`，不连接相机或 SDK。
- `route_gimbal_output.py` 是仅供实验入口使用的集中云台/模式出口；它订阅云台相对底盘角度，将一次相对车头的瞄准请求原样交给 SDK 的相对底盘 yaw 坐标（限定 ±105°），并在退出时退订。默认 `GimbalOutput` 和默认 `main.py` 的 ±30° 设置不变。
- 这不是断口真实测距。侧视线轴在画面近处居中只是**转身触发条件**，还需低头复核；单目视角、云台零位及底盘角速度误差仍可能造成失败。
- 使用的 DJI SDK 文档：[云台 `moveto`、`sub_angle` 接口](https://robomaster-dev.readthedocs.io/en/latest/python_sdk/robomaster.html)；[运动模式中 `CHASSIS_LEAD` 忽略云台 yaw、`FREE` 两轴独立](https://robomaster-dev.readthedocs.io/en/latest/text_sdk/protocol_api.html)。这些接口能力并不等于本机实车已验证。

## 安全验证顺序

离线：

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_route_gimbal tests.test_main_startup tests.test_gimbal_output -q
```

实车只由操作者在清空场地、有急停人员时执行。**先架空车轮**，通过 `route_only_main.py` 确认云台侧转时底盘不转、返回/人工停止后恢复跟随模式、正负 yaw 符号与预期一致；再以地面短距离测试。运行结束保留 `captures/route_debug/` 中对应 CSV，重点检查 `side_aim → side_advance → body_turn → reacquiring`、三个运动分量和失败原因。若方向或模式不对，立即按 SPACE/Q，不进行完整断口测试。

尚未验证：实车 `FREE`/`CHASSIS_LEAD` 切换动作、云台绝对零位、画面切线对应的地面角、侧视端点稳定性、实际角速度、转身后是否正好在线上。离线合成帧通过不证明上述事项。

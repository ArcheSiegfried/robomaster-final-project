# 离线验证记录

日期：2026-09-11

## 独立项目验证

在项目根目录、已激活虚拟环境时执行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\check_offline.ps1
```

脚本内部依次运行准确文件列表的 `py_compile`、`python -m unittest discover -s tests -v` 和 `python -m examples.offline_takeover`。结果：语法检查通过，离线测试 `14/14` 通过。接管示例依次输出：

```text
line:TRACKING
owner:external
task:running
task:completed
owner:line
line:TRACKING
```

测试覆盖路线方向和输出限幅、无条件最大色块拒绝、短时漏检降速及真实时间超时、无主动搜索、恢复限速、主动暂停、故障复位、新鲜路线显式恢复、视频失效独立锁停、最新帧入口、摄像头异常传播、唯一运动出口、公共任务类型，以及导入和示例均不加载 RoboMaster SDK。接管示例现在通过 `FakeChassis` 实际使用 `MotionOutput.claim()`、`send()` 和 `hard_stop()`，不再只用注释表示控制权切换。

静态检查确认生产代码没有旧竞速目录的绝对路径，也不依赖竞速规划器、旧搜索状态机、标定器或日志模块。SDK 底盘调用只位于 `motion_output.py`，RoboMaster 包只在 `main()` 内延迟导入。

本轮逐项核验了 `FramePacket` 的产生和主循环消费、v0.1 数据字段、暂停/复位/恢复锁定、四种停止/丢线语义、共享帧能力和底盘调用位置。公共接口字段与运行行为没有变化，只完善了示例对真实运动出口的调用。

独立导入检查由 `test_importing_main_does_not_import_robomaster` 覆盖：导入 `main` 不加载 RoboMaster 包，也不连接硬件。统一脚本本身不执行 `main()`。

## 未执行及待验证

没有运行 `main.py`，没有连接机器人、启动视频或发送运动指令。当前没有随项目交付的真实录像或场地样图，因此仍需架空车轮验证运动符号、命令超时和急停，再在现场验证 HSV、ROI、线宽筛选、反光漏检、弯道参数和真实断网停车。

## 2026-09-13 长断线第一版离线回归

本轮将公共接口升级为 v0.2，在 `TaskUpdate` 末尾增加可选 `GimbalCommand`，并增加唯一动态云台出口。实际执行：

```powershell
python -m unittest tests.test_route tests.test_gimbal_output tests.test_coordinator tests.test_task_contract -v
python scripts\check_module.py route
powershell -ExecutionPolicy Bypass -File scripts\check_offline.ps1
```

结果：相关回归 `53/53` 通过；`route.py` 静态契约检查和 `8/8` 专测通过。同步当时最新 `integration` 后再次运行统一脚本：语法检查 `36` 个 Python 文件、完整单元测试 `239/239`、离线接管示例、全部注册模块契约检查均通过，末尾输出 `OFFLINE_CHECK_OK`。

另外用 300 张 640×360 合成空白帧测量 `RouteTask.step()`：平均 `1.264 ms`，最大 `8.376 ms`。该数据只说明本机离线逐帧调用没有阻塞，不代表机器人实际帧率、网络时延或恢复成功率。

本轮同样没有运行 `main.py`、连接机器人、启动视频或发送实车命令。长断线所用云台角度、扫描方向、速度、时限、真实线段筛选及不同模块的接管优先级均待负责人组织实车验证。

## 2026-09-14 长断线第二版离线回归

根据首次实车反馈，第二版增加下方旧线检测、近全屏单端线段检测、走到物理线尾、有限空白跨越、±95° 扇形搜索、候选方向控制、旧线近端排除和分阶段交接。公共数据类型与 `TaskUpdate.step(frame, now)` 契约没有变化；协调器只增加可选 `reset()` 生命周期清理，避免人工停止、视频失效或任务结束后保留旧任务状态。

实际执行：

```powershell
python -m unittest tests.test_route tests.test_coordinator -v
python scripts\check_module.py route
powershell -ExecutionPolicy Bypass -File scripts\check_offline.ps1 -Python .\.venv\Scripts\python.exe
```

结果：路线与协调器相关回归 `52/52` 通过；`route.py`/`route_detector.py` 静态契约检查和路线专测 `19/19` 通过；同步最新 `integration` 后，统一脚本语法检查 `40` 个 Python 文件、完整单元测试 `300/300`、离线接管示例和全部登记模块检查均通过，末尾输出 `OFFLINE_CHECK_OK`。

首次实车运行还暴露了线尾阶段的时间预算错误：原 `END_APPROACH_MAX_SECONDS=2.5` 配合 `0.08 m/s` 只能覆盖约 `0.20 m`，会在到达物理线尾附近时直接失败，因而不可能进入抬头、空白跨越和扇扫。现保留硬上限但调整为 `5.0 s`（约 `0.40 m`），并增加超过旧 2.5 秒后完整进入跨越与扇扫的回归用例。该距离仍须结合实际相机视野标定。

本轮没有运行 `main.py`，没有连接机器人、相机或视频流，也没有向实车发送任何运动/云台命令。合成图只能证明状态机、限幅、几何筛选和故障路径符合当前约定，不能证明真实断口恢复成功率。抬头 ROI、HSV/形态学、0.12 m/s 空白跨越、30°/s 搜索、±95° 搜索范围、底盘 yaw 符号、旧线排除效果和云台异步完成时间仍须实车验证。

## 2026-09-15 断线专项首次实车反馈后的修正

现场运行目录 `captures/run_20260915_144216` 显示 `RouteTask` 在约 3.7 秒已经接管，
但车辆和云台没有产生可见动作。最后两条 SDK 同步消息超时发生在人工关闭机器人后，
不作为该故障原因。对该目录定时截图进行离线回放时，旧版状态机确实产生了
`pitch=-5°`、`forward=0.12 m/s` 和 `yaw=14～30°/s` 请求；因此本轮不能把问题归因于
“没有进入 RouteTask”，而是针对低速静摩擦、旧路线持续入镜、SDK 返回值未检查和
终端缺少任务阶段反馈分别修正。

实际 `log.csv` 中观察到单次 `0.406 s`、`0.500 s` 的帧间隙，之后视频继续恢复；旧
`0.22 s` 阈值会过早永久锁定 `VIDEO_LOST`。阈值现调整为 `0.60 s`，底盘命令自身的
`0.15 s` timeout 保持不变，因此视频间隙期间旧运动命令仍会先自动失效。

代码调整后，同一批现场定时截图的离线回放依次产生：抬头 `pitch=-5°`、空白跨越
`forward=0.18 m/s`、候选确认扇扫 `yaw=30°/s`、主扇扫 `yaw=45°/s`。空白跨越至少
执行 `0.8 s` 后才允许候选打断，降低抬头后仍可见的旧路线被立即重新接受的风险。
`MotionOutput` 现在不再忽略 SDK 明确返回的 `False`，`GimbalOutput` 会保留动作并在
后续帧报告 rejected/failed/exception；终端心跳会显示实际命令、云台目标和当前阶段。

离线命令：

```powershell
powershell -ExecutionPolicy Bypass -File scripts\check_offline.ps1 -Python .\.venv\Scripts\python.exe
```

结果：语法检查 `41` 个 Python 文件，完整单元测试 `322/322`，离线接管示例与全部
模块检查通过，输出 `MODULE_CHECK_OK`、`OFFLINE_CHECK_OK`。这些结果仍不能证明底盘
实际执行了新速度、云台实际到达 `-5°` 或真实新路线能够成功重获，必须由下一轮专项
实车测试确认。

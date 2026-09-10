# 离线验证记录

日期：2026-09-10

## 独立项目验证

在项目根目录执行：

```powershell
python -m py_compile __init__.py config.py models.py camera_source.py `
  line_detector.py controller.py runtime.py motion_output.py main.py `
  examples\__init__.py examples\offline_takeover.py `
  tests\test_offline.py

python -m unittest discover -s tests -v
python -m examples.offline_takeover
```

结果：语法检查通过，离线测试 `14/14` 通过。接管示例依次输出：

```text
line:TRACKING
owner:external
task:running
task:completed
line:TRACKING
```

测试覆盖路线方向和输出限幅、无条件最大色块拒绝、短时漏检降速及真实时间超时、无主动搜索、恢复限速、主动暂停、故障复位、新鲜路线显式恢复、视频失效独立锁停、最新帧入口、摄像头异常传播、唯一运动出口、公共任务类型，以及导入和示例均不加载 RoboMaster SDK。

静态检查确认生产代码没有旧竞速目录的绝对路径，也不依赖竞速规划器、旧搜索状态机、标定器或日志模块。SDK 底盘调用只位于 `motion_output.py`，RoboMaster 包只在 `main()` 内延迟导入。

## 未执行及待验证

没有运行 `main.py`，没有连接机器人、启动视频或发送运动指令。当前没有随项目交付的真实录像或场地样图，因此仍需架空车轮验证运动符号、命令超时和急停，再在现场验证 HSV、ROI、线宽筛选、反光漏检、弯道参数和真实断网停车。

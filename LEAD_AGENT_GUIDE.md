# 新负责人及代码 Agent 使用指南

## 1. 每日状态检查

每天开始先看远端而不是群消息猜状态：

```powershell
git fetch origin --prune --tags
git status --short --branch
git branch -a -vv
git log --oneline --decorate --graph --all -15
```

再到 GitHub 查看 Open/Draft PR、失败检查、阻塞评论和各工作包证据。维护一个简短表：工作包、分支/PR、当前状态、下一验收、阻塞条件、最后更新时间。不要用“写了很多”代替可运行结果。

## 2. 继续、降级、换人或砍掉

- 继续：输入已明确，最小验收可复现，最近一次提交带来可测进展。
- 降级：核心路径可保留，但泛化、速度、自动动作或覆盖范围在剩余时间内风险过高。
- 配对/换人：接口边界反复误解、三次迭代仍没有可运行切片，或阻塞技能与当前开发者明显不匹配。先保留原分支和证据，不抹去贡献。
- 砍掉：依赖的场地/规则仍未知、安全性不能验证，或会危及 P0 底座和总体拼合。把原因、替代行为和用户可见影响写入交接。

决定权在新负责人；`TASKS.md` 是候选工作包，不是固定人员安排。

## 3. 控制同时活跃模块数量

优先保持 2～3 个活跃方向：一个 P0 底座/集成、一个明确视觉任务、一个数据/证据任务。高冲突文件同一时间只允许一个已协调分支修改。依赖场地的 WP5/WP6 不要在没有输入时同时大量编码。每个分支要求 24 小时内能展示最小输入输出或明确阻塞。

## 4. PR 审查

按 `.github/pull_request_template.md` 和 `PROJECT_HANDOVER.md` 检查：

1. base 必须为 `integration`，范围只覆盖一个工作包。
2. 查看全部 diff，特别搜索相机初始化、RoboMaster import、`drive_speed`、`drive_wheels`、`move`、`sleep` 和无界循环。
3. 公共文件变化必须先有接口缺口说明和整合决定。
4. 本地复现模块测试与 `scripts/check_offline.ps1`。
5. 核对数据来源、成功/失败样例、实车与未验证边界。
6. 只有完成/失败都能停车并归还控制权时，运动任务才可进入联调。

## 5. integration 离线联调

每次合入后在干净 `integration` 运行统一脚本，并补一个最短端到端场景：同一 `FramePacket` 依次送给检测器；协调者选择唯一 owner；任务 RUNNING 才发送其 `MotionCommand`；COMPLETED/FAILED 后硬停、归还、清历史、新帧确认、显式恢复。视频失效、人工暂停和故障状态不得因检测恢复而自动启动。

失败时先回退最近一个 PR 或关闭新模块默认接入，不在 `integration` 临时大改多个模块。

## 6. 实车测试组织

实车只能由负责人人工安排。采用“离线 → 架空车轮 → 单模块低速 → 两模块组合 → 整场”。每轮指定操作、急停、记录角色，清空安全区，先测停止路径再测运动。一次只验证明确场景和少量参数；出现无法解释的运动立即停车，不继续堆补丁。

## 7. 实车记录

每次开始前保存：

```powershell
git rev-parse HEAD
git status --short
git diff -- config.py
```

记录日期、机器人/固件、相机位姿、路线/道具、光照、参数、用例、急停方式。视频/图片名包含日期和短 SHA；大文件放共享盘，在仓库索引中写路径、摘要和结果。失败同样要保存。

## 8. 稳定 integration 合入 main

确认依赖工作包、统一离线检查和本轮承诺的实车检查完成后，创建 `integration → main` PR。描述必须列出：包含的 PR、准确 SHA、实际离线/架空/实车结果、已知问题、关闭的功能和回退标签。合并后不要继续在旧 `integration` 工作；先同步 main，再把经确认的 main 合回 integration 或按仓库规则更新。

## 9. 标签和回退

稳定 `main` 使用语义清楚的新标签：

```powershell
git switch main
git pull --ff-only origin main
git tag v0.x-description
git push origin v0.x-description
```

已有标签永不移动。需要回退时，从旧标签创建新的 `fix/...` 分支修复，再走 PR；不要强推 `main` 或重写共享历史。

## 10. 最终材料

每次合并就收集源码、测试输出、数据索引、实车 SHA/参数/视频和贡献记录，最后按 `DELIVERABLES.md` 核对。先取得学校正式格式，未确认材料只能标“建议保留”。贡献按 commit、PR、配对和测试事实记录，不按猜测分配。

## 11. 可直接复制给负责人代码 Agent 的指令

```text
请作为 RoboMaster 期末项目的负责人代码 Agent，在仓库根目录工作。

开始前必须完整读取：
- PROJECT_HANDOVER.md
- MODULE_GUIDE.md
- TASKS.md
- COLLABORATION_GUIDE.md
- DELIVERABLES.md
- AGENTS.md

然后检查当前目录、Git 分支、工作树、远端关系、相关 PR/任务和目标工作包。不要覆盖来源不明的修改。

协作规则：
1. main 是稳定分支，integration 是联调分支。普通业务修改只能在从最新 integration 创建的 feat/... 或 fix/... 分支完成，并通过 PR 回到 integration。
2. models.py、camera_source.py、runtime.py、motion_output.py、main.py 是高冲突公共文件。除非负责人明确协调接口变更，不得修改或顺手重构；变更时必须说明缺口、影响、版本和集成测试。
3. 所有视觉模块只接收共享 FramePacket 或其 image，不自行连接相机。
4. 检测/任务模块只返回 VisualDetection、TaskUpdate、MotionCommand，不直接调用 RoboMaster SDK。MotionOutput 是唯一正常运动出口。
5. 任务 step 必须非阻塞；运动有限速和硬超时；完成/失败必须停车并归还控制权。普通短时漏检由基础巡线处理，长断线不得通过延长容错冒充。
6. 不运行 main.py，不连接机器人、视频流或底盘。代码 Agent 只运行离线图片、合成帧、假时钟、假 chassis 和 scripts/check_offline.ps1。任何实车步骤只能给出清单，等负责人现场人工执行。
7. 不引入插件系统、事件总线、复杂继承、大型框架或没有验收意义的空模块。
8. 不虚构场地、规则、功能、实车结果或成员贡献。报告准确命令、实际结果、失败和未验证事项。
9. 只修改当前工作包允许的文件及测试；发现需要扩大范围时先停下说明。
10. 提交前检查路径、签名、文档、git diff、密钥/代理/绝对路径和工作树；不得强推、重置共享历史或移动标签。

收到具体工作包后，先用一句话复述目标和允许范围，再检查现状，完成最小实现、离线验证和交接。若需求依赖未知场地，先完成不依赖场地的接口、样本夹具或测试，并明确阻塞，不猜测正式规则。
```

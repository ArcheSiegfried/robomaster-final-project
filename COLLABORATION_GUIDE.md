# Windows Git / GitHub 协作教程

本项目的唯一常规流程是：`任务分支 → PR 到 integration → 联调 → integration 合入 main → 稳定标签`。命令均在 PowerShell 中执行。把 `<name>` 替换为实际短名称，不要输入尖括号。

## 1. 接受仓库邀请

- 目的：获得私有仓库访问权（若所有者恢复私有）。
- 命令：无。
- 网页：登录正确 GitHub 账号，打开邮件或 GitHub Notifications 的邀请，确认仓库名后点 **Accept invitation**。
- 成功表现：能打开仓库首页和 `integration` 分支。
- 常见错误：404 通常是账号错误、邀请过期或未接受；把账号名发给仓库所有者核对，不要索要他人密码。

## 2. 第一次 clone

- 目的：得到包含完整 Git 历史的本地副本。
- 命令：

  ```powershell
  cd F:\robomaster
  git clone https://github.com/ArcheSiegfried/robomaster-final-project.git
  cd robomaster-final-project
  git remote -v
  ```

- 网页：仓库首页点 **Code → HTTPS** 可复制地址。
- 成功表现：`git remote -v` 的 fetch/push 都指向上述仓库。
- 常见错误：不要下载 ZIP 代替 clone；鉴权失败先确认邀请和 Git Credential Manager 登录。

## 3. 安装依赖并跑离线测试

- 目的：先证明本机环境正常，不碰实车。
- 命令：

  ```powershell
  python -m venv .venv
  .\.venv\Scripts\Activate.ps1
  python -m pip install -r requirements.txt
  powershell -ExecutionPolicy Bypass -File scripts\check_offline.ps1
  ```

- 网页：无。
- 成功表现：最后显示 `OFFLINE_CHECK_OK`，单元测试全部 `ok`。
- 常见错误：执行策略阻止激活时可继续用 `.\.venv\Scripts\python.exe`；依赖失败记录完整错误交给负责人。绝不用 `python main.py` 试环境。

## 4. 查看当前分支和改动

- 目的：避免在公共分支或带未知修改的目录中工作。
- 命令：`git status --short --branch`，再执行 `git branch --show-current`。
- 网页：仓库左上分支选择器可查看远端分支。
- 成功表现：开始任务前工作树没有未说明文件，当前分支名明确。
- 常见错误：若出现不是自己产生的修改，停止并询问；不要 `reset --hard`、`checkout --` 或删除文件。

## 5. 同步最新 integration

- 目的：从团队最新联调状态开始。
- 命令：

  ```powershell
  git switch integration
  git pull --ff-only origin integration
  ```

- 网页：确认远端 `integration` 最近提交与负责人通知一致。
- 成功表现：显示 `Already up to date` 或仅快进更新。
- 常见错误：`not possible to fast-forward` 表示本地有分叉；不要强推或重置，先把 `git status` 和 `git log --oneline --decorate -8` 发给负责人。

## 6. 创建任务分支

- 目的：隔离一个工作包，方便审查和回退。
- 命令：`git switch -c feat/traffic-light`；修复可用 `git switch -c fix/line-loss`。
- 网页：无。
- 成功表现：`git branch --show-current` 显示新分支。
- 常见错误：不要使用空格或中文分支名；一个分支不要混入多个不相关任务。

## 7. 修改前后看差异

- 目的：发现误改和越界文件。
- 命令：修改前运行 `git status --short`；修改后运行 `git diff --stat` 和 `git diff`。
- 网页：无。
- 成功表现：只出现工作包允许的源码、测试和小型数据。
- 常见错误：看到 `.venv`、缓存、大录像、密钥、公共文件或无关格式化时不要提交；先清理自己产生的内容或找负责人。

## 8. add、commit、push

- 目的：把审查过的改动保存到远端任务分支。
- 命令：

  ```powershell
  git add path\to\module.py tests\test_module.py
  git diff --cached
  git commit -m "feat: detect traffic light"
  git push -u origin feat/traffic-light
  ```

- 网页：推送后仓库通常显示 **Compare & pull request**。
- 成功表现：commit 成功，push 显示远端新分支。
- 常见错误：不要习惯性 `git add .`；提交身份错误先配置自己的 `user.name` 和 GitHub 可识别邮箱；绝不把 token 写进 remote URL。

## 9. 创建目标为 integration 的 PR

- 目的：让代码先进入联调分支而非稳定主线。
- 命令：无必需命令；先确保 `git status` 干净且测试通过。
- 网页：点 **Compare & pull request**，把 base 设为 `integration`，compare 设为任务分支，按模板填写。
- 成功表现：页面显示 `base: integration ← compare: feat/...`。
- 常见错误：默认 base 可能是 `main`，提交前必须改；不要删掉模板中的未验证事项。

## 10. Draft PR 与正式 PR

- 目的：区分“提前共享”与“可以合并”。
- 命令：相同分支持续 `git push` 即可更新 PR。
- 网页：未完成或需早期讨论选 **Create draft pull request**；代码、测试和材料齐全后点 **Ready for review**。
- 成功表现：Draft 显示灰色 Draft；正式 PR 显示 Open 并可请求审查。
- 常见错误：Draft 不是垃圾存档，也要写当前状态；未验证实车应写未验证，不必因此永远保持 Draft。

## 11. 根据审查意见继续修改

- 目的：在同一个 PR 中修正问题。
- 命令：保持在原任务分支，修改后重新测试，再 `git add <files>`、`git commit -m "fix: address review"`、`git push`。
- 网页：逐条回复审查意见；确认修复后由提出者或负责人解决 conversation。
- 成功表现：新 commit 自动出现在原 PR，检查重新运行。
- 常见错误：不要为同一任务另开 PR；不要改历史后 force push，除非负责人明确安排。

## 12. PR 合并后更新本地

- 目的：回到团队最新状态并清理已完成分支。
- 命令：

  ```powershell
  git switch integration
  git pull --ff-only origin integration
  git branch -d feat/traffic-light
  ```

- 网页：PR 显示 **Merged** 后可删除远端任务分支。
- 成功表现：本地 `integration` 包含 PR 提交，旧分支可正常删除。
- 常见错误：`branch is not fully merged` 时不要用 `-D`；先确认 PR 的 base 和合并状态。

## 13. 开始下一项任务

- 目的：避免新工作叠在旧分支上。
- 命令：先完成第 12 步，再从最新 `integration` 执行 `git switch -c feat/<new-name>`。
- 网页：在 Issues/工作包中确认目标、依赖和验收。
- 成功表现：新分支的共同祖先是当前 `integration`，`git status` 干净。
- 常见错误：不要在已合并旧分支继续开发，也不要把两个任务塞进同一 PR。

## 14. 整合负责人审查和合并

- 目的：保护接口、安全出口和联调质量。
- 命令：需要本地复核时用 `git fetch origin`、`git switch --detach origin/feat/<name>`，运行统一脚本后切回 `integration`。
- 网页：核对模板、Files changed、公共文件、测试证据、未验证项和依赖；选择团队约定的合并方式。
- 成功表现：PR 只改允许范围，所有必要检查通过，合并后 `integration` 可完整离线运行。
- 常见错误：不要只看“Files changed 数量”；不要因作者声称通过就跳过复现；公共接口变化必须先协调版本。

## 15. integration 进入 main

- 目的：只把已联调状态发布为稳定主线。
- 命令：

  ```powershell
  git switch integration
  git pull --ff-only origin integration
  powershell -ExecutionPolicy Bypass -File scripts\check_offline.ps1
  ```

- 网页：由负责人创建 `integration → main` PR，列出离线、实车、未验证和回退点。
- 成功表现：验证完成后 PR 合并，`main` 对应一个明确稳定状态。
- 常见错误：不要把尚未联调的任务分支直接 PR 到 `main`；没有实车条件时必须明确“仅离线”。

## 16. 创建稳定标签

- 目的：保存不可移动的回退版本。
- 命令（仅仓库所有者/负责人）：

  ```powershell
  git switch main
  git pull --ff-only origin main
  git tag v0.x-description
  git push origin v0.x-description
  ```

- 网页：仓库 **Tags/Releases** 页面核对标签提交。
- 成功表现：本地和远端标签指向同一 `main` commit。
- 常见错误：已有标签不得删除、覆盖或移动；标签命名和验证范围要写进交接记录。

## 17. 分支落后时同步

- 目的：在合并前吸收最新 `integration`。
- 命令：先提交自己的改动，再在任务分支执行 `git fetch origin`、`git merge origin/integration`，解决后重新完整测试并 push。
- 网页：PR 的 **Update branch**（若可用）也会产生同步提交。
- 成功表现：PR 不再显示落后，测试仍通过。
- 常见错误：新手优先 merge，不自行 rebase 已推送分支；工作树不干净时不要同步。

## 18. 遇到合并冲突

- 目的：保护双方代码，避免猜测覆盖。
- 命令：先运行 `git status` 并保存输出；不确定时用 `git merge --abort` 回到合并前。
- 网页：把冲突文件、双方 PR 和 `git status` 发给整合负责人。
- 成功表现：中止后原分支和修改仍在；或在负责人指导下逐文件解决并重新测试。
- 常见错误：不要选择“全部接受当前/传入”解决公共文件；不要删文件、强推或 `reset --hard`。

## 19. 新手禁止自行执行的危险操作

- 目的：避免丢历史或覆盖他人。
- 命令：不要执行 `git push --force`、`git reset --hard`、`git clean -fd`、移动/删除稳定标签、删除公共远端分支、在未知改动上批量 checkout。
- 网页：不要改仓库可见性、成员权限、分支保护或删除仓库。
- 成功表现：遇到需要这些操作的教程时先停下，把目标和状态交给负责人。
- 常见错误：所谓“重新来一次”不是破坏历史的理由；通常可用新分支、普通 merge 或新 commit 修复。

## 20. 常见报错和成功判断

- `not a git repository`：先 `cd` 到含 `.git` 的项目根目录。
- `pathspec ... did not match`：先 `git fetch origin` 并检查拼写/分支是否存在。
- `Permission denied` / 403：检查邀请、登录账号和仓库权限；不要借用 token。
- `non-fast-forward`：远端有新提交；停止 push，按第 17 步同步。
- 测试失败：保存完整输出，先修测试或解释环境，不提交“应该能过”。
- 成功的最低判断：当前分支正确、`git status` 干净、统一离线脚本成功、PR base 为 `integration`、实际验证与未验证均写清。

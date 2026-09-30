# Compound Engineering 使用指南

AI 辅助的结构化工程工作流。将开发任务拆解为 11 个阶段、6 道人工门禁，通过 22 个 skill（`.cursor/skills/compound-engineering/`）、47 个 agent（`.cursor/agents/`）实现端到端可编排工作流 — 流程编排与产物生成由 AI 驱动，关键质量与发布决策由人在门禁处完成。

## 解决什么问题

大型 monorepo（多模块仓库）中开发任务的典型痛点：需求遗漏、架构决策缺乏结构化审查、代码质量依赖个人经验、知识无法跨任务复用。不使用本工作流时，这些环节由开发者凭经验处理 — 质量高度依赖个人状态，且经验不沉淀。知识复用依赖 Stage 11 compound + knowledge 插件；未安装时工作流仍可用于交付，但复用/沉淀价值需手动补全。

## 上游同步基线

- `compound-engineering`：`compound-engineering-v2.54.1`（语义合并，非强制对齐 — 本工作流在上游基础上做了 项目特化适配，允许独立演进）
- `superpowers`：`v5.0.6`（同上）

## 首次使用

直接启动工作流即可 — 无需手动执行 `/setup`：

```
/run-ce-workflow <task-id>
```

工作流 Startup Protocol 会自动检测 `compound-engineering.local.md` 是否存在。如果不存在，自动执行 `setup --auto`：检测仓库技术栈（C++、Python、TypeScript、Proto 等），生成配置文件，决定各审计阶段启用哪些条件 agent。

如需手动重新配置 agent 选择（交互式），可运行：

```
/setup
```

## 核心工作流（11 阶段）

启动完整工作流：

```
/run-ce-workflow <task-id>
```

查看任务状态：

```
/run-ce-workflow <task-id> --status
```

从指定阶段恢复：

```
/run-ce-workflow <task-id> --stage impl_plan
```

执行到指定阶段后暂停：

```
/run-ce-workflow <task-id> --to code_audit
```

### 阶段总览

```
brainstorm → arch_plan → arch_audit → impl_plan → impl_audit → tdd → execute → code_audit → ship → verify → compound → done
                           ↑ loop                    ↑ loop                        ↑ loop
```

| 阶段          | 做什么                                                                                                                | 加载的 Skill                                                                      | 门禁  |
| ------------- | --------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------- | ----- |
| 1. brainstorm | 探索需求、调研、设计方案                                                                                              | `brainstorming`                                                               | **A** |
| 2. arch_plan  | 写架构计划                                                                                                            | `writing-arch-plans`                                                          | —     |
| 3. arch_audit | 4 个 agent 并行审查架构                                                                                               | 见 [`audit-protocol.md`](skills/run-ce-workflow/references/audit-protocol.md) | **B** |
| 4. impl_plan  | 写实现计划（完整代码 + TDD 步骤）                                                                                     | `writing-impl-plans`                                                          | —     |
| 5. impl_audit | 4+ agent 并行审查计划可执行性                                                                                         | 见 [`audit-protocol.md`](skills/run-ce-workflow/references/audit-protocol.md) | **C** |
| 6. tdd        | 创建分支，写所有失败测试                                                                                              | `test-driven-development`                                                     | **D** |
| 7. execute    | 批量执行实现，测试变绿                                                                                                | `executing-plans`                                                             | —     |
| 8. code_audit | 8+ agent 并行审查代码                                                                                                 | 见 [`audit-protocol.md`](skills/run-ce-workflow/references/audit-protocol.md) | **E** |
| 9. ship       | 合并/推送/创建 PR                                                                                                     | `finishing-a-development-branch`                                              | —     |
| 10. verify    | 验证交付物（手动/集成测试/仿真）— 阶段推进 ≠ 生产就绪，需收集验证证据                                                 | `verification`                                                                | **F** |
| 11. compound  | 将解决方案沉淀到知识库（最低可观测信号：MAP 更新、domain README 链接、`learnings-researcher` 可命中；质量需人工抽检） | `compound-docs`                                                               | —     |

> `done` 是工作流终态标记，不是第 12 个工作阶段。

**基础设施 Skill**：`ideating`、`subagent-driven-development`、`dispatching-parallel-agents` 等 auto-loaded skill 不对应特定阶段，而是作为库被其他 skill 按需调用。`reviewing-code` 也可用于 Stage 8 code_audit。

**审查合并**：审计阶段（3/5/8）的并行 agent 产出经合并去重后呈现。审计与代码审查使用不同的 severity 体系：

| 审计（audit-protocol） | 代码审查（reviewing-code） | 含义     | 门禁影响 |
| ---------------------- | ------------------------------ | -------- | -------- |
| p1 (blocking)          | P0 (Critical)                  | 必须修复 | 阻断门禁 |
| —                      | P1 (High)                      | 应当修复 | 不阻断   |
| p2 (should fix)        | P2 (Moderate)                  | 值得修复 | 不阻断   |
| p3 (nice to have)      | P3 (Low)                       | 自行决定 | 不阻断   |

冲突裁决：同维度取最高 severity、人工门禁负责最终判断。详见 [`audit-protocol.md`](skills/run-ce-workflow/references/audit-protocol.md)。

Autofix 路由也因场景不同而分两套：`document-review` 使用 `auto`（单一正确修复）/ `present`（需要判断）二级分类；`reviewing-code` 使用 `safe_auto` / `gated_auto` / `manual` / `advisory` 四级分类，粒度更细以区分代码变更的风险等级。

**并行与插队**：工作流为单任务线性主干。并行需求使用独立 task-id。热修/紧急变更可从指定阶段恢复（`--stage`），无需从头走完。注意：跳过 arch_audit/impl_audit 的变更需确认架构/计划仍有效；跳过 verify 的交付物不应视为生产就绪。

每个门禁通过后建议开启新会话 — 任务状态保存在 `docs/tasks/<task-id>.yaml` 中跨会话传递。

## 日常使用场景

**推荐路径**：新功能/重大变更走场景 1（完整 11 阶段）。需求已明确的小改走场景 3（跳过 brainstorm，无 arch_audit/impl_audit 门禁）。场景 4-11 为独立工具，可在任何阶段或不走完整流程时单独使用。

### 场景 1：完整开发一个功能

最标准的用法，走完 11 个阶段：

```
/run-ce-workflow add-lidar-filter
```

适合：新功能开发、涉及多文件的重构、需要审查的重要变更。

### 场景 2：只做头脑风暴 + 设计

不走完整流程，只用 brainstorming skill 探索方案：

```
/brainstorming
```

自动调研代码库（`repo-research-analyst`）、查找行业最佳实践（`best-practices-researcher`）、搜索历史解决方案（`learnings-researcher`），然后引导你逐步形成设计文档。

适合：探索性任务、方案评估、技术选型。

### 场景 3：写计划并执行

跳过 brainstorming，直接写计划：

```
/writing-arch-plans
```

写完后执行：

```
/executing-plans
```

适合：需求已经明确、只需要实现的任务。跳过了 brainstorm 及审计门禁，变更规模较大时建议补跑 `/reviewing-code`。

### 场景 4：TDD 开发

在当前任务中使用 TDD 方法：

```
/test-driven-development
```

强制 RED → GREEN → REFACTOR 循环：先写失败测试 → 最小代码通过 → 重构。跳过任何步骤会被阻止。

适合：有明确可测试行为的新功能和 bugfix。纯配置变更、文档更新、探索性原型可豁免。

### 场景 5：调试问题

遇到 bug、测试失败、构建问题：

```
/systematic-debugging
```

四阶段框架：根因调查 → 模式分析 → 假设验证 → 实现修复。**铁律：不完成根因调查不允许动手修。**

适合：所有技术问题，尤其是"怎么改都不对"的情况。

### 场景 6：完成开发分支

实现完成、测试通过后：

```
/finishing-a-development-branch
```

提供 4 个选项：合并到主分支 / 推送并创建 PR / 保留分支 / 丢弃。

### 场景 7：代码审查

在提交 PR 之前做结构化代码审查：

```
/reviewing-code
```

自动选择合适的 reviewer persona（correctness、security、performance 等），并行审查后合并去重，产出带 severity/confidence/route 的结构化报告。支持 4 种模式：

- 默认交互模式：自动修 safe_auto 问题，gated 问题问你
- `mode:autofix`：全自动修复 safe_auto，不问你
- `mode:report-only`：只看不动
- `mode:headless`：给其他 skill 调用的程序化模式

也可以指定 PR 或 base：

```
/reviewing-code https://github.com/org/repo/pull/123
/reviewing-code base:origin/main
```

适合：PR 前审查、实现完成后质量检查、Stage 8 code_audit。

### 场景 8：Agent-Native 审计

对代码库进行 agent-native 架构审计：

```
/agent-native-audit
```

启动 8 个并行 agent，从 Action Parity、Tools as Primitives、Context Injection、CRUD 完整性等维度打分。

### 场景 9：补全不够详细的计划

当审计阶段发现计划太笼统，使用 `deepen-plan` skill：

> deepen this plan

为计划的每个章节启动并行研究 agent，补充最佳实践、实现细节。

### 场景 10：浏览器自动化

```
/agent-browser
```

通过 `browser-use` CLI 或 `agent-browser` 进行网页交互、表单填写、截图。

### 场景 11：前端设计

```
/frontend-design
```

引导选择设计方向（极简/最大化/复古/有机等），生成有独特美感的前端代码。

## 组件参考

### Skills（22 个）

16 个 skill 未禁用模型触发（无 `disable-model-invocation`），仅在匹配触发条件时激活。`setup` 可按项目裁剪启用集合。

| Skill                                | 用途                                                                                  | 自动加载                             |
| ------------------------------------ | ------------------------------------------------------------------------------------- | ------------------------------------ |
| `run-ce-workflow`                | 11 阶段工作流编排                                                                     | 否（手动 `/run-ce-workflow`）    |
| `brainstorming`                  | 探索需求、调研、形成设计                                                              | 是                                   |
| `ideating`                       | 改进方向探索、创意生成                                                                | 是                                   |
| `writing-arch-plans`             | 写架构计划（Plan Depth + Confidence Check）                                           | 是                                   |
| `writing-impl-plans`             | 写实现计划（TDD + 完整代码）                                                          | 是                                   |
| `executing-plans`                | 批量执行实现计划                                                                      | 是                                   |
| `subagent-driven-development`    | 子 agent 驱动的并行开发                                                               | 是                                   |
| `dispatching-parallel-agents`    | 多 agent 并行派遣框架                                                                 | 是                                   |
| `test-driven-development`        | TDD：RED → GREEN → REFACTOR                                                           | 是                                   |
| `systematic-debugging`           | 四阶段调试框架                                                                        | 是                                   |
| `document-review`                | 审查和改进文档质量                                                                    | 是                                   |
| `reviewing-code`                 | 结构化代码审查（多 persona 并行、合并去重、4 种模式）                                 | 是                                   |
| `finishing-a-development-branch` | 分支集成（合并/PR/保留/丢弃）                                                         | 是                                   |
| `verification`                   | 交付后验证（手动/集成/仿真）                                                          | 是                                   |
| `compound-docs`                  | 将解决方案沉淀到 knowledge                                                        | 否（手动 `/compound-docs`）      |
| `setup`                          | 配置 agent 选择（工作流自动调用 `--auto`，手动可交互）                                | 工作流自动 / 手动 `/setup`       |
| `agent-native-architecture`      | Agent-Native 设计知识库                                                               | 是                                   |
| `frontend-design`                | 高品质前端界面设计                                                                    | 否（手动 `/frontend-design`）    |
| `agent-browser`                  | 浏览器自动化                                                                          | 是                                   |
| `deepen-plan`                    | 并行研究增强计划深度和细节                                                            | 是                                   |
| `agent-native-audit`             | Agent-Native 架构审计（8 维打分）                                                     | 否（手动 `/agent-native-audit`） |
| `ce-session-retro`               | CE 工作流复盘（6 维分析：执行质量、派遣准确性、流程优化、skill/agent 效能、开放观察） | 否（手动 `/ce-session-retro`）   |

### Agents（47 个）

核心常驻 agent（correctness、testing、maintainability 等）始终参与审查；条件 agent 由 `setup` 根据技术栈和 diff 特征自动启用。新增 agent 需有至少两个真实调用场景。

**研究类**（brainstorm / plan 阶段调研）：

| Agent                        | 用途                              |
| ---------------------------- | --------------------------------- |
| `repo-research-analyst`      | 代码库结构和模式调研              |
| `best-practices-researcher`  | 行业最佳实践调研                  |
| `framework-docs-researcher`  | 框架/库文档和 API 调研            |
| `learnings-researcher`       | 搜索历史解决方案（knowledge） |
| `git-history-analyzer`       | Git 历史考古（代码演变原因）      |
| `feishu-researcher`          | 飞书文档搜索（组织决策/约束）     |
| `issue-intelligence-analyst` | GitHub issue 主题分析和趋势识别   |

**代码审查类**（由 `reviewing-code` skill 编排或 audit 阶段派遣）：

| Agent                            | 审查维度                                                                | 类型                |
| -------------------------------- | ----------------------------------------------------------------------- | ------------------- |
| `correctness-reviewer`           | 逻辑错误、边界情况、状态 bug                                            | 始终启用            |
| `testing-reviewer`               | 测试覆盖缺口、弱断言                                                    | 始终启用            |
| `maintainability-reviewer`       | 耦合、复杂度、死代码                                                    | 始终启用            |
| `project-standards-reviewer`     | CLAUDE.md / AGENTS.md 合规                                              | 始终启用            |
| `agent-native-reviewer`          | 新功能是否 agent 可访问（审计 Stage 5 core 亦含）                       | 始终启用            |
| `learnings-researcher`           | 搜索 `docs/solutions/` 历史方案（亦列于研究类）                         | 始终启用            |
| `security-reviewer`              | 安全漏洞（注入/XSS/SSRF/auth 绕过）                                     | 条件启用            |
| `performance-reviewer`           | 代码 diff 性能审查（热循环、缓存、I/O 密集路径）— PR/code review 时触发 | 条件启用            |
| `api-contract-reviewer`          | API 契约变更、类型签名                                                  | 条件启用            |
| `data-migrations-reviewer`       | 迁移安全、schema 变更                                                   | 条件启用            |
| `reliability-reviewer`           | 错误处理、重试、超时                                                    | 条件启用            |
| `adversarial-reviewer`           | 故障场景构造（大 diff / 高危领域）                                      | 条件启用            |
| `kieran-python-reviewer`         | Python 代码质量                                                         | 条件启用            |
| `kieran-typescript-reviewer`     | TypeScript 代码质量                                                     | 条件启用            |
| `julik-frontend-races-reviewer`  | 前端竞态条件                                                            | 条件启用            |
| `cli-agent-readiness-reviewer`   | CLI agent 可读性审查                                                    | 条件启用            |
| `previous-comments-reviewer`     | PR 历史评论检查                                                         | 条件启用（PR 模式） |
| `schema-drift-detector`          | Proto Schema 变更检测                                                   | 条件启用            |
| `deployment-verification-agent`  | 部署清单和回滚方案                                                      | 条件启用            |
| `architecture-strategist`        | 架构合规、设计完整性                                                    | Stage 3, 8 audit    |
| `code-simplicity-reviewer`       | YAGNI、代码简洁性                                                       | Stage 3, 8 audit    |
| `pattern-recognition-specialist` | 设计模式、命名规范、重复代码                                            | Stage 3, 8 audit    |
| `performance-oracle`             | 深度性能审计（算法复杂度、系统瓶颈、扩展性）— 仅工作流审计阶段          | Stage 3, 5, 8 audit |
| `security-sentinel`              | 安全漏洞、OWASP 合规                                                    | Stage 5, 8 audit    |
| `document-reviewer`          | 文档质量审查（轻量 gate）                                               | Stage 1, 2, 4       |
| `data-integrity-guardian`        | 数据库迁移安全                                                          | 条件启用            |
| `data-migration-expert`          | 数据迁移验证                                                            | 条件启用            |

**文档审查类**（由 `document-review` skill 编排）：

| Agent                               | 审查维度               | 类型     |
| ----------------------------------- | ---------------------- | -------- |
| `coherence-reviewer`                | 内部一致性、术语漂移   | 始终启用 |
| `feasibility-reviewer`              | 技术可行性、架构冲突   | 始终启用 |
| `product-lens-reviewer`             | 产品策略、前提挑战     | 条件启用 |
| `design-lens-reviewer`              | UI/UX、交互设计        | 条件启用 |
| `security-lens-reviewer`            | 安全架构、威胁模型     | 条件启用 |
| `scope-guardian-reviewer`           | 范围对齐、不必要复杂度 | 条件启用 |
| `adversarial-document-reviewer` | 压力测试决策和假设     | 条件启用 |

**设计类**：

| Agent                            | 用途                     |
| -------------------------------- | ------------------------ |
| `design-implementation-reviewer` | 设计意图 vs 实现偏移检测 |
| `design-iterator`                | 迭代式 UI 设计优化       |
| `figma-design-sync`              | Figma 设计稿与实现同步   |

**工作流类**：

| Agent                        | 用途                              |
| ---------------------------- | --------------------------------- |
| `bug-reproduction-validator` | 系统性 Bug 复现                   |
| `pr-comment-resolver`        | PR 评审意见处理                   |
| `spec-flow-analyzer`         | 用户流完整性、边界条件（Stage 5） |
| `lint`                   | 代码格式化和 lint 检查            |

### 独立入口

以下能力可在工作流外独立使用，通过对应 skill 的 slash 命令调用：

| 入口                      | 用途                   | 对应 Skill               |
| ------------------------- | ---------------------- | ------------------------ |
| `/agent-native-audit` | 8 维 Agent-Native 审计 | `agent-native-audit` |

## 任务文件

每个任务的状态保存在 `docs/tasks/<task-id>.yaml`，跨会话传递。模板位于 `skills/run-ce-workflow/templates/task-state.yaml`。

YAML 是单任务的唯一状态源，建议提交到 VCS。单人单分支场景下工作良好；多人协作时通过独立 task-id 避免并发冲突。

## 外部依赖

| 资源                                                            | 用途                     | 必需          | 缺失时降级行为                                         |
| --------------------------------------------------------------- | ------------------------ | ------------- | ------------------------------------------------------ |
| `knowledge` plugin                                          | Stage 11 知识沉淀        | Stage 11 必需 | compound 阶段跳过，任务仍可标记 done，知识需手动补沉淀 |
| `agents-memory-updater` agent（由宿主环境提供，非本工作流内置） | Stage 11 记忆更新        | 可选          | 跳过记忆同步，不影响其他阶段                           |
| GitNexus                                                        | 代码影响分析、符号依赖图 | 推荐          | 索引陈旧时 agent 发出警告但不阻断；审查覆盖面可能下降  |

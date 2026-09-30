## 4. 核心原则

### 4.1 简洁至上

上下文窗口是共享资源——你的 Skill 与对话历史、其他 Skill、系统提示共同竞争空间。
**默认假设**：Agent 本身已经非常聪明。只添加它不具备的知识。
对每一段内容提出质疑：

- "Agent 真的需要这个解释吗？"
- "这个知识是 Agent 已有的常识吗？"
- "这段话值得占用的 token 成本吗？"

**好的示例**（简洁，~50 token）：

````markdown
## 提取 PDF 文本

使用 pdfplumber：

```python
import pdfplumber

with pdfplumber.open("file.pdf") as pdf:
    text = pdf.pages[0].extract_text()
```
````

````

**坏的示例**（冗长，~150 token）：
```markdown
## 提取 PDF 文本

PDF（便携文档格式）是一种常见的文件格式，包含文本、图像和其他内容。
要从 PDF 中提取文本，你需要使用一个库。有很多可用的 PDF 处理库，
但我们推荐 pdfplumber，因为它易于使用且处理大多数情况...
````

### 4.2 渐进式展开（Progressive Disclosure）

将核心指令放在 `SKILL.md` 中，将详细参考资料放在单独文件中。Agent 仅在需要时读取辅助文件。

```markdown
# Code Review

## Quick Start

[核心指令在这里]

## Reference

- 详细编码标准: [STANDARDS.md](STANDARDS.md)
- 审查示例: [examples.md](examples.md)
- 脚本说明: [scripts-reference.md](scripts-reference.md)
```

**关键规则**：引用深度不超过一层。所有辅助文件直接从 `SKILL.md` 链接。深层嵌套引用可能导致 Agent 只部分读取文件。

### 4.3 SKILL.md 控制在 500 行以内

超过 500 行的 `SKILL.md` 会降低 Agent 的理解效率。将详细内容拆分到辅助文件中。

### 4.4 自由度匹配

根据任务的脆弱性和变化性，调整指令的具体程度：

| 自由度 | 适用场景                     | 示例                 |
| ------ | ---------------------------- | -------------------- |
| **高** | 多种方式均有效，取决于上下文 | 代码审查指南         |
| **中** | 有首选模式，可接受适当变化   | 报告生成模板         |
| **低** | 操作脆弱，一致性至关重要     | 数据库迁移、告警配置 |

**类比**：把 Agent 想象成在路径上行走的机器人——

- **悬崖边的窄桥**：只有一种安全方式。提供精确的护栏和具体指令（低自由度）。
- **没有障碍的开阔地**：多条路径通向成功。给出方向并信任 Agent 自行选择（高自由度）。

### 4.5 术语一致性

在整个 Skill 中选择一个术语并始终使用它。以下是从成熟 Skill 中总结的标准化用语规范。

#### 4.5.1 章节命名标准

Skill 正文的章节标题应使用一致的命名，以下是推荐的标准章节名及其用途：

| 标准章节名                            | 用途                                               |
| ------------------------------------- | -------------------------------------------------- | ------------------------------------- |
| **Overview**                          | 1-2 句核心原则，概述 Skill 的目标                  |
| **When to Use** / **When NOT to Use** | 明确适用和排除场景                                 |
| **The Iron Law**                      | Skill 的最核心、不可违背的单条规则，用代码块强调   |
| **The Process** / **Checklist**       | 主工作流的步骤定义                                 |
| **Step N: [描述]**                    | 有序的执行步骤（用于顺序流程）                     |
| **Phase N: [描述]**                   | 主要阶段划分（用于阶段性流程，各阶段内可含子步骤） |
| **Quick Reference**                   | 汇总表格，便于快速扫描关键信息                     |
| **Common Mistakes**                   | 常见错误 + 修复方式的配对列表                      |
| **Common Rationalizations**           | "借口                                              | 现实" 对照表，封堵 Agent 的合理化绕过 |
| **Red Flags - STOP**                  | 触发立即停止的信号列表                             |
| **Integration**                       | 与其他 Skill 的关联关系                            |
| **The Bottom Line** / **Final Rule**  | 结尾总结，重申核心原则                             |

**规则**：不要自造章节名。优先使用上表中的标准名称。如果需要自定义章节，确保语义明确、风格一致。

#### 4.5.2 指令强度标记

使用一致的标记词表达指令的强制程度：

| 强度        | 标记词                       | 含义                           | 用法示例                                                                 |
| ----------- | ---------------------------- | ------------------------------ | ------------------------------------------------------------------------ |
| 🔴 绝对禁止 | **NEVER**                    | 任何情况下都不可以做           | "**Never** fix bugs without a test."                                     |
| 🔴 绝对要求 | **MUST** / **MANDATORY**     | 任何情况下都必须做，跳过即违规 | "You **MUST** create a task for each item."                              |
| 🔴 立即停止 | **STOP**                     | 触发条件满足时立即中断当前操作 | "**STOP** executing immediately when..."                                 |
| 🟡 强制要求 | **REQUIRED**                 | 必须完成，但允许标记为 skipped | "**REQUIRED SUB-SKILL:** Use superpowers:finishing-a-development-branch" |
| 🟡 禁止     | **DO NOT** / **Don't**       | 明确禁止某操作                 | "**Do NOT** invoke any implementation skill."                            |
| 🟡 始终     | **ALWAYS**                   | 无例外地执行                   | "**Always** verify tests before offering options."                       |
| 🟢 推荐     | **Recommended** / **Prefer** | 最佳实践，有充分理由可不遵循   | "**Prefer** multiple choice questions when possible."                    |

**规则**：同一 Skill 内不要混用同义的强度标记（如同时用 `MUST` 和 `REQUIRED` 表达同一强度）。选定后保持一致。

#### 4.5.3 门控与硬停用语

当需要阻止 Agent 在条件满足前继续执行时，使用以下标准化模式：

```markdown
# XML 标签式硬门控（最强，用于绝对禁止提前行动）

<HARD-GATE>
Do NOT write any code or take any implementation action
until you have presented a design and the user has approved it.
</HARD-GATE>

# 前置条件式门控（用于需要用户输入后才能继续）

**MANDATORY — do not implement until user has answered:**
Use AskQuestion to confirm:

1. ...
2. ...
   Only after the user confirms, execute the steps.

# 验证式门控（用于步骤间的质量检查）

**Before presenting options, verify tests pass.**
If tests fail: Stop. Don't proceed to Step 2.
```

#### 4.5.4 开场声明

Skill 激活时应有明确的开场声明，让用户知道当前使用了哪个 Skill：

```markdown
**Announce at start:** "I'm using the [skill-name] skill to [purpose]."
```

实际示例：

- `"I'm using the finishing-a-development-branch skill to complete this work."`
- `"I'm using Subagent-Driven Development to execute this plan."`

#### 4.5.5 核心原则声明

每个 Skill 应在 Overview 中用 **Core principle:** 前缀声明其核心哲学，一句话概括：

```markdown
**Core principle:** Evidence before claims, always.
**Core principle:** ALWAYS find root cause before attempting fixes.
**Core principle:** If you didn't watch the test fail, you don't know if it tests the right thing.
**Core principle:** Fresh subagent per task + two-stage review = high quality, fast iteration.
**Core principle:** Verify tests → Present options → Execute choice → Clean up.
```

#### 4.5.6 铁律（The Iron Law）

对于纪律性 Skill，使用代码块形式声明最核心的单条规则：

```markdown
## The Iron Law
```

NO PRODUCTION CODE WITHOUT A FAILING TEST FIRST

```

```

NO FIXES WITHOUT ROOT CAUSE INVESTIGATION FIRST

```

```

NO COMPLETION CLAIMS WITHOUT FRESH VERIFICATION EVIDENCE

```

```

**规则**：每个 Skill 最多一条铁律。铁律用全大写英文、代码块包裹，不用于普通规则。

#### 4.5.7 进度与状态用语

在涉及 TodoWrite 和步骤追踪时，使用以下标准状态词：

| 状态词                 | 语境           | 用法                 |
| ---------------------- | -------------- | -------------------- |
| `pending`              | TodoWrite 状态 | 尚未开始             |
| `in_progress`          | TodoWrite 状态 | 正在执行             |
| `completed`            | TodoWrite 状态 | 已完成               |
| `completed (skipped)`  | TodoWrite 状态 | 条件不满足，标记跳过 |
| **DONE**               | 子代理返回状态 | 任务完成             |
| **DONE_WITH_CONCERNS** | 子代理返回状态 | 完成但有疑虑         |
| **NEEDS_CONTEXT**      | 子代理返回状态 | 需要更多上下文信息   |
| **BLOCKED**            | 子代理返回状态 | 被阻塞，无法继续     |

步骤状态更新的标准写法：

```markdown
**Status Update**: Call `TodoWrite` to mark `step-3` as `in_progress`.
**Completion**: Call `TodoWrite` to mark `step-3` as `completed`.
**Completion**: Mark `step-2` as `completed` (skipped — not applicable for local mode).
```

#### 4.5.8 跨 Skill 引用

引用其他 Skill 时，使用以下标准格式：

```markdown
# 必需的子 Skill（执行过程中必须调用）

**REQUIRED SUB-SKILL:** Use superpowers:finishing-a-development-branch

# 必需的前置知识（需要先理解才能使用当前 Skill）

**REQUIRED BACKGROUND:** You MUST understand superpowers:test-driven-development

# 关联 Skill（可选，提供补充能力）

**Related skills:**

- **superpowers:test-driven-development** — For creating failing test case

# 被调用关系

**Called by:**

- **subagent-driven-development** (Step 7)
- **executing-plans** (Step 5)
```

**禁止**：不要使用 `@skills/path/SKILL.md` 语法引用 Skill，这会强制加载文件到上下文，消耗大量 token。

#### 4.5.9 验证结果标记

在表示通过/失败、正确/错误时，使用一致的视觉标记：

```markdown
# 在行内检查结果中

✅ Spec compliant — all requirements met
❌ Issues: Missing progress reporting

# 在代码审查中

✅ [Run test command] [See: 34/34 pass] "All tests pass"
❌ "Should pass now" / "Looks correct"

# 在正反对比中

✅ Good: `lower-case-with-hyphens`
❌ Avoid: `CamelCase`, `under_score`
```

#### ~~4.5.10 合理性反驳表标准格式~~

~~表头统一使用 ~~`~~Excuse | Reality~~`~~（不混用 ~~`~~Reason~~`~~、~~`~~Problem~~`~~、~~`~~Justification~~`~~ 等）：~~

```markdown
## Common Rationalizations

| Excuse                  | Reality                                    |
| ----------------------- | ------------------------------------------ |
| "Too simple to test"    | Simple code breaks. Test takes 30 seconds. |
| "Emergency, no time"    | Systematic is FASTER than guess-and-check. |
| "I'll write test after" | Tests passing immediately prove nothing.   |
```

#### 4.5.11 领域术语统一

在同一 Skill 内，避免同义词混用。选定一个术语后始终使用它：

| 推荐术语             | 避免混用                                                   |
| -------------------- | ---------------------------------------------------------- |
| "your human partner" | "user"、"developer"、"operator"（指与 Agent 协作的人）     |
| "subagent"           | "sub-agent"、"child agent"、"worker"（指被调度的子代理）   |
| "plan" / "spec"      | "document"、"blueprint"、"proposal"（指实施计划/设计文档） |
| "task"               | "item"、"step"、"ticket"（指 TodoWrite 中的工作项）        |
| "verify"             | "check"、"validate"、"confirm"（指运行命令确认结果）       |
| "dispatch"           | "launch"、"spawn"、"create"（指调度子代理）                |
| "blocker"            | "issue"、"problem"、"impediment"（指阻塞执行的障碍）       |
| "worktree"           | "workspace"、"directory"（指 git worktree）                |
| "base branch"        | "main branch"、"target branch"（指合并目标分支）           |
| "feature branch"     | "dev branch"、"working branch"（指开发分支）               |

---

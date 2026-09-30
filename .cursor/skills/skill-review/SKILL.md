---
name: skill-review
description: >
  the project Cursor SKILLS Best Practice. 
  Use when writing or reviewing a SKILL.md file in Cursor, designing skill structure, choosing Rules vs Skills, writing description fields, avoiding anti-patterns.
  Triggers: write skill, review skill, SKILL.md, skill description, 反模式, skills best practice, skills vs rules.
---

# SKILLS Best Practice — Section Index

> Source: https://the project.feishu.cn/wiki/G31swo2F5ii9cikCFeMcELgYn8e
> (Human reference only — agents use local `references/` files below.)
> **Read only the reference file relevant to your question.**

## How to Use

1. Find your topic in the table below.
2. Read only that `references/` file — do NOT load all files at once.
3. For overlapping topics, read 2–3 files at most.

## Section Index

| Topic                               | File                                     | One-line Summary                                                       |
| ----------------------------------- | ---------------------------------------- | ---------------------------------------------------------------------- |
| How to review a skill (runbook) | `references/review-runbook.md`           | 4-step review flow: index → select files → checklist → report          |
| What is a Skill?                    | `references/01-overview.md`              | Skill 定义、与对话历史的区别、核心前提                                 |
| Skills vs Rules 选择                | `references/02-skills-vs-rules.md`       | `.mdc` vs `SKILL.md` 对比表与决策树                                    |
| 文件结构 & 存储位置                 | `references/03-file-structure.md`        | 目录布局、`SKILL.md` 基本结构、元数据字段                              |
| 核心原则（简洁/门控/自由度）        | `references/04-core-principles.md`       | 4.1–4.5：简洁、渐进式展开、500行限制、自由度、门控模式                 |
| Description 写法                    | `references/05-description-writing.md`   | 黄金规则、编写公式、触发词、示例对比                                   |
| SKILL.md 正文结构                   | `references/06-body-structure.md`        | 推荐章节组织、章节规模匹配复杂度                                       |
| 常用设计模式                        | `references/07-design-patterns.md`       | 工作流、门控、模板、示例、条件分支、反馈循环、合理性反驳表             |
| Cursor 特有功能                     | `references/08-cursor-features.md`       | TodoWrite、AskQuestion 门控、Task 子代理调度、disable-model-invocation |
| 反模式                              | `references/09-anti-patterns.md`         | 9 种常见错误：叙事性内容、过多选项、时效信息、Windows 路径等           |
| 测试与迭代                          | `references/10-testing.md`               | 评估驱动开发、与 Agent 协作开发、纪律性 Skill 压力测试                 |
| 发布前检查清单                      | `references/11-pre-release-checklist.md` | 核心质量、结构、脚本、Cursor 功能、测试的完整 checklist                |
| 实际示例剖析                        | `references/examples.md`                 | add-alert-and-popup、code-review、systematic-debugging 三个案例分析    |
| Claude → Cursor 适配                | `references/cursor-adaptations.md`       | 从 Claude Best Practices 到 Cursor 的关键差异                          |

## Quick Lookup

**"如何审查一个 skill？"** → `references/review-runbook.md`

**"我应该用 Rule 还是 Skill？"** → `references/02-skills-vs-rules.md`

**"怎么写 description？"** → `references/05-description-writing.md`

**"Skill 里怎么用 TodoWrite / AskQuestion / Task？"** → `references/08-cursor-features.md`

**"我的 Skill 有哪些常见错误？"** → `references/09-anti-patterns.md`

**"发布前要检查什么？"** → `references/11-pre-release-checklist.md`

**"怎么设计门控让 Agent 不跳步骤？"** → `references/04-core-principles.md`

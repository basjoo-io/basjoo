## 5. 编写有效的 Description

`description` 是 Skill 被发现和激活的关键。Agent 在启动时加载所有 Skill 的 `name` 和 `description` 元数据，并据此判断何时读取完整的 `SKILL.md`。

### 5.1 黄金规则

**Description = 何时使用（When），而非做什么（What）。**
这是从实际测试中得到的关键发现：当 description 概括了 Skill 的工作流程时，Agent 可能会直接按照 description 行动，跳过阅读完整的 `SKILL.md`。

```yaml
# ❌ 坏：概括了工作流 → Agent 可能按 description 执行而跳过正文
description: >-
  Code review between tasks - dispatches subagent per task
  with code review between tasks

# ❌ 坏：流程细节太多
description: >-
  Use for TDD - write test first, watch it fail,
  write minimal code, refactor

# ✅ 好：只描述触发条件，不透露流程
description: >-
  Use when executing implementation plans with independent
  tasks in the current session

# ✅ 好：触发条件 + 能力描述
description: >-
  Review code for correctness, style, and performance
  following Google Style Guides and team standards.
  Use when reviewing pull requests, diffs, or when the
  user asks for code review.
```

### 5.2 编写公式

```plaintext
[做什么的一句话概述]。Use when [具体触发场景/症状/关键词]。
```

**包含两部分**：

1. **WHAT**：Skill 的能力（简洁）
2. **WHEN**：触发场景（具体，含关键词）

### 5.3 第三人称

Description 被注入到系统提示中，视角不一致会导致发现问题：

- ✅ "Processes Excel files and generates reports"
- ❌ "I can help you process Excel files"
- ❌ "You can use this to process Excel files"

### 5.4 包含多语言触发词

如果你的用户可能使用不同语言发出请求，在 description 中包含多语言关键词：

```yaml
# 项目中实际使用的示例
description: >-
  Adds a new alert and configures its popup text.
  Use when the user asks to add an alert, 添加告警,
  配置弹窗, 弹窗内容, set popup for alert.
```

### 5.5 示例对比

| 场景     | ❌ 模糊                | ✅ 具体                                                                                                                                                                                 |
| -------- | ---------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| PDF 处理 | "Helps with documents" | "Extract text and tables from PDF files, fill forms, merge documents. Use when working with PDF files or when the user mentions PDFs, forms, or document extraction."                   |
| Git 提交 | "Does stuff with git"  | "Generate descriptive commit messages by analyzing git diffs. Use when the user asks for help writing commit messages or reviewing staged changes."                                     |
| 代码审查 | "Reviews code"         | "Review code for correctness, style, and performance following Google Style Guides and team standards. Use when reviewing pull requests, diffs, or when the user asks for code review." |

---

## 8. Cursor 特有功能集成

### 8.1 TodoWrite 集成

`TodoWrite` 是 Cursor 的任务追踪工具（tool call，非函数调用）。Skill 通过文本指令驱动 Agent 调用此工具：

```markdown
**MANDATORY**: At session start, call the `TodoWrite` tool with:

- `merge`: false
- `todos`: array of { id, content, status } objects

Example instruction in a SKILL.md:

## Step 1: Initialize Execution Roadmap (MANDATORY)

**IMMEDIATELY** upon starting, call `TodoWrite` to create the roadmap:

- id: "step-1", content: "Step 1: Analyze inputs", status: "in_progress"
- id: "step-2", content: "Step 2: Run tools", status: "pending"
- id: "step-3", content: "Step 3: Generate report", status: "pending"
```

**最佳实践**：

- 在 Skill 开头声明 `MANDATORY: use TodoWrite`
- 为主要步骤定义 todo 项
- 每步完成时标记为 `completed`
- 对于复杂步骤，可嵌套子步骤

### 8.2 AskQuestion 门控

`AskQuestion` 工具允许以结构化方式收集用户输入，比自由文本问答更高效。
在 Skill 中通过文本指令驱动 Agent 调用此工具：

```markdown
Call the AskQuestion tool with:

- title: "Alert Configuration"
- questions:
  - id: "team", prompt: "Which team?",
    options: [{id: "common", label: "Common"}, {id: "planning", label: "Planning"},
    {id: "perception", label: "Perception"}, {id: "control", label: "Control"}]
  - id: "severity", prompt: "Severity?",
    options: [{id: "info", label: "INFO"}, {id: "warning", label: "WARNING"},
    {id: "error", label: "ERROR"}, {id: "fatal", label: "FATAL"}]
  - id: "features", prompt: "Features?", allow_multiple: true,
    options: [{id: "popup", label: "popup"}, {id: "voice", label: "voice"},
    {id: "issue", label: "issue"}]
```

### 8.3 子代理（Task）调度

对于大型 Skill，可以调度子代理并行处理：

```markdown
## Step 4: Dispatch Sub-Agents

Use max concurrent sub-agents = 6.

- Split files into balanced shards (2-6 files per shard)
- Each sub-agent gets ONE file shard for ONE scope
- Use Task tool with subagent_type="generalPurpose"
```

### 8.4 `disable-model-invocation` 标志

设置此标志后，Agent 不会自动触发此 Skill——只有用户显式调用时才激活：

```yaml
---
name: shell
description: >-
  Runs the rest of a /shell request as a literal shell command.
  Use only when the user explicitly invokes /shell.
disable-model-invocation: true
---
```

**适用场景**：

- 命令行直通（如 `/shell`）
- 可能产生副作用的危险操作
- 仅在特定命令触发时需要的 Skill

### 8.5 辅助脚本

预制脚本比 Agent 临时生成的代码更可靠：

```plaintext
code-review/
├── SKILL.md
├── templates.md
├── adjudication-guide.md
└── scripts/
    ├── fetch_and_checkout_remote_branch.sh
    ├── fetch_pr_comments.py
    ├── review_precheck.py
    └── clang_tidy_review.py
```

**优势**：

- 比生成代码更可靠
- 节省 token（无需在上下文中包含代码）
- 节省时间（无需代码生成）
- 确保跨次调用的一致性

**明确执行意图**：在 Skill 中说清楚是"运行"还是"阅读"脚本：

- "Run `analyze_form.py` to extract fields"（执行）
- "See `analyze_form.py` for the extraction algorithm"（阅读参考）

---

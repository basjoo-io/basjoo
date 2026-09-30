## 7. 常用设计模式

### 7.1 工作流模式（Workflow）

将复杂操作分解为清晰的步骤，配合检查清单追踪进度。

```markdown
## 告警添加工作流

| Step | Where                            | Action            |
| ---- | -------------------------------- | ----------------- |
| 1    | `alert_id/<team>_alert_id.proto` | 添加新枚举值      |
| 2    | Business code                    | 调用 TriggerAlert |
| 3    | Module's BUILD                   | 添加依赖          |
| 4    | `alert_property.pb.txt`          | 配置告警属性      |
| 5    | (Optional) i18n files            | 添加弹窗文案      |
```

**适用场景**：数据库迁移、配置添加、部署流程等多步骤操作。

### 7.2 门控模式（Gating）

在执行前强制 Agent 收集必要信息：

```markdown
**MANDATORY — do not implement until user has answered:**
Use AskQuestion to confirm:

1. **Which team?** (Common / Planning / Perception / Control)
2. **Alert name and enum value?**
3. **Which optional features?** (popup / voice / issue)
4. **Severity level?** (INFO / WARNING / ERROR / FATAL)

Only after the user confirms, execute the steps.
```

更强的门控使用 XML 标签强调：

```markdown
<HARD-GATE>
Do NOT write any code or take any implementation action
until you have presented a design and the user has approved it.
</HARD-GATE>
```

**适用场景**：需要用户决策的操作、不可逆操作、多选项配置。

### 7.3 模板模式（Template）

提供输出格式模板，确保一致性：

```markdown
## Report Structure

ALWAYS use this template:

\`\`\`markdown

# [Analysis Title]

## Executive Summary

[One-paragraph overview]

## Key Findings

- Finding 1 with data
- Finding 2 with data

## Recommendations

1. Actionable recommendation
2. Actionable recommendation
   \`\`\`
```

### 7.4 示例模式（Examples）

当输出质量依赖于看到示例时，提供输入/输出对：

```markdown
## Commit Message Format

**Example 1:**
Input: Added user authentication with JWT tokens
Output:
```

feat(auth): implement JWT-based authentication

Add login endpoint and token validation middleware

```

**Example 2:**
Input: Fixed bug where dates displayed incorrectly
Output:
```

fix(reports): correct date formatting in timezone conversion

```

```

**原则**：一个优秀的示例胜过多个平庸的示例。

### 7.5 条件分支模式（Conditional Workflow）

引导 Agent 在决策点做出正确选择：

```markdown
## Document Modification

1. Determine the modification type:

   **Creating new content?** → Follow "Creation workflow"
   **Editing existing content?** → Follow "Editing workflow"
```

对于复杂的决策逻辑，可使用 Graphviz 流程图（Cursor 支持 `dot` 格式渲染）：

```plaintext
digraph workflow {
    "Input type?" [shape=diamond];
    "PR URL" [shape=box];
    "Local diff" [shape=box];
    "File list" [shape=box];

    "Input type?" -> "PR URL" [label="GitHub URL"];
    "Input type?" -> "Local diff" [label="git diff"];
    "Input type?" -> "File list" [label="user provided"];
}
```

### 7.6 反馈循环模式（Feedback Loop）

对于质量关键的任务，实现"执行 → 验证 → 修复 → 重复"的循环：

```markdown
## Document Editing Process

1. Make your edits
2. **Validate immediately**: `python scripts/validate.py output/`
3. If validation fails:
   - Review the error message
   - Fix the issues
   - Run validation again
4. **Only proceed when validation passes**
```

### 7.7 合理性反驳表（Rationalization Table）

对于纪律性 Skill（强制 Agent 遵守特定流程），预先封堵 Agent 可能的"合理化借口"：

```markdown
## Common Rationalizations

| Excuse                       | Reality                                     |
| ---------------------------- | ------------------------------------------- |
| "Too simple to need process" | Simple bugs have root causes too.           |
| "Emergency, no time"         | Systematic is FASTER than guess-and-check.  |
| "I'll write test after"      | Tests-after prove nothing. Test first.      |
| "Multiple fixes saves time"  | Can't isolate what worked. Causes new bugs. |
```

**适用场景**：TDD 流程、调试流程、代码审查标准等需要严格遵守的 Skill。

---

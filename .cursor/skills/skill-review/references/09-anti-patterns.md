## 9. 反模式

### 9.1 ❌ 叙事性内容

```markdown
# 坏：像讲故事一样描述

"In session 2025-10-03, we found that the empty projectDir caused..."

# 好：直接给出规则和解决方案

## Empty Directory Handling

If `projectDir` is empty, create it with default content.
```

### 9.2 ❌ 过多选项导致选择困难

```markdown
# 坏：列举太多选择

"You can use pypdf, or pdfplumber, or PyMuPDF, or pdf2image, or..."

# 好：给出默认推荐 + 备选

"Use pdfplumber for text extraction.
For scanned PDFs requiring OCR, use pdf2image with pytesseract instead."
```

### 9.3 ❌ 时效性信息

```markdown
# 坏：会过时

"If you're doing this before August 2025, use the old API."

# 好：用 "旧模式" 折叠区

## Current Method

Use the v2 API endpoint.

## Old Patterns (deprecated)

<summary>Legacy v1 API</summary>
...
```

### 9.4 ❌ Windows 路径格式

```markdown
# 坏

scripts\helper.py

# 好

scripts/helper.py
```

### 9.5 ❌ 模糊的 Skill 名称

```markdown
# 坏：含义不明

helper, utils, tools, documents

# 好：动名词形式，清楚描述行为

processing-pdfs, analyzing-spreadsheets, reviewing-code
```

### 9.6 ❌ 深层嵌套引用

```markdown
# 坏：引用链过深

SKILL.md → advanced.md → details.md → actual-info.md

# 好：从 SKILL.md 直接链接所有资料（一层深度）

SKILL.md → advanced.md
SKILL.md → details.md
SKILL.md → actual-info.md
```

### 9.7 ❌ Description 中概括工作流

```yaml
# 坏：Agent 可能按 description 执行而跳过正文
description: Dispatches subagent per task with code review between tasks

# 好：只描述触发条件
description: Use when executing implementation plans with independent tasks
```

### 9.8 ❌ 在流程图中放置代码

```markdown
# 坏：无法复制粘贴，难以阅读

step1 [label="import fs"];
step2 [label="read file"];

# 好：流程图只用于决策逻辑，代码放在代码块中
```

---

## 3. Skill 文件结构

### 目录布局

每个 Skill 是一个目录，包含必需的 `SKILL.md` 和可选的辅助文件：

```plaintext
skill-name/
├── SKILL.md              # 必需 — 主指令文件
├── reference.md          # 可选 — 详细参考资料
├── templates.md          # 可选 — 模板定义
├── examples.md           # 可选 — 使用示例
└── scripts/              # 可选 — 工具脚本
    ├── validate.py
    └── helper.sh
```

### 存储位置

| 路径                           |
| ------------------------------ |
| `.cursor/skills/<skill-name>/` |

**禁止**：不要在 `~/.cursor/skills-cursor/` 下创建 Skill，该目录由 Cursor 系统内部管理。

### SKILL.md 基本结构

```markdown
---
name: your-skill-name
description: >-
  Brief description of what this skill does and when to use it.
  Use when [specific trigger conditions].
---

# Skill Title

## Instructions

Clear, step-by-step guidance for the agent.

## Examples

Concrete examples demonstrating the skill.
```

### 元数据字段

| 字段                       | 必需 | 说明                                                                                                  | 内部规范                                                                                                                                                                |
| -------------------------- | ---- | ----------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `name`                     | 是   | Skill 标识符。仅允许小写字母、数字和连字符，**必须与父文件夹名一致**。                                | 使用 `lower-case-with-hyphens` 格式，与 `.mdc` 文件命名风格保持一致。用描述性动词短语命名，如 `add-alert-and-popup`、`code-review`。 |
| `description`              | 是   | 描述 Skill 的功能和适用场景。Agent 据此判断是否与当前任务相关并决定是否加载。                         | 使用英文撰写，但**同时包含中文触发词**以支持双语使用（如 `添加告警, 配置弹窗`）。格式：`[能力概述]. Use when [触发场景/中文关键词].`                                    |
| `license`                  | 否   | 许可证名称或对随附许可证文件的引用。                                                                  | 项目 Skill（`.cursor/skills/`）无需填写，也不需要添加版权声明，默认受项目整体许可证约束。仅对外发布的共享 Skill 需显式声明。                                            |
| `compatibility`            | 否   | 环境要求（系统依赖包、网络访问等）。                                                                  | 如果 Skill 依赖特定工具链（如 `clang-tidy`、`clang-format`）或 Python 脚本，应在此注明所需工具及获取方式。                                                                    |
| `metadata`                 | 否   | 任意键值映射，用于附加元数据。                                                                        | 可用于标记 Skill 归属团队（如 `team: planning`）或版本（如 `version: v4`），目前非强制要求。                                                                            |
| `disable-model-invocation` | 否   | 设为 `true` 时，Skill 仅在通过 `/skill-name` 显式调用时才加载。Agent 不会根据上下文自动应用此 Skill。 | 仅用于有副作用或需显式触发的 Skill。大多数项目 Skill 应**省略此字段**（默认允许自动触发）。                                                                             |

**name与文件夹名一致性**示例：

```yaml
add-alert-and-popup/     ← 文件夹名
├── SKILL.md             ← name: add-alert-and-popup  ✅ 一致

# ❌ 不匹配：文件夹名 add-alert，但 name 写了 add-alert-and-popup
name: add-alert-and-popup

# ✅ 匹配：文件夹名和 name 完全一致
name: add-alert-and-popup
```

---

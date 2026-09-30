## 附录 B：从 Claude Best Practices 到 Cursor 的适配要点

| Claude 概念                      | Cursor 对应                                             |
| -------------------------------- | ------------------------------------------------------- |
| YAML frontmatter                 | 相同格式，额外支持 `disable-model-invocation`           |
| Skill 目录在 `~/.claude/skills/` | `~/.cursor/skills/`（个人）或 `.cursor/skills/`（项目） |
| `Skill` 工具加载                 | Cursor 基于 description 自动发现                        |
| Bash/Read 工具访问文件           | Cursor 的 Read/Shell/Grep 等工具                        |
| 代码执行环境                     | Cursor IDE 中的终端和工具                               |
| Context window 管理              | 相同原则，Cursor 中 Rules 也竞争 token                  |
| MCP 工具引用                     | `ServerName:tool_name` 全限定名格式                     |

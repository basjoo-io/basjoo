## 附录 A：实际项目中的 Skill 示例剖析

### 示例 1：`add-alert-and-popup`（低自由度 + 门控）

**特点**：

- 强制门控：执行前必须用 AskQuestion 确认 4 个参数
- 表格式检查清单：每步的位置和操作清晰定义
- 条件步骤：步骤 5/6/7 根据用户选择可选执行
- 多语言触发词：description 包含中英文关键词
- 跨车辆配置：明确要求在所有车辆目录下添加相同配置

### 示例 2：`code-review`（高复杂度编排器）

**特点**：

- 强制 TodoWrite：第一步就初始化完整的执行路线图
- 8 步工作流：从获取 PR 到清理工作区的完整流程
- 脚本化工具：`review_precheck.py`、`clang_tidy_review.py` 处理 ~60% 的规则
- 渐进式展开：`templates.md`、`adjudication-guide.md`、`scripts-reference.md` 按需加载
- 子代理调度：Step 4 并行分发最多 6 个子代理
- 条件分支：PR 模式 vs 本地模式有不同的处理路径
- 快速路径优化：小型 PR 可跳过子代理直接内联审查

### 示例 3：`systematic-debugging`（纪律性 Skill）

**特点**：

- "铁律"声明：`NO FIXES WITHOUT ROOT CAUSE INVESTIGATION FIRST`
- 四阶段流程：根因调查 → 模式分析 → 假设测试 → 实施
- 红旗列表：Agent 自检是否在合理化绕过流程
- 合理性反驳表：预先封堵 8 种常见借口
- 3 次失败阈值：尝试 3+ 次修复后强制质疑架构
- 跨技能引用：`superpowers:test-driven-development`、`superpowers:verification-before-completion`

---

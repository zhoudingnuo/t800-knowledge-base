<!-- 来源: https://engineai.com.cn/open/docs/t800/native-sdk/4?product=pm01 -->

# 4. 自定义 Runner

## 1 第一步：拷贝 `passive` 骨架

假设需要创建的 runner 叫 demo_runner

- 拷贝 Runner 目录：`src/runner/passive` -> `src/runner/demo`
- 拷贝参数类目录：`src/data/_param/passive_param` -> `src/data/_param/demo_param`
- 拷贝参数配置：`assets/config/<robot>/passive/default.yaml` -> `assets/config/<robot>/demo/default.yaml`
- 关键替换部分速查表：
 | 替换项 | passive 模板 | demo 示例 | 说明
 | 参数类名与默认 tag
 | `PassiveParam("passive")` | `DemoParam("demo")`
 | 需要与 `mode.yaml`/`param_tag` 对齐
 | Runner 中参数类型 | `std::shared_ptr<data::PassiveParam>` | `std::shared_ptr<data::DemoParam>` | 包括构造和 `Enter()` 里的 `create<...>()`
 | 注册宏字符串 | `REGISTER_RUNNER(PassiveRunner, "passive_runner", kMotion)` | `REGISTER_RUNNER(DemoRunner, "demo_runner", kMotion)` | 必须与 task_motion 里的 `runner[].name` 完全一致

 | 参数配置路径 | `assets/config/<robot>/passive/default.yaml` | `assets/config/<robot>/demo/default.yaml` | 后续在 `mode.yaml` 中通过 `scope` 引用

## 2 第二步：task 中增加 runner

- 在 `assets/config/<robot>/mode.yaml`（`sim` 和 `robot`）都加上新 `tag -> scope`
 | YAML
```
- tag: demo
  scope: demo/default.yaml        # 与新增的参数配置路径一致
```

- 在 `task_motion/*.yaml` 或 `task_resident/*.yaml` 增加任务条目，挂载：

- 假设增加的 demo_runner 是 motion task
 | YAML
```
- motion: demo                    # task名
  period: 0.002                   # runner的运行频率
  runner:                         # 该task需要的runner列表
    - name: demo_runner           # 新增的runner，名字与**REGISTER_RUNNER一致**
      enabled: true
      param_tag: sitting          # 名字与mode.yaml中的tag一致
  manual_transition:              # demo只能切换这个列表中的motion
    - passive
  key: [LB, X]                    # 绑定切入该motion需要的遥控器按键
```

## 3 第三步：扩展参数类

- 按需新增 `LOAD_PARAM(...)` 字段，并在 `Update()` 中补齐读取逻辑。
- 需要约束时可补 `Check()`（范围、维度、非空等）。

例子：如果你的 Runner 需要加载模型文件（如策略网络）

3.1 在参数 YAML 中新增字段（例如 `policy_path`）：

 | YAML
```
policy_file: "demo/policies/policy.mnn"
```

3.2 在参数类中新增读取字段：

 | C++
```
class DemoParam : public BasicParam {
 public:
  DemoParam(std::string_view tag = "demo") : BasicParam(tag) {}

  std::string LOAD_PARAM(policy_file);

  void Update() override { LOAD_PARAM(policy_file); }
};
```

3.3 在 Runner `Enter()` 中加载模型：

 | C++
```
bool DemoRunner::Enter() {
  // 加载参数
  if (!param_tag_.empty()) {
    param_ = data::ParamManager::create<data::DemoParam>(param_tag_);
  }

  // 加载模型
  model_ = std::make_unique<math::MNNModel>(
      common::PathJoin(common::GlobalPathManager::GetInstance().GetConfigPath(), param_->policy_file));
  return true;
}
```

3.4 在 Runner 头文件中增加成员：`std::unique_ptr<math::MNNModel> model_;`

这样就形成了完整链路：YAML 配置 -> Param `LOAD_PARAM` -> Runner `Enter()` 实际生效

## 4 第四步：扩展 Runner 逻辑

- `motion runner` 额外维护 `SetupContext()` / `TeardownContext()`，管理全局参数
- 在 `Run()` 中持续写输出（`GetMutableOutput().SetCommand(...)`）
- 把耗时的复位逻辑放在 `TryExit`，一次性清理/复位放在 `Exit`

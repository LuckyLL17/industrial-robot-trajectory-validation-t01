# 工业机器人离线轨迹规划与碰撞验证服务

纯后端 Node.js + TypeScript + Fastify 服务，默认无需 Docker、数据库或消息中间件即可运行。

## 启动

```bash
npm install
npm run dev
```

默认端口：`6303`。健康检查：`GET /health`。

## 业务闭环

- 领域状态模型与事件输入
- 可回放的时间序列/仿真流程
- 约束校验与风险门禁
- 方案版本、审计轨迹与异常处置
- 所有状态暂存于进程内，便于本机演示；可扩展为数据库、事件总线和实时流处理架构

## 增强业务闭环

- TCP 工具标定误差和标定有效期管理
- 工件基准坐标系登记
- 动态障碍物上报与清除
- 轨迹发布前的碰撞、标定、动态障碍物多重安全门禁
- 执行中关节跟踪误差和 TCP 力阈值保护停机
- 轨迹 release、执行遥测和安全审计

扩展接口：`/api/calibrations`、`/api/frames`、`/api/dynamic-obstacles`、`/api/paths/:id/release`、`/api/executions/:id/telemetry`。

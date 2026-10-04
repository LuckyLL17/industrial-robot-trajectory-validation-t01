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

## 坐标系验证与轨迹转换

产线换型时把轨迹从车身基准坐标系转换到焊装单元：

- `POST /api/frames/:id/verifications`：提交名义点 `nominal` 与实测点 `measured`（至少 3 对，一一对应），用 Kabsch 算法求最优刚体变换（平移 + Rz·Ry·Rx 欧拉角），返回 RMS/最大/逐点残差，按 `toleranceMm`（默认 2mm）判定通过与否并记录递增验证版本；相同输入重复提交幂等返回，共线等退化几何只记录失败版本，不生成无法解释的变换
- `GET /api/frames/:id/verifications`：验证版本历史（含失败版本）；frame 重新验证后 `currentVerificationId` 只指向最新通过版本，失败不影响已通过的旧版本
- `POST /api/paths/:id/frame-association`：路径只能关联**已验证通过**的 frame，关联锚定具体验证版本（快照），重复提交幂等
- `GET /api/paths/:id/frame-association`：读取关联；frame 重新验证后关联仍锚定旧快照并标记 `stale`，不隐式改变转换结果
- `POST /api/paths/:id/transform-preview`：只读转换预览，返回转换后的 waypoint 副本（位置 + 姿态 + 工艺参数），**不修改原始 waypoints**；默认用关联快照，也可显式传 `frameId` / `verificationVersion` 指定
- `GET /api/paths` / `GET /api/paths/:id`：路径只读接口
- 错误码：`INSUFFICIENT_POINTS`(422)、`POINT_COUNT_MISMATCH` / `MALFORMED_INPUT`(400)、`FRAME_NOT_FOUND` / `PATH_NOT_FOUND`(404)、`FRAME_NOT_VERIFIED` / `NO_FRAME_ASSOCIATED` / `VERIFICATION_VERSION_NOT_PASSED`(409)
- 未关联 frame 的旧路径，其原有 validate/plan/execute/release 闭环行为完全不变；上线前预置的 `frame-body` 保留合成 v1（单位变换，带 `SYNTHETIC_SEED` 警告），建议重新提交实测点验证

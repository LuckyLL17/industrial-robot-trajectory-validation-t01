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

产线换型时，把车身基准坐标系下的轨迹转换到焊装单元坐标系：

- `POST /api/frames/:id/verifications`：提交至少 3 对不共线的 `nominalPoints`（车身系名义点）与 `measuredPoints`（单元系实测点），服务端用 Kabsch（3x3 SVD）求解刚体平移 `translation`、旋转（`rotationEuler` ZYX 内旋 + `rotationMatrix`）和逐点残差（`residualMaxMm` / `residualRmsMm`），按 `toleranceMm`（默认 2mm）判定 `passed`，并生成自增验证版本。
  - 点数不足返回 `400 points-insufficient`，点集共线返回 `400 points-degenerate`；验证失败也会落版本（`passed:false`），frame 状态置为 `failed`，重新验证通过后恢复 `verified`。
  - 相同点集与容差重复提交幂等返回原版本（`idempotent:true`），不会产生重复或难解释的状态。
- `GET /api/frames/:id/verifications`、`GET /api/frames/:id/verifications/:vid`：版本列表与详情（失败版本同样可查）。
- `POST /api/paths/:id/frame`：把路径关联到**已验证**的 frame（可显式指定 `verificationId` 钉住版本，否则取最新通过版本）；重复关联同一版本幂等，关联历史保留在 `frameRefHistory`。
- `POST /api/paths/:id/transform-preview`：基于关联 frame（或 body 临时指定 `frameId` / `verificationId`，`useLatest:true` 取最新通过版本）输出 `transformedWaypoints` 预览，位置与姿态（ZYX 角随旋转复合）一并转换，**不修改路径原始 waypoints**（响应中 `originalWaypointsUnchanged:true`，并回显 `source`）。
- `GET /api/paths/:id/frame`：读取关联状态；未关联返回 `frameRef:null`（路径仍可读），关联的 frame 不存在时返回 `frameExists:false`，存在更新的已验证版本时给出 `updateAvailable`。

规则：frame 不存在 404；frame 从未验证或所指验证失败时关联/预览返回 409 并附验证摘要；frame 最新一次验证失败期间，钉住旧通过版本的路径仍可读取和预览。所有状态仍为进程内暂存。

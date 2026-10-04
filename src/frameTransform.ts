import { randomUUID } from 'node:crypto';
import type { FastifyInstance } from 'fastify';

// 坐标系验证与轨迹转换模块
// - 接收名义点(nominal)/实测点(measured)，用 Kabsch 算法求最优刚体变换（平移+旋转）
// - 记录逐版本的验证结果（含残差），重复提交幂等，不产生难以解释的中间状态
// - 路径可关联“已验证”的 frame；转换预览不修改原始 waypoints
// 旋转约定：R = Rz(rz)·Ry(ry)·Rx(rx)，角度为度（外参 XYZ / 内参 ZYX，工业机器人常用）

type Point3 = { x: number; y: number; z: number };
type Euler = { rx: number; ry: number; rz: number };
type Store = {
  frames: any[];
  paths: any[];
  log: (action: string, payload: any) => void;
};

const DEFAULT_TOLERANCE_MM = 2;

// ---------- 基础向量/矩阵工具 ----------
const I3 = (): number[][] => [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
const dot = (a: number[], b: number[]) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a: number[], b: number[]): number[] => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const vnorm = (a: number[]) => Math.hypot(a[0], a[1], a[2]);
const matMul = (a: number[][], b: number[][]) =>
  a.map((row, i) => [0, 1, 2].map((j) => dot(row, [b[0][j], b[1][j], b[2][j]])));
const matVec = (a: number[][], v: number[]) => a.map((row) => dot(row, v));
const det3 = (m: number[][]) =>
  m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
  m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
  m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

// 对称 3x3 矩阵特征分解：A = V·diag(d)·Vᵀ，特征值降序（Jacobi 循环）
function symEigen3(Ain: number[][]) {
  const a = Ain.map((r) => r.slice());
  const V = I3();
  for (let sweep = 0; sweep < 100; sweep++) {
    let maxOff = 0;
    for (const [p, q] of [[0, 1], [0, 2], [1, 2]] as const) {
      const apq = a[p][q];
      if (Math.abs(apq) < 1e-14) continue;
      maxOff = Math.max(maxOff, Math.abs(apq));
      const tau = (a[q][q] - a[p][p]) / (2 * apq);
      const t = (tau >= 0 ? 1 : -1) / (Math.abs(tau) + Math.sqrt(1 + tau * tau));
      const c = 1 / Math.sqrt(1 + t * t);
      const s = t * c;
      a[p][p] -= t * apq;
      a[q][q] += t * apq;
      a[p][q] = 0;
      a[q][p] = 0;
      for (const r of [0, 1, 2]) {
        if (r === p || r === q) continue;
        const g = a[r][p];
        const h = a[r][q];
        a[r][p] = c * g - s * h;
        a[p][r] = a[r][p];
        a[r][q] = s * g + c * h;
        a[q][r] = a[r][q];
      }
      for (let r = 0; r < 3; r++) {
        const g = V[r][p];
        const h = V[r][q];
        V[r][p] = c * g - s * h;
        V[r][q] = s * g + c * h;
      }
    }
    if (maxOff < 1e-12) break;
  }
  const d = [a[0][0], a[1][1], a[2][2]];
  const order = [0, 1, 2].sort((i, j) => d[j] - d[i]);
  // V[row][col]：列为按特征值降序排列的特征向量
  const Vord = [0, 1, 2].map((row) => order.map((j) => V[row][j]));
  return { d: order.map((i) => d[i]), V: Vord };
}

// ---------- 欧拉角（度）与旋转矩阵互转 ----------
const deg2rad = (d: number) => (d * Math.PI) / 180;
const rad2deg = (r: number) => (r * 180) / Math.PI;
function eulerToMatrix(e: Euler): number[][] {
  const [cx, sx] = [Math.cos(deg2rad(e.rx)), Math.sin(deg2rad(e.rx))];
  const [cy, sy] = [Math.cos(deg2rad(e.ry)), Math.sin(deg2rad(e.ry))];
  const [cz, sz] = [Math.cos(deg2rad(e.rz)), Math.sin(deg2rad(e.rz))];
  const Rx: number[][] = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]];
  const Ry: number[][] = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]];
  const Rz: number[][] = [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]];
  return matMul(matMul(Rz, Ry), Rx);
}
function matrixToEuler(m: number[][]): Euler {
  const clamp = (x: number) => Math.max(-1, Math.min(1, x));
  let rx: number, ry: number, rz: number;
  if (Math.abs(m[2][0]) < 1 - 1e-9) {
    ry = rad2deg(Math.asin(clamp(-m[2][0])));
    rx = Math.atan2(m[2][1], m[2][2]);
    rz = Math.atan2(m[1][0], m[0][0]);
  } else {
    // 万向节锁（pitch ≈ ±90°）
    ry = m[2][0] <= -1 ? 90 : -90;
    rx = Math.atan2(-m[1][2], m[1][1]);
    rz = 0;
  }
  return { rx: rad2deg(rx), ry, rz: rad2deg(rz) };
}
const round6 = (n: number) => Math.round(n * 1e6) / 1e6;

// ---------- Kabsch：名义点 -> 实测点 的最优刚体变换 ----------
function kabsch(nominal: Point3[], measured: Point3[]) {
  const P = nominal.map((p) => [p.x, p.y, p.z]);
  const Q = measured.map((p) => [p.x, p.y, p.z]);
  const centroid = (M: number[][]) => {
    const c = [0, 0, 0];
    for (const row of M) row.forEach((v, i) => (c[i] += v));
    return c.map((v) => v / M.length);
  };
  const cn = centroid(P);
  const cm = centroid(Q);
  const Pc = P.map((r) => r.map((v, i) => v - cn[i]));
  const Qc = Q.map((r) => r.map((v, i) => v - cm[i]));
  // 共线性检查：中心化协方差的最小一维展布相对过小 => 旋转轴不可观
  const spread = (M: number[][]) => {
    const C: number[][] = [0, 1, 2].map(() => [0, 0, 0]);
    for (const r of M)
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[i][j] += r[i] * r[j];
    return symEigen3(C).d.map((d) => Math.sqrt(Math.max(0, d)));
  };
  const degenerate = (s: number[]) => s[2] < 1e-9 * (s[0] || 1);
  const coplanar = (s: number[]) => !degenerate(s) && s[1] < 1e-9 * (s[0] || 1);
  const sp = spread(Pc);
  const sq = spread(Qc);
  if (degenerate(sp) || degenerate(sq))
    return { error: 'DEGENERATE_GEOMETRY', message: '点位近似共线或重合，无法唯一确定旋转' };
  const underdetermined = coplanar(sp) || coplanar(sq);
  // H = Pcᵀ·Qc，对 HᵀH 做特征分解得到 SVD
  const H: number[][] = [0, 1, 2].map(() => [0, 0, 0]);
  for (let k = 0; k < P.length; k++)
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) H[i][j] += Pc[k][i] * Qc[k][j];
  const HtH: number[][] = [0, 1, 2].map((i) =>
    [0, 1, 2].map((j) => dot([H[0][i], H[1][i], H[2][i]], [H[0][j], H[1][j], H[2][j]])),
  );
  const { d, V: Vcols } = symEigen3(HtH);
  const sigma = d.map((x) => Math.sqrt(Math.max(0, x)));
  // U 列 = H·v_j/σ_j；σ 退化时取 u_j = v_j，保证可构造合法旋转
  const Ucols = sigma.map((s, j) => {
    const v = [Vcols[0][j], Vcols[1][j], Vcols[2][j]];
    if (s < 1e-10 * (sigma[0] || 1)) return v;
    const u = matVec(H, v).map((x) => x / s);
    const n = vnorm(u);
    return n < 1e-9 ? v : u.map((x) => x / n);
  });
  const U: number[][] = [0, 1, 2].map((i) => [Ucols[0][i], Ucols[1][i], Ucols[2][i]]);
  const V: number[][] = Vcols;
  // Vᵀ：Vt[i][j] = V[j][i]
  const Vt: number[][] = [0, 1, 2].map((i) => [0, 1, 2].map((j) => V[j][i]));
  const dSign = det3(matMul(U, Vt));
  const Dm: number[][] = [[1, 0, 0], [0, 1, 0], [0, 0, dSign]];
  // 上面解出的 (U,D,V) 是 measured→nominal 的配准 R_qp。
  // 需要 nominal→measured：R = R_qpᵀ（纯旋转），t = cm − R·cn
  const Rqp = matMul(matMul(U, Dm), Vt);
  const R: number[][] = [0, 1, 2].map((i) => [0, 1, 2].map((j) => Rqp[j][i]));
  const t = matVec(R, cn).map((v, i) => cm[i] - v);
  const perPoint = nominal.map((p, i) => {
    const q = matVec(R, [p.x, p.y, p.z]).map((v, j) => v + t[j]);
    const r = [measured[i].x - q[0], measured[i].y - q[1], measured[i].z - q[2]];
    return { index: i, errorMm: round6(vnorm(r)), residual: { x: round6(r[0]), y: round6(r[1]), z: round6(r[2]) } };
  });
  const sse = perPoint.reduce((s, r) => s + r.errorMm ** 2, 0);
  return {
    matrix: R,
    translation: { x: round6(t[0]), y: round6(t[1]), z: round6(t[2]) },
    rotation: matrixToEuler(R),
    warnings: underdetermined
      ? [{ code: 'PLANAR_POINT_SET', message: '点位近似共面，平面法向旋转欠约束，建议补充不在同一平面的测点' }]
      : [],
    residuals: {
      rmsMm: round6(Math.sqrt(sse / P.length)),
      maxMm: round6(Math.max(...perPoint.map((r) => r.errorMm))),
      perPoint,
    },
  };
}

// 用验证得到的刚体变换转换单个 waypoint（返回新对象，不改原值）
function transformWaypoint(w: any, R: number[][], t: number[]) {
  const p = matVec(R, [w.x, w.y, w.z]).map((v, i) => round6(v + t[i]));
  const Rw = eulerToMatrix({ rx: w.rx ?? 0, ry: w.ry ?? 0, rz: w.rz ?? 0 });
  const e = matrixToEuler(matMul(R, Rw));
  return {
    x: p[0], y: p[1], z: p[2],
    rx: round6(e.rx), ry: round6(e.ry), rz: round6(e.rz),
    speed: w.speed, blend: w.blend, ...(w.process ? { process: { ...w.process } } : {}),
  };
}

const isPoint = (p: any): p is Point3 =>
  p && typeof p === 'object' && ['x', 'y', 'z'].every((k) => Number.isFinite(Number(p[k])));

function fingerprint(pairs: [Point3, Point3][], toleranceMm: number) {
  const flat = pairs
    .map(([n, m]) => [n.x, n.y, n.z, m.x, m.y, m.z].map((v) => round6(Number(v))))
    .sort((a, b) => a.join(',').localeCompare(b.join(',')))
    .flat();
  return JSON.stringify({ tol: round6(toleranceMm), pts: flat });
}

// ---------- 路由注册 ----------
export function registerFrameRoutes(app: FastifyInstance, store: Store) {
  const { frames, paths, log } = store;
  // frameVerifications / frameAssociations 为模块自身的进程内模型
  const frameVerifications: any[] = [];
  const frameAssociations: any[] = [];
  const now = () => new Date().toISOString();
  const findFrame = (id: string) => frames.find((f) => f.id === id);
  const findPath = (id: string) => paths.find((p) => p.id === id);
  const passedVersions = (frameId: string) =>
    frameVerifications.filter((v) => v.frameId === frameId && v.passed);
  const latestPassed = (frameId: string) => passedVersions(frameId).at(-1);
  const findVerification = (frameId: string, version: number) =>
    frameVerifications.find((v) => v.frameId === frameId && v.version === version);
  const associationOf = (pathId: string) =>
    frameAssociations.filter((a) => a.pathId === pathId).at(-1);

  // 预置 frame（如 frame-body）在引入验证模块前已标记 verified：
  // 补一条 identity 合成 v1，保持旧演示行为；新登记 frame 仍须真实验证
  for (const f of frames) {
    if (f.status === 'verified' && !frameVerifications.some((v) => v.frameId === f.id)) {
      const R = eulerToMatrix({ rx: 0, ry: 0, rz: 0 });
      const seed = {
        id: randomUUID(),
        frameId: f.id,
        version: 1,
        input: { pointCount: 0, toleranceMm: null, fingerprint: `seed:${f.id}`, synthetic: true },
        createdAt: f.createdAt || now(),
        passed: true,
        status: 'verified',
        warnings: [{ code: 'SYNTHETIC_SEED', message: '模块上线前预置的已验证 frame，采用单位变换占位，建议重新提交实测点验证' }],
        transform: {
          translation: { x: 0, y: 0, z: 0 },
          rotation: { rx: 0, ry: 0, rz: 0 },
          matrix: R,
        },
        residuals: { rmsMm: 0, maxMm: 0, perPoint: [] },
      };
      frameVerifications.push(seed);
      f.currentVerificationId = seed.id;
      f.verificationVersion = 1;
    }
  }

  // 提交坐标系验证（名义点 + 实测点）
  app.post('/api/frames/:id/verifications', async (req: any, res) => {
    const frame = findFrame((req.params as any).id);
    if (!frame) return res.code(404).send({ errorCode: 'FRAME_NOT_FOUND', error: 'frame 不存在' });
    const b = req.body as any;
    const nominal = b?.nominal;
    const measured = b?.measured;
    if (!Array.isArray(nominal) || !Array.isArray(measured))
      return res.code(400).send({ errorCode: 'MALFORMED_INPUT', error: 'nominal 和 measured 必须为点位数组 {x,y,z}' });
    if (!nominal.every(isPoint) || !measured.every(isPoint))
      return res.code(400).send({ errorCode: 'MALFORMED_INPUT', error: '每个点位必须包含有限数值 x、y、z' });
    if (nominal.length !== measured.length)
      return res.code(400).send({ errorCode: 'POINT_COUNT_MISMATCH', error: `名义点 ${nominal.length} 个与实测点 ${measured.length} 个数量不一致` });
    if (nominal.length < 3)
      return res.code(422).send({ errorCode: 'INSUFFICIENT_POINTS', error: `至少需要 3 对配准点，当前 ${nominal.length} 对`, pointCount: nominal.length, minimum: 3 });
    const toleranceMm = b.toleranceMm == null ? DEFAULT_TOLERANCE_MM : Number(b.toleranceMm);
    if (!(toleranceMm > 0))
      return res.code(400).send({ errorCode: 'MALFORMED_INPUT', error: 'toleranceMm 必须为正数' });

    const pairs: [Point3, Point3][] = nominal.map((n: Point3, i: number) => [n, measured[i]]);
    const fp = fingerprint(pairs, toleranceMm);
    const existing = frameVerifications
      .filter((v) => v.frameId === frame.id && v.input.fingerprint === fp)
      .at(-1);
    if (existing) {
      // 完全相同的输入重复提交：幂等返回原记录，不新增版本、不改 frame 状态
      return { ...existing, duplicate: true, message: '相同输入已提交过，返回既有验证版本' };
    }

    const fit = kabsch(nominal, measured);
    const version = frameVerifications.filter((v) => v.frameId === frame.id).length + 1;
    const v = {
      id: randomUUID(),
      frameId: frame.id,
      version,
      input: { pointCount: nominal.length, toleranceMm: round6(toleranceMm), fingerprint: fp },
      createdAt: now(),
    } as any;
    if ('error' in fit) {
      // 几何退化：不给出无法解释的平移/旋转，仅记录失败版本
      Object.assign(v, {
        passed: false,
        status: 'failed',
        reason: fit.error,
        transform: null,
        residuals: null,
      });
    } else {
      const passed = fit.residuals.rmsMm <= toleranceMm;
      Object.assign(v, {
        passed,
        status: passed ? 'verified' : 'failed',
        warnings: fit.warnings,
        transform: {
          translation: fit.translation,
          rotation: fit.rotation,
          matrix: fit.matrix, // 进程内使用，不直接暴露
        },
        residuals: fit.residuals,
      });
    }
    frameVerifications.push(v);
    frame.status = v.status;
    frame.currentVerificationId = v.passed ? v.id : frame.currentVerificationId || null;
    frame.verificationVersion = v.version;
    frame.lastVerifiedAt = v.createdAt;
    log('frame.verify', {
      frameId: frame.id,
      verificationId: v.id,
      version: v.version,
      passed: v.passed,
      rmsMm: v.residuals?.rmsMm ?? null,
      reason: v.reason || null,
    });
    // 深拷贝后再剥离内部矩阵，避免污染存储的验证记录
    const pub: any = {
      ...v,
      input: { ...v.input },
      warnings: (v.warnings || []).map((w: any) => ({ ...w })),
      transform: v.transform
        ? { translation: { ...v.transform.translation }, rotation: { ...v.transform.rotation } }
        : null,
      residuals: v.residuals
        ? {
            rmsMm: v.residuals.rmsMm,
            maxMm: v.residuals.maxMm,
            perPoint: v.residuals.perPoint.map((r: any) => ({ ...r, residual: { ...r.residual } })),
          }
        : null,
    };
    return pub;
  });

  // 查询 frame 的验证版本历史（含失败版本，便于追溯）
  app.get('/api/frames/:id/verifications', async (req: any, res) => {
    const frame = findFrame((req.params as any).id);
    if (!frame) return res.code(404).send({ errorCode: 'FRAME_NOT_FOUND', error: 'frame 不存在' });
    return {
      frameId: frame.id,
      currentVerificationId: frame.currentVerificationId || null,
      verifications: frameVerifications
        .filter((v) => v.frameId === frame.id)
        .map((v) => {
          const pub = { ...v, warnings: v.warnings || [], transform: v.transform ? { translation: v.transform.translation, rotation: v.transform.rotation } : null };
          return pub;
        }),
    };
  });

  // 列出 frame 及其当前验证状态（只读）
  app.get('/api/frames', async () =>
    frames.map((f) => ({
      id: f.id,
      name: f.name,
      origin: f.origin,
      rotation: f.rotation,
      status: f.status,
      createdAt: f.createdAt || null,
      currentVerificationId: f.currentVerificationId || null,
      verificationVersion: f.verificationVersion || null,
      lastVerifiedAt: f.lastVerifiedAt || null,
    })),
  );

  // 路径关联已验证 frame（幂等；不触碰 path.waypoints，也不改动旧路径的既有字段）
  app.post('/api/paths/:id/frame-association', async (req: any, res) => {
    const path = findPath((req.params as any).id);
    if (!path) return res.code(404).send({ errorCode: 'PATH_NOT_FOUND', error: 'path not found' });
    const frameId = (req.body as any)?.frameId;
    if (!frameId) return res.code(400).send({ errorCode: 'MALFORMED_INPUT', error: 'frameId 必填' });
    const frame = findFrame(frameId);
    if (!frame) return res.code(404).send({ errorCode: 'FRAME_NOT_FOUND', error: 'frame 不存在' });
    const verification = latestPassed(frameId);
    if (!verification)
      return res.code(409).send({
        errorCode: 'FRAME_NOT_VERIFIED',
        error: `frame ${frameId} 尚无通过的验证版本，无法关联`,
        frameStatus: frame.status,
      });

    const prev = associationOf(path.id);
    if (prev && prev.frameId === frameId && prev.frameVerificationId === verification.id) {
      return { ...prev, unchanged: true, stale: false, message: '路径已关联该 frame 的当前验证版本，状态未变化' };
    }
    const assoc = {
      id: randomUUID(),
      pathId: path.id,
      frameId,
      frameName: frame.name,
      frameVerificationId: verification.id,
      frameVerificationVersion: verification.version,
      createdAt: prev?.createdAt || now(),
      updatedAt: now(),
      stale: false,
    };
    frameAssociations.push(assoc);
    log('frame.associate', {
      pathId: path.id,
      frameId,
      frameVerificationId: verification.id,
      previousFrameId: prev?.frameId || null,
    });
    return assoc;
  });

  // 读取路径的 frame 关联（未关联时返回 404，但路径本身仍可通过既有接口读取）
  app.get('/api/paths/:id/frame-association', async (req: any, res) => {
    const path = findPath((req.params as any).id);
    if (!path) return res.code(404).send({ errorCode: 'PATH_NOT_FOUND', error: 'path not found' });
    const assoc = associationOf(path.id);
    if (!assoc)
      return res.code(404).send({ errorCode: 'NO_FRAME_ASSOCIATED', error: '该路径尚未关联 frame', pathId: path.id });
    const frame = findFrame(assoc.frameId);
    return {
      ...assoc,
      // frame 重新验证后，关联仍锚定旧快照，这里仅标记，不隐式改变转换结果
      stale: frame?.currentVerificationId !== assoc.frameVerificationId,
      frameStatus: frame?.status || null,
    };
  });

  // 转换预览：只读，不修改原始 waypoints，不改变关联
  app.post('/api/paths/:id/transform-preview', async (req: any, res) => {
    const path = findPath((req.params as any).id);
    if (!path) return res.code(404).send({ errorCode: 'PATH_NOT_FOUND', error: 'path not found' });
    const b = (req.body as any) || {};
    const assoc = associationOf(path.id);
    let frameId = b.frameId || assoc?.frameId;
    if (!frameId)
      return res.code(409).send({
        errorCode: 'NO_FRAME_ASSOCIATED',
        error: '路径未关联 frame，请先关联或在请求体中指定 frameId',
      });
    const frame = findFrame(frameId);
    if (!frame) return res.code(404).send({ errorCode: 'FRAME_NOT_FOUND', error: 'frame 不存在' });

    let verification: any;
    if (b.verificationVersion != null) {
      verification = findVerification(frameId, Number(b.verificationVersion));
      if (!verification)
        return res.code(404).send({ errorCode: 'VERIFICATION_VERSION_NOT_FOUND', error: `frame ${frameId} 不存在验证版本 ${b.verificationVersion}` });
      if (!verification.passed)
        return res.code(409).send({ errorCode: 'VERIFICATION_VERSION_NOT_PASSED', error: `验证版本 ${verification.version} 未通过，不能用于转换预览`, version: verification.version });
    } else if (b.frameId) {
      verification = latestPassed(frameId);
      if (!verification)
        return res.code(409).send({ errorCode: 'FRAME_NOT_VERIFIED', error: `frame ${frameId} 尚无通过的验证版本` });
    } else {
      // 默认锚定关联时的验证快照：frame 重新验证不会悄悄改变既有路径的预览结果
      verification = frameVerifications.find((v) => v.id === assoc.frameVerificationId);
      if (!verification || !verification.passed)
        return res.code(409).send({
          errorCode: 'ASSOCIATED_VERIFICATION_UNAVAILABLE',
          error: '关联快照对应的验证版本不可用，请重新关联 frame',
        });
    }

    const R = verification.transform.matrix;
    const t = [verification.transform.translation.x, verification.transform.translation.y, verification.transform.translation.z];
    const previewWaypoints = path.waypoints.map((w: any) => transformWaypoint(w, R, t));
    return {
      pathId: path.id,
      frameId,
      frameName: frame.name,
      verificationId: verification.id,
      verificationVersion: verification.version,
      associatedSnapshot: !b.frameId && b.verificationVersion == null,
      stale: assoc ? frame.currentVerificationId !== assoc.frameVerificationId : false,
      transform: { translation: verification.transform.translation, rotation: verification.transform.rotation },
      residuals: { verificationRmsMm: verification.residuals.rmsMm, verificationMaxMm: verification.residuals.maxMm, toleranceMm: verification.input.toleranceMm },
      waypointCount: previewWaypoints.length,
      previewWaypoints,
      note: '预览结果为转换副本，原始 waypoints 未修改；未指定 frameId 时使用关联时的验证快照',
    };
  });
}

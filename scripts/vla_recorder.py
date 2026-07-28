"""
VLARecorder — record each grasp as a LeRobot episode for VLA training (π0/SmolVLA/ACT).

A background thread samples, at `fps`, the wrist camera + arm state + the action the
pickup declares (target EE pose + gripper command), buffering frames in memory.
Because node.move() spins the ROS executor while it blocks, the cached node.color /
node.tcp / node.gs stay fresh during motion, so the thread only READS them — no
concurrent ROS spinning (which is not thread-safe).

Reward gate: an episode is committed to the trainable LeRobot dataset ONLY if it
passes (success AND reward >= threshold). Failed / low-quality grasps are NOT
imitated — they are logged to attempts_log.jsonl for analysis but never enter the
dataset. This is standard imitation-learning practice: only train on good demos.

Schema (single UR5 + MAGPIE gripper, end-effector control):
    observation.images.wrist : (H, W, 3) uint8 video, palm/wrist RealSense
    observation.state        : [x,y,z, rx,ry,rz, grip_mm, grip_force]   (8,)
    action                   : [x,y,z, rx,ry,rz, grip_cmd]              (7,)
                               grip_cmd: 0=open, 1=closed (commanded)
    task                     : language instruction, e.g. "pick up the red cube"

Usage (notebook):
    rec = VLARecorder(node, '~/magpie_control/data/lerobot_magpie', fps=10)
    rec.start_episode('pick up the red cube')
    rec.set_action(target_pose_mat, grip=0.0)   # call before each move / grip change
    ...
    rec.end_episode()                            # stop sampling
    rec.commit(success=held, reward=grasp_quality, threshold=0.6, extra={...})
    # at end of the collection session:
    rec.finalize()
"""

import json
import pathlib
import threading
import time

import numpy as np

try:
    import cv2
except Exception:                                   # pragma: no cover
    cv2 = None

STATE_NAMES  = ['x', 'y', 'z', 'rx', 'ry', 'rz', 'grip_mm', 'grip_force', 'wrist_fz']
ACTION_NAMES = ['x', 'y', 'z', 'rx', 'ry', 'rz', 'grip_cmd']

# Topics (match c04 / the RealSense + MAGPIE publishers)
COLOR_TOPIC   = '/camera/gripper_camera/camera/color/image_raw'
TCP_TOPIC     = '/arm/tcp_pose'
GRIPPER_TOPIC = '/gripper/state'


def _rotvec(R):
    """3x3 rotation matrix -> axis-angle rotation vector (3,)."""
    R = np.asarray(R, dtype=float)
    if cv2 is not None:
        return cv2.Rodrigues(R)[0].flatten()
    a = np.arccos(np.clip((np.trace(R) - 1.) / 2., -1., 1.))
    if a < 1e-8:
        return np.zeros(3)
    return (a / (2. * np.sin(a))) * np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def _quat_to_rotvec(w, x, y, z):
    a = 2.0 * np.arccos(np.clip(w, -1., 1.))
    s = np.sin(a / 2.)
    return np.zeros(3) if s < 1e-10 else a * np.array([x, y, z]) / s


class _RecorderNode:
    """Standalone ROS node — its OWN subscriptions + executor, spun in a background
    thread. Independent of the kernel node, so it keeps receiving camera/TCP/gripper
    messages even while the notebook's main thread is blocked in time.sleep() during
    the grasp close — capturing the gripper dynamics the cached-attribute approach
    misses. Mirrors how slip_guard_node stays live during motion."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from sensor_msgs.msg import Image
        from geometry_msgs.msg import PoseStamped, WrenchStamped
        from magpie_msgs.msg import GripperState
        from cv_bridge import CvBridge

        self._bridge = CvBridge()
        self.color = None        # HxWx3 uint8 RGB
        self.tcp   = None        # 4x4
        self.grip_mm = 0.0
        self.grip_force = 0.0
        self.wrist_fz = 0.0      # OptoForce wrist Z-force (0 if FT not publishing)

        img_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.node = Node('vla_recorder')
        self.node.create_subscription(
            Image, COLOR_TOPIC,
            lambda m: setattr(self, 'color', self._bridge.imgmsg_to_cv2(m, 'rgb8')),
            img_qos)
        self.node.create_subscription(PoseStamped, TCP_TOPIC, self._tcp_cb, 1)
        self.node.create_subscription(GripperState, GRIPPER_TOPIC, self._grip_cb, 1)
        self.node.create_subscription(
            WrenchStamped, 'ft_sensor/wrench',
            lambda m: setattr(self, 'wrist_fz', float(m.wrench.force.z)), 10)

        # MultiThreadedExecutor.spin() runs continuously in its own thread, so the
        # 500Hz TCP / gripper / image callbacks are drained as fast as they arrive —
        # independent of the notebook main thread blocking in node.move()/time.sleep().
        # (A SingleThreadedExecutor spin_once loop starved TCP behind the heavy image
        # callback, so the arm got recorded as waypoint jumps instead of motion.)
        from rclpy.executors import MultiThreadedExecutor
        self._exec = MultiThreadedExecutor(num_threads=3)
        self._exec.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._spin_thread.start()

    def _tcp_cb(self, m):
        from magpie_control import poses
        p = m.pose
        rv = _quat_to_rotvec(p.orientation.w, p.orientation.x,
                             p.orientation.y, p.orientation.z)
        self.tcp = poses.pose_vec_to_mtrx(
            [p.position.x, p.position.y, p.position.z, *rv])

    def _grip_cb(self, m):
        self.grip_mm = float(getattr(m, 'position', 0.) or 0.)
        self.grip_force = float(getattr(m, 'force', 0.) or 0.)

    def shutdown(self):
        try:
            self._exec.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass


class VLARecorder:
    def __init__(self, node, root='~/magpie_control/data/lerobot_magpie',
                 repo_id='magpie/grasp', fps=10, cam='wrist', own_node=True,
                 rerun=None, trim_static=False):
        self.node     = node          # kernel node (fallback source)
        # trim_static: drop long runs of stationary frames (arm idle during model compute)
        # at end_episode — the idle gap otherwise teaches the policy that hovering is an
        # action (V0 post-grasp dithering). Keeps up to _STATIC_KEEP consecutive static
        # frames so short, real pauses survive.
        self.trim_static = bool(trim_static)
        self.own_node = own_node      # use a dedicated ROS recorder node (robust)
        self.rerun    = rerun         # None | 'web' | 'spawn' | 'connect' | 'save'
        self._rr      = None
        self._rr_t    = 0
        self._rnode   = None
        self.root    = pathlib.Path(root).expanduser()
        self.repo_id = repo_id
        self.fps     = int(fps)
        self.cam     = cam
        self.img_key = f'observation.images.{cam}'
        self._ds     = None
        self._thread = None
        self._stop   = threading.Event()
        self._buf    = []
        self._action = None          # (target_mat, grip_cmd)
        self._task   = ''
        # NOTE: do NOT create self.root here — LeRobotDataset.create() requires the
        # root to not exist yet. Keep the attempts log OUTSIDE the dataset root.
        self._attempts_log = self.root.parent / f'{self.root.name}_attempts.jsonl'

    # ── dataset lazy init (needs image H,W from a live frame) ────────────────
    def _ensure_ds(self, h, w):
        if self._ds is not None:
            return
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        feats = {
            self.img_key: {'dtype': 'video', 'shape': (h, w, 3),
                           'names': ['height', 'width', 'channel']},
            'observation.state': {'dtype': 'float32', 'shape': (len(STATE_NAMES),),
                                  'names': STATE_NAMES},
            'action': {'dtype': 'float32', 'shape': (len(ACTION_NAMES),),
                       'names': ACTION_NAMES},
        }
        if (self.root / 'meta').exists():
            try:
                self._ds = LeRobotDataset(self.repo_id, root=str(self.root))
                print(f'  [VLA] resumed existing dataset ({self._ds.num_episodes} episodes)')
                return
            except Exception as e:
                # Corrupted trailing file (partial write without finalize) — find and remove it,
                # then rewind info.json to match the last good episode in the episodes meta.
                print(f'  [VLA] resume failed ({type(e).__name__}), scanning for corrupted files...')
                import json as _json, pyarrow.parquet as _pq
                _repaired = False
                for _f in sorted((self.root / 'data').rglob('*.parquet'), reverse=True):
                    try:
                        _pq.read_metadata(_f)
                    except Exception:
                        print(f'  [VLA] removing corrupted {_f.name}')
                        # Also remove the matching video shard
                        _idx = _f.stem  # e.g. file-001
                        _chunk = _f.parent.name
                        for _vid in (self.root / 'videos').rglob(f'{_chunk}/{_idx}.mp4'):
                            _vid.unlink(missing_ok=True)
                        _f.unlink()
                        _repaired = True
                if _repaired:
                    # Recount good episodes from the episodes meta parquet
                    _ep_files = list((self.root / 'meta' / 'episodes').rglob('*.parquet'))
                    _n_ep = sum(_pq.read_metadata(f).num_rows for f in _ep_files)
                    _n_fr = sum(_pq.read_metadata(f).num_rows
                                for f in (self.root / 'data').rglob('*.parquet'))
                    _info_path = self.root / 'meta' / 'info.json'
                    _info = _json.loads(_info_path.read_text())
                    _info['total_episodes'] = _n_ep
                    _info['total_frames']   = _n_fr
                    _info['splits']         = {'train': f'0:{_n_ep}'}
                    _info_path.write_text(_json.dumps(_info, indent=4))
                    print(f'  [VLA] repaired: {_n_ep} episode(s), {_n_fr} frames — retrying open...')
                    self._ds = LeRobotDataset(self.repo_id, root=str(self.root))
                    print(f'  [VLA] resumed after repair ({self._ds.num_episodes} episodes)')
                    return
                raise RuntimeError(
                    f'A LeRobot dataset already exists at {self.root} but could not be '
                    f'reopened to append ({e}). Either finalize+train on it, or point '
                    f'VLARecorder at a fresh root.')
        self._ds = LeRobotDataset.create(
            repo_id=self.repo_id, fps=self.fps, features=feats,
            root=str(self.root), robot_type='ur5_magpie', use_videos=True,
            image_writer_threads=4)
        print(f'  [VLA] created new dataset at {self.root}')

    # ── snapshots ────────────────────────────────────────────────────────────
    def _read(self):
        """Latest (color, tcp, grip_mm, grip_force, wrist_fz) from the dedicated
        recorder node if active, else the kernel node's cached attributes."""
        if self._rnode is not None:
            return (self._rnode.color, self._rnode.tcp,
                    self._rnode.grip_mm, self._rnode.grip_force, self._rnode.wrist_fz)
        n, gs = self.node, self.node.gs
        w = getattr(n, 'wrench', None)
        wfz = float(w.wrench.force.z) if w is not None else 0.0
        return (n.color, n.tcp,
                float(getattr(gs, 'position', 0.) or 0.) if gs is not None else 0.,
                float(getattr(gs, 'force', 0.) or 0.) if gs is not None else 0.,
                wfz)

    def _state_vec(self):
        _, tcp, gmm, gf, wfz = self._read()
        if tcp is None:
            return None
        t = np.asarray(tcp, dtype=float)
        return np.array([*t[:3, 3], *_rotvec(t[:3, :3]), gmm, gf, wfz], dtype=np.float32)

    def _action_vec(self):
        if self._action is None:
            _, tcp, _, _, _ = self._read()
            if tcp is None:
                return None
            t = np.asarray(tcp, dtype=float)
            return np.array([*t[:3, 3], *_rotvec(t[:3, :3]), 0.], dtype=np.float32)
        mat, grip = self._action
        m = np.asarray(mat, dtype=float)
        return np.array([*m[:3, 3], *_rotvec(m[:3, :3]), float(grip)], dtype=np.float32)

    # ── public API ────────────────────────────────────────────────────────────
    def set_action(self, target_mat, grip):
        """Declare the action the pickup is commanding: target EE pose (4x4) and
        gripper command (0=open, 1=closed). The recorder logs it alongside state."""
        self._action = (np.asarray(target_mat, dtype=float), float(grip))

    def _rerun_setup(self):
        """Live viewing via Rerun. 'web' serves a browser viewer (best for a remote
        robot), 'save' writes a .rrd to open later, 'spawn' opens a local window,
        'connect' attaches to a running viewer."""
        if self.rerun is None or self._rr is not None:
            return
        try:
            import rerun as rr
            rr.init('magpie_grasp', spawn=(self.rerun == 'spawn'))
            if self.rerun == 'web':
                rr.serve_web()
                print('  [VLA] Rerun live: open the web viewer URL printed above')
            elif self.rerun == 'connect':
                rr.connect_grpc()
            elif self.rerun == 'save':
                p = str(self.root.parent / f'{self.root.name}_live.rrd')
                rr.save(p)
                print(f'  [VLA] Rerun recording → {p}  (view: `rerun {p}`)')
            self._rr = rr
        except Exception as e:
            print(f'  [VLA] Rerun unavailable ({e}) — continuing without live view')
            self.rerun = None

    def _rerun_log(self, img, st, ac, task):
        rr = self._rr
        rr.set_time('frame', sequence=self._rr_t)
        rr.log(f'wrist/{self.cam}', rr.Image(img))
        rr.log('state/grip_force_N', rr.Scalars(float(st[7])))
        rr.log('state/grip_mm',      rr.Scalars(float(st[6])))
        rr.log('state/tcp_z',        rr.Scalars(float(st[2])))
        rr.log('action/grip_cmd',    rr.Scalars(float(ac[6])))
        if self._rr_t == 0:
            rr.log('task', rr.TextLog(task))
        self._rr_t += 1

    def start_episode(self, task):
        self._rerun_setup()
        if self.own_node and self._rnode is None:
            try:
                self._rnode = _RecorderNode()
                time.sleep(0.4)   # let first messages arrive
            except Exception as e:
                print(f'  [VLA] dedicated recorder node unavailable ({e}) — '
                      f'falling back to kernel node cache')
                self._rnode = None
        self._task   = task
        self._buf    = []
        self._action = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        dt = 1.0 / self.fps
        while not self._stop.is_set():
            t0  = time.time()
            img = self._read()[0]
            st  = self._state_vec()
            ac  = self._action_vec()
            if img is not None and st is not None and ac is not None:
                img = np.ascontiguousarray(img, dtype=np.uint8)
                self._buf.append((img, st, ac))
                if self._rr is not None:
                    try:
                        self._rerun_log(img, st, ac, self._task)
                    except Exception:
                        pass
            slp = dt - (time.time() - t0)
            if slp > 0:
                time.sleep(slp)

    _STATIC_KEEP = 5     # max consecutive static frames kept (0.5s at 10Hz)

    def _trim_static_frames(self):
        """Drop frames where BOTH the state and the declared action are unchanged vs the
        previous kept frame, once more than _STATIC_KEEP such frames run consecutively.
        Removes the arm-idle model-compute gap while preserving short genuine pauses."""
        if len(self._buf) < 3:
            return
        kept, static_run = [self._buf[0]], 0
        for img, st, ac in self._buf[1:]:
            _, pst, pac = kept[-1]
            moved = (np.linalg.norm(st[:3] - pst[:3]) > 5e-4          # >0.5mm TCP motion
                     or abs(st[6] - pst[6]) > 0.5                      # gripper moving
                     or np.linalg.norm(ac - pac) > 1e-3)               # action changed
            if moved:
                static_run = 0
                kept.append((img, st, ac))
            else:
                static_run += 1
                if static_run <= self._STATIC_KEEP:
                    kept.append((img, st, ac))
        dropped = len(self._buf) - len(kept)
        if dropped:
            print(f'  [VLA] trimmed {dropped} static frames ({len(self._buf)} -> {len(kept)})')
        self._buf = kept

    def end_episode(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.)
        if self.trim_static:
            self._trim_static_frames()
        n = len(self._buf)
        if n > 1:
            tcp = np.array([b[1][:3] for b in self._buf])
            moving = int((np.linalg.norm(np.diff(tcp, axis=0), axis=1) > 5e-4).sum())
            uniq   = len({tuple(p) for p in np.round(tcp, 3)})
            src    = 'dedicated node' if self._rnode is not None else 'KERNEL FALLBACK'
            warn   = '' if (uniq > 10 or n < 12) else '  ⚠ jumpy (check recorder node)'
            print(f'  [VLA] buffered {n} frames | src={src} | {moving}/{n-1} moving | '
                  f'{uniq} unique TCP poses{warn}')
        return n

    def commit(self, success, reward, threshold=0.6, extra=None):
        """Write the buffered episode to the LeRobot dataset IF it passes the gate
        (success AND reward >= threshold). Always log the attempt for analysis."""
        n = len(self._buf)
        passed = bool(success) and reward is not None and float(reward) >= threshold
        rec = {'task': self._task, 'success': bool(success),
               'reward': None if reward is None else float(reward),
               'threshold': threshold, 'n_frames': n, 'kept': bool(passed),
               'ts': time.strftime('%Y-%m-%dT%H:%M:%S')}
        if extra:
            rec.update(extra)
        self._attempts_log.parent.mkdir(parents=True, exist_ok=True)
        with open(self._attempts_log, 'a') as f:
            f.write(json.dumps(rec) + '\n')

        if not passed or n == 0:
            why = ('no frames captured' if n == 0
                   else 'object DROPPED' if not success
                   else f'reward {float(reward):.2f} < gate {threshold}')
            # Keep the FULL failed episode (frames + trajectory) so it's recoverable
            # for review/relabel — the trainable dataset stays clean, failures go to a
            # separate <name>_failed/ dir. (Previously the frames were discarded.)
            if n > 0:
                try:
                    _fp = self._save_failed_episode(rec, why)
                    print(f'  [VLA] NOT trained ({why}) — full data KEPT at {_fp}')
                except Exception as _fe:
                    print(f'  [VLA] NOT trained ({why}); failed-data dump skipped: {_fe}')
            else:
                print(f'  [VLA] episode NOT saved ({why}) — no frames')
            self._buf = []
            return False

        h, w = self._buf[0][0].shape[:2]
        self._ensure_ds(h, w)
        for img, st, ac in self._buf:
            self._ds.add_frame({
                self.img_key:        img,
                'observation.state': st,
                'action':            ac,
                'task':              self._task,
            })
        self._ds.save_episode()
        print(f'  [VLA] episode SAVED ✓  {n} frames @ {self.fps}Hz  reward={float(reward):.2f}  '
              f'task="{self._task}"')
        self._buf = []
        return True

    def _save_failed_episode(self, rec, why):
        """Dump a FAILED episode's full frames + trajectory so it's recoverable.
        Writes <name>_failed/<ts>_<task>.mp4 (wrist video) + .json (states/actions/
        metadata). Keeps the trainable dataset clean while losing nothing."""
        import cv2, json as _j, time as _t
        d = self.root.parent / f'{self.root.name}_failed'
        d.mkdir(parents=True, exist_ok=True)
        stem = str(d / f"{_t.strftime('%Y%m%d_%H%M%S')}_{(self._task or 'obj')[:20].replace(' ', '_')}")
        h, w = self._buf[0][0].shape[:2]
        vw = cv2.VideoWriter(stem + '.mp4', cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (w, h))
        traj = []
        for img, st, ac in self._buf:
            vw.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
            traj.append({'state': [float(x) for x in st], 'action': [float(x) for x in ac]})
        vw.release()
        meta = dict(rec); meta['reason'] = why
        _j.dump({'meta': meta, 'trajectory': traj}, open(stem + '.json', 'w'))
        return stem + '.mp4'

    def flush_episode(self):
        """Close the current episode's parquet/video writers so the file is immediately
        readable. Does NOT shut down the recorder node — more episodes can follow.
        Call this after each commit() to avoid corrupt files if the kernel dies."""
        if self._ds is None:
            return
        try:
            self._ds._close_writer()
            self._ds.meta._close_writer()
            enc = getattr(self._ds, '_streaming_encoder', None)
            if enc is not None:
                enc.close()
        except Exception as e:
            print(f'  [VLA] flush_episode warning: {e}')
        n = self._ds.num_episodes
        self._ds = None   # force _ensure_ds to re-open cleanly on next commit
        print(f'  [VLA] episode flushed to disk ({n} total) — file is valid ✓')

    def finalize(self):
        """Flush parquet/video writers — REQUIRED before the dataset can be loaded
        for training. Call once at the end of a collection session."""
        if self._ds is not None:
            self._ds.finalize()
            print(f'  [VLA] dataset finalized → {self.root} '
                  f'({self._ds.num_episodes} episodes). Ready to train.')
        if self._rnode is not None:
            self._rnode.shutdown()
            self._rnode = None

"""Run state and scoring logic for the PARC 2026 referee (no ROS imports)."""

import json
import math
import os
import random
import threading
import time
from datetime import datetime

# Mirrors parc_msgs/RunStatus constants.
IDLE, ARMED, RUNNING, FINISHED, TIMED_OUT, TERMINATED = range(6)
STATE_NAMES = ['IDLE', 'ARMED', 'RUNNING', 'FINISHED', 'TIMED_OUT', 'TERMINATED']
ENDED = (FINISHED, TIMED_OUT, TERMINATED)

METHOD_COMMAND, METHOD_VOICE = 0, 1


class Visit:
    def __init__(self, tag_id, elapsed_s, image_file, image_check, detected_ids,
                 pose, msg_stamp):
        self.tag_id = tag_id
        self.elapsed_s = elapsed_s
        self.wall_time = datetime.now().isoformat(timespec='seconds')
        self.image_file = image_file
        self.image_check = image_check      # match | mismatch | not_detected | unchecked
        self.detected_ids = detected_ids
        self.pose = pose                    # (x, y, yaw) at the visit, or None
        self.hold = 'pending' if pose else 'unknown'   # pending | ok | moved | unknown
        self.max_move_m = 0.0
        self.received_mono = time.monotonic()
        self.msg_stamp = msg_stamp
        self.verdict = None                 # None | 'confirmed' | 'rejected'
        self.order_status = ''              # filled in by Run.evaluate()


class Attempt:
    def __init__(self, number):
        self.number = number
        self.start_method = None
        self.heard = ''
        self.false_start = False
        self.visits = []
        self.light = 0
        self.dangerous = 0
        self.voice_bonus = None             # None = follow start_method; True/False = judge override


class RunState:
    """All referee state. Every public method is thread-safe."""

    def __init__(self, tag_ids=(1, 4, 7, 9), time_limit_s=600.0, hold_time_s=3.0,
                 move_threshold_m=0.05, move_threshold_rad=0.087, log_dir='~/parc_runs',
                 safe_floor_zero=True, terminate_keeps_tag_points=True):
        self.tag_ids = list(tag_ids)
        self.time_limit_s = time_limit_s
        self.hold_time_s = hold_time_s
        self.move_threshold_m = move_threshold_m
        self.move_threshold_rad = move_threshold_rad
        self.log_root = os.path.expanduser(log_dir)
        self.safe_floor_zero = safe_floor_zero
        self.terminate_keeps_tag_points = terminate_keeps_tag_points

        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)
        self.version = 0
        self.events = []                    # (elapsed or None, text) for the event log
        self.pose = None                    # latest odometry (x, y, yaw)
        self.arm_pose = None
        self._clear_run()

    # ------------------------------------------------------------------ helpers
    def _clear_run(self):
        self.state = IDLE
        self.team = ''
        self.order = []
        self.t0 = None
        self.t_end = None
        self.end_reason = ''
        self.attempts = []
        self.run_dir = None
        self.events = []

    def _bump(self):
        self.version += 1
        self.changed.notify_all()

    def _log(self, text):
        self.events.append((round(self.elapsed(), 1) if self.t0 else None, text))

    def _attempt(self):
        return self.attempts[-1] if self.attempts else None

    def elapsed(self):
        if self.t0 is None:
            return 0.0
        end = self.t_end if self.t_end is not None else time.monotonic()
        return max(0.0, end - self.t0)

    @staticmethod
    def parse_order(text):
        ids = [int(p) for p in str(text).replace(',', ' ').split()]
        return ids

    # ------------------------------------------------------------- judge actions
    def arm(self, team, order=None):
        with self.lock:
            if self.state in (ARMED, RUNNING):
                raise ValueError('A run is already in progress. Terminate or reset it first.')
            if self.state in ENDED:
                self.save()
            order = list(order) if order else random.sample(self.tag_ids, len(self.tag_ids))
            if sorted(order) != sorted(self.tag_ids):
                raise ValueError(f'Order must contain each of {self.tag_ids} exactly once.')
            self._clear_run()
            self.team = (team or 'Unnamed team').strip()
            self.order = order
            self.state = ARMED
            self.attempts = [Attempt(1)]
            self.arm_pose = self.pose
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            safe_team = ''.join(c if c.isalnum() else '_' for c in self.team)
            self.run_dir = os.path.join(self.log_root, f'{stamp}_{safe_team}')
            os.makedirs(self.run_dir, exist_ok=True)
            self._log(f'Armed for {self.team}. Order: {", ".join(map(str, order))}')
            self._bump()
            return order

    def restart(self):
        """Manual intervention: new attempt, clock keeps running."""
        with self.lock:
            if self.state not in (ARMED, RUNNING):
                raise ValueError('No run in progress.')
            if self.t0 is None:
                raise ValueError('The run has not started yet.')
            self.attempts.append(Attempt(len(self.attempts) + 1))
            self.state = ARMED
            self.arm_pose = self.pose
            self._log(f'Restart: attempt {len(self.attempts)}. Clock keeps running.')
            self._bump()

    def terminate(self, reason):
        with self.lock:
            if self.state not in (ARMED, RUNNING):
                raise ValueError('No run in progress.')
            if self.t0 is None:
                self.t0 = time.monotonic()
            self.t_end = time.monotonic()
            self.state = TERMINATED
            self.end_reason = reason or 'Terminated by judge'
            self._log(f'Terminated: {self.end_reason}')
            self.save()
            self._bump()

    def end_run(self):
        """Judge ends the run normally (e.g. the team concedes). Points are kept."""
        with self.lock:
            if self.state not in (ARMED, RUNNING) or self.t0 is None:
                raise ValueError('No run in progress.')
            self._end(FINISHED, 'Ended by judge')
            self.save()
            self._bump()

    def reset(self):
        with self.lock:
            if self.state != IDLE:
                self.save()
            self._clear_run()
            self._bump()

    def set_verdict(self, index, verdict):
        with self.lock:
            a = self._attempt()
            if a is None or not 0 <= index < len(a.visits):
                raise ValueError('No such visit.')
            if verdict not in (None, 'confirmed', 'rejected'):
                raise ValueError('Verdict must be confirmed, rejected or null.')
            a.visits[index].verdict = verdict
            self._log(f'Judge: visit {index + 1} (tag {a.visits[index].tag_id}) {verdict or "cleared"}')
            self.save()
            self._bump()

    def add_contact(self, kind, delta):
        with self.lock:
            a = self._attempt()
            if a is None:
                raise ValueError('No run in progress.')
            if kind == 'light':
                a.light = max(0, a.light + delta)
            elif kind == 'dangerous':
                a.dangerous = max(0, a.dangerous + delta)
            else:
                raise ValueError('Contact kind must be light or dangerous.')
            self._log(f'Judge: {kind} contact {"+1" if delta > 0 else "-1"}')
            self.save()
            self._bump()

    def set_voice_bonus(self, value):
        with self.lock:
            a = self._attempt()
            if a is None:
                raise ValueError('No run in progress.')
            a.voice_bonus = value
            self._log(f'Judge: voice bonus {"awarded" if value else "not awarded" if value is False else "automatic"}')
            self._bump()

    # ------------------------------------------------------------ robot inputs
    def on_start(self, method, heard):
        with self.lock:
            if self.state != ARMED:
                self._log(f'Start signal ignored (state {STATE_NAMES[self.state]})')
                self._bump()
                return False
            a = self._attempt()
            a.start_method = method
            a.heard = heard or ''
            if self.t0 is None:
                self.t0 = time.monotonic()
            self.state = RUNNING
            how = f'voice ("{a.heard}")' if method == METHOD_VOICE else 'command'
            self._log(f'Start signal received, attempt {a.number}, by {how}')
            self._bump()
            return True

    def on_visit(self, tag_id, image_bytes, msg_stamp, image_check, detected_ids):
        with self.lock:
            if self.state != RUNNING:
                self._log(f'Visit to tag {tag_id} ignored (state {STATE_NAMES[self.state]})')
                self._bump()
                return None
            a = self._attempt()
            image_file = None
            if image_bytes:
                ext = 'png' if image_bytes[:4] == b'\x89PNG' else 'jpg'
                image_file = f'attempt{a.number}_visit{len(a.visits) + 1}_tag{tag_id}.{ext}'
                with open(os.path.join(self.run_dir, image_file), 'wb') as f:
                    f.write(image_bytes)
            v = Visit(tag_id, round(self.elapsed(), 2), image_file, image_check,
                      detected_ids, self.pose, msg_stamp)
            a.visits.append(v)
            self.evaluate()
            self._log(f'Tag {tag_id} reported ({v.order_status.replace("_", " ")}), image: {image_check.replace("_", " ")}')
            if v.order_status == 'in_order' and tag_id == self.order[-1]:
                self._end(FINISHED, 'Last tag in the order reported')
            self.save()
            self._bump()
            return v

    def on_finish(self, reason):
        with self.lock:
            if self.state != RUNNING:
                return
            self._end(FINISHED, 'Finish signal: ' + ('gave up' if reason == 1 else 'completed'))
            self.save()
            self._bump()

    def on_odom(self, x, y, yaw):
        with self.lock:
            self.pose = (x, y, yaw)
            changed = False
            a = self._attempt()
            if self.state == ARMED and a and not a.false_start:
                if self.arm_pose is None:
                    self.arm_pose = self.pose
                elif self._moved(self.arm_pose, self.pose):
                    a.false_start = True
                    self._log('FALSE START: robot moved before the start signal')
                    changed = True
            if a and self.state in (RUNNING,) + ENDED:
                for v in a.visits:
                    if v.hold == 'pending':
                        d = math.hypot(x - v.pose[0], y - v.pose[1])
                        v.max_move_m = max(v.max_move_m, d)
                        if self._moved(v.pose, self.pose):
                            v.hold = 'moved'
                            self._log(f'Tag {v.tag_id}: robot moved during the {self.hold_time_s:.0f} s hold')
                            changed = True
            if changed:
                self._bump()

    def tick(self):
        """Called periodically: timeout and hold completion."""
        with self.lock:
            changed = False
            if self.state in (ARMED, RUNNING) and self.t0 is not None \
                    and self.elapsed() >= self.time_limit_s:
                self.t_end = self.t0 + self.time_limit_s
                self._end(TIMED_OUT, 'Time limit reached')
                self.save()
                changed = True
            a = self._attempt()
            if a:
                now = time.monotonic()
                for v in a.visits:
                    if v.hold == 'pending' and now - v.received_mono >= self.hold_time_s:
                        v.hold = 'ok'
                        changed = True
            if changed:
                self._bump()

    # ---------------------------------------------------------------- internals
    def _moved(self, p0, p1):
        d = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        dyaw = abs(math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2])))
        return d > self.move_threshold_m or dyaw > self.move_threshold_rad

    def _end(self, state, reason):
        if self.t_end is None:
            self.t_end = time.monotonic()
        self.state = state
        self.end_reason = reason
        self._log(f'Run ended: {reason}')

    def evaluate(self):
        """Apply the order and skip rules to the current attempt's visits."""
        a = self._attempt()
        if a is None:
            return
        last_idx, counted = -1, set()
        for v in a.visits:
            if v.tag_id not in self.order:
                v.order_status = 'unknown_tag'
            elif v.verdict == 'rejected':
                v.order_status = 'rejected'
            elif v.tag_id in counted:
                v.order_status = 'duplicate'
            elif self.order.index(v.tag_id) <= last_idx:
                v.order_status = 'out_of_order'
            else:
                v.order_status = 'in_order'
                last_idx = self.order.index(v.tag_id)
                counted.add(v.tag_id)

    def score(self):
        a = self._attempt()
        if a is None:
            return None
        self.evaluate()
        in_order = [v for v in a.visits if v.order_status == 'in_order']
        confirmed = sum(1 for v in in_order if v.verdict == 'confirmed')
        pending = sum(1 for v in in_order if v.verdict is None)
        tag_points = 20 * confirmed
        safe = min(20, 5 * confirmed) - 2 * a.light - 5 * a.dangerous
        if self.safe_floor_zero:
            safe = max(0, safe)
        if self.state == TERMINATED:
            safe = 0
            if not self.terminate_keeps_tag_points:
                tag_points = 0
        voice = a.voice_bonus if a.voice_bonus is not None else (
            a.start_method == METHOD_VOICE and not a.false_start)
        bonus = 5 if voice else 0
        return {'tag_points': tag_points, 'safe_points': safe, 'voice_bonus': bonus,
                'total': tag_points + safe + bonus, 'confirmed': confirmed,
                'pending': pending}

    # ----------------------------------------------------------------- output
    def tags_reported(self):
        a = self._attempt()
        if not a:
            return []
        seen, out = set(), []
        for v in a.visits:
            if v.tag_id not in seen:
                seen.add(v.tag_id)
                out.append(v.tag_id)
        return out

    def to_dict(self):
        with self.lock:
            a = self._attempt()
            self.evaluate()
            return {
                'version': self.version,
                'state': STATE_NAMES[self.state],
                'team': self.team,
                'order': self.order,
                'elapsed_s': round(self.elapsed(), 1),
                'time_limit_s': self.time_limit_s,
                'clock_running': self.t0 is not None and self.t_end is None,
                'end_reason': self.end_reason,
                'attempt': a.number if a else 0,
                'start_method': {None: '', METHOD_COMMAND: 'command',
                                 METHOD_VOICE: 'voice'}.get(a.start_method, '') if a else '',
                'heard': a.heard if a else '',
                'false_start': a.false_start if a else False,
                'light': a.light if a else 0,
                'dangerous': a.dangerous if a else 0,
                'voice_bonus_override': a.voice_bonus if a else None,
                'visits': [{
                    'index': i, 'tag_id': v.tag_id, 'elapsed_s': v.elapsed_s,
                    'image': f'/api/image/{a.number}/{i}' if v.image_file else None,
                    'image_check': v.image_check, 'detected_ids': v.detected_ids,
                    'hold': v.hold, 'verdict': v.verdict, 'order_status': v.order_status,
                } for i, v in enumerate(a.visits)] if a else [],
                'score': self.score(),
                'odom': self.pose is not None,
                'events': self.events[-30:],
            }

    def image_path(self, attempt_no, index):
        with self.lock:
            if not self.run_dir or not 1 <= attempt_no <= len(self.attempts):
                return None
            visits = self.attempts[attempt_no - 1].visits
            if not 0 <= index < len(visits) or not visits[index].image_file:
                return None
            return os.path.join(self.run_dir, visits[index].image_file)

    def save(self):
        """Write the run record to <log_dir>/<run>/run.json."""
        with self.lock:
            if not self.run_dir:
                return
            record = {
                'team': self.team, 'order': self.order, 'state': STATE_NAMES[self.state],
                'elapsed_s': round(self.elapsed(), 2), 'end_reason': self.end_reason,
                'score_last_attempt': self.score(),
                'attempts': [{
                    'number': a.number, 'start_method': a.start_method, 'heard': a.heard,
                    'false_start': a.false_start, 'light': a.light, 'dangerous': a.dangerous,
                    'voice_bonus_override': a.voice_bonus,
                    'visits': [{k: getattr(v, k) for k in (
                        'tag_id', 'elapsed_s', 'wall_time', 'image_file', 'image_check',
                        'detected_ids', 'hold', 'max_move_m', 'verdict', 'order_status',
                        'msg_stamp')} for v in a.visits],
                } for a in self.attempts],
                'events': self.events,
            }
            with open(os.path.join(self.run_dir, 'run.json'), 'w') as f:
                json.dump(record, f, indent=2)

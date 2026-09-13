"""GUI command arbitration; software interlocks, not a safety-rated controller.

Only this service writes motor commands for the web interface. A cancellation
prevents subsequent commands; it cannot retract a target already in a drive.
"""
from collections import deque
from dataclasses import dataclass
import math
import threading
import time
from datetime import datetime
from core.gui_journal import updated_at

from core.motor import (CURRENT_LIMIT, PROFILE_ACCEL, PROFILE_DECEL,
                        PROFILE_VELOCITY, PROTECT_OVER_CURRENT)


class Rejected(ValueError):
    pass


class Cancelled(Exception):
    pass


def finite(value, label):
    if isinstance(value, bool):
        raise Rejected(f'{label}必须是有限数值')
    try:
        value = float(value)
    except (ValueError, TypeError):
        raise Rejected(f'{label}必须是有限数值') from None
    if not math.isfinite(value):
        raise Rejected(f'{label}必须是有限数值')
    return value


@dataclass(frozen=True)
class Limits:
    minimum: float
    maximum: float

    @classmethod
    def read(cls, node):
        spec = node.get('limits') or {}
        if spec.get('verified') is not True:
            raise Rejected(f"J{node['id']} 未配置并核实软限位")
        low = finite(spec.get('min_deg'), '最小角度')
        high = finite(spec.get('max_deg'), '最大角度')
        if low >= high:
            raise Rejected('最小角度必须小于最大角度')
        return cls(low, high)

    def check(self, value):
        value = finite(value, '目标角度')
        if not self.minimum <= value <= self.maximum:
            raise Rejected(f'角度 {value:g}° 超出 [{self.minimum:g}, {self.maximum:g}]°')
        return value


@dataclass(frozen=True)
class HomingProfile:
    order: int
    current: float
    search: float
    backoff: float
    final: float
    velocity: float
    acceleration: float
    release: float

    @classmethod
    def read(cls, node):
        spec = node.get('homing') or {}
        try:
            order = int(spec['order'])
        except (KeyError, TypeError, ValueError):
            raise Rejected(f"J{node['id']} 缺少有效的碰撞调零顺序") from None
        values = {key: finite(spec.get(source), f"J{node['id']} {source}") for key, source in
                  [('current', 'current'), ('search', 'search_deg'),
                   ('backoff', 'backoff_deg'), ('final', 'final_deg'),
                   ('velocity', 'velocity'), ('acceleration', 'acceleration')]}
        release = finite(spec.get('release_deg', 0), f"J{node['id']} release_deg")
        if order < 1 or values['current'] <= 0 or values['velocity'] <= 0 or values['acceleration'] <= 0:
            raise Rejected(f"J{node['id']} 碰撞调零参数必须为有效正值")
        if values['search'] == 0:
            raise Rejected(f"J{node['id']} 寻零目标不能为 0")
        return cls(order=order, release=release, **values)


class GuiControl:
    def __init__(self, controller, nodes, stale_seconds=2.0, move_timeout=30.0,
                 homing_defaults=None, commissioning_defaults=None,
                 manual_defaults=None, journal=None):
        self.controller = controller
        self.journal = journal
        self.nodes = {n['id']: n for n in nodes}
        if not self.nodes or len(self.nodes) != len(nodes):
            raise Rejected('节点配置为空或包含重复 ID')
        self.stale_seconds = stale_seconds
        self.move_timeout = move_timeout
        self.lock = threading.RLock()
        self.cancel_event = threading.Event()
        self.closed = False
        self.latched = False
        self.active = None
        self.active_node = None
        self.worker = None
        self.last_result = '等待操作'
        self.events = deque(maxlen=200)
        self.progress = ''
        self.communication_error = ''
        self.homed_nodes = set()
        self.last_targets = {node_id: None for node_id in self.nodes}
        defaults = homing_defaults or {}
        self.homing_stall_seconds = finite(defaults.get('stall_seconds', .2), '堵转确认时间')
        self.homing_timeout = finite(defaults.get('search_timeout_seconds', 40), '寻零超时')
        self.backoff_timeout = finite(defaults.get('backoff_timeout_seconds', 30), '回退超时')
        self.stall_velocity = finite(defaults.get('stall_velocity', .4), '堵转速度阈值')
        self.movement_threshold = finite(defaults.get('movement_threshold_deg', 2), '最小运动量')
        self.protection_current = finite(defaults.get('protection_current', 20), '寻零保护电流')
        self.restored_protection_current = finite(defaults.get('restored_protection_current', 12), '运行保护电流')
        self.return_current = finite(defaults.get('return_current', 10), '调零回位电流')
        self.return_velocity = finite(defaults.get('return_velocity', 2), '调零回位速度')
        self.return_acceleration = finite(defaults.get('return_acceleration', 5), '调零回位加速度')
        self.collision_confirm_seconds = finite(defaults.get('collision_confirm_seconds', .6), '限位确认时间')
        self.collision_stability = finite(defaults.get('collision_stability_deg', .3), '限位稳定误差')
        self.verification_backoff = finite(defaults.get('verification_backoff_deg', 8), '二次确认回退角度')
        self.collision_repeatability = finite(defaults.get('collision_repeatability_deg', 2), '限位重复精度')
        self.verification_current_factor = finite(defaults.get('verification_current_factor', 1.2), '二次确认电流倍率')
        self.release_timeout = finite(defaults.get('release_timeout_seconds', 8), '起始限位回退超时')
        self.homing_command_retry = finite(defaults.get('command_retry_seconds', .5), '调零指令重发间隔')
        self.homing_motion_epsilon = finite(defaults.get('command_motion_epsilon_deg', .2), '调零启动位移阈值')
        self.drive_feedback_timeout = finite(defaults.get('drive_feedback_timeout_seconds', 8),
                                             '驱动状态确认超时')
        if min(self.homing_command_retry, self.homing_motion_epsilon,
               self.drive_feedback_timeout, self.return_current,
               self.return_velocity, self.return_acceleration) <= 0:
            raise Rejected('调零重发、回位和状态确认参数必须大于 0')
        if self.return_current >= self.restored_protection_current:
            raise Rejected('调零回位电流必须低于运行保护电流')
        commissioning = commissioning_defaults or {}
        self.commissioning_step_max = finite(commissioning.get('max_step_deg', 2), '调试最大步距')
        self.commissioning_velocity = finite(commissioning.get('velocity', 2), '调试速度')
        self.commissioning_acceleration = finite(commissioning.get('acceleration', 2), '调试加速度')
        self.commissioning_margin = finite(commissioning.get('hard_stop_margin_deg', 2), '机械限位余量')
        if min(self.commissioning_step_max, self.commissioning_velocity,
               self.commissioning_acceleration, self.commissioning_margin) <= 0:
            raise Rejected('调试模式参数必须大于 0')
        manual = manual_defaults or {}
        self.manual_velocity = finite(manual.get('velocity', 10), '手动运行速度')
        self.manual_acceleration = finite(manual.get('acceleration', 10), '手动运行加速度')
        self.manual_max_step = finite(manual.get('max_step_deg', 2), '手动最大点动步距')
        self.manual_min_scale = finite(manual.get('min_speed_scale', .1), '手动最小速度倍率')
        if min(self.manual_velocity, self.manual_acceleration,
               self.manual_max_step, self.manual_min_scale) <= 0 or self.manual_min_scale > 1:
            raise Rejected('手动运行参数无效')
        configured_pose = manual.get('safe_pose') or {}
        self.safe_pose = {}
        for raw_node_id, value in configured_pose.items():
            try:
                node_id = int(raw_node_id)
            except (TypeError, ValueError):
                raise Rejected(f'安全姿态包含非法关节编号：{raw_node_id}') from None
            if node_id not in self.nodes:
                raise Rejected(f'安全姿态包含未知关节 J{node_id}')
            self.safe_pose[node_id] = Limits.read(self.nodes[node_id]).check(value)

    @property
    def progress(self):
        return getattr(self, '_progress', '')

    @progress.setter
    def progress(self, value):
        previous = getattr(self, '_progress', '')
        self._progress = value
        if value and value != previous and hasattr(self, 'last_targets'):
            self.log(value, category='正在重试' if '重发' in value else '阶段更新',
                     update_result=False)

    def log(self, message, category=None, update_result=True):
        with self.lock:
            message = str(message)
            if category is None:
                category = ('任务超时' if '超时' in message else
                            '驱动器故障' if '驱动器故障' in message else
                            '任务失败' if '失败' in message else
                            '软件锁定' if self.latched else '运行事件')
            wall, mono = time.time(), time.monotonic()
            details = []
            try:
                for status in self.controller.get_all_motor_status():
                    n = status['node_id']
                    details.append(dict(status, target=self.last_targets.get(n),
                                        status_updated=updated_at(status.get('status_received_at'), wall, mono),
                                        position_updated=updated_at(status.get('position_received_at'), wall, mono),
                                        current_updated=updated_at(status.get('current_received_at'), wall, mono)))
            except Exception:
                pass
            event = {'time': datetime.now().astimezone().isoformat(timespec='milliseconds'),
                     'category': category, 'message': message, 'task': self.active,
                     'stage': self.progress, 'node': self.active_node,
                     'locked': self.latched, 'details': details}
            self.events.append(event)
            if self.journal:
                self.journal.append(event)
            if update_result:
                self.last_result = message

    def fresh(self, status):
        return (status['status_age'] <= self.stale_seconds and
                status['position_age'] <= self.stale_seconds)

    def check_nodes(self, ids, enabled=False, limits=False, allow_fault=False):
        if self.closed:
            raise Rejected('控制服务已关闭')
        if not self.controller.is_initialized() or self.communication_error:
            raise Rejected('CAN 不可用：' + (self.communication_error or '未连接'))
        statuses = {s['node_id']: s for s in self.controller.get_all_motor_status()}
        for node_id in ids:
            if node_id not in self.nodes or node_id not in statuses:
                raise Rejected(f'J{node_id} 未初始化')
            status = statuses[node_id]
            if not self.fresh(status):
                raise Rejected(f"J{node_id} 反馈过期（状态 {status['status_age']:.2f}s，"
                               f"位置 {status['position_age']:.2f}s）")
            if not allow_fault and any(status['errors'].values()):
                raise Rejected(f'J{node_id} 存在驱动器故障')
            if enabled and not status['enabled']:
                raise Rejected(f'J{node_id} 未使能')
            if limits:
                Limits.read(self.nodes[node_id]).check(status['position'])
        return statuses

    def active_statuses(self, ids, enabled=None, limits=False, allow_fault=False):
        """Validate an active task without treating one dropped status reply as offline.

        A command may be retried only while position feedback remains fresh. A
        fresh status frame is still required before declaring state transitions
        or target completion.
        """
        if self.closed:
            raise Rejected('控制服务已关闭')
        if not self.controller.is_initialized() or self.communication_error:
            raise Rejected('CAN 不可用：' + (self.communication_error or '未连接'))
        statuses = {item['node_id']: item for item in self.controller.get_all_motor_status()}
        result = {}
        for node_id in ids:
            if node_id not in self.nodes or node_id not in statuses:
                raise Rejected(f'J{node_id} 未初始化')
            status = statuses[node_id]
            if status['position_age'] > self.stale_seconds:
                raise Rejected(f"J{node_id} 反馈过期（状态 {status['status_age']:.2f}s，"
                               f"位置 {status['position_age']:.2f}s）")
            if not allow_fault and any(status['errors'].values()):
                raise Rejected(f'J{node_id} 存在驱动器故障')
            # A reported disabled state is actionable even if its timestamp is
            # aging; an old enabled state is never enough to declare success.
            if enabled is True and not status['enabled']:
                raise Rejected(f'J{node_id} 未使能')
            if enabled is False and status['enabled'] and status['status_age'] <= self.stale_seconds:
                raise Rejected(f'J{node_id} 仍处于使能状态')
            if limits:
                Limits.read(self.nodes[node_id]).check(status['position'])
            result[node_id] = status
        return result

    def reason(self, ids=None):
        with self.lock:
            if self.latched:
                return '停止锁定：检查设备后解除软件锁定'
            if self.active:
                return f'任务进行中：{self.active}'
            try:
                checked = list(self.nodes) if ids is None else ids
                self.check_nodes(checked)
                missing = [n for n in checked if self.nodes[n].get('homing') and n not in self.homed_nodes]
                if missing:
                    raise Rejected('本次启动尚未完成碰撞调零：' + ', '.join(f'J{n}' for n in missing))
                self.check_nodes(checked, limits=True)
            except Rejected as error:
                return str(error)
            return ''

    def homing_reason(self):
        with self.lock:
            if self.latched:
                return '停止锁定：检查设备后解除软件锁定'
            if self.active:
                return f'任务进行中：{self.active}'
            try:
                profiles = [HomingProfile.read(node) for node in self.nodes.values()]
                if len({p.order for p in profiles}) != len(profiles):
                    raise Rejected('碰撞调零顺序包含重复值')
                if any(p.current * self.verification_current_factor >= self.protection_current
                       for p in profiles):
                    raise Rejected('二次确认电流必须低于寻零保护电流')
                self.check_nodes(self.nodes, allow_fault=True)
            except Rejected as error:
                return str(error)
            return ''

    def commissioning_envelope(self, node_id):
        if node_id not in self.nodes:
            raise Rejected(f'未知关节 J{node_id}')
        profile = HomingProfile.read(self.nodes[node_id])
        extent = abs(profile.search)
        if extent <= self.commissioning_margin * 2:
            raise Rejected(f'J{node_id} 临时调试包络无效')
        if profile.search < 0:
            return Limits(self.commissioning_margin, extent - self.commissioning_margin)
        return Limits(-extent + self.commissioning_margin, -self.commissioning_margin)

    def commissioning_reason(self, node_id):
        with self.lock:
            if self.latched:
                return '停止锁定：检查设备后解除软件锁定'
            if self.active:
                return f'任务进行中：{self.active}'
            try:
                if set(self.nodes) - self.homed_nodes:
                    raise Rejected('请先完成全部关节自动调零')
                status = self.check_nodes([node_id], enabled=True)[node_id]
                self.commissioning_envelope(node_id).check(status['position'])
            except Rejected as error:
                return str(error)
            return ''

    def validate_commissioning_limits(self, node_id, minimum, maximum):
        with self.lock:
            reason = self.commissioning_reason(node_id)
            if reason:
                raise Rejected(reason)
            limits = Limits(finite(minimum, '最小软限位'), finite(maximum, '最大软限位'))
            if limits.minimum >= limits.maximum:
                raise Rejected('最小软限位必须小于最大软限位')
            envelope = self.commissioning_envelope(node_id)
            envelope.check(limits.minimum)
            envelope.check(limits.maximum)
            current = self.check_nodes([node_id], enabled=True)[node_id]['position']
            limits.check(current)
            return {'min_deg': limits.minimum, 'max_deg': limits.maximum, 'verified': True}

    def apply_verified_limits(self, node_id, limits):
        # Call only after the atomic configuration-file replacement succeeds.
        checked = self.validate_commissioning_limits(node_id, limits['min_deg'], limits['max_deg'])
        with self.lock:
            self.nodes[node_id]['limits'] = checked
        self.log(f"J{node_id} 软限位已核实并生效：[{checked['min_deg']:g}, {checked['max_deg']:g}]°")

    def _checkpoint(self):
        if self.closed or self.latched or self.cancel_event.is_set():
            raise Cancelled('已取消；已下发的目标不会被撤回')

    def _wait(self, seconds):
        if self.cancel_event.wait(seconds):
            raise Cancelled('已取消；已下发的目标不会被撤回')
        self._checkpoint()

    def submit(self, name, operation):
        with self.lock:
            if self.closed or self.latched:
                raise Rejected('停止锁定或服务关闭，拒绝新任务')
            if self.active is not None:
                raise Rejected(f'请先结束当前任务：{self.active}')
            self.cancel_event.clear()
            self.active = name
            self.progress = ''

            def run():
                try:
                    self._checkpoint()
                    operation()
                    self._checkpoint()
                    self.log(f'{name}：完成')
                except Cancelled as error:
                    self.log(f'{name}：{error}')
                except Exception as error:
                    with self.lock:
                        self.latched = True
                        self.cancel_event.set()
                    self.log(f'{name}：失败，已锁定后续指令；{error}')
                finally:
                    with self.lock:
                        self.active = None
                        self.active_node = None

            self.worker = threading.Thread(target=run, daemon=True, name='gui-command')
            self.worker.start()

    def validate_targets(self, targets):
        if not targets:
            raise Rejected('未选择关节')
        missing = [n for n in targets if self.nodes.get(n, {}).get('homing') and n not in self.homed_nodes]
        if missing:
            raise Rejected('本次启动尚未完成碰撞调零：' + ', '.join(f'J{n}' for n in missing))
        self.check_nodes(targets, enabled=True, limits=True)
        result = {}
        for node_id, value in targets.items():
            if node_id not in self.nodes:
                raise Rejected(f'未知关节 {node_id}')
            result[node_id] = Limits.read(self.nodes[node_id]).check(value)
        return result

    def _manual_profile(self, speed_scale):
        speed_scale = finite(speed_scale, '速度倍率')
        if not self.manual_min_scale <= speed_scale <= 1:
            raise Rejected(f'速度倍率必须在 {self.manual_min_scale * 100:g}%–100% 之间')
        return self.manual_velocity * speed_scale, self.manual_acceleration * speed_scale

    def _configure_manual_profile(self, targets, speed_scale):
        velocity, acceleration = self._manual_profile(speed_scale)
        for node_id in targets:
            motor = self.controller.motors[node_id]
            for index, value in ((PROFILE_VELOCITY, velocity),
                                 (PROFILE_ACCEL, acceleration),
                                 (PROFILE_DECEL, acceleration)):
                self._checkpoint()
                motor.set_float_config(index, value)
                self._wait(.03)

    def _move(self, targets, speed_scale=None, require_target_reached=True,
              position_tolerance=1.0):
        # Validate the entire group before sending its first command.
        targets = self.validate_targets(targets)
        if speed_scale is not None:
            self._configure_manual_profile(targets, speed_scale)
        sent_at = {}
        last_command_at = {}
        command_count = {node_id: 1 for node_id in targets}
        for node_id, target in targets.items():
            with self.lock:
                self._checkpoint()
                self.check_nodes(targets, enabled=True, limits=True)
                sent_at[node_id] = time.monotonic()
                last_command_at[node_id] = sent_at[node_id]
                self.last_targets[node_id] = target
                self.controller.motors[node_id].set_position(target)
                self.log(f'J{node_id} 已发送目标 {target:g}°', category='运动指令', update_result=False)
        deadline = time.monotonic() + self.move_timeout
        while True:
            self._checkpoint()
            statuses = self.active_statuses(targets, enabled=True, limits=True)
            now = time.monotonic()
            # Require post-command status AND position responses, not old zero values.
            complete = {
                n: ((statuses[n]['position_received_at'] or 0) > sent_at[n] and
                    (statuses[n]['status_received_at'] or 0) > sent_at[n] and
                    statuses[n]['status_age'] <= self.stale_seconds and
                    (statuses[n]['target_reached'] or not require_target_reached) and
                    abs(statuses[n]['position'] - target) <= position_tolerance)
                for n, target in targets.items()
            }
            if all(complete.values()):
                return
            for node_id, target in targets.items():
                if not complete[node_id] and now - last_command_at[node_id] >= self.homing_command_retry:
                    self.controller.motors[node_id].set_position(target)
                    last_command_at[node_id] = now
                    command_count[node_id] += 1
                    self.progress = (f'等待新鲜到位反馈；J{node_id} 已重发目标 '
                                     f'{command_count[node_id]} 次')
            if now >= deadline:
                raise Rejected('等待到位超时；已下发目标可能仍在执行，请检查机械臂')
            self._wait(0.05)

    def move(self, targets, speed_scale=None, name='定位',
             require_target_reached=True, position_tolerance=1.0):
        targets = self.validate_targets(targets)
        if speed_scale is not None:
            self._manual_profile(speed_scale)
        position_tolerance = finite(position_tolerance, '到位容差')
        if position_tolerance <= 0:
            raise Rejected('到位容差必须大于 0')
        self.submit(name, lambda: self._move(targets, speed_scale,
                                             require_target_reached,
                                             position_tolerance))

    def jog(self, node_id, delta, speed_scale):
        node_id = int(node_id)
        delta = finite(delta, '点动步距')
        if delta == 0 or abs(delta) > self.manual_max_step:
            raise Rejected(f'单次点动步距必须大于 0 且不超过 {self.manual_max_step:g}°')
        status = self.check_nodes([node_id], enabled=True, limits=True)[node_id]
        target = Limits.read(self.nodes[node_id]).check(status['position'] + delta)
        self.move({node_id: target}, speed_scale, f'J{node_id} 点动 {delta:+g}°')

    def midpoint_pose(self):
        return {node_id: (limits.minimum + limits.maximum) / 2
                for node_id in self.nodes
                for limits in (Limits.read(self.nodes[node_id]),)}

    def configured_safe_pose(self):
        if set(self.safe_pose) != set(self.nodes):
            missing = sorted(set(self.nodes) - set(self.safe_pose))
            raise Rejected('安全姿态配置不完整：' + ', '.join(f'J{n}' for n in missing))
        return dict(self.safe_pose)

    def sequence(self, steps, name='轨迹回放', speed_scale=None):
        # Freeze & validate every target and interval before starting the sequence.
        if not steps or len(steps) > 10000:
            raise Rejected('轨迹必须包含 1–10000 个点')
        prepared = []
        for targets, delay in steps:
            delay = finite(delay, '等待时间')
            if not 0 <= delay <= 60:
                raise Rejected('等待时间必须在 0–60 秒之间')
            prepared.append((self.validate_targets(targets), delay))
        if speed_scale is not None:
            self._manual_profile(speed_scale)

        def run():
            if speed_scale is not None:
                involved = {node_id: target for targets, _ in prepared
                            for node_id, target in targets.items()}
                self._configure_manual_profile(involved, speed_scale)
            for index, (targets, delay) in enumerate(prepared, 1):
                with self.lock:
                    self.progress = f'{index} / {len(prepared)}'
                self._move(targets)
                self._wait(delay)
        self.submit(name, run)

    def commissioning_move(self, node_id, delta):
        node_id = int(node_id)
        delta = finite(delta, '调试步距')
        if delta == 0 or abs(delta) > self.commissioning_step_max:
            raise Rejected(f'单次调试步距必须大于 0 且不超过 {self.commissioning_step_max:g}°')
        reason = self.commissioning_reason(node_id)
        if reason:
            raise Rejected(reason)
        status = self.check_nodes([node_id], enabled=True)[node_id]
        target = self.commissioning_envelope(node_id).check(status['position'] + delta)
        profile = HomingProfile.read(self.nodes[node_id])

        def run():
            motor = self.controller.motors[node_id]
            for index, value in ((PROFILE_VELOCITY, self.commissioning_velocity),
                                 (PROFILE_ACCEL, self.commissioning_acceleration),
                                 (PROFILE_DECEL, self.commissioning_acceleration)):
                self._checkpoint()
                motor.set_float_config(index, value)
                self._wait(.05)
            sent_at = time.monotonic()
            motor.set_position(target)
            self.last_targets[node_id] = target
            self.progress = f'J{node_id}：软限位调试到 {target:g}°'
            last_command_at = sent_at
            command_count = 1
            deadline = time.monotonic() + self.move_timeout
            while True:
                self._checkpoint()
                current = self.active_statuses([node_id], enabled=True)[node_id]
                if ((current['position_received_at'] or 0) > sent_at and
                        (current['status_received_at'] or 0) > sent_at and
                        current['status_age'] <= self.stale_seconds and
                        current['target_reached'] and abs(current['position'] - target) <= .5):
                    break
                now = time.monotonic()
                if now - last_command_at >= self.homing_command_retry:
                    motor.set_position(target)
                    last_command_at = now
                    command_count += 1
                    self.progress = (f'J{node_id} 调试移动等待反馈；已重发目标 '
                                     f'{command_count} 次')
                if time.monotonic() >= deadline:
                    raise Rejected(f'J{node_id} 调试移动到位超时')
                self._wait(.05)
            for index, value in ((PROFILE_VELOCITY, profile.velocity),
                                 (PROFILE_ACCEL, profile.acceleration),
                                 (PROFILE_DECEL, profile.acceleration)):
                self._checkpoint()
                motor.set_float_config(index, value)
                self._wait(.05)
        self.submit(f'J{node_id} 软限位调试 {delta:+g}°', run)

    def _homing_status(self, node_id, allow_over_current=True, allow_stale_status=False,
                       allow_drive_faults=False):
        if not allow_stale_status:
            status = self.check_nodes([node_id], allow_fault=True)[node_id]
        else:
            if self.closed:
                raise Rejected('控制服务已关闭')
            if not self.controller.is_initialized() or self.communication_error:
                raise Rejected('CAN 不可用：' + (self.communication_error or '未连接'))
            statuses = {item['node_id']: item for item in self.controller.get_all_motor_status()}
            if node_id not in statuses:
                raise Rejected(f'J{node_id} 未初始化')
            status = statuses[node_id]
            # During homing, a fresh position reply proves that the node is still
            # communicating. Do not abort solely because an independent status
            # request was dropped; the active poll continues requesting it.
            if status['position_age'] > self.stale_seconds:
                raise Rejected(f"J{node_id} 反馈过期（状态 {status['status_age']:.2f}s，"
                               f"位置 {status['position_age']:.2f}s）")
        other_faults = [name for name, present in status['errors'].items()
                        if present and (name != 'over_current' or not allow_over_current)]
        if other_faults and not allow_drive_faults:
            raise Rejected(f"J{node_id} 驱动器故障：{', '.join(other_faults)}")
        return status

    def _wait_drive_feedback(self, node_id, sent_at, enabled, resend, timeout=None):
        timeout = self.drive_feedback_timeout if timeout is None else timeout
        deadline = time.monotonic() + timeout
        last_command_at = sent_at
        command_count = 1
        while True:
            self._checkpoint()
            status = self._homing_status(node_id, allow_stale_status=True,
                                         allow_drive_faults=True)
            if ((status['status_received_at'] or 0) > sent_at and
                    status['enabled'] is enabled and not any(status['errors'].values())):
                return status
            now = time.monotonic()
            if now - last_command_at >= self.homing_command_retry:
                resend()
                last_command_at = now
                command_count += 1
                state = '使能' if enabled else '失能 / 故障清除'
                self.progress = f'J{node_id}：等待{state}确认，已重发 {command_count} 次'
            if time.monotonic() >= deadline:
                state = '使能' if enabled else '失能 / 故障清除'
                raise Rejected(f'J{node_id} 未收到新的{state}反馈')
            self._wait(.05)

    def _send_disable_and_reset(self, motor):
        motor.disable()
        self._wait(.05)
        motor.error_resets()

    def _reset_and_enable_for_homing(self, node_id, current):
        motor = self.controller.motors[node_id]
        motor.disable()
        self._wait(.1)
        reset_at = time.monotonic()
        motor.error_resets()
        self._wait_drive_feedback(
            node_id, reset_at, enabled=False,
            resend=lambda: self._send_disable_and_reset(motor))
        motor.set_float_config(CURRENT_LIMIT, current)
        self._wait(.05)
        motor.set_stop_damping_mode()
        self._wait(.1)
        enabled_at = time.monotonic()
        motor.enable()
        self._wait_drive_feedback(node_id, enabled_at, enabled=True, resend=motor.enable)

    def _release_homing_stop(self, node_id, profile, distance, current):
        """Clear a stop fault and verify a real move opposite the search direction."""
        self.progress = f'J{node_id}：解除限位并验证回退'
        self._reset_and_enable_for_homing(node_id, current)
        status = self._homing_status(node_id, allow_over_current=False,
                                     allow_stale_status=True)
        start = status['position']
        direction = 1 if profile.search < 0 else -1
        target = start + direction * distance
        self.log(f'J{node_id} 发送限位回退：{start:.2f}° → {target:.2f}°，电流限制 {current:.2f}')
        self._wait_homing_position(node_id, target, self.release_timeout)
        moved = abs(self._homing_status(
            node_id, allow_over_current=False, allow_stale_status=True)['position'] - start)
        if moved < min(self.movement_threshold, distance * .5):
            raise Rejected(f'J{node_id} 回退指令已发送但没有确认到有效位移；请核查电流、故障复位和机械卡阻')

    def _search_homing_stop(self, node_id, profile, detection_current):
        motor = self.controller.motors[node_id]
        start = self._homing_status(node_id, allow_stale_status=True)['position']
        try:
            limits = Limits.read(self.nodes[node_id])
            configured_span = limits.maximum - limits.minimum + 10
        except Rejected:
            configured_span = 0
        travel = max(abs(profile.search), configured_span)
        target = start + (-travel if profile.search < 0 else travel)
        sent_at = time.monotonic()
        motor.set_position(target)
        self.last_targets[node_id] = target
        last_command_at = sent_at
        command_count = 1
        movement_confirmed = False
        self.progress = (f'J{node_id}：从 {start:.2f}° 向'
                         f"{'负' if profile.search < 0 else '正'}方向搜索 {travel:g}°")
        deadline = time.monotonic() + self.homing_timeout
        stable_since = None
        stable_position = None
        last_position_feedback = None
        while True:
            self._checkpoint()
            status = self._homing_status(node_id, allow_stale_status=True)
            if not movement_confirmed:
                movement_confirmed = abs(status['position'] - start) >= self.homing_motion_epsilon
                now = time.monotonic()
                if not movement_confirmed and now - last_command_at >= self.homing_command_retry:
                    motor.set_position(target)
                    last_command_at = now
                    command_count += 1
                    self.progress = (f'J{node_id}：等待启动，已重发搜索目标 '
                                     f'{command_count} 次')
            # Some drives stop periodic value replies immediately after latching
            # over-current. Capture the fresh drive fault before freshness expires;
            # the second approach below still has to reproduce the same position.
            if (status['errors'].get('over_current') and
                    (status['status_received_at'] or 0) > sent_at and
                    (status['position_received_at'] or 0) > sent_at):
                self.log(f"J{node_id} 过流候选：位置 {status['position']:.2f}°，"
                         f"速度 {status['velocity']:.2f}，电流 {status['current']:.2f}")
                return status['position'], abs(status['position'] - start)
            feedback_times = [status.get(name) for name in
                              ('status_received_at', 'position_received_at',
                               'velocity_received_at', 'current_received_at')]
            feedback_fresh = all(
                value is not None and value > sent_at and
                time.monotonic() - value <= self.stale_seconds
                for value in feedback_times[1:])
            if (feedback_fresh and
                    status['position_received_at'] != last_position_feedback):
                last_position_feedback = status['position_received_at']
                drive_limit = bool(status.get('current_limit') or
                                   status['errors'].get('over_current'))
                stopped = (abs(status['velocity']) < self.stall_velocity and
                           (drive_limit or abs(status['current']) >= detection_current * .6))
                if stopped:
                    if stable_position is None or abs(status['position'] - stable_position) > self.collision_stability:
                        stable_position = status['position']
                        stable_since = time.monotonic()
                    elif time.monotonic() - stable_since >= self.collision_confirm_seconds:
                        self.log(f"J{node_id} 停转候选：位置 {status['position']:.2f}°，"
                                 f"速度 {status['velocity']:.2f}，电流 {status['current']:.2f}，"
                                 f"过流={bool(status['errors'].get('over_current'))}，"
                                 f"限流={bool(status.get('current_limit'))}")
                        return status['position'], abs(status['position'] - start)
                else:
                    stable_since = None
                    stable_position = None
            if time.monotonic() >= deadline:
                now = time.monotonic()
                ages = {name: (now - status.get(name) if status.get(name) is not None else float('inf'))
                        for name in ('status_received_at', 'position_received_at',
                                     'velocity_received_at', 'current_received_at')}
                raise Rejected(
                    f"J{node_id} 搜索限位超时；位置={status['position']:.2f}°，"
                    f"速度={status['velocity']:.2f}，电流={status['current']:.2f}，"
                    f"到位={bool(status['target_reached'])}，限流={bool(status.get('current_limit'))}，"
                    f"过流={bool(status['errors'].get('over_current'))}，"
                    f"反馈年龄(状态/位置/速度/电流)="
                    f"{ages['status_received_at']:.2f}/{ages['position_received_at']:.2f}/"
                    f"{ages['velocity_received_at']:.2f}/{ages['current_received_at']:.2f}s")
            self._wait(.05)

    def _set_homing_config(self, motor, profile, protection):
        settings = ((CURRENT_LIMIT, profile.current),
                    (PROTECT_OVER_CURRENT, protection),
                    (PROFILE_VELOCITY, profile.velocity),
                    (PROFILE_ACCEL, profile.acceleration),
                    (PROFILE_DECEL, profile.acceleration))
        for index, value in settings:
            self._checkpoint()
            motor.set_float_config(index, value)
            self._wait(.05)

    def _set_return_config(self, motor):
        """Restore load-bearing output before moving away from the hard stop."""
        settings = ((PROTECT_OVER_CURRENT, self.restored_protection_current),
                    (CURRENT_LIMIT, self.return_current),
                    (PROFILE_VELOCITY, self.return_velocity),
                    (PROFILE_ACCEL, self.return_acceleration),
                    (PROFILE_DECEL, self.return_acceleration))
        for index, value in settings:
            self._checkpoint()
            motor.set_float_config(index, value)
            self._wait(.05)

    def _wait_homing_position(self, node_id, target, timeout):
        start = self._homing_status(node_id, allow_over_current=False,
                                    allow_stale_status=True)['position']
        sent_at = time.monotonic()
        motor = self.controller.motors[node_id]
        motor.set_position(target)
        self.last_targets[node_id] = target
        self.progress = f'J{node_id}：回位到 {target:g}°'
        last_command_at = sent_at
        command_count = 1
        movement_confirmed = abs(start - target) < self.homing_motion_epsilon
        deadline = time.monotonic() + timeout
        while True:
            self._checkpoint()
            status = self._homing_status(node_id, allow_over_current=False,
                                         allow_stale_status=True)
            if ((status['position_received_at'] or 0) > sent_at and
                    abs(status['position'] - target) < 1.5):
                return
            if not movement_confirmed:
                movement_confirmed = abs(status['position'] - start) >= self.homing_motion_epsilon
                now = time.monotonic()
                if not movement_confirmed and now - last_command_at >= self.homing_command_retry:
                    motor.set_position(target)
                    last_command_at = now
                    command_count += 1
                    self.progress = (f'J{node_id}：等待回退启动，已重发目标 '
                                     f'{command_count} 次')
            if time.monotonic() >= deadline:
                raise Rejected(f'J{node_id} 回退到 {target:g}° 超时')
            self._wait(.1)

    def _home_joint(self, node_id, profile):
        motor = self.controller.motors[node_id]
        completed = False
        with self.lock:
            self.active_node = node_id
        try:
            self.progress = f'J{node_id}：准备低电流寻零'
            motor.disable()
            self._wait(.1)
            reset_at = time.monotonic()
            motor.error_resets()
            self._wait_drive_feedback(
                node_id, reset_at, enabled=False,
                resend=lambda: self._send_disable_and_reset(motor))
            self._set_homing_config(motor, profile, self.protection_current)
            motor.set_stop_damping_mode()
            self._wait(.1)
            enabled_at = time.monotonic()
            motor.enable()
            self._wait_drive_feedback(node_id, enabled_at, enabled=True, resend=motor.enable)

            if profile.release:
                start = self._homing_status(node_id, allow_stale_status=True)['position']
                self.progress = f'J{node_id}：探零前释放 {profile.release:g}°'
                motor.set_position(start + profile.release)
                self._wait(1.5)
                if self._homing_status(
                        node_id, allow_stale_status=True)['errors'].get('over_current'):
                    self._reset_and_enable_for_homing(node_id, profile.current)

            self.progress = f'J{node_id}：向 {profile.search:g}° 搜索物理限位'
            candidate, moved = self._search_homing_stop(node_id, profile, profile.current)
            verification_current = profile.current * self.verification_current_factor
            if moved < self.movement_threshold:
                self.log(f'J{node_id} 起步位置已在限位，先执行反向回退')
                self._release_homing_stop(node_id, profile, 30, verification_current)
                candidate, moved = self._search_homing_stop(node_id, profile, verification_current)
                if moved < self.movement_threshold:
                    raise Rejected(f'J{node_id} 回退后再次搜索仍无有效行程')

            self.progress = f'J{node_id}：二次接近确认限位重复性'
            self._release_homing_stop(node_id, profile, self.verification_backoff,
                                      verification_current)
            repeated, repeated_move = self._search_homing_stop(
                node_id, profile, verification_current)
            if repeated_move < self.movement_threshold:
                raise Rejected(f'J{node_id} 二次接近没有形成有效行程')
            if abs(repeated - candidate) > self.collision_repeatability:
                raise Rejected(f'J{node_id} 两次停转位置不一致（{candidate:.2f}° / {repeated:.2f}°）；可能电流不足、静摩擦或机构卡阻，拒绝写零点')

            self.progress = f'J{node_id}：已触限，写入零点并回退'
            motor.error_resets()
            self._wait(.1)
            motor.disable()
            self._wait(.2)
            motor.set_home()
            self._wait(.5)
            self._set_return_config(motor)
            self.log(f'J{node_id} 写零完成：回位电流 {self.return_current:g}，'
                     f'保护电流 {self.restored_protection_current:g}，'
                     f'速度 {self.return_velocity:g}，加速度 {self.return_acceleration:g}')
            motor.enable()
            self._wait(.2)
            self._wait_homing_position(node_id, profile.backoff, self.backoff_timeout)
            completed = True
        finally:
            if not completed:
                try:
                    motor.disable()
                    self.log(f'J{node_id} 调零未完成，已尝试失能该关节')
                except Exception as error:
                    self.log(f'J{node_id} 调零未完成且失能发送失败：{error}')

    def home_all(self):
        reason = self.homing_reason()
        if reason:
            raise Rejected(reason)
        profiles = {node_id: HomingProfile.read(node) for node_id, node in self.nodes.items()}
        ordered = sorted(profiles, key=lambda node_id: profiles[node_id].order)

        def run():
            self.homed_nodes.clear()
            for node_id in ordered:
                self._checkpoint()
                self._home_joint(node_id, profiles[node_id])
            # These are reviewed homing-recovery positions, not general user targets.
            for node_id in sorted(profiles):
                self.progress = f'J{node_id}：移动到调零后的待机姿态'
                try:
                    self._wait_homing_position(node_id, profiles[node_id].final, self.backoff_timeout)
                except Exception:
                    try:
                        self.controller.motors[node_id].disable()
                        self.log(f'J{node_id} 待机归位未完成，已尝试失能该关节')
                    finally:
                        raise
            self.homed_nodes.update(profiles)
            self.progress = '全部关节已完成本次启动的碰撞调零'
        self.submit('自动碰撞调零', run)

    def enable(self):
        ids = list(self.nodes)
        missing = [n for n in ids if self.nodes[n].get('homing') and n not in self.homed_nodes]
        if missing:
            raise Rejected('请先完成本次启动的碰撞调零')
        self.check_nodes(ids, limits=True)

        def run():
            sent_at = {}
            last_command_at = {}
            command_count = {node_id: 1 for node_id in ids}
            for node_id in ids:
                with self.lock:
                    self._checkpoint()
                    self.active_statuses(ids, limits=True)
                    sent_at[node_id] = time.monotonic()
                    last_command_at[node_id] = sent_at[node_id]
                    self.controller.motors[node_id].enable()
            deadline = time.monotonic() + self.drive_feedback_timeout
            while True:
                self._checkpoint()
                states = self.active_statuses(ids, limits=True)
                confirmed = {
                    node_id: (states[node_id]['enabled'] and
                              states[node_id]['status_age'] <= self.stale_seconds and
                              (states[node_id]['status_received_at'] or 0) > sent_at[node_id] and
                              not any(states[node_id]['errors'].values()))
                    for node_id in ids
                }
                if all(confirmed.values()):
                    return
                now = time.monotonic()
                for node_id in ids:
                    if (not confirmed[node_id] and
                            now - last_command_at[node_id] >= self.homing_command_retry):
                        self.controller.motors[node_id].enable()
                        last_command_at[node_id] = now
                        command_count[node_id] += 1
                        self.progress = (f'等待使能反馈；J{node_id} 已重发 '
                                         f'{command_count[node_id]} 次')
                if time.monotonic() >= deadline:
                    raise Rejected('使能反馈超时（可能存在部分关节已使能）')
                self._wait(0.05)
        self.submit('使能', run)

    def cancel(self, reason='用户取消'):
        with self.lock:
            self.cancel_event.set()
            self.latched = True
            self.log(reason + '；后续指令已锁定，已下发运动未保证停止')

    def disable(self):
        # Latch synchronously, before the caller returns or a bus write starts.
        self.cancel('失能请求')

        def run():
            failures = []
            for attempt in range(3):
                for node_id, motor in list(self.controller.motors.items()):
                    try:
                        with self.lock:
                            motor.disable()
                    except Exception as error:
                        failures.append(f'第{attempt + 1}轮 J{node_id}: {error}')
                if attempt < 2:
                    time.sleep(.1)
            self.log('失能发送失败：' + '; '.join(failures) if failures else
                     '失能指令已发送；请以新鲜的驱动器反馈确认，失能可能导致重力下坠')
        # Repeated clicks must not create concurrent writers.
        with self.lock:
            if getattr(self, 'disable_worker', None) and self.disable_worker.is_alive():
                return
            self.disable_worker = threading.Thread(target=run, daemon=True, name='gui-disable')
            self.disable_worker.start()

    def disable_and_confirm(self):
        """Disable every joint and require a new disabled status from each one."""
        self.cancel('工作结束失能请求')
        ids = list(self.nodes)
        sent_at = {node_id: None for node_id in ids}
        last_command_at = {node_id: 0.0 for node_id in ids}
        command_count = {node_id: 0 for node_id in ids}
        deadline = time.monotonic() + self.drive_feedback_timeout
        while True:
            states = {s['node_id']: s for s in self.controller.get_all_motor_status()}
            confirmed = {
                node_id: (sent_at[node_id] is not None and node_id in states and
                          not states[node_id]['enabled'] and
                          states[node_id]['status_age'] <= self.stale_seconds and
                          (states[node_id]['status_received_at'] or 0) > sent_at[node_id])
                for node_id in ids
            }
            if all(confirmed.values()):
                self.log('工作结束：全部关节已由新鲜反馈确认失能')
                return
            now = time.monotonic()
            for node_id in ids:
                if confirmed[node_id] or now - last_command_at[node_id] < self.homing_command_retry:
                    continue
                motor = self.controller.motors[node_id]
                with self.lock:
                    motor.disable()
                sent_at[node_id] = now
                last_command_at[node_id] = now
                command_count[node_id] += 1
                if hasattr(motor, 'reference_status'):
                    motor.reference_status()
                self.progress = (f'工作结束失能：等待 J{node_id} 新鲜反馈；'
                                 f'已发送 {command_count[node_id]} 次')
            if time.monotonic() >= deadline:
                pending = ', '.join(f'J{n}' for n in ids if not confirmed[n])
                raise Rejected(f'工作结束失能确认超时：{pending}；保留程序运行')
            time.sleep(.05)

    def unlock(self):
        with self.lock:
            if self.active or (getattr(self, 'disable_worker', None) and self.disable_worker.is_alive()):
                raise Rejected('等待当前任务退出')
            self.check_nodes(self.nodes)
            self.latched = False
            self.cancel_event.clear()
            self.log('已解除软件锁定；未清除驱动器故障，未自动恢复运动')

    def reset_errors(self):
        # Reset is a maintenance operation: preserve the latch throughout.
        with self.lock:
            if self.active or self.closed or (getattr(self, 'disable_worker', None) and self.disable_worker.is_alive()):
                raise Rejected('当前任务未结束')
            states = self.check_nodes(self.nodes, allow_fault=True)
            if any(s['enabled'] for s in states.values()):
                raise Rejected('请先确认所有驱动器已失能')
            self.latched = True
            self.active = '故障复位'
            self.cancel_event.clear()

            def maintenance():
                try:
                    for attempt in range(3):
                        for motor in self.controller.motors.values():
                            with self.lock:
                                if self.closed or self.cancel_event.is_set():
                                    raise Cancelled('复位已取消')
                                self.active_statuses(self.nodes, enabled=False,
                                                     allow_fault=True)
                                motor.error_resets()
                        if attempt < 2:
                            time.sleep(.1)
                    self.log('复位请求已重复发送 3 轮；检查反馈后手动解除软件锁定')
                except Exception as error:
                    self.log(f'故障复位失败：{error}')
                finally:
                    with self.lock:
                        self.active = None
            self.worker = threading.Thread(target=maintenance, daemon=True)
            self.worker.start()

    def close(self):
        self.cancel('服务关闭')
        with self.lock:
            self.closed = True
        for worker in (self.worker, getattr(self, 'disable_worker', None)):
            if worker:
                worker.join(timeout=2)

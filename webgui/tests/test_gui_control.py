"""Fault injection for GUI arbitration without physical hardware."""
import struct
import threading
import time
import unittest
from unittest.mock import Mock, patch

import can
from core.gui_control import GuiControl, HomingProfile, Rejected
from core.motor import (CURRENT_LIMIT, MOTOR_CURRENT, POSITION, PROFILE_ACCEL,
                        PROFILE_DECEL, PROFILE_VELOCITY, PROTECT_OVER_CURRENT,
                        VELOCITY, Motor)
from core.motorcontroller import MotorController


class FakeMotor:
    def __init__(self):
        self.sent = []
        self.enabled = True
        self.position = 0
        self.timestamp = time.monotonic()
        self.stale = False
        self.fault = False
        self.arrive = True
        self.polling = False

    def set_position(self, target):
        self.sent.append(target)
        if self.arrive:
            self.position = target
            self.timestamp = time.monotonic()

    def enable(self):
        self.enabled = True
        self.timestamp = time.monotonic()

    def disable(self):
        self.enabled = False
        self.timestamp = time.monotonic()

    def error_resets(self):
        self.fault = False
        self.timestamp = time.monotonic()

    def set_float_config(self, index, value):
        self.sent.append(('config', index, value))

    def set_stop_damping_mode(self):
        self.sent.append(('position_mode',))

    def set_home(self):
        self.sent.append(('home',))
        self.position = 0
        self.timestamp = time.monotonic()

    def status(self, node_id):
        if self.polling:
            self.timestamp = time.monotonic()
        return dict(node_id=node_id, enabled=self.enabled, position=self.position,
                    target_reached=self.arrive, status_age=99 if self.stale else 0,
                    position_age=99 if self.stale else 0,
                    velocity=0, current=2, current_limit=self.fault,
                    errors={'over_current': self.fault},
                    status_received_at=self.timestamp, position_received_at=self.timestamp,
                    velocity_received_at=self.timestamp, current_received_at=self.timestamp)


class FakeController:
    def __init__(self):
        self.motors = {1: FakeMotor(), 2: FakeMotor()}

    def is_initialized(self):
        return True

    def get_all_motor_status(self):
        return [m.status(n) for n, m in self.motors.items()]


class GuiControlTests(unittest.TestCase):
    def setUp(self):
        self.controller = FakeController()
        self.nodes = [dict(id=n, limits=dict(min_deg=-90, max_deg=90, verified=True))
                      for n in (1, 2)]
        self.control = GuiControl(self.controller, self.nodes, move_timeout=0.15)

    def tearDown(self):
        self.control.close()

    def join(self):
        self.control.worker.join(5)
        self.assertFalse(self.control.worker.is_alive())

    def test_unconfigured_limits_fail_closed(self):
        self.nodes[0]['limits']['verified'] = False
        with self.assertRaises(Rejected):
            self.control.move({1: 10})
        self.assertEqual(self.controller.motors[1].sent, [])

    def test_validate_entire_group_before_any_send(self):
        for value in (100, float('nan'), float('inf'), None, True):
            with self.subTest(value=value), self.assertRaises(Rejected):
                self.control.move({1: 10, 2: value})
        self.assertEqual(self.controller.motors[1].sent, [])

    def test_manual_jog_uses_fresh_position_soft_limit_and_scaled_profile(self):
        motor = self.controller.motors[1]
        motor.position = 10
        self.control.homed_nodes = {1, 2}
        self.control.jog(1, .5, .25)
        self.join()
        self.assertEqual(motor.sent[:3], [
            ('config', PROFILE_VELOCITY, 2.5),
            ('config', PROFILE_ACCEL, 2.5),
            ('config', PROFILE_DECEL, 2.5),
        ])
        self.assertEqual(motor.sent[-1], 10.5)
        self.assertEqual(self.control.last_targets[1], 10.5)

    def test_manual_jog_rejects_step_beyond_limit_before_send(self):
        motor = self.controller.motors[1]
        motor.position = 89
        self.control.homed_nodes = {1, 2}
        with self.assertRaises(Rejected):
            self.control.jog(1, 2, .25)
        self.assertEqual(motor.sent, [])

    def test_normal_move_retries_target_when_only_status_feedback_is_stale(self):
        motor = self.controller.motors[1]
        motor.position = 0
        motor.arrive = False
        self.control.homed_nodes = {1, 2}
        self.control.homing_command_retry = .01
        attempts = 0
        status_stale = False

        def accept_after_retries(target):
            nonlocal attempts, status_stale
            attempts += 1
            motor.sent.append(target)
            motor.timestamp = time.monotonic()
            status_stale = attempts < 3
            if attempts == 3:
                motor.position = target
                motor.arrive = True

        original_status = motor.status

        def status(node_id):
            value = original_status(node_id)
            value['status_age'] = 99 if status_stale else 0
            value['position_age'] = 0
            return value

        motor.status = status
        motor.set_position = accept_after_retries
        self.control.move({1: 10})
        self.join()
        self.assertFalse(self.control.latched)
        self.assertEqual(motor.sent, [10, 10, 10])

    def test_enable_retries_each_unconfirmed_node(self):
        self.control.homed_nodes = {1, 2}
        motor = self.controller.motors[1]
        motor.enabled = False
        self.controller.motors[2].enabled = False
        self.control.homing_command_retry = .01
        self.control.drive_feedback_timeout = .2
        attempts = 0
        status_stale = False

        def enable_after_retries():
            nonlocal attempts, status_stale
            attempts += 1
            status_stale = attempts < 3
            if attempts == 3:
                motor.enabled = True
                motor.timestamp = time.monotonic()

        original_status = motor.status

        def status(node_id):
            value = original_status(node_id)
            value['status_age'] = 99 if status_stale else 0
            value['position_age'] = 0
            return value

        motor.enable = enable_after_retries
        motor.status = status
        self.control.enable()
        self.join()
        self.assertFalse(self.control.latched)
        self.assertEqual(attempts, 3)

    def test_configured_safe_pose_requires_every_joint(self):
        self.control.safe_pose = {1: 0}
        with self.assertRaises(Rejected):
            self.control.configured_safe_pose()
        self.control.safe_pose = {1: 0, 2: 1}
        self.assertEqual(self.control.configured_safe_pose(), {1: 0, 2: 1})

    def test_preflight_entire_sequence(self):
        with self.assertRaises(Rejected):
            self.control.sequence([({1: 10}, 0), ({1: 200}, 0)])
        self.assertEqual(self.controller.motors[1].sent, [])

    def test_stale_fault_disabled_block(self):
        motor = self.controller.motors[1]
        for attribute, value in [('stale', True), ('fault', True), ('enabled', False)]:
            previous = getattr(motor, attribute)
            setattr(motor, attribute, value)
            with self.assertRaises(Rejected):
                self.control.move({1: 10})
            setattr(motor, attribute, previous)
        self.assertEqual(motor.sent, [])

    def test_success_requires_new_feedback(self):
        self.control.move({1: 10})
        self.join()
        self.assertFalse(self.control.latched)
        self.assertEqual(self.controller.motors[1].sent, [10])

    def test_old_reached_flag_is_not_completion(self):
        motor = self.controller.motors[1]
        motor.set_position = lambda target: motor.sent.append(target)
        self.control.move({1: 0})
        self.join()
        self.assertTrue(self.control.latched)
        self.assertIn('超时', self.control.last_result)

    def test_exit_pose_accepts_fresh_position_within_one_degree(self):
        motor = self.controller.motors[1]

        def position_without_reached(target):
            motor.sent.append(target)
            motor.position = target + .9
            motor.timestamp = time.monotonic()
            motor.arrive = False

        motor.set_position = position_without_reached
        self.control.move({1: 10}, require_target_reached=False,
                          position_tolerance=1.0)
        self.join()
        self.assertFalse(self.control.latched)
        self.assertIn('完成', self.control.last_result)

    def test_stalled_normal_move_requests_hold_and_latches(self):
        motor = self.controller.motors[1]
        motor.set_position = lambda target: motor.sent.append(target)
        self.control.following_error = 5
        self.control.no_progress_seconds = .03
        self.control.progress_epsilon = .1
        self.control.homing_command_retry = .01
        self.control.move_timeout = .3
        self.control.move({1: 20})
        self.join()
        self.assertTrue(self.control.latched)
        self.assertTrue(self.control.protective_stop_reason)
        self.assertEqual(motor.sent[-1], 0)
        self.assertTrue(any(e['category'] == '保护停止' for e in self.control.events))

    def test_task_reservation_is_atomic(self):
        entered = threading.Event()
        def task():
            entered.set()
            self.control._wait(10)
        self.control.submit('长任务', task)
        self.assertTrue(entered.wait(1))
        with self.assertRaises(Rejected):
            self.control.submit('重复任务', lambda: None)
        self.control.cancel()
        self.join()
        self.assertTrue(self.control.latched)

    def test_cancel_prevents_next_target(self):
        reached = threading.Event()
        motor = self.controller.motors[1]
        original = motor.set_position
        def send(target):
            original(target)
            reached.set()
        motor.set_position = send
        self.control.sequence([({1: 10}, 10), ({1: 20}, 0)])
        self.assertTrue(reached.wait(1))
        self.control.cancel()
        self.join()
        self.assertEqual(motor.sent, [10])
        with self.assertRaises(Rejected):
            self.control.move({1: 20})

    def test_lost_feedback_latches_running_task(self):
        motor = self.controller.motors[1]
        original = motor.set_position
        def send(target):
            original(target)
            motor.stale = True
        motor.set_position = send
        self.control.move({1: 10})
        self.join()
        self.assertTrue(self.control.latched)

    def test_disable_attempts_remaining_nodes_on_failure(self):
        self.controller.motors[1].disable = Mock(side_effect=RuntimeError('bus error'))
        self.control.disable()
        self.control.disable_worker.join(1)
        self.assertFalse(self.controller.motors[2].enabled)
        self.assertTrue(self.control.latched)
        self.assertIn('失败', self.control.last_result)

    def test_exit_disable_requires_new_disabled_feedback(self):
        self.control.disable_and_confirm()
        self.assertTrue(self.control.latched)
        self.assertFalse(any(m.enabled for m in self.controller.motors.values()))
        self.assertIn('确认失能', self.control.last_result)

    def test_exit_disable_retries_unconfirmed_joint(self):
        motor = self.controller.motors[1]
        calls = 0

        def delayed_disable():
            nonlocal calls
            calls += 1
            if calls >= 2:
                motor.enabled = False
                motor.timestamp = time.monotonic()

        motor.disable = delayed_disable
        self.control.disable_and_confirm()
        self.assertGreaterEqual(calls, 2)

    def test_reset_fault_never_unlatches_or_enables(self):
        for motor in self.controller.motors.values():
            motor.enabled = False
            motor.fault = True
        self.control.cancel()
        self.control.reset_errors()
        self.join()
        self.assertTrue(self.control.latched)
        self.assertFalse(any(m.enabled for m in self.controller.motors.values()))
        self.control.unlock()
        self.assertFalse(self.control.latched)

    def test_position_and_status_have_independent_freshness(self):
        motor = Motor(Mock(), 1, 80)
        self.assertEqual(motor.get_status_dict()['position_age'], float('inf'))
        motor.update_status(can.Message(data=struct.pack('<II', 0, 0)))
        self.assertLess(motor.get_status_dict()['status_age'], 1)
        self.assertEqual(motor.get_status_dict()['position_age'], float('inf'))
        motor.update_status_all(can.Message(data=struct.pack('<fI', 1, 2)))
        self.assertLess(motor.get_status_dict()['position_age'], 1)

    def test_gui_motion_poll_requests_only_required_values(self):
        bus = Mock()
        motor = Motor(bus, 1, 80)
        motor.reference_motion_feedback(inter_request_delay=0)
        self.assertEqual(bus.send.call_count, 3)
        requested = [call.args[0].data[0] for call in bus.send.call_args_list]
        self.assertEqual(requested, [POSITION, VELOCITY, MOTOR_CURRENT])
        bus.reset_mock()
        motor.reference_position_feedback()
        self.assertEqual(bus.send.call_count, 1)
        self.assertEqual(bus.send.call_args.args[0].data[0], POSITION)

    def test_post_homing_return_restores_load_bearing_current_and_slow_profile(self):
        motor = self.controller.motors[1]
        self.control._set_return_config(motor)
        self.assertEqual(motor.sent, [
            ('config', PROTECT_OVER_CURRENT, 12),
            ('config', CURRENT_LIMIT, 10),
            ('config', PROFILE_VELOCITY, 2),
            ('config', PROFILE_ACCEL, 5),
            ('config', PROFILE_DECEL, 5),
        ])

    def test_controller_counts_can_error_frames(self):
        with patch('core.motorcontroller.can.Bus', return_value=Mock()):
            controller = MotorController()
        controller.on_message_received(can.Message(
            arbitration_id=0, data=bytes.fromhex('0000040000000001'),
            is_extended_id=False, is_error_frame=True))
        health = controller.get_bus_health()
        self.assertEqual(health['error_frames'], 1)
        self.assertEqual(health['last_error_data'], '0000040000000001')

    def test_homing_does_not_require_pre_homing_soft_limits(self):
        motor = self.controller.motors[1]
        motor.polling = True
        self.controller.motors = {1: motor}
        self.nodes = [dict(id=1, limits=dict(min_deg=None, max_deg=None, verified=False),
                           homing=dict(order=1, current=2, search_deg=-360,
                                       backoff_deg=10, final_deg=12,
                                       velocity=10, acceleration=10))]
        self.control.close()
        self.control = GuiControl(self.controller, self.nodes,
                                  homing_defaults={'collision_confirm_seconds': .05}, move_timeout=.15)
        original = motor.set_position
        def collide_or_arrive(target):
            original(target)
            motor.fault = target < 0
            motor.position = 5 if target < 0 else target
            motor.timestamp = time.monotonic()
        motor.set_position = collide_or_arrive
        self.assertEqual(self.control.homing_reason(), '')
        self.control.home_all()
        self.join()
        self.assertFalse(self.control.latched)
        self.assertEqual(self.control.homed_nodes, {1})
        self.assertIn(('home',), motor.sent)
        with self.assertRaises(Rejected):
            self.control.move({1: 0})

    def test_cancelled_homing_attempts_to_disable_joint(self):
        motor = self.controller.motors[1]
        motor.polling = True
        self.controller.motors = {1: motor}
        self.nodes = [dict(id=1, limits=dict(min_deg=None, max_deg=None, verified=False),
                           homing=dict(order=1, current=2, search_deg=-360,
                                       backoff_deg=10, final_deg=12,
                                       velocity=10, acceleration=10))]
        self.control.close()
        self.control = GuiControl(self.controller, self.nodes,
                                  homing_defaults={'search_timeout_seconds': 10})
        searching = threading.Event()
        def keep_searching(target):
            motor.sent.append(target)
            if target < 0:
                searching.set()
                motor.arrive = False
        motor.set_position = keep_searching
        self.control.home_all()
        self.assertTrue(searching.wait(2))
        self.control.cancel()
        self.join()
        self.assertFalse(motor.enabled)
        self.assertNotIn(('home',), motor.sent)

    def test_homing_releases_when_starting_on_the_limit(self):
        motor = self.controller.motors[1]
        motor.polling = True
        self.controller.motors = {1: motor}
        self.nodes = [dict(id=1, limits=dict(min_deg=0, max_deg=100, verified=True),
                           homing=dict(order=1, current=2, search_deg=-360,
                                       backoff_deg=10, final_deg=12,
                                       velocity=10, acceleration=10))]
        self.control.close()
        self.control = GuiControl(
            self.controller, self.nodes,
            homing_defaults={'collision_confirm_seconds': .05,
                             'verification_backoff_deg': 8}, move_timeout=.2)
        original = motor.set_position
        def collision_at_zero(target):
            original(target)
            motor.fault = target < 0
            motor.position = 0 if target < 0 else target
            motor.timestamp = time.monotonic()
        motor.set_position = collision_at_zero
        self.control.home_all()
        self.join()
        self.assertFalse(self.control.latched)
        self.assertIn(30, motor.sent)
        self.assertIn(('home',), motor.sent)

    def test_non_repeatable_air_wall_is_not_written_as_home(self):
        motor = self.controller.motors[1]
        motor.polling = True
        motor.position = 10
        self.controller.motors = {1: motor}
        self.nodes = [dict(id=1, limits=dict(min_deg=0, max_deg=100, verified=True),
                           homing=dict(order=1, current=2, search_deg=-360,
                                       backoff_deg=10, final_deg=12,
                                       velocity=10, acceleration=10))]
        self.control.close()
        self.control = GuiControl(
            self.controller, self.nodes,
            homing_defaults={'collision_confirm_seconds': .05,
                             'verification_backoff_deg': 8}, move_timeout=.2)
        collisions = iter((50, 40))
        original = motor.set_position
        def inconsistent_stop(target):
            original(target)
            motor.fault = target < 0
            motor.position = next(collisions) if target < 0 else target
            motor.timestamp = time.monotonic()
        motor.set_position = inconsistent_stop
        self.control.home_all()
        self.join()
        self.assertTrue(self.control.latched)
        self.assertNotIn(('home',), motor.sent)
        self.assertIn('两次停转位置不一致', self.control.last_result)

    def test_fresh_overcurrent_is_captured_before_drive_stops_reporting(self):
        motor = self.controller.motors[1]
        motor.position = 20
        self.controller.motors = {1: motor}
        self.nodes = [dict(id=1, limits=dict(min_deg=0, max_deg=353, verified=True),
                           homing=dict(order=1, current=3, search_deg=-360,
                                       backoff_deg=180, final_deg=180,
                                       velocity=10, acceleration=10))]
        self.control.close()
        self.control = GuiControl(self.controller, self.nodes, stale_seconds=.05)
        profile = HomingProfile.read(self.nodes[0])
        def collide(target):
            motor.sent.append(target)
            motor.position = -100
            motor.fault = True
            motor.timestamp = time.monotonic()
        motor.set_position = collide
        position, moved = self.control._search_homing_stop(1, profile, 3)
        self.assertEqual(position, -100)
        self.assertEqual(moved, 120)

    def test_homing_retries_target_while_status_reply_is_stale_but_position_is_fresh(self):
        motor = self.controller.motors[1]
        motor.position = 20
        motor.arrive = False
        self.controller.motors = {1: motor}
        self.nodes = [dict(id=1, limits=dict(min_deg=0, max_deg=353, verified=True),
                           homing=dict(order=1, current=3, search_deg=-360,
                                       backoff_deg=180, final_deg=180,
                                       velocity=10, acceleration=10))]
        self.control.close()
        self.control = GuiControl(
            self.controller, self.nodes, stale_seconds=.05,
            homing_defaults={'search_timeout_seconds': .2,
                             'command_retry_seconds': .01})
        profile = HomingProfile.read(self.nodes[0])
        sends = 0

        def accept_after_retries(target):
            nonlocal sends
            sends += 1
            motor.sent.append(target)
            motor.stale_status = True
            motor.timestamp = time.monotonic()
            if sends == 3:
                motor.position = 18
                motor.fault = True

        original_status = motor.status

        def status(node_id):
            value = original_status(node_id)
            value['status_age'] = 99 if getattr(motor, 'stale_status', False) else 0
            return value

        motor.status = status
        motor.set_position = accept_after_retries
        position, moved = self.control._search_homing_stop(1, profile, 3)
        self.assertEqual(sends, 3)
        self.assertEqual(position, 18)
        self.assertEqual(moved, 2)

    def test_homing_retries_enable_until_new_status_confirmation(self):
        motor = self.controller.motors[1]
        motor.enabled = False
        sent_at = time.monotonic()
        motor.timestamp = sent_at - 1
        attempts = 0

        def enable_after_retries():
            nonlocal attempts
            attempts += 1
            if attempts == 3:
                motor.enabled = True
                motor.timestamp = time.monotonic()

        original_status = motor.status

        def status(node_id):
            value = original_status(node_id)
            value['status_age'] = 99
            value['position_age'] = 0
            return value

        motor.status = status
        self.control.homing_command_retry = .01
        self.control.drive_feedback_timeout = .2
        self.control._wait_drive_feedback(
            1, sent_at, enabled=True, resend=enable_after_retries)
        self.assertEqual(attempts, 3)

    def test_commissioning_requires_homing_and_uses_small_bounded_steps(self):
        motor = self.controller.motors[1]
        motor.position = 10
        self.controller.motors = {1: motor}
        self.nodes = [dict(id=1, limits=dict(min_deg=None, max_deg=None, verified=False),
                           homing=dict(order=1, current=2, search_deg=-360,
                                       backoff_deg=10, final_deg=12,
                                       velocity=10, acceleration=10))]
        self.control.close()
        self.control = GuiControl(self.controller, self.nodes, move_timeout=.2,
                                  commissioning_defaults={'max_step_deg': 2})
        with self.assertRaises(Rejected):
            self.control.commissioning_move(1, 1)
        self.control.homed_nodes.add(1)
        with self.assertRaises(Rejected):
            self.control.commissioning_move(1, 3)
        self.control.commissioning_move(1, 2)
        self.join()
        self.assertFalse(self.control.latched)
        self.assertEqual(motor.position, 12)
        self.assertEqual(self.control.validate_commissioning_limits(1, 3, 20),
                         {'min_deg': 3.0, 'max_deg': 20.0, 'verified': True})
        with self.assertRaises(Rejected):
            self.control.validate_commissioning_limits(1, -1, 20)


if __name__ == '__main__':
    unittest.main()

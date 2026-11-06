#!/usr/bin/env python3
"""Pure conversion and safety-state tests; no ROS master or Gazebo needed."""

import importlib.util
import math
import pathlib
import types
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "thruster_output_adapter.py"
SPEC = importlib.util.spec_from_file_location("thruster_output_adapter", str(SCRIPT))
adapter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(adapter)


class ConversionTests(unittest.TestCase):
    def test_source_branch_and_parameter_order(self):
        self.assertAlmostEqual(adapter.glf_force(0.), .0414149, delta=.0001)
        self.assertEqual(adapter.glf_force(.01), 0.)
        self.assertAlmostEqual(adapter.glf_force(adapter.BRANCH_ABOVE), 1.2094, delta=.0001)
        self.assertEqual(adapter.glf_force(-1.), -100.)
        self.assertAlmostEqual(adapter.glf_force(1.), 242.6659, delta=.001)

    def test_force_round_trips_and_discontinuity_tolerance(self):
        for force in (-100., -80., -10., -.1, .02, .1, .5, 1., 2., 20., 100., 200., 242.):
            command = adapter.inverse_glf(force)
            self.assertTrue(math.isfinite(command))
            self.assertGreaterEqual(command, -1.)
            self.assertLessEqual(command, 1.)
            self.assertAlmostEqual(adapter.glf_force(command), force, delta=.58)
        self.assertEqual(adapter.inverse_glf(0.), 0.)

    def test_inverse_every_reachable_sample(self):
        for step in range(-100, 101):
            command = adapter.float32(step / 100.)
            force = adapter.glf_force(command)
            recovered = adapter.inverse_glf(force)
            self.assertAlmostEqual(adapter.glf_force(recovered), force, delta=.0001)

    def test_common_scaling_keeps_signs_and_ratios(self):
        requested = (600., -200., 100., -50.)
        scaled = adapter.scale_forces(requested)
        ratios = tuple(actual / desired for actual, desired in zip(scaled, requested))
        for ratio in ratios:
            self.assertAlmostEqual(ratio, ratios[0])
            self.assertGreater(ratio, 0.)
            self.assertLess(ratio, 1.)
        reverse, forward = adapter.force_limits()
        self.assertLessEqual(max(scaled), forward)
        self.assertGreaterEqual(min(scaled), reverse)
        commands = adapter.convert_raw_frame(tuple(force / 800. for force in requested))
        for command, desired in zip(commands, scaled):
            self.assertAlmostEqual(adapter.glf_force(command), desired, delta=.58)

    def test_custom_limits_and_invalid_values(self):
        self.assertEqual(adapter.force_limits(50., -30.), (-30., 50.))
        self.assertEqual(adapter.convert_raw_frame((0.,) * 4), (0.,) * 4)
        for invalid in (math.nan, math.inf, -math.inf):
            with self.assertRaises(ValueError):
                adapter.inverse_glf(invalid)
            with self.assertRaises(ValueError):
                adapter.convert_raw_frame((invalid, 0., 0., 0.))
        with self.assertRaises(ValueError):
            adapter.scale_forces((1., 2., 3.))


class GuardTests(unittest.TestCase):
    def setUp(self):
        self.guard = adapter.OutputGuard()
        self.inputs(10.)

    def inputs(self, now, command=(.2, 0., 0., 0., 0., 0.), status="navigating"):
        self.guard.set_command(command, now)
        self.guard.set_status(status, now)
        for name in ("odometry", "scan", "gps", "imu"):
            self.guard.observe(name, now)

    def frame(self, now):
        for name, value in zip(adapter.THRUSTERS, (.1, -.1, .1, -.1)):
            self.guard.set_raw(name, value, now)

    def enable(self, now=10.):
        token = self.guard.begin_reset(now)
        self.assertIsNotNone(token)
        self.assertFalse(self.guard.tick(now))
        self.assertTrue(self.guard.finish_reset(token, True, now))
        self.frame(now + .01)
        self.assertTrue(self.guard.tick(now + .01))

    def test_start_and_reenable_require_ack_and_new_complete_frame(self):
        self.frame(10.)
        self.assertFalse(self.guard.tick(10.))
        token = self.guard.begin_reset(10.)
        self.assertFalse(self.guard.tick(10.))
        self.assertTrue(self.guard.finish_reset(token, True, 10.))
        self.assertFalse(self.guard.tick(10.))
        for name in adapter.THRUSTERS[:-1]:
            self.guard.set_raw(name, .1, 10.01)
        self.assertFalse(self.guard.tick(10.01))
        self.guard.set_raw(adapter.THRUSTERS[-1], .1, 10.01)
        self.assertTrue(self.guard.tick(10.01))
        self.guard.set_status("goal_reached", 10.02)
        self.assertFalse(self.guard.tick(10.02))
        self.inputs(10.03)
        self.assertFalse(self.guard.tick(10.03))
        self.enable(10.03)

    def test_all_input_timeouts_and_invalid_raw_close_gate(self):
        for stale_name in adapter.OutputGuard.DEFAULT_TIMEOUTS:
            self.setUp()
            self.enable()
            self.inputs(10.6)
            self.frame(10.6)
            if stale_name == "raw":
                self.guard.frame_stamps = (10.,) * 4
            else:
                self.guard.samples[stale_name] = 10.6 - self.guard.timeouts[stale_name] - .01
            self.assertFalse(self.guard.tick(10.6), stale_name)
            self.assertTrue(self.guard.reset_required, stale_name)
        self.setUp()
        self.enable()
        self.guard.set_raw("left_front", math.nan, 10.1)
        self.assertFalse(self.guard.tick(10.1))

    def test_active_zero_target_preserves_feedback_braking_without_new_reset(self):
        self.enable()
        reset_ack = self.guard.reset_ack
        self.inputs(10.1, command=(0.,) * 6)
        for name in adapter.THRUSTERS:
            self.guard.set_raw(name, -27.77 / 800., 10.1)
        self.assertTrue(self.guard.tick(10.1))
        self.assertEqual(self.guard.reset_ack, reset_ack)
        self.assertFalse(self.guard.reset_required)
        commands = adapter.convert_raw_frame(self.guard.frame)
        self.assertTrue(all(adapter.glf_force(command) < 0. for command in commands))
        self.inputs(10.2, command=(0.,) * 6, status="aligning")
        self.assertTrue(self.guard.tick(10.2))
        self.assertEqual(self.guard.reset_ack, reset_ack)

    def test_zero_target_can_enable_only_after_reset_ack_and_new_frame(self):
        for status in ("navigating", "aligning"):
            self.setUp()
            self.inputs(10., command=(0.,) * 6, status=status)
            self.assertFalse(self.guard.tick(10.))
            self.enable()

    def test_inactive_states_always_stop_even_with_a_braking_frame(self):
        for status in ("goal_reached", "blocked", "invalid_goal", "planning", "waiting_for_inputs", "unexpected"):
            self.setUp()
            self.enable()
            self.inputs(10.1, command=(0.,) * 6, status=status)
            for name in adapter.THRUSTERS:
                self.guard.set_raw(name, -.1, 10.1)
            self.assertFalse(self.guard.tick(10.1), status)
            self.assertTrue(self.guard.reset_required, status)

    def test_stale_or_invalid_zero_target_closes_active_gate(self):
        self.enable()
        self.inputs(10.1, command=(0.,) * 6)
        self.frame(10.1)
        self.guard.samples["command"] = 9.
        self.assertFalse(self.guard.tick(10.1))
        self.setUp()
        self.inputs(10., command=(0.,) * 6)
        self.enable()
        self.guard.set_command((0., 0., math.nan, 0., 0., 0.), 10.1)
        self.assertFalse(self.guard.tick(10.1))

    def test_rotation_only_alignment_remains_enabled(self):
        self.inputs(10.2, command=(0., 0., 0., 0., 0., .1), status="aligning")
        self.enable(10.2)

    def test_failed_or_superseded_reset_never_opens_gate(self):
        token = self.guard.begin_reset(10.)
        self.assertFalse(self.guard.finish_reset(token, False, 10.))
        token = self.guard.begin_reset(10.01)
        self.guard.set_status("blocked", 10.02)
        self.assertFalse(self.guard.finish_reset(token, True, 10.03))
        self.inputs(10.04)
        self.frame(10.05)
        self.assertFalse(self.guard.tick(10.05))

    def test_frame_synchronization_and_clock_reset(self):
        self.enable()
        self.guard.pending_raw.clear()
        for index, name in enumerate(adapter.THRUSTERS):
            self.guard.set_raw(name, .2, 10.1 + index * .1)
        self.assertEqual(self.guard.frame, (.1, -.1, .1, -.1))
        self.assertFalse(self.guard.tick(1.))
        self.assertTrue(self.guard.reset_required)
        self.assertEqual(self.guard.samples, {})


class SensorTimestampTests(unittest.TestCase):
    def setUp(self):
        self.node = adapter.ThrusterOutputAdapter.__new__(adapter.ThrusterOutputAdapter)
        self.node.future_tolerance = .05
        self.node.rospy = types.SimpleNamespace(
            Time=types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: 10.)))

    def stamp(self, value):
        message = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: value)))
        return self.node.sensor_stamp(message, self.node.now())

    def test_separate_clock_connection_allows_only_small_future_skew(self):
        self.assertEqual(self.stamp(9.9), 9.9)
        self.assertEqual(self.stamp(10.), 10.)
        self.assertEqual(self.stamp(10.001), 10.)
        self.assertEqual(self.stamp(10.05), 10.)
        self.assertTrue(math.isnan(self.stamp(10.050001)))
        self.assertTrue(math.isnan(self.stamp(100.)))

    def test_invalid_and_stale_timestamps_are_not_replaced_by_current_time(self):
        for value in (0., -1., math.nan, math.inf, -math.inf):
            self.assertTrue(math.isnan(self.stamp(value)))
        stale_stamp = self.stamp(1.)
        self.assertEqual(stale_stamp, 1.)
        self.assertFalse(adapter.OutputGuard.fresh(stale_stamp, 10., .5))
        self.node.future_tolerance = 0.
        self.assertTrue(math.isnan(self.stamp(10.001)))


if __name__ == "__main__":
    unittest.main()

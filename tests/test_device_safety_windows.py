"""独立设备前后包络、提前定位、延后回位及页面保存回归。"""
from dataclasses import replace
from pathlib import Path
import queue
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from tests._offline_imports import install_noop_logger_module
install_noop_logger_module()

from model.utils.MachineSafetyWindow import (
    WINDOW_KEYS, machine_window_bounds, machine_window_distances,
    with_machine_window_defaults,
)
from model.utils.TomlLoader import TomlLoader
from model.motionplan.MotionFrameByFramePlanning import (
    MotionFrameByFramePlanning, PlannedXAxisRange, RegionBox, ThreeRegionBoxes,
    RuntimeQueueGeometry, XN_SIDE_AXIS_SEQUENCE, XN_SIDE_X_AXES, XN_SIDE_Y_AXES,
    Variable2DWindowObservation, build_variable_2d_window_observation,
)
from model.plc.MovingFrameData import AxisData
from control import MachineConfigFrameControl as ui

ROOT = Path(__file__).resolve().parents[1]

def frame(*rows):
    return SimpleNamespace(FrameData=[SimpleNamespace(H_Axis=y, V_Axis_Min=lo, V_Axis_Max=hi)
                                      for y, lo, hi in rows])

class DeviceWindowTests(unittest.TestCase):
    def setUp(self):
        self.config = TomlLoader.load(str(ROOT / 'model/tomls/MotionPlanningConfig.toml'))
        self.config['frame_by_frame']['x_arrival_confirmations'] = 2
        self.config['frame_by_frame']['short_stroke_threshold_mm'] = 0
        self.config['frame_by_frame']['x_position_tolerance_mm'] = 10
        self.planner = MotionFrameByFramePlanning(self.config)
        self.machine = dict(
            type='xn_side', install_orietation='left', z_position=2000, x_position=0,
            yunque_safety_window_front_mm=30, yunque_safety_window_back_mm=10,
            axis_type=list(XN_SIDE_AXIS_SEQUENCE), min_limit_pos=[0, 0], max_limit_pos=[1500, 1000],
            max_limit_speed=[400, 500], safe_pos=[0, 0], origin_pos=[1000, 1400, 1800, 2400],
            x_pos_speed=100, y_pos_speed=100,
        )
        self.actual = dict.fromkeys(XN_SIDE_AXIS_SEQUENCE, 0)
        self.geometry = RuntimeQueueGeometry(
            object(), {a: PlannedXAxisRange(a, 500, 900, 200, 600, 0, 1000, 100, 200)
                       for a in XN_SIDE_X_AXES},
            y_targets=dict.fromkeys(XN_SIDE_Y_AXES, 300), targets_prepared=True,
        )
        self.empty, self.valid = frame(), frame((1500, 500, 900))
        self.manager = SimpleNamespace(stack_size=400, frame_revision=0,
                                      frame_stack={s: [self.empty]*400 for s in ('left', 'right')})
        self.two_d = dict(type='out_lift', install_orietation='left', z_position=2000,
                          x_position=0, out_front_x_offset=100, origin_pos=[0],
                          min_limit_pos=[0, 0], max_limit_pos=[5000, 1600], safe_pos=[0, 0],
                          variable_2d_safety_window_back_mm=30, variable_2d_safety_window_front_mm=10)

    def advance(self, phase='at_station', geometry=None):
        return self.planner._advance_side(replace(geometry or self.geometry, arrival_phase=phase),
                                          self.machine, 'left', self.actual, 10)[0]

    def put(self, index, value=None):
        self.manager.frame_stack['left'][index] = self.valid if value is None else value
        self.manager.frame_revision += 1

    def test_defaults_and_asymmetric_bounds(self):
        for kind, expected in [('xn_side', 200), ('out_lift', 300)]:
            self.assertEqual(machine_window_distances({'type': kind}), (expected, expected))
        self.assertEqual(machine_window_bounds(self.machine, 400), (197, 202, 200))
        for key in WINDOW_KEYS['xn_side']:
            for invalid in [-10, 1, 15, True, 1.5, '20']:
                with self.subTest(key=key, invalid=invalid), self.assertRaises(ValueError):
                    machine_window_distances(dict(self.machine, **{key: invalid}))
        for values in [dict(yunque_safety_window_front_mm=2010), dict(yunque_safety_window_back_mm=2000)]:
            with self.assertRaisesRegex(ValueError, '超出'):
                machine_window_bounds(dict(self.machine, **values), 400)
        self.assertEqual(machine_window_bounds(dict(self.machine, yunque_safety_window_front_mm=0,
                                                    yunque_safety_window_back_mm=0), 400), (200, 201, 200))

    def test_new_fields_are_only_in_matching_device_sections(self):
        machines = TomlLoader.load(str(ROOT / 'model/tomls/MachineConfig1.toml'))
        all_keys = set(sum(WINDOW_KEYS.values(), ()))
        for machine in machines.values():
            self.assertEqual(set(machine) & all_keys, set(WINDOW_KEYS[machine['type']]))
            machine_window_bounds(machine, 1000)
        self.assertNotIn('yunque_safety_window_mm', self.config['frame_by_frame'])
        self.assertNotIn('variable_2d_safety_window_mm', self.config['frame_by_frame'])

    def test_preposition_never_sprays_or_reciprocates_before_station(self):
        first = self.advance('preposition')
        self.assertEqual(first['y1'].Pos, 300)
        self.assertEqual(first['x1'].Pos, 0)
        self.actual.update(dict.fromkeys(XN_SIDE_Y_AXES, 300))
        self.advance('preposition')
        self.assertEqual(self.advance('preposition')['x1'].Pos, 200)
        self.actual.update(dict.fromkeys(XN_SIDE_X_AXES, 200))
        for _ in range(5):
            command = self.advance('preposition')
            self.assertTrue(all(c.Status == 0 for c in command.values()))
            self.assertEqual(command['x1'].Pos, 200)
            self.assertFalse(self.planner.motion_started['left'])
        self.assertEqual(self.advance()['x1'].Status, 1)
        self.assertEqual(self.advance()['x1'].Pos, 600)

    def test_leave_station_latches_actual_position_and_next_piece_repositions(self):
        self.actual.update(dict.fromkeys(XN_SIDE_Y_AXES, 300))
        self.actual.update(dict.fromkeys(XN_SIDE_X_AXES, 200))
        for _ in range(5): self.advance()
        self.assertTrue(self.planner.motion_started['left'])
        self.actual['x1'] = 345
        waiting = self.advance('waiting')
        self.assertEqual(waiting['x1'], AxisData(Pos=345, Speed=0, Status=0))
        self.actual['x1'] = 350
        self.assertEqual(self.advance('waiting')['x1'].Pos, 345)
        self.assertFalse(self.planner.motion_started['left'])
        command = self.advance('preposition')
        self.assertEqual(command['x1'].Pos, 350)
        self.assertTrue(all(c.Status == 0 for c in command.values()))

    def test_leave_before_positioning_complete_cancels_targets(self):
        self.advance('preposition')
        self.actual['y1'] = 70
        command = self.advance('waiting')
        self.assertEqual(command['y1'], AxisData(Pos=70, Speed=0, Status=0))

    def test_window_edit_restarts_position_confirmation(self):
        self.actual.update(dict.fromkeys(XN_SIDE_Y_AXES, 300))
        self.actual.update(dict.fromkeys(XN_SIDE_X_AXES, 200))
        for _ in range(5): self.advance()
        self.machine['yunque_safety_window_front_mm'] = 40
        self.assertEqual(self.advance()['x1'].Status, 0)
        self.assertFalse(self.planner.motion_started['left'])

    def test_both_clearance_conditions_and_boundaries(self):
        devices = {('xn_side', 'left'): self.machine}
        def clear(): return self.planner._behind_empty_by_machine(self.manager, devices)[('xn_side', 'left')]
        self.assertTrue(clear())
        for index in (100, 199, 200, 201):
            self.put(index)
            self.assertFalse(clear(), index)
            self.put(index, self.empty)
        for index in (99, 202):
            self.put(index)
            self.assertTrue(clear(), index)
            self.put(index, self.empty)
        self.put(202)
        self.assertTrue(clear())
        self.machine['yunque_safety_window_back_mm'] = 20
        self.assertFalse(clear())  # 同一队列修订号，参数变化也刷新缓存

    def test_front_larger_than_idle_distance_does_not_cancel_preposition(self):
        self.machine['yunque_safety_window_front_mm'] = 1100
        self.put(90)
        result = self.planner._behind_empty_by_machine(self.manager, {('xn_side', 'left'): self.machine})
        self.assertFalse(result[('xn_side', 'left')])

    def test_2d_early_valid_tail_and_empty_hold_then_return(self):
        self.put(197)
        observation = self.planner._build_variable_2d_observation(self.manager, 'left', 200, self.two_d)
        self.assertEqual(observation.state, 'valid')
        initial, _ = self.planner._advance_out_lift('left', self.two_d, observation, False)
        self.assertEqual((initial.Pos, initial.Speed, initial.Status), (400, 1500, 5000))
        for at in (201, 202):
            self.put(197, self.empty)
            self.put(at)
            observation = self.planner._build_variable_2d_observation(self.manager, 'left', 200, self.two_d)
            held, _ = self.planner._advance_out_lift('left', self.two_d, observation, False)
            self.assertEqual(held, initial)
            self.put(at, self.empty)
        returned, _ = self.planner._advance_out_lift('left', self.two_d, observation, True)
        self.assertEqual(returned, AxisData())

    def test_2d_rejects_internal_hole_and_uses_explicit_station_index(self):
        observation = build_variable_2d_window_observation(
            [self.valid, self.empty, self.valid, self.empty, self.empty], self.two_d, station_index=3)
        self.assertEqual(observation.state, 'mixed')
        observation = build_variable_2d_window_observation(
            [self.empty, self.empty, self.empty, self.valid, self.empty], self.two_d, station_index=3)
        self.assertEqual(observation.state, 'valid')

    def test_geometry_window_reference_direction_and_internal_gap(self):
        boxes = ThreeRegionBoxes(RegionBox(100, 300, 1300, 1600),
                                 RegionBox(150, 170, 1600, 1900), RegionBox(100, 300, 1900, 2900))
        self.put(197)
        module = 'model.motionplan.MotionFrameByFramePlanning.'
        with patch(module + '_detect_frame_edge_sequence', return_value=[None]*5), \
             patch(module + '_build_region_box_sequence', return_value=[None]*4+[boxes]), \
             patch(module + '_build_x_axis_ranges', return_value=self.geometry.motions):
            geometry = self.planner._build_side_geometry(self.manager, 'left', 200, 10, self.machine)
            self.assertEqual((geometry.station_index, geometry.reference_index), (1, 4))
            self.assertEqual(geometry.arrival_phase, 'preposition')
            self.assertIsNotNone(geometry.motions)
            self.put(199)  # 与197之间缺帧，不跨工件生成目标
            invalid = self.planner._build_side_geometry(self.manager, 'left', 200, 10, self.machine)
            self.assertIsNone(invalid.boxes)
            self.assertIsNone(invalid.motions)
        self.put(197, self.empty)
        self.put(199, self.empty)
        self.put(196)  # 前窗口外
        geometry = self.planner._build_side_geometry(self.manager, 'left', 200, 10, self.machine)
        self.assertEqual(geometry.arrival_phase, 'waiting')

    def test_bottom_y_uses_selected_window_without_mutating_source_boxes(self):
        module = 'model.motionplan.MotionFrameByFramePlanning.'
        self.planner.minimum_gun_y_distance_mm = 300
        base = ThreeRegionBoxes(RegionBox(100, 300, 2100, 2300),
                                None, RegionBox(100, 300, 1500, 1700))
        cases = [
            ('asymmetric', {201: 1400, 200: 1500, 199: 1450, 198: 1480, 197: 1490,
                            202: 1100, 196: 1100}, 30, 10, 1400, 'at_station'),
            ('equal', dict.fromkeys(range(197, 202), 1500), 30, 10, 1500, 'at_station'),
            ('single', {201: 1100, 200: 1500, 199: 1100}, 0, 0, 1500, 'at_station'),
            ('preposition', {199: 1500, 198: 1400, 197: 1450}, 30, 10, 1400, 'preposition'),
            ('tail', {201: 1400, 200: 1500}, 30, 10, 1400, 'at_station'),
            ('gap', {201: 1400, 200: 1500, 198: 1300}, 30, 10, None, 'at_station'),
        ]
        for side in ('left', 'right'):
            for name, heights, front, back, expected, phase in cases:
                with self.subTest(side=side, case=name):
                    stack = [self.empty] * 400
                    for index, height in heights.items():
                        stack[index] = frame((height, 100, 300))
                    self.manager.frame_stack[side] = stack
                    machine = dict(self.machine, install_orietation=side,
                                   yunque_safety_window_front_mm=front,
                                   yunque_safety_window_back_mm=back,
                                   out_down_y_offset=50, in_down_y_offset=0, in_up_y_offset=0)
                    source_boxes = []

                    def build_boxes(rows, *args, **kwargs):
                        source_boxes[:] = [None if len(row) == 0 else replace(
                            base, lower=replace(base.lower, y_min=float(row.height[0])))
                            for row in rows]
                        return source_boxes

                    with patch(module + '_detect_frame_edge_sequence', return_value=[]), \
                         patch(module + '_build_region_box_sequence', side_effect=build_boxes), \
                         patch(module + '_build_x_axis_ranges', return_value=self.geometry.motions):
                        geometry = self.planner._build_side_geometry(
                            self.manager, side, 200, 10, machine)
                    self.assertEqual(geometry.arrival_phase, phase)
                    if expected is None:
                        self.assertIsNone(geometry.boxes)
                        self.assertIsNone(geometry.motions)
                        continue
                    original = source_boxes[geometry.reference_index]
                    self.assertEqual(geometry.boxes, replace(
                        original, lower=replace(original.lower, y_min=expected)))
                    self.assertEqual(original.lower.y_min, 1500)
                    self.assertIsNot(geometry.boxes, original)
                    self.assertEqual(geometry.motions, {
                        axis: replace(motion, window_min_detected_x=500)
                        for axis, motion in self.geometry.motions.items()})
                    targets = self.planner._calculate_side_y_targets(geometry.boxes, side, machine)
                    # 枪2绝对高度1700，枪间距300，枪1绝对高度最高1400。
                    self.assertEqual(targets, {
                        'y1': min(expected - 50 - 1000, 400), 'y2': 300, 'y3': 300})

    def test_window_cache_is_per_device_and_updates_without_queue_change(self):
        machines = {s: dict(self.machine, install_orietation=s) for s in ('left', 'right')}
        with patch.object(self.planner, '_build_side_geometry', return_value=self.geometry) as build:
            self.planner._geometry_by_side(self.manager, 0, machines, 10)
            self.planner._geometry_by_side(self.manager, 0, machines, 10)
            self.assertEqual(build.call_count, 2)
            machines['left']['yunque_safety_window_back_mm'] = 20
            self.planner._geometry_by_side(self.manager, 0, machines, 10)
            self.assertEqual(build.call_count, 4)
            self.assertEqual(machine_window_bounds(machines['right'], 400), (197, 202, 200))

    def test_invalid_runtime_window_emits_safe_frame(self):
        machines = {str(i): dict(self.machine, install_orietation=s) for i,s in enumerate(('left', 'right'))}
        machines['0']['yunque_safety_window_front_mm'] = 2010
        output = self.planner.build_moving_frame(strategy_name='frame_by_frame', frame_queue_manager=self.manager,
            plc_data=SimpleNamespace(Operate=6, HeartBeat=7, ChainCountCM=0, AxisList=[AxisData()]*16),
            machine_config=machines, read_data_config={'y_threshold': 10}, lidar_status=0, raw_data_timeout_active=False)
        self.assertTrue(self.planner.last_frame_was_safe)
        self.assertEqual(output.Enable, 0)
        self.assertEqual(len(output.to_bytes()), 110)

    def test_page_save_reload_defaults_notification_and_reject_before_write(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'MachineConfig1.toml'
            path.write_text('[0]\ntype="xn_side"\nz_position=2000\n', encoding='utf-8')
            notifications = queue.Queue()
            with patch.object(ui, '_get_toml_path', return_value=str(path)), \
                 patch.object(ui, '_get_param_range_rules', return_value={}):
                loaded = ui.load_machine_config_for_ui(0)
                self.assertEqual(machine_window_distances(loaded), (200, 200))
                ui.save_machine_config_from_ui(0, {'yunque_safety_window_front_mm': 40,
                    'yunque_safety_window_back_mm': 80}, notifications)
                self.assertEqual(machine_window_distances(ui.load_machine_config_for_ui(0)), (80, 40))
                self.assertEqual(notifications.get()['machine']['yunque_safety_window_back_mm'], 80)
                original = path.read_bytes()
                for value in (-10, 15, 2010):
                    with self.assertRaises(ValueError):
                        ui.save_machine_config_from_ui(0, {'yunque_safety_window_front_mm': value}, notifications)
                    self.assertEqual(path.read_bytes(), original)
                self.assertTrue(notifications.empty())

    def test_complete_wire_cycle_preposition_spray_tail_hold_and_return(self):
        machines = {str(i): dict(self.machine, install_orietation=side)
                    for i, side in enumerate(('left', 'right'))}
        plc = SimpleNamespace(Operate=6, HeartBeat=9, ChainCountCM=0,
                              AxisList=[AxisData() for _ in range(16)])
        module = 'model.motionplan.MotionFrameByFramePlanning.'
        boxes = ThreeRegionBoxes(RegionBox(100, 300, 1300, 1600),
                                 RegionBox(150, 170, 1600, 1900), RegionBox(100, 300, 1900, 2900))
        def prepare(g, *_):
            return replace(g, targets_prepared=True, y_targets=self.geometry.y_targets)
        def build():
            result = self.planner.build_moving_frame(
                strategy_name='frame_by_frame', frame_queue_manager=self.manager,
                plc_data=plc, machine_config=machines, read_data_config={'y_threshold': 10},
                lidar_status=0, raw_data_timeout_active=False)
            self.assertFalse(self.planner.last_frame_was_safe, self.planner.last_error)
            self.assertEqual(len(result.to_bytes()), 110)
            for i, command in enumerate(result.AxisList):
                plc.AxisList[i] = AxisData(Pos=command.Pos)
            return result
        with patch(module + '_detect_frame_edge_sequence', side_effect=lambda rows, **kw: [None]*len(rows)), \
             patch(module + '_build_region_box_sequence', side_effect=lambda rows, *a, **kw:
                   [boxes if len(row) else None for row in rows]), \
             patch(module + '_build_x_axis_ranges', return_value=self.geometry.motions), \
             patch.object(self.planner, '_prepare_side_geometry', side_effect=prepare):
            self.put(197)
            for _ in range(7):
                result = build()
                self.assertTrue(all(command.Status == 0 for command in result.AxisList))
            self.assertEqual(result.AxisList[1].Pos, 200)
            self.put(197, self.empty)
            self.put(200)
            result = build()
            self.assertEqual((result.AxisList[1].Pos, result.AxisList[1].Status), (600, 1))
            self.put(200, self.empty)
            self.put(201)
            result = build()
            self.assertEqual((result.AxisList[1].Pos, result.AxisList[1].Status), (600, 0))
            self.assertFalse(self.planner._return_triggered['left'])
            self.put(201, self.empty)
            self.put(150)  # 后窗口已空，但下一件在来料1000mm内
            self.assertEqual(build().AxisList[1].Pos, 600)
            self.assertFalse(self.planner._return_triggered['left'])
            self.put(150, self.empty)
            result = build()
            self.assertTrue(self.planner._return_triggered['left'])
            self.assertEqual((result.AxisList[1].Pos, result.AxisList[0].Pos), (0, 300))
            for _ in range(6): result = build()
            self.assertTrue(self.planner._return_complete['left'])
            self.assertFalse(result.Enable & 2)

    def test_zero_distances_require_actual_station_and_allow_immediate_return(self):
        self.machine.update(yunque_safety_window_front_mm=0, yunque_safety_window_back_mm=0)
        self.put(199)
        g = self.planner._build_side_geometry(self.manager, 'left', 200, 10, self.machine)
        self.assertEqual(g.arrival_phase, 'waiting')
        self.put(199, self.empty)
        self.put(201)
        self.assertTrue(self.planner._behind_empty_by_machine(
            self.manager, {('xn_side', 'left'): self.machine})[('xn_side', 'left')])

    def test_invalid_early_geometry_holds_all_axes_without_powder(self):
        for bad in (replace(self.geometry, motions=None),
                    replace(self.geometry, y_targets=None),
                    replace(self.geometry, boxes=None)):
            self.planner.reset()
            commands = self.advance('preposition', bad)
            self.assertTrue(all(c.Speed == 0 and c.Status == 0 for c in commands.values()))

    def test_ui_fields_are_device_specific_and_only_for_frame_strategy(self):
        # wx未安装时只加载对话框的参数选择方法，不启动窗口。
        import importlib.util
        import sys
        spec = importlib.util.spec_from_file_location('device_window_ui_test', ROOT/'view/MachineConfigFrame.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {'wx': SimpleNamespace(Dialog=object)}):
            spec.loader.exec_module(module)
        dialog = module.MachineConfigFrame.__new__(module.MachineConfigFrame)
        all_keys = set(sum(WINDOW_KEYS.values(), ()))
        for strategy in ('frame_by_frame', 'complete_workpiece', 'continuous_bidirectional'):
            dialog.strategy_name = strategy
            for sn, kind in ((0, 'xn_side'), (1, 'xn_side'), (2, 'out_lift'), (3, 'out_lift')):
                keys = dict(dialog._get_param_order_for_config(sn, {'type': kind}))
                expected = set(WINDOW_KEYS[kind]) if strategy == 'frame_by_frame' else set()
                self.assertEqual(set(keys) & all_keys, expected)

    def test_replay_keeps_prearrival_and_postdeparture_events(self):
        from tools.frame_queue_motion_replay import simulate_frame_queue_motion
        machines = {}
        for sn, side, kind in ((0, 'left', 'xn_side'), (1, 'right', 'xn_side'),
                               (2, 'left', 'out_lift'), (3, 'right', 'out_lift')):
            machines[str(sn)] = dict(self.machine if kind == 'xn_side' else self.two_d,
                                     install_orietation=side, z_position=100)
        result = simulate_frame_queue_motion(
            frames_by_side={side: [self.empty] for side in ('left', 'right')},
            machine_configs=machines, planning_config=self.config,
            read_config={'frame_by_frame_queue_size': 100, 'max_pulse': 100000,
                         'pulse_to_mm': 1, 'combined_y_min': 0, 'combined_y_max': 3000},
            system_config_path=str(ROOT/'model/tomls/SystemConfig.toml'), y_threshold=10)
        self.assertEqual(result.events[0].tick, 7)
        self.assertIsNone(result.events[0].source_frame_number)
        self.assertGreater(result.events[-1].tick, 12)
        self.assertEqual(result.safe_count, 0)


    def test_plc_runtime_update_preserves_device_window_fields(self):
        from control.PlcCommunicationProcess import PlcCommunicationProcess
        process = PlcCommunicationProcess(queue.Queue(), queue.Queue(), queue.Queue())
        process.machine_config = {'0': dict(self.machine), '2': dict(self.two_d)}
        for sn, kind in ((0, 'xn_side'), (2, 'out_lift')):
            front, back = WINDOW_KEYS[kind]
            process._handle_config_update(sn, {front: 40, back: 80})
            self.assertEqual(process.runtime_machine_config[sn][front], 40)
            self.assertEqual(process.runtime_machine_config[sn][back], 80)
            self.assertEqual(machine_window_distances(process.machine_config[str(sn)]),
                             (80, 40) if kind == "xn_side" else (40, 80))


    def test_viewer_strip_uses_station_not_window_midpoint(self):
        from matplotlib.figure import Figure
        from tools.frame_contour_viewer import ContourProcessingRenderer
        for front, back in ((10, 20), (20, 10), (0, 20), (10, 0), (0, 0)):
            with self.subTest(front=front, back=back):
                axis = Figure().subplots()
                count = front + back + 1
                diagnostic = SimpleNamespace(window_nonempty=(True,)*count,
                    window_modes=('direct',)*count, window_station_index=back,
                    window_obstacle_min_x={})
                ContourProcessingRenderer._draw_window_strip(axis, diagnostic)
                self.assertEqual(list(axis.lines[0].get_xdata()), [back, back])
                self.assertEqual(list(axis.get_xticks()), sorted({0, back, count-1}))
                labels = [label.get_text() for label in axis.get_xticklabels()]
                self.assertIn('当前', labels)
                if back: self.assertIn(f'出料 {back}帧 ({back*10}mm)', labels)
                if front: self.assertIn(f'来料 {front}帧 ({front*10}mm)', labels)


    def test_device_specific_direction_mapping_and_viewer(self):
        from tools.frame_contour_viewer import build_side_contour_diagnostic
        for original in (self.machine, self.two_d):
            with self.subTest(kind=original['type']):
                machine = dict(original)
                front_key, back_key = WINDOW_KEYS[machine['type']]
                machine.update({front_key: 500, back_key: 100})
                outgoing, incoming = (100, 500) if machine["type"] == "xn_side" else (500, 100)
                self.assertEqual(machine_window_distances(machine), (outgoing, incoming))
                self.assertEqual(machine_window_bounds(machine, 400),
                                 (200-incoming//10, 201+outgoing//10, 200))
                diagnostic = build_side_contour_diagnostic(
                    side='left', device_type=machine['type'], frames=[],
                    event=SimpleNamespace(device_source_frame_numbers={}, tick=200,
                                          debug_snapshot=None, reasons=()),
                    planning_parameters=SimpleNamespace(geometry=self.planner.geometry_parameters,
                                                        y_threshold=10),
                    machine_configs={'0': machine})
                self.assertEqual(diagnostic.window_station_index, outgoing//10)
                self.assertEqual(len(diagnostic.window_modes), 61)


    def test_partial_defaults_do_not_copy_value_from_opposite_field(self):
        for kind, default in (('xn_side', 200), ('out_lift', 300)):
            front, back = WINDOW_KEYS[kind]
            for present, missing in ((front, back), (back, front)):
                source = {'type': kind, present: 500}
                filled = with_machine_window_defaults(source)
                self.assertEqual(filled[present], 500)
                self.assertEqual(filled[missing], default)
                self.assertNotIn(missing, source)


    def test_spray_window_range_defaults_and_load_failure(self):
        keys = sum(WINDOW_KEYS.values(), ())
        for config in ({}, {keys[0] + '_max': 600}):
            with patch.object(ui.TomlLoader, 'load', return_value=config):
                rules = ui._get_param_range_rules()
            for key in keys:
                self.assertEqual(rules[key], (0, config.get(key + '_max', 1000)))
        with patch.object(ui.TomlLoader, 'load', side_effect=OSError('unavailable')):
            rules = ui._get_param_range_rules()
        for key in keys:
            self.assertEqual(rules[key], (0, 1000))

    def test_spray_ranges_validate_saves_for_all_four_devices(self):
        import contextlib
        import io
        with tempfile.TemporaryDirectory() as directory:
            config_dir = Path(directory)/'model'/'tomls'
            config_dir.mkdir(parents=True)
            machine_path = config_dir/'MachineConfig1.toml'
            machine_path.write_text((ROOT/'model/tomls/MachineConfig1.toml').read_text('utf-8'), encoding='utf-8')
            (config_dir/'ReadDataConfig.toml').write_text('frame_by_frame_queue_size = 1000\n', encoding='utf-8')
            spray_path = config_dir/'SprayConfig.toml'
            spray_path.write_text((ROOT/'model/tomls/SprayConfig.toml').read_text('utf-8'), encoding='utf-8')
            notifications = queue.Queue()
            with patch.object(ui.os, 'getcwd', return_value=directory), contextlib.redirect_stdout(io.StringIO()):
                for sn, kind in ((0, 'xn_side'), (1, 'xn_side'), (2, 'out_lift'), (3, 'out_lift')):
                    for key in WINDOW_KEYS[kind]:
                        for value in (0, 100, 500, 1000):
                            ui.save_machine_config_from_ui(sn, {key: value}, notifications)
                            self.assertEqual(ui.load_machine_config_for_ui(sn)[key], value)
                            self.assertEqual(notifications.get_nowait()['machine'][key], value)
                        for value in (-10, 1010, 15):
                            before = machine_path.read_bytes()
                            with self.assertRaises(ValueError) as error:
                                ui.save_machine_config_from_ui(sn, {key: value}, notifications)
                            self.assertIn(key, str(error.exception))
                            if value != 15:
                                self.assertIn('0 ~ 1000', str(error.exception))
                            self.assertEqual(machine_path.read_bytes(), before)
                            self.assertTrue(notifications.empty())
                # 单项范围修改后下次保存立即使用新范围，其他项仍为0~1000。
                for kind in WINDOW_KEYS:
                    sn = 0 if kind == 'xn_side' else 2
                    for key in WINDOW_KEYS[kind]:
                        spray_path.write_text(f'{key}_min = 100\n{key}_max = 600\n', encoding='utf-8')
                        rules = ui._get_param_range_rules(sn)
                        self.assertEqual(rules[key], (100, 600))
                        for other in sum(WINDOW_KEYS.values(), ()):
                            if other != key: self.assertEqual(rules[other], (0, 1000))
                        for value in (100, 600):
                            ui.save_machine_config_from_ui(sn, {key: value}, notifications)
                            notifications.get_nowait()
                        for value in (90, 610):
                            before = machine_path.read_bytes()
                            with self.assertRaisesRegex(ValueError, '100 ~ 600'):
                                ui.save_machine_config_from_ui(sn, {key: value}, notifications)
                            self.assertEqual(machine_path.read_bytes(), before)
                            self.assertTrue(notifications.empty())
                # 范围允许但窗口越过队列边界时，仍禁止保存。
                spray_path.write_text('', encoding='utf-8')
                configs = TomlLoader.load(str(machine_path))
                configs['0']['z_position'] = 100
                TomlLoader.save(configs, str(machine_path))
                before = machine_path.read_bytes()
                with self.assertRaisesRegex(ValueError, '超出帧队列范围'):
                    ui.save_machine_config_from_ui(0, {'yunque_safety_window_front_mm': 500}, notifications)
                self.assertEqual(machine_path.read_bytes(), before)
                self.assertTrue(notifications.empty())


if __name__ == '__main__':
    unittest.main()

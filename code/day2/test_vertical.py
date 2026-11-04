"""Mathematics, timestamped recordings, notebook decisions and motor-free capture."""

import ast
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import types
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
from scipy.stats import chi2

from . import vertical as v
from . import vertical_live as live
from .notebook_tools import (_run_capture, capture_vertical, simulated_imu_pair,
                             read_imu, plot_imu_calibration)
from race_common.motor_stop import LOCKED_BIT


def completed_notebook_filter():
    """Fill the four exercise expressions and execute the notebook's own scaffolding."""
    notebook = json.loads(Path(__file__).with_name('03_height_velocity.ipynb').read_text())
    cells = {cell['id']: ''.join(cell['source']) for cell in notebook['cells']}
    namespace = {'np': np, 'vertical': v}
    mechanics = cells['day2-mechanics']
    # B maps a one-element acceleration vector to a two-element state. H, R,
    # K and the residual use matrix shapes in the measurement update.
    for name, expression in {
        'x_predicted': 'F @ x + B @ acceleration',
        'C_predicted': 'F @ C @ F.T + Q',
        'K': 'C_predicted @ H.T @ np.linalg.inv(R)',
        'x_filtered': 'x_column + K @ residual',
    }.items():
        marker = f'{name} = None  #'
        if mechanics.count(marker) != 1:
            raise AssertionError(f'Expected one exercise expression for {name}')
        mechanics = mechanics.replace(marker, f'{name} = {expression}  #')
    exec(mechanics, namespace)
    return cells, namespace


class VerticalTests(unittest.TestCase):
    def test_visible_notebook_model_gate_and_rest_update_drive_replay(self):
        cells, namespace = completed_notebook_filter()
        exec(cells['day2-gate-policy'], namespace)
        namespace['settings'] = v.Settings()
        namespace['replace'] = __import__('dataclasses').replace
        exec(cells['day2-rest-settings'], namespace)
        gate = namespace['our_accept_range']
        for residual, variance, threshold, accepted in (
                (.25, .0625, 1., True), (.26, .0625, 1., False), (10., .01, None, True)):
            self.assertEqual(gate(residual, variance, threshold), accepted)
            self.assertEqual(v.accept_range(residual, variance, threshold), accepted)
        x, C = np.array([.3, .2]), np.array([[.04, .01], [.01, .09]])
        rest_step = namespace['our_zero_velocity_update']
        corrected, covariance = rest_step(x, C, .0004)
        for actual, expected in zip((corrected, covariance), v.zero_velocity_update(x, C, .0004)):
            np.testing.assert_allclose(actual, expected)
        self.assertLess(abs(corrected[1]), abs(x[1]))
        self.assertNotEqual(corrected[0], x[0])  # Cross covariance also corrects height.
        self.assertGreater(np.linalg.eigvalsh(covariance).min(), 0.)
        data = v.with_outlier(v.select())
        with patch.object(v, 'accept_range', wraps=gate) as accept, \
                patch.object(v, 'zero_velocity_update', wraps=rest_step) as rest:
            result = v.run(data, predict_step=namespace['our_predict'],
                           update_step=namespace['our_update'], gate_step=accept,
                           stationary_step=rest, residual_gate=lambda dimension, alpha: 9.,
                           stationary_intervals=[(16., 19.)])
            self.assertEqual(accept.call_count, np.isfinite(result['chi2']).sum())
            self.assertEqual(rest.call_count, result['stationary_update'].sum())
            self.assertGreater(rest.call_count, 0)
        rejected = v.run(data, gate_step=lambda residual, R, threshold: False)
        self.assertFalse(rejected['accepted'].any())

    def test_notebook_matrix_expressions_preserve_state_and_covariance_shapes(self):
        _, namespace = completed_notebook_filter()
        predict, update = namespace['our_predict'], namespace['our_update']
        x = np.array([.3, -.2])
        C = np.array([[.04, .01], [.01, .09]])
        for dt in (.03, .05, .2):
            for acceleration in (-.4, 0., .4):
                for variance in (.000025, .04):
                    with self.subTest(dt=dt, acceleration=acceleration, variance=variance):
                        predicted, covariance = predict(x.copy(), C.copy(), acceleration, dt, .08)
                        self.assertEqual(predicted.shape, (2,))
                        self.assertEqual(covariance.shape, (2, 2))
                        expected_x, expected_C = v.predict(x, C, acceleration, dt, .08)
                        np.testing.assert_allclose(predicted, expected_x)
                        np.testing.assert_allclose(covariance, expected_C)
                        filtered, filtered_C = update(predicted, covariance, .35, variance)
                        self.assertEqual(filtered.shape, (2,))
                        self.assertEqual(filtered_C.shape, (2, 2))
                        expected_x, expected_C = v.update(expected_x, expected_C, .35, variance)
                        np.testing.assert_allclose(filtered, expected_x)
                        np.testing.assert_allclose(filtered_C, expected_C)

    def test_notebook_filter_matches_reference_replay_with_gating_dropout_and_rest(self):
        _, namespace = completed_notebook_filter()
        data = v.with_outlier(v.select(seed=20))
        options = dict(range_gap=(8., 11.), stationary_intervals=[(16., 19.)],
                       residual_gate=lambda dimension, alpha: chi2.ppf(1 - alpha, df=dimension))
        reference = v.run(data, **options)
        replay = v.run(data, predict_step=namespace['our_predict'],
                       update_step=namespace['our_update'], **options)
        self.assertEqual(replay['state'].shape, (len(data.time_s), 2))
        self.assertEqual(replay['covariance'].shape, (len(data.time_s), 2, 2))
        for field in ('state', 'covariance', 'predicted_residual', 'predicted_residual_variance',
                      'chi2', 'gain'):
            with self.subTest(field=field):
                np.testing.assert_allclose(replay[field], reference[field], atol=1e-14, equal_nan=True)
        for field in ('accepted', 'stationary_update', 'time_s'):
            np.testing.assert_array_equal(replay[field], reference[field])

    def test_e2_plot_labels_frozen_normalization_and_undefined_constant_channels(self):
        import matplotlib.pyplot as plt

        existing_figures = plt.get_fignums()
        values = np.tile(np.linspace(-1, 1, 100)[:, None], (1, 6))
        standardized = values / values.std(axis=0)
        standardized[:, 0] = np.nan  # A constant calibration channel cannot be normalized.
        validation = 3 + 2 * standardized
        names = [f'{sensor}.{axis}' for sensor in ('acc', 'gyro') for axis in 'xyz']
        for held_out in (validation, None):
            figures = []
            with patch.object(plt, 'show', side_effect=lambda: figures.append(plt.gcf())):
                plot_imu_calibration(values, names, standardized, held_out)
            self.assertEqual(len(figures), 2)
            axes = figures[1].axes
            self.assertIn('undefined', axes[0].texts[0].get_text())
            labels = axes[1].get_legend_handles_labels()[1]
            self.assertIn('Calibration: self-standardized', labels)
            self.assertEqual('Held-out: frozen calibration' in labels, held_out is not None)
            self.assertIn('calibration RMS', axes[1].get_xlabel())
        self.assertEqual(plt.get_fignums(), existing_figures)

    def test_simulated_validation_matches_frozen_sensor_assumptions(self):
        calibration = v.Calibration(1., .003)
        settings = v.Settings(acceleration_std_m_s2=.025, range_std_m=.016)
        raw, metadata, truth = v.synthetic(20, calibration=calibration, settings=settings)
        time = raw[:, 0]
        acceleration = np.r_[0, np.diff(truth[:, 1]) / np.diff(time)]
        acceleration_noise = (raw[:, 3] - 1.) * v.GRAVITY - acceleration
        range_noise = raw[:, 4] / 1000 - truth[:, 0]
        self.assertLess(abs(acceleration_noise.mean()), .004)
        self.assertAlmostEqual(acceleration_noise.std(), .025, delta=.003)
        self.assertAlmostEqual(range_noise.std(), .016, delta=.002)
        data = v.select(seed=20, calibration=calibration, settings=settings)
        self.assertEqual(data.calibration, calibration)
        self.assertEqual(metadata['sensor_assumptions'], data.metadata['sensor_assumptions'])
        self.assertEqual(data.metadata['sensor_assumptions']['range_std_m'], settings.range_std_m)
        # A noisy real recording still requires the original rest/geometry checks.
        with self.assertRaisesRegex(ValueError, 'stationary'):
            v.prepare(raw, dict(metadata, source='recording'), calibration=calibration)
        other = v.select(seed=21, calibration=calibration, settings=settings)
        self.assertNotEqual(data.metadata['input_sha256'], other.metadata['input_sha256'])

    def test_e2_simulated_pair_is_independent_labelled_and_preserves_existing_files(self):
        with tempfile.TemporaryDirectory() as directory:
            training, held_out = simulated_imu_pair(directory)
            values, validation = read_imu(training), read_imu(held_out)
            self.assertEqual(values.shape, (3000, 6))
            self.assertFalse(np.array_equal(values, validation))
            bias = values[:, 3:].mean(axis=0)
            self.assertLess(np.linalg.norm((validation[:, 3:] - bias).mean(axis=0)), .01)
            for path in (training, held_out):
                receipt = json.loads(Path(str(path) + '.capture.json').read_text())
                self.assertEqual(receipt['source'], 'simulation')
                self.assertEqual(receipt['status'], 'complete')
            original = training.read_bytes()
            self.assertEqual(simulated_imu_pair(directory), (training, held_out))
            self.assertEqual(training.read_bytes(), original)
            training.write_bytes(original + b'changed')
            with self.assertRaisesRegex(ValueError, 'changed'):
                simulated_imu_pair(directory)

    def test_irregular_timestamp_prediction_and_cross_covariance_velocity_correction(self):
        x, C = np.array([.3, .2]), np.diag([.04, .09])
        for dt in (.03, .12, .2):
            F, B = v.model(dt)
            predicted, covariance = v.predict(x, C, .4, dt, .1)
            np.testing.assert_allclose(predicted, [x[0] + x[1]*dt + .2*dt**2, x[1] + .4*dt])
            np.testing.assert_allclose(covariance, F @ C @ F.T + np.outer(B, B)*.01)
            self.assertGreater(covariance[1, 0], 0)
            corrected, filtered = v.update(predicted, covariance, predicted[0]+.01, .000025)
            self.assertGreater(corrected[1], predicted[1])
            self.assertLess(filtered[1, 1], covariance[1, 1])
            no_cross = np.diag(np.diag(covariance))
            corrected_no_cross, _ = v.update(predicted, no_cross, predicted[0]+.01, .000025)
            self.assertEqual(corrected_no_cross[1], predicted[1])

    def test_focused_expression_checks_detect_bad_gain_or_missing_process_noise(self):
        v.check_steps(v.predict, v.update)
        def wrong_predict(x, C, a, dt, sigma):
            F, B = v.model(dt)
            return F @ x + B*a, F @ C @ F.T
        with self.assertRaises(AssertionError):
            v.check_steps(wrong_predict, v.update)
        with self.assertRaises(AssertionError):
            v.check_steps(v.predict, lambda x, C, m, V: (x, C))

    def test_timestamped_measurements_and_inputs_use_actual_elapsed_time(self):
        data = v.Data(np.array([5., 5.10, 5.25]), np.array([2., 2., 2.]),
                      np.array([.3, np.nan, np.nan]), .3, v.Calibration(1., .005), {})
        result = v.run(data)
        np.testing.assert_allclose(result['state'][-1], [.3 + .5*2*.25**2, 2*.25])
        self.assertFalse(result['accepted'].any())

    def test_a_range_gap_must_actually_remove_observations_and_allow_recovery(self):
        data = v.select()
        with self.assertRaisesRegex(ValueError, 'before, during and after'):
            v.run(data, range_gap=(19, 20))
        data.height_m[(data.time_s >= 8) & (data.time_s <= 11)] = np.nan
        with self.assertRaisesRegex(ValueError, 'before, during and after'):
            v.run(data, range_gap=(8, 11))

    def test_timestamp_errors_and_epoch_roundoff(self):
        for values in ([0, 0], [1, .9], [0, .301], [0, np.nan], [0, np.inf]):
            with self.subTest(values=values), self.assertRaises(ValueError):
                v.timestamps(values)
        v.timestamps([1700000000.1, 1700000000.4])
        with self.assertRaises(ValueError):
            v.timestamps([1700000000.1, 1700000000.4001])

    def test_calibration_uses_only_initial_rest_and_validation_freezes_it(self):
        raw, metadata, truth = v.synthetic(0)
        calibration = v.calibrate(raw)
        changed = raw.copy()
        changed[changed[:, 0] >= 5, 3] += .05
        self.assertEqual(v.calibrate(changed), calibration)
        prepared = v.prepare(changed, metadata, calibration=calibration, truth=truth)
        self.assertEqual(prepared.calibration, calibration)
        np.testing.assert_allclose(prepared.acceleration_m_s2,
                                   (changed[changed[:, 0] >= 5, 3] - calibration.acceleration_at_rest_g)*v.GRAVITY)
        self.assertEqual(prepared.time_s[0], 5.)
        validation = v.select(seed=20, calibration=calibration)
        self.assertEqual(validation.calibration, calibration)
        self.assertNotEqual(validation.metadata['input_sha256'], v.select(seed=0).metadata['input_sha256'])

    def test_bad_rest_tilt_and_first_range_fail_instead_of_silent_fallback(self):
        for column, value in ((3, .7), (1, .3), (4, 0), (3, float('nan'))):
            raw, metadata, _ = v.synthetic()
            raw[:100, column] = value
            with self.subTest(column=column), self.assertRaises(ValueError):
                v.prepare(raw, metadata)
        raw, metadata, _ = v.synthetic()
        raw[100, 4] = 0
        with self.assertRaisesRegex(ValueError, 'first filtering sample'):
            v.prepare(raw, metadata)
        with self.assertRaises(FileNotFoundError):
            v.select('nonexistent-vertical-recording')

    def test_range_gaps_grow_covariance_and_missing_returns_skip_updates(self):
        data = v.select()
        baseline = v.run(data)
        dropout = v.run(data, range_gap=(8, 11))
        mask = (data.time_s >= 8) & (data.time_s <= 11)
        self.assertFalse(dropout['accepted'][mask].any())
        self.assertTrue(np.isnan(dropout['predicted_residual'][mask]).all())
        index = np.flatnonzero(mask)[-1]
        self.assertGreater(dropout['covariance'][index, 0, 0], baseline['covariance'][index, 0, 0])
        self.assertGreater(dropout['covariance'][index, 1, 1], baseline['covariance'][index, 1, 1])
        data.height_m[20] = np.nan
        self.assertFalse(v.run(data)['accepted'][20])

    def test_fault_is_copied_and_gating_preserves_the_prediction(self):
        data = v.select()
        original = data.height_m.copy()
        fault = v.with_outlier(data)
        np.testing.assert_array_equal(data.height_m, original)
        index = np.flatnonzero(fault.height_m != data.height_m)[0]
        gated = v.run(fault, residual_gate=lambda dimension, alpha: chi2.ppf(1-alpha, dimension))
        settings = v.Settings(range_std_m=data.calibration.range_std_m)
        x, C = v.predict(gated['state'][index-1], gated['covariance'][index-1],
                         data.acceleration_m_s2[index], data.time_s[index]-data.time_s[index-1],
                         settings.acceleration_std_m_s2)
        self.assertFalse(gated['accepted'][index])
        np.testing.assert_allclose(gated['state'][index], x)
        np.testing.assert_allclose(gated['covariance'][index], C)
        self.assertGreater(gated['chi2'][index], gated['gate_threshold'])
        self.assertTrue(np.isfinite(gated['predicted_residual'][index]))
        with self.assertRaises(ValueError):
            v.run(data, residual_gate=lambda _m, _alpha: -1)

    def test_known_rest_has_finite_covariance_and_false_rest_biases_motion(self):
        data = v.select()
        correct = v.run(data, stationary_intervals=[(16, 19)])
        false = v.run(data, stationary_intervals=[(8, 9)])
        baseline = v.run(data)
        index = np.flatnonzero(data.time_s <= 9)[-1]
        self.assertGreater(abs(false['state'][index, 1] - data.truth[index, 1]),
                           abs(baseline['state'][index, 1] - data.truth[index, 1]))
        self.assertTrue(np.all(np.linalg.eigvalsh(correct['covariance']) > 0))
        self.assertEqual(correct['settings']['stationary_velocity_std_m_s'], .02)

    def test_csv_bom_hash_schema_and_output_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            raw, metadata, _ = v.synthetic()
            for encoding in ('utf-8', 'utf-8-sig'):
                with (root/'raw.csv').open('w', encoding=encoding, newline='') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(v.RAW_COLUMNS)
                    writer.writerows(raw)
                details = dict(metadata, format='vertical-sensors-v1', columns=list(v.RAW_COLUMNS),
                               raw_sha256=hashlib.sha256((root/'raw.csv').read_bytes()).hexdigest())
                (root/'recording.json').write_text(json.dumps(details), encoding=encoding)
                loaded = v.select(root)
                self.assertIsNone(loaded.truth)
                np.testing.assert_allclose(loaded.height_m, raw[raw[:, 0] >= 5, 4]/1000)
            live_root = root/'live-capture'
            live_root.mkdir()
            live_raw = np.column_stack((raw[:, 0], np.full((len(raw), 3), .25), raw[:, 1:]))
            with (live_root/'raw.csv').open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(v.LIVE_COLUMNS)
                writer.writerows(live_raw)
            details = dict(metadata, format='vertical-sensors-v2', columns=list(v.LIVE_COLUMNS),
                           channels=list(v.LIVE_SENSOR_COLUMNS),
                           vertical_analysis_channels=list(v.VERTICAL_SENSOR_COLUMNS),
                           raw_sha256=hashlib.sha256((live_root/'raw.csv').read_bytes()).hexdigest())
            (live_root/'recording.json').write_text(json.dumps(details))
            loaded = v.select(live_root)
            expected = v.prepare(raw, metadata)
            np.testing.assert_allclose(loaded.acceleration_m_s2, expected.acceleration_m_s2)
            np.testing.assert_allclose(loaded.height_m, expected.height_m)
            data = v.select()
            result = v.run(data, range_gap=(8, 11))
            output = root/'comparison'
            import matplotlib.pyplot as plt
            close = plt.close
            with patch.object(plt, 'close'):
                v.save_comparison(data, {'baseline': result}, output)
                figure = plt.gcf()
                colors = [next(line.get_color() for line in axis.lines if line.get_label() == 'baseline')
                          for axis in figure.axes]
                self.assertEqual(colors, [colors[0]] * 3)
            close(figure)
            with (output/'baseline.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            for row in rows:
                residual = float(row['predicted_residual_m'])
                if np.isfinite(residual):
                    self.assertAlmostEqual(residual**2 / float(row['predicted_residual_variance_m2']), float(row['chi2']))
            saved = json.loads((output/'settings.json').read_text())
            self.assertEqual(saved['source']['input_sha256'], data.metadata['input_sha256'])
            self.assertEqual(saved['settings']['baseline']['range_gap_s'], [8, 11])
            self.assertGreater((output/'comparison.png').stat().st_size, 1000)
            with self.assertRaises(FileExistsError):
                v.save_comparison(data, {'baseline': result}, output)
            changed = v.with_outlier(data)
            with self.assertRaisesRegex(ValueError, 'must match'):
                v.save_comparison(changed, {'baseline': result}, root/'mismatched')
            self.assertFalse((root/'mismatched').exists())
            (root/'raw.csv').write_text('tampered')
            with self.assertRaisesRegex(ValueError, 'hash'):
                v.load_raw(root)

    def test_capture_confirmation_failure_timeout_and_command_bound(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'capture'
            with patch('day2.notebook_tools._run_capture') as runner:
                with self.assertRaises(ValueError):
                    capture_vertical(output, 'test://radio')
                runner.assert_not_called()
            with patch('day2.notebook_tools._run_capture', side_effect=subprocess.TimeoutExpired('test', 50)):
                with self.assertRaises(TimeoutError):
                    capture_vertical(output, 'test://radio', ready_for_motor_stop=True)
            with patch('day2.notebook_tools._run_capture', return_value=subprocess.CompletedProcess([], 1, 'failed', '')) as runner:
                with self.assertRaises(RuntimeError):
                    capture_vertical(output, 'test://radio', ready_for_motor_stop=True)
                self.assertEqual(runner.call_args.kwargs['timeout_s'], 50)
                self.assertIn('day2.vertical_live', runner.call_args.args[0])
                self.assertIn('--capture-worker', runner.call_args.args[0])
                self.assertNotIn('day2.live', runner.call_args.args[0])

    def test_vertical_capture_overwrite_is_opt_in(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)/'capture'
            output.mkdir()
            marker = output/'previous.txt'
            marker.write_text('keep unless overwrite is requested')
            with patch('day2.notebook_tools._run_capture') as runner:
                with self.assertRaises(FileExistsError):
                    capture_vertical(output, 'test://radio', ready_for_motor_stop=True)
                self.assertTrue(marker.is_file())
                runner.assert_not_called()
                runner.return_value = subprocess.CompletedProcess([], 1, 'failed', '')
                with self.assertRaises(RuntimeError):
                    capture_vertical(output, 'test://radio',
                                     ready_for_motor_stop=True, overwrite=True)
                self.assertFalse(output.exists())

    def test_cli_supervises_worker_and_propagates_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            arguments = ['--output', str(Path(directory)/'capture'), '--uri', 'test://no-radio',
                         '--prepare-for-handling']
            with patch('day2.notebook_tools._run_capture',
                       return_value=subprocess.CompletedProcess([], 0)) as runner:
                with patch.object(live, 'collect') as collect:
                    self.assertEqual(live.main(arguments), 0)
                    collect.assert_not_called()
                self.assertIn('--capture-worker', runner.call_args.args[0])
                self.assertEqual(runner.call_args.kwargs['timeout_s'], 50)
            with patch('day2.notebook_tools._run_capture',
                       return_value=subprocess.CompletedProcess([], 1)):
                with self.assertRaises(SystemExit) as error:
                    live.main(arguments)
                self.assertEqual(error.exception.code, 1)
            with patch.object(live, 'collect') as collect:
                self.assertEqual(live.main([*arguments, '--capture-worker']), 0)
                collect.assert_called_once_with(Path(directory)/'capture', 'test://no-radio', 20)

    def test_cli_timeout_covers_connection_and_parameter_waits(self):
        # The real CLI worker imports this fake cflib instead of opening a radio.
        bootstrap = '''
import os
from pathlib import Path
import sys
import time
import types
modules = {name: types.ModuleType(name) for name in (
    'cflib', 'cflib.crtp', 'cflib.crazyflie', 'cflib.crazyflie.log',
    'cflib.crazyflie.syncCrazyflie')}
modules['cflib'].crtp = modules['cflib.crtp']
modules['cflib.crtp'].init_drivers = lambda: None
modules['cflib.crazyflie'].Crazyflie = lambda **kwargs: object()
modules['cflib.crazyflie.log'].LogConfig = object
def stall(phase):
    if os.environ['STALL_PHASE'] == phase:
        Path(os.environ['STALL_MARKER']).write_text(phase)
        time.sleep(60)
class Connection:
    cf = object()
    def __init__(self, *args, **kwargs): pass
    def open_link(self):
        stall('connection')
    def wait_for_params(self): stall('parameters')
    def __exit__(self, *args): pass
modules['cflib.crazyflie.syncCrazyflie'].SyncCrazyflie = Connection
sys.modules.update(modules)
import notebook_radio
notebook_radio.crazyflie.init = lambda: None
'''
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'sitecustomize.py').write_text(bootstrap)
            code = str(Path(live.__file__).resolve().parents[1])
            for phase in ('connection', 'parameters'):
                with self.subTest(phase=phase):
                    output, marker = root/phase, root/f'{phase}.marker'
                    environment = dict(os.environ, STALL_PHASE=phase, STALL_MARKER=str(marker),
                                       PYTHONPATH=os.pathsep.join((str(root), code)))
                    def shortened_timeout(command, *, cwd, env, timeout_s):
                        self.assertEqual(timeout_s, 50)
                        return _run_capture(command, cwd=cwd, env=environment, timeout_s=2)
                    start = time.monotonic()
                    with patch('day2.notebook_tools._run_capture', side_effect=shortened_timeout):
                        with self.assertRaises(SystemExit) as error:
                            live.main(['--output', str(output), '--uri', 'test://no-radio',
                                       '--prepare-for-handling'])
                    self.assertEqual(error.exception.code, 1)
                    self.assertLess(time.monotonic() - start, 6)
                    self.assertEqual(marker.read_text(), phase)
                    self.assertTrue((output/'raw.csv').is_file())
                    metadata = json.loads((output/'recording.json').read_text())
                    self.assertNotEqual(metadata['status'], 'complete')
                    with self.assertRaisesRegex(ValueError, 'complete'):
                        v.load_raw(output)

    def test_live_status_values_and_no_flight_commands(self):
        valid = dict(zip(live.STATUS_CHANNELS, [LOCKED_BIT, 0, 0, 0, 0]))
        live.check_status(valid)
        for info, motor in ((0, 0), (LOCKED_BIT | 2, 0), (LOCKED_BIT, 1), (-64, 0), (np.nan, 0)):
            with self.subTest(info=info, motor=motor), self.assertRaises(ValueError):
                live.check_status(dict(valid, **{'supervisor.info': info, 'motor.m1': motor}))
        source = Path(live.__file__).read_text()
        calls = [node.func.attr for node in ast.walk(ast.parse(source))
                 if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)]
        for name in calls:
            self.assertFalse(name.startswith('send_'))
            self.assertNotIn(name, ('set_value', 'takeoff', 'land', 'request_arming'))

    def fake_capture(self, output, *, unsafe=False, lose_status=False):
        cf = MagicMock()
        cf.param.get_value.return_value = '1'
        cf.log.toc.get_element_by_complete_name.side_effect = lambda channel: (
            types.SimpleNamespace(ctype=('uint16_t' if channel in live.STATUS_CHANNELS
                                         or channel == 'range.zrange' else 'float'))
            if channel in (*live.STATUS_CHANNELS, *live.SENSOR_CHANNELS) else None)
        logs = []
        class Callbacks:
            def __init__(self): self.callbacks = []
            def add_callback(self, fn): self.callbacks.append(fn)
            def emit(self, *args):
                for fn in self.callbacks: fn(*args)
        class Log:
            def __init__(self, name, period_in_ms):
                self.name, self.variables, self.started = name, [], False
                self.data_received_cb, self.error_cb = Callbacks(), Callbacks()
                self.stop, self.delete = MagicMock(), MagicMock()
                logs.append(self)
            def add_variable(self, name, kind): self.variables.append((name, kind))
            def start(self):
                self.started = True
                if len(logs) != 2 or not all(log.started for log in logs): return
                for index in range(401):
                    stamp = ((1 << 24) - 100 + index * 50) % (1 << 24)
                    info = LOCKED_BIT
                    motors = [int(unsafe and index == 10), 0, 0, 0]
                    if not (lose_status and 10 <= index <= 20):
                        logs[0].data_received_cb.emit(stamp, dict(zip(live.STATUS_CHANNELS, [info, *motors])), logs[0])
                    sensors = [index*.01, -index*.01, index*.02, 0, 0, 1, 300]
                    logs[1].data_received_cb.emit(stamp, dict(zip(live.SENSOR_CHANNELS, sensors)), logs[1])
        connection = MagicMock()
        connection.cf = cf
        modules = {name: types.ModuleType(name) for name in (
            'cflib', 'cflib.crtp', 'cflib.crazyflie', 'cflib.crazyflie.log', 'cflib.crazyflie.syncCrazyflie')}
        modules['cflib'].crtp = modules['cflib.crtp']
        modules['cflib.crtp'].init_drivers = MagicMock()
        modules['cflib.crazyflie'].Crazyflie = MagicMock(return_value=cf)
        modules['cflib.crazyflie.log'].LogConfig = Log
        modules['cflib.crazyflie.syncCrazyflie'].SyncCrazyflie = MagicMock(return_value=connection)
        with patch.dict('sys.modules', modules), patch('notebook_radio.crazyflie.init'), \
             patch('notebook_radio.engage_motor_stop', return_value={'value': 1}) as stop:
            try:
                self.capture_logs = logs
                return live.collect(output, 'test://no-radio')
            finally:
                stop.assert_called_once_with(cf)
                self.assertFalse(cf.param.set_value.called)
                for log in logs:
                    log.stop.assert_called_once()
                    log.delete.assert_called_once()

    def test_full_capture_wrap_calibration_cleanup_and_reusable_raw_file(self):
        with tempfile.TemporaryDirectory() as directory:
            output = self.fake_capture(Path(directory)/'capture')
            data = v.select(output)
            self.assertAlmostEqual(data.time_s[-1], 20)
            np.testing.assert_allclose(data.acceleration_m_s2, 0)
            metadata = json.loads((output/'recording.json').read_text())
            self.assertEqual(metadata['status'], 'complete')
            self.assertEqual(metadata['channels'], list(live.SENSOR_CHANNELS))
            self.assertEqual(metadata['columns'], list(v.LIVE_COLUMNS))
            self.assertNotIn('stateEstimate.z', metadata['channels'])
            self.assertEqual(self.capture_logs[1].variables, [
                (channel, 'uint16_t' if channel == 'range.zrange' else 'float')
                for channel in live.SENSOR_CHANNELS
            ])
            self.assertEqual(self.capture_logs[0].variables, [
                (channel, 'uint16_t') for channel in live.STATUS_CHANNELS
            ])
            self.assertEqual(sum(2 if ctype == 'uint16_t' else 4
                                for _, ctype in self.capture_logs[1].variables), 26)
            with (output/'raw.csv').open(newline='') as stream:
                reader = csv.DictReader(stream)
                self.assertEqual(reader.fieldnames, list(v.LIVE_COLUMNS))
                rows = list(reader)
                self.assertAlmostEqual(float(rows[1]['gyro.x']), .01)
                self.assertAlmostEqual(float(rows[1]['gyro.y']), -.01)
                self.assertAlmostEqual(float(rows[1]['gyro.z']), .02)

    def test_motor_or_status_failure_preserves_partial_input_and_stop(self):
        for kwargs in ({'unsafe': True}, {'lose_status': True}):
            with self.subTest(kwargs=kwargs), tempfile.TemporaryDirectory() as directory:
                output = Path(directory)/'capture'
                with self.assertRaises(ValueError):
                    self.fake_capture(output, **kwargs)
                self.assertGreater((output/'raw.csv').stat().st_size, 50)
                self.assertEqual(json.loads((output/'recording.json').read_text())['status'], 'failed')
                with self.assertRaisesRegex(ValueError, 'complete'):
                    v.load_raw(output)


if __name__ == '__main__':
    unittest.main()

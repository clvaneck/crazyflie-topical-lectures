"""Survey acquisition, fitting and notebook fallback checks without a radio."""

import ast
from contextlib import contextmanager
import csv
import hashlib
import json
from pathlib import Path
import queue
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

from . import survey


class SurveyTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def test_circle_rectangle_export_and_planner_schema_are_shared(self):
        from trajectory_tools.course import construct_course, read_json

        template = read_json(Path(__file__).resolve().parents[2] / 'examples/course.shuttle.synthetic.json')
        for shape in ('circle', 'rectangle'):
            with self.subTest(shape=shape):
                points, source = survey.example_outline(shape)
                fitted = survey.fit_survey(points, shape, [1, 0, 0], source)
                path = survey.save_survey(fitted, self.root / (shape + '.json'), 'gate')
                record = read_json(path)
                self.assertIn('SIMULATED', record['survey']['source']['source'])
                np.testing.assert_allclose(record['centre'], [0, 0, .8], atol=.01)
                self.assertGreater(record['normal'][0], .99)
                course = construct_course(template, [record], ['gate:+', 'gate:-'], 2)
                self.assertEqual(len(course['hoops']), 4)
                self.assertIs(course['enabled_for_flight'], False)
                with self.assertRaises(FileExistsError):
                    survey.save_survey(fitted, path, 'gate')
                if shape == 'rectangle':
                    self.assertAlmostEqual(record['radius_m'], min(record['width_m'], record['height_m']) / 2, places=5)

    def test_modified_fit_or_source_cannot_be_exported(self):
        for change in ('points', 'normal', 'source'):
            with self.subTest(change=change):
                points, source = survey.example_outline()
                fitted = survey.fit_survey(points, 'circle', [1, 0, 0], source)
                if change == 'points':
                    fitted.points[0, 0] += 1
                elif change == 'normal':
                    fitted.fit.normal[:] *= -1
                else:
                    fitted.source['source'] = 'different capture'
                path = self.root / (change + '.json')
                with self.assertRaisesRegex(ValueError, 'changed since fitting'):
                    survey.save_survey(fitted, path, 'gate')
                self.assertFalse(path.exists())

    def test_bad_geometry_and_spacing_fail(self):
        for points, shape, spacing in ((np.zeros((20, 3)), 'circle', .02),
                                        (np.zeros((5, 3)), 'rectangle', .02),
                                        (np.full((10, 3), np.nan), 'circle', .02),
                                        (np.ones((20, 3)), 'circle', np.nan)):
            with self.assertRaises(ValueError):
                survey.fit_survey(points, shape, [1, 0, 0], {}, spacing)

    def collect(self, failure=None):
        clock = {'now': 0.0, 'index': 0}

        class Events:
            def get(self, timeout):
                index = clock['index']
                clock['index'] += 1
                stamp = index // 2
                arrival = stamp * .05
                if failure == 'gap' and index >= 14:
                    arrival += 1
                clock['now'] = arrival
                if index % 2 == 0:
                    data = {'supervisor.info': 1 << 6, **{name: 0 for name in survey.MOTOR_CHANNELS}}
                    if failure == 'motor' and index >= 14:
                        data['motor.m1'] = 10
                    stream = 'motors'
                else:
                    data = dict(zip(survey.POSE_CHANNELS, [0, np.cos(stamp), np.sin(stamp), 2]))
                    if failure == 'lighthouse' and index >= 14:
                        data['lighthouse.status'] = 1
                    if failure == 'invalid' and index >= 14:
                        data['stateEstimate.x'] = float('nan')
                    stream = 'pose'
                if failure == 'interrupt' and index >= 14:
                    raise KeyboardInterrupt
                return stream, stamp * 50 + 1, data, arrival

        cf = Mock()
        stop_check = {'value': 1, 'motor_outputs': [0, 0, 0, 0]}

        @contextmanager
        def stopped(*args):
            if failure == 'stop':
                raise TimeoutError('stop not verified')
            yield Mock(cf=cf), stop_check

        configs = [Mock(), Mock()]
        errors = queue.Queue()
        if failure == 'tail':
            configs[0].stop.side_effect = lambda: errors.put('connection lost at trace end')
        with (
            patch.object(survey, 'motor_stopped_connection', side_effect=stopped),
            patch('day2.notebook_tools.add_log_config', side_effect=configs) as logs,
            patch.object(survey.queue, 'Queue', side_effect=[Events(), errors]),
            patch.object(survey.time, 'monotonic', side_effect=lambda: clock['now']),
        ):
            if failure:
                with self.assertRaises(KeyboardInterrupt if failure == 'interrupt' else Exception):
                    survey._collect('radio://test', self.root, .6, .1)
            else:
                survey._collect('radio://test', self.root, .6, .1)
        if failure == 'stop':
            logs.assert_not_called()
        else:
            self.assertEqual(logs.call_count, 2)
            for config in configs:
                config.stop.assert_called_once()
                config.delete.assert_called_once()
        return json.loads((self.root / 'recording.json').read_text())

    def test_capture_uses_verified_stop_and_has_a_hash_checked_recording(self):
        metadata = self.collect()
        self.assertEqual(metadata['status'], 'complete')
        points, source = survey.read_capture(self.root)
        self.assertGreater(len(points), 5)
        self.assertEqual(source['motor_stop_check']['value'], 1)
        self.assertEqual(points.shape[1], 3)
        path = self.root / 'positions.csv'
        path.write_text(path.read_text() + '1,1,1,1,1,1\n')
        with self.assertRaisesRegex(ValueError, 'changed since capture'):
            survey.read_capture(self.root)

    def test_capture_failures_retain_diagnostics_but_cannot_be_fitted(self):
        for failure in ('motor', 'lighthouse', 'gap', 'invalid', 'interrupt', 'stop', 'tail'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                self.root = Path(directory)
                metadata = self.collect(failure)
                self.assertEqual(metadata['status'], 'failed')
                self.assertIn('reason', metadata)
                self.assertTrue((self.root / 'positions.csv').exists())
                with self.assertRaisesRegex(ValueError, 'not fit-ready'):
                    survey.read_capture(self.root)

    def test_wrapper_timeout_interruption_and_validation_fail_closed(self):
        for error in (subprocess.TimeoutExpired('worker', 60), KeyboardInterrupt()):
            path = self.root / type(error).__name__
            with patch.object(survey, 'run_worker', side_effect=error):
                with self.assertRaises(type(error)):
                    survey.capture_outline('radio://test', path, ready_for_motor_stop=True)
            self.assertEqual(json.loads((path / 'recording.json').read_text())['status'], 'failed')
            with self.assertRaises(FileExistsError):
                survey.capture_outline('radio://test', path, ready_for_motor_stop=True)
        with patch.object(survey, 'run_worker') as worker:
            for kwargs in ({}, {'ready_for_motor_stop': True, 'duration_s': float('nan')},
                           {'ready_for_motor_stop': True, 'duration_s': 121}):
                with self.assertRaises(ValueError):
                    survey.capture_outline('radio://test', self.root / 'invalid', **kwargs)
            worker.assert_not_called()

    def test_receipts_and_csv_accept_utf8_bom_and_reject_incomplete_trace(self):
        self.collect()
        metadata_path = self.root / 'recording.json'
        raw = self.root / 'positions.csv'
        raw.write_bytes(b'\xef\xbb\xbf' + raw.read_bytes())
        metadata = json.loads(metadata_path.read_text())
        metadata['positions_sha256'] = hashlib.sha256(raw.read_bytes()).hexdigest()
        metadata_path.write_text(json.dumps(metadata), encoding='utf-8-sig')
        survey.read_capture(self.root)
        metadata['duration_s'] = 5
        metadata_path.write_text(json.dumps(metadata))
        with self.assertRaisesRegex(ValueError, 'incomplete'):
            survey.read_capture(self.root)

    def test_survey_module_has_no_flight_or_arming_path(self):
        tree = ast.parse(Path(survey.__file__).read_text())
        names = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        self.assertFalse(names & {'commander', 'high_level_commander', 'send_arming_request', 'set_value'})


class SurveyNotebookTests(unittest.TestCase):
    def cells(self):
        path = Path(__file__).with_name('01_survey.ipynb')
        return {cell['id']: ''.join(cell['source']) for cell in json.loads(path.read_text())['cells']
                if cell['cell_type'] == 'code'}

    def execute(self, cells, namespace, name):
        exec(compile(cells[name], '<' + name + '>', 'exec'), namespace)

    def setup_notebook(self, directory, shape='circle'):
        cells = self.cells()
        cells['survey-setup'] = cells['survey-setup'].replace(
            'output = root / "code/day3/data" / run_id', 'output = Path(' + repr(str(directory)) + ') / "data"')
        cells['survey-choices'] = cells['survey-choices'].replace('shape = "circle"', 'shape = ' + repr(shape))
        namespace = {'__name__': '__main__'}
        self.execute(cells, namespace, 'survey-setup')
        namespace['display'] = lambda *args: None
        namespace['check_drone'] = Mock(side_effect=AssertionError('offline notebook accessed a radio'))
        self.execute(cells, namespace, 'survey-radio')
        self.execute(cells, namespace, 'survey-choices')
        return cells, namespace

    def test_run_all_fallback_fits_plots_and_exports_both_shapes_without_hardware(self):
        import matplotlib.image as image

        for shape in ('circle', 'rectangle'):
            with self.subTest(shape=shape), tempfile.TemporaryDirectory() as directory:
                cells, namespace = self.setup_notebook(directory, shape)
                namespace['capture_outline'] = Mock(side_effect=AssertionError('offline capture'))
                for name in ('survey-capture', 'survey-selection', 'survey-fit', 'survey-save'):
                    self.execute(cells, namespace, name)
                namespace['capture_outline'].assert_not_called()
                saved = namespace['saved_survey']
                self.assertTrue(saved.name.startswith('example-'))
                self.assertIn('SIMULATED', json.loads(saved.read_text())['survey']['source']['source'])
                self.assertGreater(image.imread(namespace['output'] / 'survey-fit.png').std(), .01)

    def test_failed_selection_clears_previous_fit_and_blocks_export_without_silent_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            cells, namespace = self.setup_notebook(directory)
            for name in ('survey-capture', 'survey-selection', 'survey-fit'):
                self.execute(cells, namespace, name)
            self.assertIsNotNone(namespace['survey'])
            for source in ('saved', 'live'):
                namespace['data_source'] = source
                namespace['recording_directory'] = Path(directory) / 'missing'
                with self.assertRaises((FileNotFoundError, ValueError)):
                    self.execute(cells, namespace, 'survey-selection')
                self.assertIsNone(namespace['survey'])
                self.assertIsNone(namespace['points'])
                with self.assertRaisesRegex(ValueError, 'fit it before saving'):
                    self.execute(cells, namespace, 'survey-save')


if __name__ == '__main__':
    unittest.main()

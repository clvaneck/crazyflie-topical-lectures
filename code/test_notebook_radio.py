"""One radio identity convention across the practical notebooks."""

from contextlib import contextmanager
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import ModuleType
import unittest
from unittest.mock import MagicMock, patch

import notebook_radio as radio


class NotebookRadioTests(unittest.TestCase):
    @staticmethod
    def completed_identity(command, **_kwargs):
        result = Path(command[command.index('--result') + 1])
        result.write_text(json.dumps({'uri': 'radio://0/16/2M/E7E7E7E702',
                                      'motor_stop_check': {'value': 1}}))
        return subprocess.CompletedProcess(command, 0, '')

    def test_label_discovery_retains_the_actual_channel(self):
        found = 'radio://0/51/2M/E7E7E7E707'
        with patch.object(radio.crazyflie, 'find_drone', return_value=found) as find:
            self.assertEqual(radio.assigned_uri(7), found)
            find.assert_called_once_with(7)

    def test_explicit_uri_uses_day_one_validation(self):
        with patch.object(radio.crazyflie, 'resolve_uri', return_value='assigned') as resolve:
            self.assertEqual(radio.assigned_uri(uri='explicit'), 'assigned')
            resolve.assert_called_once_with('explicit')

    def test_zero_label_reaches_day_one_discovery(self):
        uri = 'radio://0/2/2M/E7E7E7E700'
        with patch.object(radio.crazyflie, 'find_drone', return_value=uri) as find, \
             patch.object(radio.crazyflie, 'resolve_uri') as resolve:
            self.assertEqual(radio.assigned_uri(0), uri)
        find.assert_called_once_with(0)
        resolve.assert_not_called()

    def test_identity_worker_preserves_zero_label_and_returned_address(self):
        uri = 'radio://0/2/2M/E7E7E7E700'

        def completed(command, **kwargs):
            selection = json.loads(command[command.index('--selection') + 1])
            self.assertEqual(selection['label'], 0)
            result = Path(command[command.index('--result') + 1])
            result.write_text(json.dumps({'uri': uri, 'motor_stop_check': {'value': 1}}))
            return subprocess.CompletedProcess(command, 0, '')

        with patch.object(radio, 'run_worker', side_effect=completed), \
             patch('builtins.input', return_value='yes'):
            self.assertEqual(radio.check_drone(0), uri)

    def test_rejected_identity_does_not_return_a_usable_uri(self):
        with patch.object(radio, 'run_worker', side_effect=self.completed_identity), \
             patch('builtins.input', return_value='no'):
            with self.assertRaisesRegex(radio.crazyflie.DroneError, 'unconfirmed'):
                radio.check_drone(2)

    def test_successful_worker_precedes_confirmation_and_preserves_integer_label(self):
        output = io.StringIO()
        with patch.object(radio, 'run_worker', side_effect=self.completed_identity) as worker, \
             patch('builtins.input', return_value='yes') as confirm, \
             redirect_stdout(output):
            self.assertEqual(radio.check_drone(10), 'radio://0/16/2M/E7E7E7E702')
        command = worker.call_args.args[0]
        self.assertEqual(json.loads(command[command.index('--selection') + 1])['label'], 10)
        self.assertEqual(worker.call_args.kwargs['timeout_s'], 30)
        confirm.assert_called_once()
        self.assertIn("Confirm the LEDs on your drone to continue.", output.getvalue())

    def test_blink_confirmation_can_be_skipped_without_skipping_worker_checks(self):
        output = io.StringIO()
        with patch.object(radio, 'run_worker', side_effect=self.completed_identity) as worker, \
             patch('builtins.input') as confirm, \
             redirect_stdout(output):
            self.assertEqual(radio.check_drone(2, confirm_blink=False),
                             'radio://0/16/2M/E7E7E7E702')
        worker.assert_called_once()
        confirm.assert_not_called()
        self.assertIn("Blink confirmation skipped", output.getvalue())
        self.assertIn("Firmware stop remains engaged.", output.getvalue())

    def test_failed_timed_out_or_missing_worker_result_never_prompts(self):
        for result in (subprocess.TimeoutExpired('identity', 30),
                       subprocess.CompletedProcess([], 1, 'connection failed'),
                       subprocess.CompletedProcess([], 0, '')):
            effect = result if isinstance(result, Exception) else None
            with self.subTest(result=result), \
                 patch.object(radio, 'run_worker', side_effect=effect, return_value=result), \
                 patch('builtins.input') as confirm:
                with self.assertRaises((TimeoutError, radio.crazyflie.DroneError)):
                    radio.check_drone(2)
                confirm.assert_not_called()

    def test_invalid_timeout_does_not_launch_a_worker(self):
        for timeout in (0, -1, float('nan'), float('inf')):
            with self.subTest(timeout=timeout), patch.object(radio, 'run_worker') as worker:
                with self.assertRaises(ValueError):
                    radio.check_drone(2, timeout_s=timeout)
                worker.assert_not_called()

    def test_unverified_worker_receipt_never_prompts(self):
        def unverified(command, **kwargs):
            result = self.completed_identity(command, **kwargs)
            path = Path(command[command.index('--result') + 1])
            receipt = json.loads(path.read_text())
            receipt['motor_stop_check']['value'] = 0
            path.write_text(json.dumps(receipt))
            return result

        with patch.object(radio, 'run_worker', side_effect=unverified), \
             patch('builtins.input') as confirm:
            with self.assertRaisesRegex(radio.crazyflie.DroneError, 'stop was not verified'):
                radio.check_drone(2)
            confirm.assert_not_called()

    def test_verified_stop_precedes_status_and_identification(self):
        events = []
        connection = MagicMock()

        @contextmanager
        def stopped(uri, cache):
            events.append('stop verified')
            yield connection, {'value': 1}
            events.append('closed')
            self.assertFalse(result_path.exists())

        found = 'radio://0/16/2M/E7E7E7E702'
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(radio, 'assigned_uri', return_value=found), \
             patch.object(radio, 'motor_stopped_connection', side_effect=stopped), \
             patch.object(radio.crazyflie, 'status', side_effect=lambda scf: events.append('status')), \
             patch.object(radio.crazyflie, 'identify', side_effect=lambda scf: events.append('blink')), \
             patch('builtins.input') as confirm:
            result_path = Path(directory) / 'result.json'
            radio._identify({'label': 2}, directory, result_path)
            self.assertEqual(json.loads(result_path.read_text())['uri'], found)
            confirm.assert_not_called()
        self.assertEqual(events, ['stop verified', 'status', 'blink', 'closed'])
        connection.cf.param.set_value.assert_not_called()

    def test_stop_failure_prevents_identity_checks(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(radio, 'assigned_uri', return_value='assigned'), \
             patch.object(radio, 'motor_stopped_connection', side_effect=TimeoutError), \
             patch.object(radio.crazyflie, 'status') as status, \
             patch.object(radio.crazyflie, 'identify') as blink:
            with self.assertRaises(TimeoutError):
                radio._identify({'label': 2}, directory, Path(directory) / 'result.json')
            self.assertFalse((Path(directory) / 'result.json').exists())
            status.assert_not_called()
            blink.assert_not_called()

    def test_interrupted_or_failed_connection_always_closes_the_underlying_link(self):
        for phase in ('open', 'parameters', 'stop', 'body'):
            for error_type in (KeyboardInterrupt, RuntimeError):
                cf = MagicMock()
                connection = MagicMock(cf=cf)
                failure = error_type('interrupted ' + phase)
                modules = {name: ModuleType(name) for name in
                           ('cflib', 'cflib.crazyflie', 'cflib.crazyflie.syncCrazyflie')}
                modules['cflib.crazyflie'].Crazyflie = MagicMock(return_value=cf)
                modules['cflib.crazyflie.syncCrazyflie'].SyncCrazyflie = MagicMock(return_value=connection)
                if phase == 'open':
                    connection.open_link.side_effect = failure
                if phase == 'parameters':
                    connection.wait_for_params.side_effect = failure
                with self.subTest(phase=phase, error=error_type), patch.dict(sys.modules, modules), \
                     patch.object(radio.crazyflie, 'init'), \
                     patch.object(radio, 'engage_motor_stop',
                                  side_effect=failure if phase == 'stop' else None) as stop:
                    with self.assertRaises(error_type):
                        with radio.motor_stopped_connection('assigned', './cache'):
                            raise failure
                    cf.close_link.assert_called_once()
                    if phase in ('open', 'parameters'):
                        stop.assert_not_called()

    def test_real_sync_connection_interrupted_during_toc_wait_is_closed(self):
        try:
            import cflib.crazyflie.syncCrazyflie as sync
        except ImportError:
            self.skipTest('Optional cflib not installed; fake lifecycle cases run separately.')
        cf, waiter = MagicMock(), MagicMock()
        waiter.wait.side_effect = KeyboardInterrupt
        with patch.object(radio.crazyflie, 'init'), \
             patch('cflib.crazyflie.Crazyflie', return_value=cf), \
             patch.object(sync, 'Event', return_value=waiter), \
             patch.object(radio, 'engage_motor_stop') as stop:
            with self.assertRaises(KeyboardInterrupt):
                with radio.motor_stopped_connection('assigned', './cache'):
                    self.fail('Interrupted connection must not yield')
            cf.open_link.assert_called_once_with('assigned')
            cf.close_link.assert_called_once()
            stop.assert_not_called()

    def test_interruption_during_worker_setup_kills_and_reaps_the_process(self):
        processes = []
        popen = subprocess.Popen

        def start(*args, **kwargs):
            process = popen(*args, **kwargs)
            processes.append(process)
            return process

        with patch.object(radio.subprocess, 'Popen', side_effect=start), \
             patch.object(radio.time, 'monotonic', side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                radio.run_worker([sys.executable, '-c', 'import time; time.sleep(60)'],
                                 cwd=None, env=None, timeout_s=30)
        self.assertEqual(len(processes), 1)
        self.assertIsNotNone(processes[0].poll())

    def test_identity_deadline_covers_discovery_connection_and_parameters(self):
        bootstrap = '''
import os
from pathlib import Path
import sys
import time
import types
def stall(phase):
    if os.environ['STALL_PHASE'] == phase:
        Path(os.environ['STALL_MARKER']).write_text(phase)
        time.sleep(60)
modules = {name: types.ModuleType(name) for name in (
    'cflib', 'cflib.crtp', 'cflib.crazyflie', 'cflib.crazyflie.log',
    'cflib.crazyflie.syncCrazyflie')}
modules['cflib'].crtp = modules['cflib.crtp']
modules['cflib.crazyflie'].Crazyflie = lambda **kwargs: types.SimpleNamespace(close_link=lambda: stall('cleanup'))
modules['cflib.crazyflie.log'].LogConfig = object
class Connection:
    def __init__(self, uri, cf): self.cf = cf
    def open_link(self): stall('connection')
    def wait_for_params(self): stall('parameters')
modules['cflib.crazyflie.syncCrazyflie'].SyncCrazyflie = Connection
sys.modules.update(modules)
import day1.crazyflie as day_one
day_one.init = lambda: None
def find(label):
    stall('discovery')
    return 'radio://0/16/2M/E7E7E7E702'
day_one.find_drone = find
day_one.status = lambda connection: stall('status')
day_one.identify = lambda connection: stall('blink')
import race_common.motor_stop as motor_stop
def stop(cf):
    stall('stop')
    return {'value': 1}
motor_stop.engage_motor_stop = stop
'''
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'sitecustomize.py').write_text(bootstrap)
            code = str(Path(radio.__file__).parent)
            for phase in ('discovery', 'connection', 'parameters', 'stop', 'status', 'blink', 'cleanup', 'success'):
                marker = root / (phase + '.marker')
                environment = {'STALL_PHASE': phase, 'STALL_MARKER': str(marker),
                               'PYTHONPATH': os.pathsep.join((str(root), code))}
                processes = []
                popen = subprocess.Popen

                def start(*args, **kwargs):
                    process = popen(*args, **kwargs)
                    processes.append(process)
                    return process

                before = time.monotonic()
                with self.subTest(phase=phase), patch.dict(os.environ, environment), \
                     patch.object(radio.subprocess, 'Popen', side_effect=start), \
                     patch('builtins.input', return_value='yes') as confirm:
                    if phase == 'success':
                        self.assertEqual(radio.check_drone(2, cache_directory=root, timeout_s=2),
                                         'radio://0/16/2M/E7E7E7E702')
                        confirm.assert_called_once()
                    else:
                        with self.assertRaisesRegex(TimeoutError, 'worker stopped'):
                            radio.check_drone(2, cache_directory=root, timeout_s=2)
                        confirm.assert_not_called()
                self.assertLess(time.monotonic() - before, 5)
                if phase != 'success':
                    self.assertEqual(marker.read_text(), phase)
                self.assertEqual(len(processes), 1)
                self.assertIsNotNone(processes[0].poll())

    def test_survey_capture_and_flight_identification_reuse_the_discovered_uri(self):
        path = Path(__file__).parent / 'day3/03_flight.ipynb'
        uri = 'radio://0/51/2M/E7E7E7E707'
        flight = json.loads(path.read_text())
        connection = next(''.join(cell['source']) for cell in flight['cells']
                          if cell['id'] == 'flight-connect')
        check = MagicMock(return_value=uri)
        scope = dict(LABEL='7', check_drone=check)
        exec(connection.replace('LABEL = None', 'LABEL = "7"'), scope)
        self.assertEqual(scope['uri'], uri)
        check.assert_called_once_with('7')
        notebook = json.loads(path.with_name('01_survey.ipynb').read_text())
        capture_cell = next(''.join(cell['source']) for cell in notebook['cells']
                            if cell['id'] == 'survey-capture')
        capture = MagicMock(return_value=Path('capture-01'))
        scope = dict(uri=uri, data_source='live', output=Path('.'), capture_name='capture-01',
                     duration_s=30, preparation_s=5, capture_outline=capture)
        exec(capture_cell, scope)
        capture.assert_called_once_with(uri, Path('capture-01'), ready_for_motor_stop=True,
                                        duration_s=30, preparation_s=5)

    def test_connection_instructions_describe_live_connection_only(self):
        root = Path(__file__).resolve().parent
        for path in ('day2/02_calibration.ipynb', 'day2/03_height_velocity.ipynb',
                     'day3/03_flight.ipynb', 'day3/01_survey.ipynb'):
            notebook = json.loads((root / path).read_text())
            cells = {cell['id']: ''.join(cell['source']) for cell in notebook['cells']}
            guide = next(text for name, text in cells.items()
                         if name.endswith(('-radio-guide', '-connect-guide')))
            code = next(text for name, text in cells.items() if name.endswith(('-radio', '-connect')))
            with self.subTest(path=path):
                self.assertIn("Set `LABEL` to", guide)
                self.assertNotIn('offline', guide.lower())
                self.assertNotIn('offline', code.lower())

    def test_radio_cells_require_a_label_before_connection(self):
        root = Path(__file__).resolve().parent
        for path, setup_id, prefix in (
                ('day2/02_calibration.ipynb', 'day2-calibration-setup', 'day2-calibration'),
                ('day2/03_height_velocity.ipynb', 'day2-setup', 'day2-vertical'),
                ('day3/03_flight.ipynb', 'flight-setup', 'flight'),
                ('day3/01_survey.ipynb', 'survey-setup', 'survey')):
            notebook = json.loads((root / path).read_text())
            cells = {c['id']: ''.join(c['source']) for c in notebook['cells']}
            source = cells[prefix + ('-connect' if prefix == 'flight' else '-radio')].split('\n# retreieve', 1)[0]
            scope = {'check_drone': radio.check_drone, 'output': root, 'uri': None}
            with self.subTest(path=path), patch.object(radio, 'assigned_uri') as connect:
                if path == 'day2/02_calibration.ipynb':
                    with self.assertRaisesRegex(ValueError, 'Set LABEL'):
                        exec(source, scope)
                else:
                    exec(source, scope)
                self.assertIsNone(scope['uri'])
                connect.assert_not_called()
            self.assertNotIn(prefix + '-identity', cells)
            self.assertIn('from notebook_radio import check_drone', cells[setup_id])
            self.assertNotIn('drone_uri(LABEL)', cells[setup_id])

    def test_day_two_and_three_connection_cells_accept_zero(self):
        root = Path(__file__).resolve().parent
        uri = 'radio://0/2/2M/E7E7E7E700'
        for path in ('day2/02_calibration.ipynb', 'day2/03_height_velocity.ipynb',
                     'day3/03_flight.ipynb', 'day3/01_survey.ipynb'):
            notebook = json.loads((root / path).read_text())
            source = next(''.join(cell['source']).split('\n# retreieve', 1)[0]
                          for cell in notebook['cells'] if cell['id'].endswith(('-radio', '-connect')))
            check = MagicMock(return_value=uri)
            scope = {'check_drone': check, 'output': root}
            with self.subTest(path=path):
                exec(source.replace('LABEL = None', 'LABEL = 0', 1), scope)
                expected_kwargs = {'cache_directory': root / '.cflib-cache'}
                if path == 'day3/03_flight.ipynb':
                    expected_kwargs = {}
                if path in ('day2/02_calibration.ipynb', 'day2/03_height_velocity.ipynb'):
                    expected_kwargs['confirm_blink'] = False
                check.assert_called_once_with(0, **expected_kwargs)
                self.assertEqual(scope['uri'], uri)


if __name__ == '__main__':
    unittest.main()

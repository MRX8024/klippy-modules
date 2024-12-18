import numpy as np

class AccelerometerHelper:
    def __init__(self, printer):
        self.printer = printer
        self.is_finished = False
        self.samples = []
        self.request_start_time = None
        self.request_end_time = 9e9
        self.gcode = self.printer.lookup_object('gcode')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.reactor = self.printer.get_reactor()

    def wait_samples(self):
        while True:
            now = self.reactor.monotonic()
            self.reactor.pause(now + 0.010)
            if self.samples:
                last_mcu_time = self.samples[-1][0]
                if last_mcu_time > self.request_end_time:
                    # Return last batch mcu sample time
                    return self.samples[-1][0]
                self.samples.clear()

    def finish_measurements(self):
        self.wait_samples()
        self.is_finished = True
        return self.samples

    def handle_batch(self, batch):
        samples = batch['data']
        ptime = self.toolhead.get_last_move_time()
        now = self.reactor.monotonic()
        pmtime = self.toolhead.mcu.estimated_print_time(now)
        last_mcu_time = samples[-1][0]
        self.gcode.respond_info(
            f"print_time: {ptime:.10f}, "
            f"mcu_time: {pmtime:.10f} "
            f"sample_mcu_time: {last_mcu_time:.10f}, ")
        if self.is_finished:
            return False
        self.samples.extend(samples)
        return True


class AccelTimingTest:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.chip_name = self.config.get('accel_chip')
        # Register commands
        self.gcode.register_command('ACCEL_TIMING_TEST', self.cmd_ACCEL_TEST,
                                    desc='Run ACCEL_TIMING_TEST')

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.kin = self.toolhead.get_kinematics()
        self.reactor = self.printer.get_reactor()
        self.chip_config = self.printer.lookup_object(self.chip_name)

    def send(self, params):
        self.gcode.run_script_from_command(params)

    def cmd_ACCEL_TEST(self, gcmd):
        # To inital pos
        now = self.reactor.monotonic()
        if 'xy' not in self.kin.get_status(now)['homed_axes']:
            self.send('G28 X Y')
        self.send('G0 X10 Y10 F6000')
        # Switch to incremental coords
        self.send('G91')
        # Start handle adxl measurements
        aclient = AccelerometerHelper(self.printer)
        self.chip_config.batch_bulk.add_client(aclient.handle_batch)
        now = self.reactor.monotonic()
        self.reactor.pause(now + 0.1)
        # Run moves with measurements
        moves = [50 * (1 if i % 2 == 0 else -1) for i in range(10)]
        for move in moves:
            aclient.request_start_time = self.toolhead.get_last_move_time()
            self.send(f'G0 X{move}')
            # split_moves = np.arange(0, move, move / 5)
            # for spl_move in split_moves:
            #     self.send(f'G0 X{spl_move}')
            #     now = self.reactor.monotonic()
            #     self.reactor.pause(now + 0.050)
            aclient.request_end_time = self.toolhead.get_last_move_time()
            self.gcode.respond_info(f"measure_start_time: {aclient.request_start_time}")
            self.gcode.respond_info(f"measure_end_time: {aclient.request_end_time}")
            last_mcu_time_in_batch = aclient.wait_samples()
            # Filter samples in timing limits
            # samples = [s for s in samples if
            #            aclient.request_start_time <= s[0] <= aclient.request_end_time]
            ptime = self.toolhead.get_last_move_time()
            delta = ptime - last_mcu_time_in_batch
            self.gcode.respond_info('')
            self.gcode.respond_info(
                f"now_print_time: {ptime}, "
                f"last_mcu_time_in_batch: {last_mcu_time_in_batch}, "
                f"delta: {delta}, "
                f"get_sample_delta: {ptime - aclient.request_end_time}")
            self.gcode.respond_info('')
        aclient.finish_measurements()
        self.send('G90')


def load_config(config):
    return AccelTimingTest(config)

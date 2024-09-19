# flake8: noqa
# reference: https://github.com/maselab/tsnd
import datetime
import time
from enum import IntEnum
from tsnd5_driver.common_utils import check_range
from tsnd5_driver.thread_utils import ReusableLoopThread
from queue import Queue, Empty
from threading import RLock, Thread
from logging import getLogger
import serial


# TODO update comment style as npy style
class TSND151:
    """This class is a controller for TSND151"""

    _LOGGER_ = getLogger("TSND151")

    _START_BIT_ = b'\x9A'
    _OK_BIT_ = b'\x00'
    _NG_BIT_ = b'\x01'
    _OVERWRITE_OK_BIT_ = b'\x00'
    _OVERWRITE_NG_BIT_ = b'\x01'

    class OptionButtonMode(IntEnum):
        DISABLE = 0
        STOP = 1
        START_STOP = 2
        EVENT = 3
        EVENT_WITH_BUZZER = 4

    _RESPONSE_ARG_LEN_MAP_ = {
        b'\x8F': 1
        , b'\x90': 30
        , b'\x92': 8
        , b'\x93': 13
        , b'\x97': 3
        , b'\x99': 3
        , b'\x9B': 3
        , b'\x9D': 2
        , b'\x9F': 5
        , b'\xA1': 3
        , b'\xA3': 1
        , b'\xA6': 1
        , b'\xAA': 12
        , b'\xAB': 9
        , b'\xAD': 1
        , b'\xAF': 1
        , b'\xB1': 4
        , b'\xB3': 1
        , b'\xB6': 1
        , b'\xB7': 24
        , b'\xB8': 60
        , b'\xB9': 1
        , b'\xBA': 5
        , b'\xBB': 3
        , b'\xBC': 1
        , b'\xBD': 12
        , b'\xBE': 12
        , b'\xD1': 1
        , b'\xD3': 1
        , b'\xD6': 3
        , b'\xD8': 78
        , b'\xDA': 7
        , b'\xDC': 28
        , b'\xDD': 1
        , b'\x80': 22
        , b'\x81': 13
        , b'\x82': 9
        , b'\x83': 7
        , b'\x84': 9
        , b'\x85': 6
        , b'\x86': 13
        , b'\x87': 5
        , b'\x88': 1
        , b'\x89': 1
        , b'\x8A': 30
        , b'\x8B': 22
        , b'\x8C': 12}

    _RESPONSE_CODE_MAP_ = {
        'simple': b'\x8F'
        , 'acc_gyro_data': b'\x80'
        , 'magnetism_data': b'\x81'
        , 'atmosphere_data': b'\x82'
        , 'battery_voltage_data': b'\x83'
        , 'start_recording': b'\x88'
        , 'stop_recording': b'\x89'
        , 'quaternion_acc_gyro_data': b'\x8A'
        , 'time': b'\x92'
        , 'recording_time_settings': b'\x93'
        , 'acc_range': b'\xA3'
        , 'overwrite_protection': b'\xAF'
        , 'saved_entry_num': b'\xB6'
        , 'saved_entry_info': b'\xB7'
        , 'saved_entry_end': b'\xB9'
        , 'mode': b'\xBC'
        , 'option_button_behavior': b'\xAD'
    }

    _CMD_CODE_MAP_ = {
        'set_time': 0x11
        , 'get_time': 0x12
        , 'start': 0x13
        , 'get_recording_time_settings': 0x14
        , 'stop': 0x15
        , 'set_acc_and_gyro_interval': 0x16
        , 'set_magnetism_interval': 0x18
        , 'set_atmosphere_interval': 0x1A
        , 'set_battery_voltage_measurement': 0x1C
        , 'set_option_button_behavior': 0x2C
        , 'get_option_button_behavior': 0x2D
        , 'set_acc_range': 0x23
        , 'set_gyro_range': 0x25
        , 'set_overwrite_protection': 0x2E
        , 'get_overwrite_protection': 0x2F
        , 'clear_saved_data': 0x35
        , 'get_saved_entry_num': 0x36
        , 'get_saved_entry_info': 0x37
        , 'get_saved_entry': 0x39
        , 'get_mode': 0x3C
        , 'set_auto_power_off': 0x50
        , 'set_quaternion_interval': 0x55
    }

    def __init__(self, response_wait_timeout=5, auto_recovery=True, response_wait_auto_recovery_limit=5):
        """**Use TSND151.open instead of this**

        Do not use it directory.
        Please use TSND151.open.

        Parameters
        ----------
        response_wait_timeout: int, float
            timeout value to wait response

        auto_recovery: bool
            If True, it will try to recover from serial connection error.

        """

        self.response_wait_timeout = response_wait_timeout
        self._auto_recovery = auto_recovery
        self._recovered = False
        self._serial_property = {}
        self._recording_start_time = None
        self._response_wait_auto_recovery_limit = response_wait_auto_recovery_limit

        self.serial = None
        self._response_queue_map = {
            self._RESPONSE_CODE_MAP_['simple']: Queue()
            , self._RESPONSE_CODE_MAP_['recording_time_settings']: Queue()
            , self._RESPONSE_CODE_MAP_['start_recording']: Queue()
            , self._RESPONSE_CODE_MAP_['stop_recording']: Queue()
            , self._RESPONSE_CODE_MAP_['mode']: Queue()
            , self._RESPONSE_CODE_MAP_['overwrite_protection']: Queue()
            , self._RESPONSE_CODE_MAP_['saved_entry_num']: Queue()
            , self._RESPONSE_CODE_MAP_['saved_entry_info']: Queue()
            , self._RESPONSE_CODE_MAP_['saved_entry_end']: Queue()
            , self._RESPONSE_CODE_MAP_['acc_range']: Queue()
            , self._RESPONSE_CODE_MAP_['time']: Queue()
            , self._RESPONSE_CODE_MAP_['option_button_behavior']: Queue()
        }

        self.serial_lock = RLock()
        self.__close = False
        self.sensor_to_local_time_gap_in_microsecond = 0
        self._read_response_thread = ReusableLoopThread(self._in_loop_read_response)
        self._recording_will_stop_at = None

        self.wait_sec_on_auto_close_for_stability = 0.2

    @property
    def recording_start_time(self):
        return self._recording_start_time

    @property
    def auto_recovery(self):
        return self._auto_recovery

    @property
    def recovered(self):
        return self._recovered

    def set_response_queue(self, resp_code, q):
        """Set a queue to store responses from the sensor with specified code.

        Extended description of function.

        Parameters
        ----------
        resp_code: string
            Please use a key of self._RESPONSE_CODE_MAP_.
        q: queue. or None to stop store response

        Raises
        ------
        ValueError
            If resp_code is not in self._RESPONSE_CODE_MAP_.

        """
        if resp_code not in TSND151._RESPONSE_CODE_MAP_:
            raise ValueError("Invalid response code")

        self._response_queue_map[TSND151._RESPONSE_CODE_MAP_[resp_code]] = q

    def get_response_queue(self, resp_code):
        if resp_code not in TSND151._RESPONSE_CODE_MAP_:
            raise ValueError("Invalid response code")

        return self._response_queue_map[TSND151._RESPONSE_CODE_MAP_[resp_code]]

    def clear_all_queue(self):
        """Drop all response already received."""
        for q in self._response_queue_map.values():
            if q is not None:
                while not q.empty():
                    q.get()

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self.close(self.wait_sec_on_auto_close_for_stability)

    def __del__(self):
        self.close(self.wait_sec_on_auto_close_for_stability)

    @staticmethod
    def open(path_to_serial_port
             , timeout_sec=1
             , baudrate=115200
             , wait_sec_on_open_for_stability=2
             , wait_sec_on_auto_close_for_stability=2
             , response_wait_timeout=5):
        tsnd151 = TSND151(response_wait_timeout=response_wait_timeout)
        tsnd151._serial_property['port'] = path_to_serial_port
        tsnd151._serial_property['baudrate'] = baudrate
        tsnd151._serial_property['timeout'] = timeout_sec

        with tsnd151.serial_lock:
            tsnd151.__close = False
            tsnd151.serial = tsnd151._open_serial()
            tsnd151.wait_sec_on_auto_close_for_stability = wait_sec_on_auto_close_for_stability
            time.sleep(wait_sec_on_open_for_stability)

        tsnd151._read_response_thread.start()

        return tsnd151

    def close(self, wait_sec_for_stability=0.2):
        self.__close = True

        with self.serial_lock:
            if self.is_serial_ready():
                self.serial.close()
                del self.serial
                time.sleep(wait_sec_for_stability)
                self.serial = None

        self._read_response_thread.stop()



    def is_closed(self):
        if self.__close:
            return True
        elif self.auto_recovery:
            return False
        else:
            return not self.is_serial_ready()

    def is_serial_ready(self):
        with self.serial_lock:
            return self.serial is not None and self.serial.is_open

    def read(self, num=1, ping_check_interval = 2):
        res = b''

        last_read_time = datetime.datetime.now()

        while len(res) < num and not self.is_closed():
            with self.serial_lock:
                if not self.is_serial_ready():
                    raise IOError("Serial is not ready")
                _b = self.serial.read(num - len(res)) # it return silently when timed out

            if len(_b) == 0:
                if self.is_recording():
                    if (datetime.datetime.now() - last_read_time).total_seconds() > self._serial_property['timeout']:
                        raise TimeoutError()
                elif (datetime.datetime.now() - last_read_time).total_seconds() > ping_check_interval:
                    self._ping()
            else:
                last_read_time = datetime.datetime.now()
                res += _b

        if self.is_closed():
            raise IOError("Serial is closed")

        return res

    def read_response(self):
        while not self.is_closed():
            b = self.read()
            if b == self._START_BIT_:
                break

        cmd = self.read()
        if cmd not in self._RESPONSE_ARG_LEN_MAP_:
            raise ValueError(f"Invalid cmd_code is received: {cmd}")
        args = self.read(self._RESPONSE_ARG_LEN_MAP_[cmd])
        bcc = self.read()

        check = self._START_BIT_[0] ^ cmd[0]
        for b in args:
            check ^= b

        if check.to_bytes(1, 'little') != bcc:
            raise IOError("Invalid Verification Bit")

        return cmd, args


    def _open_serial(self, open_timeout=5):

        q = Queue()

        def __open_serial():
            s = serial.Serial(port=self._serial_property['port'], 
                              baudrate=self._serial_property['baudrate'], 
                              timeout=0.01) # 10 msec, it is just a check interval.
            q.put(s)
            time.sleep(open_timeout)
            if not q.empty(): # timed out while open. the base thread going to other operations.
                try:
                    s.close()
                except serial.SerialException as e:
                    self._LOGGER_.warning(f'Serial can not be closed on open_serial garbage collection. cause: {e}')
                except IOError as e:
                    self._LOGGER_.warning(f'Serial can not be closed on open_serial garbage collection. cause: {e}')
                finally:
                    del s

        t = Thread(target=__open_serial)
        t.start()
        try:
            return q.get(timeout=open_timeout)
        except Empty as e:
            raise TimeoutError("Timeout occured to open serial port")

    def _in_loop_read_response(self):

        if self.is_closed():
            time.sleep(0.01)
            return

        error = None
        try:
            cmd, args = self.read_response()
            if cmd in self._response_queue_map:
                q = self._response_queue_map[cmd]
                if q is not None:
                    q.put(args)

        except serial.SerialException as e:
            error = e
        except TimeoutError as e:
            error = e
        except IOError as e:
            error = e
        except ValueError as e:
            self._LOGGER_.warning(e)
            error = None

        if error is not None and not self.is_closed() and self.auto_recovery:
            error = self.recover_connection(error)

        if error is not None and not self.is_closed():
            if self.auto_recovery:
                if datetime.datetime.now().microsecond % 10 == 0:
                    self._LOGGER_.warning(f'Auto recovery will be done. {error}')
                time.sleep(0.1)
            else:
                raise error

    def recover_connection(self, error):
        if self.is_closed():
            error = None  # pass
        elif self.auto_recovery:
                # recovery
            error = None
            self._recovered = True

            with self.serial_lock:
                if self.is_serial_ready():
                    try:
                        self.serial.close()
                        time.sleep(0.1)
                    except serial.SerialException as e:
                        self._LOGGER_.warning(f'Serial can not be closed on auto recovery. cause: {e}')
                    except IOError as e:
                        self._LOGGER_.warning(f'Serial can not be closed on auto recovery. cause: {e}')
                    finally:
                        del self.serial
                        self.serial = None

                try:
                    self.serial = self._open_serial()
                except serial.SerialException as e:
                    error = e
                except TimeoutError as e:
                    error = e
                except IOError as e:
                    error = e
        
        return error


    def send(self, cmd, args=(0x00,)):
        with self.serial_lock:
            if not self.is_serial_ready():
                raise IOError("Serial is not ready")

            self.serial.write(self.build_cmd(cmd, args))
            self.serial.flush()


    @staticmethod
    def build_cmd(cmd_code, args):
        """cmd is array like object of int (of byte)"""
        total_cmd = [TSND151._START_BIT_[0], cmd_code]

        if isinstance(args, (list, tuple)):
            total_cmd.extend(args)
        else:
            total_cmd.append(args)

        # calc verification bit.
        bcc = 0x00
        for c in total_cmd:
            bcc ^= c

        total_cmd.append(bcc)
        return total_cmd

    def wait_response(self, code_name, timeout_sec=None):
        """
        Waits for a response with the specified code from the response queue.

        Args:
            code_name (str): The name of the response code to wait for.
            timeout_sec (float, optional): The maximum time to wait for the response, in seconds.
                If None, the default response_wait_timeout is used.

        Returns:
            object: The response object from the queue.

        Raises:
            NotImplementedError: If an invalid response code is provided.
            TimeoutError: If the timeout period is exceeded while waiting for the response.
            IOError: If the connection for TSND151 is lost.

        """

        if timeout_sec is None:
            timeout_sec = self.response_wait_timeout

        code = self._RESPONSE_CODE_MAP_[code_name]
        if code not in self._response_queue_map:
            raise NotImplementedError(f"Invalid response code: {code}")

        q = self._response_queue_map[code]

        wait_start = datetime.datetime.now()
        while not self._read_response_thread.check_should_be_stop():
            try:
                return q.get(timeout= min(timeout_sec, 0.1))  # 0.1 is a check interval
            except Empty:
                is_timed_out = (datetime.datetime.now() - wait_start).total_seconds() > timeout_sec
                if is_timed_out:
                    raise TimeoutError(f'Timeout occurred while waiting response for {code_name}')

    def check_success(self):
        return self.wait_response('simple') == self._OK_BIT_

    def set_time(self, dt=None):
        if dt is not None and not isinstance(dt, datetime.datetime):
            raise ValueError('Invalid dt. type(dt) have to be datetime.datetime.')

        if not self.check_is_cmd_mode():
            return False

        if dt is None:
            dt = datetime.datetime.now()
        msec = int(round(dt.microsecond / 1000)).to_bytes(2, "little", signed=False)
        self.send(self._CMD_CODE_MAP_['set_time'], [
            dt.year % 100, dt.month,
            dt.day, dt.hour, dt.minute,
            dt.second, msec[0], msec[1]])

        return self.check_success()

    def get_time(self):
        if not self.check_is_cmd_mode():
            return None

        self.send(self._CMD_CODE_MAP_['get_time'])
        resp = self.wait_response('time')
        return self.parse_time(resp)

    def calc_sensor_time_gap_in_microsecond(self, loop_n=10):
        """Calculate time gap between the sensor and local

        It try to get time of sensor loop_n times, and
        calculate time gap as a mean of them.
        The calculated time gap is saved as tsnd151.sensor_to_local_time_gap_in_microsecond.
        So, local's time = tsnd151.sensor_to_local_time_gap_in_microsecond + sensor's time

        Parameters
        ----------
        loop_n: int
            loop number to calculate a mean of time gap

        Returns
        -------
        float
            time gap in microsecond or None if failed

        """
        if not self.check_is_cmd_mode():
            return None

        total_gap = 0
        for x in range(loop_n):
            l_dt = datetime.datetime.now()
            s_dt = self.get_time()
            total_gap += (l_dt - s_dt).total_seconds()
            time.sleep(0.001)

        self.sensor_to_local_time_gap_in_microsecond = total_gap / loop_n * 1000000
        return self.sensor_to_local_time_gap_in_microsecond

    def check_is_cmd_mode(self, cannot_send_cmd_warn=True):
        mode = self.get_mode()
        is_cmd_mode = mode == 0 or mode == 2
        if cannot_send_cmd_warn and not is_cmd_mode:
            self._LOGGER_.warning('Mode is recording. Command cannot be sent.')

        return is_cmd_mode

    def set_acc_range(self, g=4):
        """g: 2 or 4 or 8 or 16"""

        if not self.check_is_cmd_mode():
            return False

        if g == 2:
            flag = 0x00
        elif g == 4:
            flag = 0x01
        elif g == 8:
            flag = 0x02
        elif g == 16:
            flag = 0x03
        else:
            raise ValueError(f"Invalid acc range: (2,4,8,16) but {g}")

        self.send(self._CMD_CODE_MAP_['set_acc_range'], [flag])
        resp = self.wait_response('acc_range')
        return resp[0] == flag

    def set_gyro_range(self, dps=2000):
        """dps: 250, 500, 1000, 2000"""

        if not self.check_is_cmd_mode():
            return False

        if dps == 250:
            flag = 0x00
        elif dps == 500:
            flag = 0x01
        elif dps == 1000:
            flag = 0x02
        elif dps == 2000:
            flag = 0x03
        else:
            raise ValueError(f"Invalid gyro range: (250,500,1000,2000) but {dps}")

        self.send(self._CMD_CODE_MAP_['set_gyro_range'], [flag])
        return self.check_success()

    def set_quaternion_interval(self, interval_in_5ms_unit, avg_num_for_send=1, avg_num_for_save=0):
        """
        interval_in_5ms_unit: interval in 5 ms unit.
                              0: off, on: 1-51 (means 5-255 ms interval) 
        avg_num_for_send    : N of SMA for sending via Bluetooth. (def:1)
                              0: off (not send), enable: 1-255.
                              e.g., If it is 2 and interval is set 5 ms, data will be sent every 10 ms.
        avg_num_for_save    : N of SMA for saving data in device memory. (def:0)
                              0: off (not save), enable: 1-255.
                              e.g., If it is 2 and interval is set 5 ms, data will be saved every 10 ms.

        If quaternion is enabled,
        NOTE1: +-2000 gyro range is forced.
        NOTE2: quaternion_acc_gyro_data response is used, no acc_gyro_data.
        """

        check_range("interval_in_5ms_unit", interval_in_5ms_unit, 0, 51)
        check_range("avg_num_for_send", avg_num_for_send, 0, 255)
        check_range("avg_num_for_save", avg_num_for_save, 0, 255)

        if not self.check_is_cmd_mode():
            return False

        self.send(self._CMD_CODE_MAP_['set_quaternion_interval'],
                  [interval_in_5ms_unit * 5, avg_num_for_send, avg_num_for_save])
        return self.check_success()

    def set_acc_and_gyro_interval(self, interval_in_ms, avg_num_for_send=1, avg_num_for_save=0):
        """
        interval_in_ms   : interval in ms.
                           0: off, on: 1-255 
        avg_num_for_send : N of SMA for sending via Bluetooth. (def:1)
                           0: off (not send), enable: 1-255.
                           e.g., If it is 2 and interval is set 5 ms, data will be sent every 10 ms.
        avg_num_for_save : N of SMA for saving data in device memory. (def:0)
                           0: off (not save), enable: 1-255.
                           e.g., If it is 2 and interval is set 5 ms, data will be saved every 10 ms.
        """
        check_range("interval_in_ms", interval_in_ms, 0, 255)
        check_range("avg_num_for_send", avg_num_for_send, 0, 255)
        check_range("avg_num_for_save", avg_num_for_save, 0, 255)

        if not self.check_is_cmd_mode():
            return False

        self.send(self._CMD_CODE_MAP_['set_acc_and_gyro_interval'],
                  [interval_in_ms, avg_num_for_send, avg_num_for_save])
        return self.check_success()

    def set_magnetism_interval(self, interval_in_ms, avg_num_for_send=1, avg_num_for_save=0):
        """
        interval_in_ms   : interval in ms.
                           0: off, on: 10-255 
        avg_num_for_send : N of SMA for sending via Bluetooth. (def:1)
                           0: off (not send), enable: 1-255.
                           e.g., If it is 2 and interval is set 5 ms, data will be sent every 10 ms.
        avg_num_for_save : N of SMA for saving data in device memory. (def:0)
                           0: off (not save), enable: 1-255.
                           e.g., If it is 2 and interval is set 5 ms, data will be saved every 10 ms.
        """

        if interval_in_ms != 0:
            check_range("interval_in_ms", interval_in_ms, 10, 255)
        check_range("avg_num_for_send", avg_num_for_send, 0, 255)
        check_range("avg_num_for_save", avg_num_for_save, 0, 255)

        if not self.check_is_cmd_mode():
            return False

        self.send(self._CMD_CODE_MAP_['set_magnetism_interval'],
                  [interval_in_ms, avg_num_for_send, avg_num_for_save])
        return self.check_success()

    def set_atmosphere_interval(self, interval_in_10ms_unit, avg_num_for_send=1, avg_num_for_save=0):
        """
        interval_in_10ms_unit: interval in 10 ms unit.
                               0: off, on: 4-255 (means 40-2550 ms interval) 
        avg_num_for_send     : N of SMA for sending via Bluetooth. (def:1)
                               0: off (not send), enable: 1-255.
                               e.g., If it is 2 and interval is set 10 ms, data will be sent every 20 ms.
        avg_num_for_save     : N of SMA for saving data in device memory. (def:0)
                               0: off (not save), enable: 1-255.
                               e.g., If it is 2 and interval is set 10 ms, data will be saved every 20 ms.
        """

        if interval_in_10ms_unit != 0:
            check_range("interval_in_10ms_unit", interval_in_10ms_unit, 4, 255)
        check_range("avg_num_for_send", avg_num_for_send, 0, 255)
        check_range("avg_num_for_save", avg_num_for_save, 0, 255)

        if not self.check_is_cmd_mode():
            return False

        self.send(self._CMD_CODE_MAP_['set_atmosphere_interval'],
                  [interval_in_10ms_unit, avg_num_for_send, avg_num_for_save])
        return self.check_success()

    def set_battery_voltage_measurement(self, send=False, save=False):
        """
        send: sending via Bluetooth or not. (def:False)
        send: sending to device memory or not. (def:False)
        """
        if not isinstance(send, bool):
            ValueError(f"Invalid 'send' (have to be bool):{send}")
        if not isinstance(save, bool):
            ValueError(f"Invalid 'save' (have to be bool):{save}")

        if not self.check_is_cmd_mode():
            return False

        self.send(self._CMD_CODE_MAP_['set_battery_voltage_measurement'], [send, save])
        return self.check_success()


    def set_option_button_behavior(self, mode: OptionButtonMode):
        """
        mode: mode of the option button
        """
        try:
            mode = TSND151.OptionButtonMode(mode)
        except ValueError as e:
            ValueError(f"Invalid 'mode' (have to be one of the TSND151.OptionButtonMode): {mode}")

        if not self.check_is_cmd_mode():
            return False

        self.send(self._CMD_CODE_MAP_['set_option_button_behavior'], [mode])
        return self.check_success()


    def get_option_button_behavior(self):
        """
        return: one of the TSND151.OptionButtonMode
        """
        self.send(self._CMD_CODE_MAP_['get_option_button_behavior'])
        res = self.wait_response('option_button_behavior')
        return TSND151.OptionButtonMode(int.from_bytes(res, byteorder='little'))

    def set_overwrite_protection(self, enable=False):
        """
        enable: enable overwrite protection for device memory or not
        """
        if not isinstance(enable, bool):
            ValueError(f"Invalid 'enable', {enable}, (have to be bool).")

        if not self.check_is_cmd_mode():
            return False

        self.send(self._CMD_CODE_MAP_['set_overwrite_protection'], [enable])
        return self.check_success()

    def get_overwrite_protection(self):
        """
        return: overwrite protection for device memory is enabled or not
        """
        if not self.check_is_cmd_mode():
            return None

        self.send(self._CMD_CODE_MAP_['get_overwrite_protection'])
        res = self.wait_response('overwrite_protection')
        return res == TSND151._OVERWRITE_NG_BIT_

    def set_auto_power_off(self, minutes):
        """
        minutes: 0:off, 1-20:minutes for auto power off
        """
        if minutes != 0:
            check_range("minutes", minutes, 1, 20)

        if not self.check_is_cmd_mode():
            return False

        self.send(self._CMD_CODE_MAP_['set_auto_power_off'], [minutes])
        return self.check_success()

    def is_recording(self):
        return (self._recording_will_stop_at is not None 
                and datetime.datetime.now() < self._recording_will_stop_at)

    def start_recording(self, force_restart=False, return_start_time_hms=False):

        if not self.check_is_cmd_mode(False):
            self._LOGGER_.warning('Recording already')
            if not force_restart:
                return None
            elif not self.stop_recording():
                self._LOGGER_.warning('Failed to stop recording')
                return None

        flag = [0, 0, 1, 1, 0, 0, 0,  # start immediately
                0, 0, 1, 1, 0, 0, 0]  # run forever
        self.send(self._CMD_CODE_MAP_['start'], flag)

        resp = self.wait_response('recording_time_settings')
        start_time = None
        if resp[0] == 1:
            if return_start_time_hms:
                start_time = datetime.datetime(resp[1] + 2000, resp[2], resp[3], resp[4], resp[5], resp[6])
            else:
                start_time = datetime.datetime(resp[1] + 2000, resp[2], resp[3], 0, 0, 0)

        run_forever = True
        for res, src in zip(resp[7:14], [100, 1, 1, 0, 0, 0]):  # run forever flags
            if res != src:
                run_forever = False

        if not run_forever:
            stop_time = datetime.datetime(resp[7] + 2000, resp[8], resp[9], resp[10], resp[11], resp[12])
            self._LOGGER_.warning(f'Set run forever, but stop time has been set:{stop_time}')
        else:
            stop_time = datetime.datetime(9999, 1, 1, 0, 0, 0)

        self._recording_will_stop_at = stop_time
        self._recovered = False
        self._recording_start_time = start_time

        self.wait_response('start_recording')
        return start_time

    def stop_recording(self):
        if self.check_is_cmd_mode(False):
            self._LOGGER_.info('Stopped already')
            return True

        cmd_code = self._CMD_CODE_MAP_['stop']
        self.send(cmd_code)

        resp = self.check_success()
        if not resp:
            return False

        self.wait_response('stop_recording')
        self._recording_will_stop_at = None
        self._recording_start_time = None
        return True

    def get_recording_time_settings(self, return_start_time_hms=False):
        if not self.check_is_cmd_mode():
            return None

        self.send(self._CMD_CODE_MAP_['get_recording_time_settings'])
        resp = self.wait_response('recording_time_settings')

        scheduled = resp[0] == 1
        if return_start_time_hms:
            start_time = datetime.datetime(resp[1] + 2000, resp[2], resp[3], resp[4], resp[5], resp[6])
        else:
            start_time = datetime.datetime(resp[1] + 2000, resp[2], resp[3], 0, 0, 0)

        stop_time = datetime.datetime(resp[7] + 2000, resp[8], resp[9], resp[10], resp[11], resp[12])

        return scheduled, start_time, stop_time

    def get_mode(self):
        return self._get_mode()
    
    def _ping(self):
        self._get_mode(0.2)

    def _get_mode(self, timeout_sec=None):
        """
        return: 0: USB_CMD, 1:USB_RECORDING, 2: BLT_CMD, 3:BLT_RECORDING
        """
        self.send(self._CMD_CODE_MAP_['get_mode'])
        resp = self.wait_response('mode', timeout_sec)

        return resp[0]

    def get_saved_entry_num(self):
        """
        return: number of entry saved in the device
        """
        if not self.check_is_cmd_mode():
            return None

        self.send(self._CMD_CODE_MAP_['get_saved_entry_num'])
        resp = self.wait_response('saved_entry_num')
        return resp[0]

    def get_saved_entry_start_date(self, entry_num: int, remove_hms=True):
        """return start_date of saved entry information.

        It is an utility to calculate entry's timestamp.

        Parameters
        ----------
        entry_num: int
           target entry number
           1 <= entry_num <= 80

        remove_hms: bool
            if true, only year, month, day is included in the return value.

        Returns
        -------
        bool
            return a datetime.datetime if success
            return None if fail

        """
        assert 1 <= entry_num <= 80

        if not self.check_is_cmd_mode():
            return None

        if entry_num > self.get_saved_entry_num():
            return None

        self.send(self._CMD_CODE_MAP_['get_saved_entry_info'], [entry_num])
        dt = self.parse_time(self.wait_response('saved_entry_info'))
        if remove_hms:
            dt = dt.replace(hour=0, minute=0, second=0, microsecond=0)

        return dt

    def get_saved_entry(self, entry_num: int, timeout_sec = 600):
        """Start saved entry download.

        It start a download of the specified saved entry,
        and block until end of the entry.
        If specified entry_num > tsnd151.get_saved_entry_num(),
        it will return immediately.

        Parameters
        ----------
        entry_num: int
           target entry number
           1 <= entry_num <= 80

        timeout_sec: int, float
            timeout value to wait end of the entry
            default is 600 sec

        Returns
        -------
        bool
            return True if success

        """
        assert 1 <= entry_num <= 80

        if not self.check_is_cmd_mode():
            return False

        if entry_num > self.get_saved_entry_num():
            return False

        self.send(self._CMD_CODE_MAP_['get_saved_entry'], [entry_num])
        self.wait_response('saved_entry_end', timeout_sec)
        return True

    def clear_saved_entry(self):
        """clear saved data"""
        if not self.check_is_cmd_mode():
            return False

        self.send(self._CMD_CODE_MAP_['clear_saved_data'])
        return self.check_success()

    @staticmethod
    def parse_acc_gyro(bytes_):
        """
        return: (ms, acc, gyro): acc=(ax, ay, az) in 0.1mg, gyro=(gx, gy, gz) in 0.01dps
        """
        ms = int.from_bytes(bytes_[0:4], "little")
        acc = [(int.from_bytes(bytes_[i:(i + 3)], "little", signed=True)) for i in range(4, 13, 3)]
        gyro = [(int.from_bytes(bytes_[i:(i + 3)], "little", signed=True)) for i in range(13, 22, 3)]

        return ms, acc, gyro  # ms, acc, gyro

    @staticmethod
    def parse_quaternion_acc_gyro(bytes_):
        """
        return: (ms, quaternion, acc, gyro): acc=(ax, ay, az) in 0.1mg, gyro=(gx, gy, gz) in 0.01dps
        """
        ms = int.from_bytes(bytes_[0:4], "little", signed=False)
        quat = [int.from_bytes(bytes_[i:(i + 2)], "little", signed=True) for i in range(4, 12, 2)]
        acc = [(int.from_bytes(bytes_[i:(i + 3)], "little", signed=True)) for i in range(12, 21, 3)]
        gyro = [(int.from_bytes(bytes_[i:(i + 3)], "little", signed=True)) for i in range(21, 30, 3)]
        return ms, quat, acc, gyro

    @staticmethod
    def parse_atmosphere(bytes_):
        """"Parse result of atmosphere sensor.

        Parameters
        ----------
        bytes_: byte
            an atmosphere record

        Returns
        -------
        tuple
            (ms, atmosphere, temperature)
        """
        ms = int.from_bytes(bytes_[0:4], "little", signed=False)
        atm = int.from_bytes(bytes_[4:7], "little", signed=False)
        temp = int.from_bytes(bytes_[7:9], "little", signed=True)

        return ms, atm, temp

    @staticmethod
    def parse_time(bytes_):
        """
        return: (ms, quaternion, acc, gyro): acc=(ax, ay, az) in 0.1mg, gyro=(gx, gy, gz) in 0.01dps
        """
        return datetime.datetime(
            year=2000 + int.from_bytes(bytes_[0:1], "little", signed=False)
            , month=int.from_bytes(bytes_[1:2], "little", signed=False)
            , day=int.from_bytes(bytes_[2:3], "little", signed=False)
            , hour=int.from_bytes(bytes_[3:4], "little", signed=False)
            , minute=int.from_bytes(bytes_[4:5], "little", signed=False)
            , second=int.from_bytes(bytes_[5:6], "little", signed=False)
            , microsecond=1000 * int.from_bytes(bytes_[6:8], "little", signed=False)
        )
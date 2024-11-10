# flake8: noqa
# reference: https://github.com/maselab/tsnd
import threading
import time
import os
import gzip
import queue
import warnings


############ thread utilities #############
class StopLoop(Exception):
    'Exception raised from in_loop_func when stop_flag of ReusableLoopThread is on.'
    pass


class ReusableLoopThread():
    def __init__(self, in_loop_func, before_loop_func=None, after_loop_func=None):
        self.__stop_flag = False
        self.__thread = None
        self.__in_loop_func = in_loop_func
        self.__before_loop_func = before_loop_func
        self.__after_loop_func = after_loop_func

    def _loop(self):
        if self.__before_loop_func is not None:
            self.__before_loop_func()

        try:
            while not self.__stop_flag:
                self.__in_loop_func()
        except StopLoop:
            pass
        finally:
            if self.__after_loop_func is not None:
                self.__after_loop_func()

    def start(self):
        if self.is_alive():
            self.stop()

        self.clear_stop_flag()
        self.__thread = threading.Thread(target=self._loop)
        self.__thread.start()

    def stop(self):
        self.set_stop_flag()
        if self.is_alive():
            self.__thread.join()

    def is_alive(self):
        return self.__thread is not None and self.__thread.is_alive()

    def set_stop_flag(self):
        self.__stop_flag = True

    def clear_stop_flag(self):
        self.__stop_flag = False

    def check_should_be_stop(self, raise_error=True):
        if self.__stop_flag and raise_error:
            raise StopLoop
        return self.__stop_flag


class TextLoggingThread(ReusableLoopThread):
    '''
        to_text_func: return should be ended with newline
        write_func  : lambda fout, data: write_data_to_fout
    '''

    def __init__(self, queue, to_text_func=None, write_func=None, queue_timeout=5, log_dir="log", prefix="log",
                 postfix="", size_limit=1024 * 1024 * 1024, flush_check_interval=10, use_localtime=True):
        self.queue = queue
        self.to_text_func = to_text_func
        self.write_func = write_func
        self.log_dir = log_dir
        self.queue_timeout = queue_timeout
        self.prefix = prefix
        self.postfix = postfix
        self.size_limit = size_limit
        self.flush_check_interval = flush_check_interval
        self.lastFlushTime = time.time()
        self.use_localtime = use_localtime

        self.current_fpath = None
        self.fout = None
        self.backupData = None
        super().__init__(self._in_loop_func, self._before_loop_func, self._after_loop_func)

    def _before_loop_func(self):
        os.makedirs(self.log_dir, exist_ok=True)
        self.lastFlushTime = time.time()

    def _in_loop_func(self):
        try:
            data = self.queue.get(timeout=self.queue_timeout) if not self.backupData else self.backupData
        except queue.Empty:
            return  # do nothing

        self.backupData = data

        try:
            if not self.fout:
                if not self.current_fpath:
                    fname = '.'.join((self.prefix, time.strftime("%Y-%m-%d_%H-%M-%S",
                                                                 time.localtime() if self.use_localtime else time.gmtime()),
                                      self.postfix, "gz"))
                    self.current_fpath = os.path.join(self.log_dir, fname)
                self.fout = gzip.open(self.current_fpath, mode="at")
                self.lastFlushTime = time.time()

            if self.to_text_func is not None:
                data = self.to_text_func(data)

            if self.write_func is not None:
                self.write_func(self.fout, data)
            else:
                l = len(data)
                written_l = 0
                while (written_l < l):
                    written_l += self.fout.write(data[written_l:])

            if (time.time() - self.lastFlushTime > self.flush_check_interval):  # check flush
                self.fout.flush()
                if (os.path.getsize(self.current_fpath) > self.size_limit):
                    self.fout.close()
                    self.fout = None
                    self.current_fpath = None

            self.backupData = None
        except OSError as e:
            warnings.warn('Error occurs during logging. retry forever\n  errno: [%d] msg: [%s]' % (e.errno, e.strerror))
            try:
                if self.fout:
                    self.fout.close()
            except:
                pass
            self.fout = None
            self.current_fpath = None

    def _after_loop_func(self):
        if self.fout:
            self.fout.close()
            self.fout = None
            self.current_fpath = None
import os
import time


class P2ImageReceiveCsvLogger:
    def __init__(self, path, topic_name, run_id, run_start_ns=None):
        self.path = os.path.abspath(os.path.expanduser(path))
        self.topic_name = topic_name
        self.run_id = run_id
        self.sequence = 0
        self.run_start_ns = run_start_ns if run_start_ns else time.monotonic_ns()
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.file = open(self.path, 'a', buffering=65536, encoding='utf-8')
        if self.file.tell() == 0:
            self.file.write('run_id,receive_ns,elapsed_ns,sequence,frame_id,topic_name\n')

    def write(self, msg):
        receive_ns = time.monotonic_ns()
        self.sequence += 1
        self.file.write(
            f'{self.run_id},'
            f'{receive_ns},'
            f'{receive_ns - self.run_start_ns},'
            f'{self.sequence},'
            f'{msg.header.frame_id},'
            f'{self.topic_name}\n'
        )

    def flush(self):
        self.file.flush()

    def close(self):
        self.file.flush()
        self.file.close()

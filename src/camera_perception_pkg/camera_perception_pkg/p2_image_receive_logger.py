import os


class P2ImageReceiveCsvLogger:
    def __init__(self, path, topic_name):
        self.path = os.path.abspath(os.path.expanduser(path))
        self.topic_name = topic_name
        self.sequence = 0
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self.file = open(self.path, 'a', buffering=65536, encoding='utf-8')
        if self.file.tell() == 0:
            self.file.write('sequence,frame_id,topic_name\n')

    def write(self, msg):
        self.sequence += 1
        self.file.write(
            f'{self.sequence},'
            f'{msg.header.frame_id},'
            f'{self.topic_name}\n'
        )

    def close(self):
        self.file.flush()
        self.file.close()

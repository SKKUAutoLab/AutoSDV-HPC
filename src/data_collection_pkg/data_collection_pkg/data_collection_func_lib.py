from datetime import datetime
import keyboard
import serial
import cv2
import os
import time
import uuid

message = """
 ____    ___   ____    ____  
|  _ \  / _ \ / ___|  |___ \ 
| |_) || | | |\___ \    __) |
|  _ < | |_| | ___) |  / __/ 
|_| \_\ \___/ |____/  |_____|
                             
    _            _                 
   / \    _   _ | |_   ___         
  / _ \  | | | || __| / _ \  _____ 
 / ___ \ | |_| || |_ | (_) ||_____|
/_/   \_\ \__,_| \__| \___/        
                                   
__     __       _      _        _       
\ \   / /  ___ | |__  (_)  ___ | |  ___ 
 \ \ / /  / _ \| '_ \ | | / __|| | / _ \\
  \ V /  |  __/| | | || || (__ | ||  __/
   \_/    \___||_| |_||_| \___||_| \___|                           

"""
print(message)
print("ROS2 기반 자율주행 설계 및 구현")
print('Sungkyunkwan University Automation Lab.')
print('')
print('------------------Authors------------------')
print('Hyeong-Keun Hong <whaihong@g.skku.edu>')
print('Jinsun Lee <with23skku@g.skku.edu>')
print('Siwoo Lee <edenlee@g.skku.edu>')
print('Jae-Wook Jeon <jwjeon@skku.edu>')
print('------------------------------------------')

class Data_Collect:
    def __init__(self, path, cam_num, max_steering=7, image_width=640, image_height=480, keyboard_sensing_period=0.1):
        self.data_collection_path = os.path.join(path, datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        self.left_speed = 0
        self.right_speed = 0
        self.steering = 0
        self.max_steering = max_steering
        self.cap = cv2.VideoCapture(cam_num)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
        self.frame_num = 0
        self.keyboard_sensing_period = keyboard_sensing_period  # 키 입력 감지 주기
        self.exit = False  # 프로세스 종료 플래그
        self.last_input_time = time.time()  # 마지막 키 입력 시간

    def process(self):
        """
        한 번의 키보드 입력을 처리하고 상태를 반환.
        :return: {'exit': True/False} - 프로세스 종료 여부 포함.
        """
        current_time = time.time()
        if current_time - self.last_input_time < self.keyboard_sensing_period:
            # 입력 대기 시간 동안 추가 입력 무시
            return {"exit": self.exit}

        # 키보드 입력 처리
        if keyboard.is_pressed('w'):
            self.left_speed = min(self.left_speed + 10, 250)
            self.right_speed = min(self.right_speed + 10, 250)
        elif keyboard.is_pressed('s'):
            self.left_speed = max(self.left_speed - 10, -250)
            self.right_speed = max(self.right_speed - 10, -250)
        elif keyboard.is_pressed('a'):
            self.steering = max(self.steering - 1, -self.max_steering)
        elif keyboard.is_pressed('d'):
            self.steering = min(self.steering + 1, self.max_steering)
        elif keyboard.is_pressed('r'):
            self.steering = 0
            self.left_speed = 0
            self.right_speed = 0
        elif keyboard.is_pressed('c'):
            print("Saving frame...")
            if not os.path.exists(self.data_collection_path):
                os.makedirs(self.data_collection_path)
            file_name = f"{self.data_collection_path}/{self.frame_num}_steer:{self.steering}_left_speed:{self.left_speed}_right_speed:{self.left_speed}.png"
            ret, frame = self.cap.read()
            if ret:
                cv2.imwrite(file_name, frame)
                self.frame_num += 1
        elif keyboard.is_pressed('f'):
            self.left_speed = 0
            self.right_speed = 0
            self.steering = 0
            print("You pressed 'f'. Exiting...")
            self.exit = True

        # 입력 처리 후 시간 업데이트
        self.last_input_time = current_time

        # 디버깅 출력
        print(f"Steering: {self.steering}, Left Speed: {self.left_speed}, Right Speed: {self.right_speed}")

        return {"exit": self.exit}

    def get_control_values(self):
        """현재 속도 및 조향 값을 반환."""
        return {
            "steering": self.steering,
            "left_speed": self.left_speed,
            "right_speed": self.right_speed,
        }

    def cleanup(self):
        """자원 정리"""
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        print("Program finished")

"""Isolated Pico/CDR regression. Source ROS + HPC install first.
Requires adjacent S32G3_ZCU/build-pico-host/pico_interop_test and the local
bridge listening on 127.0.0.1:27447, ROS domain 93 (pico_interop.json5).
The test intentionally sends boundary-valued commands ONLY on that domain.
"""
import os,subprocess,time,json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
os.environ['RMW_IMPLEMENTATION']='rmw_cyclonedds_cpp';os.environ['ROS_DOMAIN_ID']='93';os.environ['CYCLONEDDS_URI']=(ROOT / 'config/dds/cyclonedds_bridge_local.xml').as_uri()
os.environ['ROS_LOCALHOST_ONLY']='';os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='SYSTEM_DEFAULT'
import rclpy
from sensor_msgs.msg import Image
from diagnostic_msgs.msg import DiagnosticArray
from interfaces_pkg.msg import MotionCommand
from rclpy.qos import qos_profile_sensor_data
rclpy.init();n=rclpy.create_node('pico_interop_ros');seen={};health=0

def image(m):
 size=int(m.header.frame_id)
 assert (m.width,m.height,m.encoding)==(640,480,'jpeg')
 assert bytes(m.data)==bytes(i%251 for i in range(size))
 seen[size]=seen.get(size,0)+1

def diagnostic(m):
 global health
 s=m.status[0];assert (s.name,s.hardware_id,s.message)==('pico_interop','test','ok')
 assert (s.values[0].key,s.values[0].value)==('codec','CDR');health+=1
n.create_subscription(Image,'/image_01_raw',image,qos_profile_sensor_data)
n.create_subscription(DiagnosticArray,'/ivn/zcu_health',diagnostic,10)
pub=n.create_publisher(MotionCommand,'/topic_control_signal',qos_profile_sensor_data)
m=MotionCommand(frame_id=65535,steering=-2147483648,left_speed=2147483647,right_speed=-1)
results=[]
try:
 for proto in ('tcp','udp'):
  seen={};health=0
  p=subprocess.Popen([str(ROOT.parent / 'S32G3_ZCU/build-pico-host/pico_interop_test'),f'{proto}/127.0.0.1:27447'],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
  try:
   end=time.monotonic()+18
   while p.poll() is None and time.monotonic()<end:
    pub.publish(m);rclpy.spin_once(n,timeout_sec=.05)
   out=p.communicate(timeout=3)[0]
   assert p.returncode==0,out
   assert set(seen)=={0,43200,65503} and min(seen.values())>2 and health>5,(seen,health,out)
   results.append({'transport':proto,'image_sizes_received':seen,'health':health,'probe':out})
  finally:
   if p.poll() is None:p.kill();p.wait()
 print(json.dumps(results))
finally:n.destroy_node();rclpy.shutdown()

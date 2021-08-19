#!/usr/bin/python
import rclpy
import yaml
import sys
import os
from python_qt_binding.QtWidgets import QApplication, QWidget, QVBoxLayout,QHBoxLayout,QGridLayout, QLabel, QSlider, QLineEdit, QPushButton, QCheckBox
from python_qt_binding.QtCore import Signal, Qt,  pyqtSlot
from python_qt_binding.QtGui import QFont
from threading import Thread
import signal
from geometry_msgs.msg import Quaternion
from functools import reduce
from scipy.spatial.transform import Rotation
from numpy import pi
from ament_index_python import get_package_share_directory

font = QFont("Helvetica", 9, QFont.Bold)
topic_font = QFont("Helvetica", 10, QFont.Bold)

def rsetattr(obj, attr, val, cast=True):
    pre, _, post = attr.rpartition('.')
    if pre:
        return setattr(rgetattr(obj, pre), post, val)
    if cast:
        return setattr(obj, post, type(getattr(obj, post))(val))
    else:
        return setattr(obj, post, val)

def rgetattr(obj, attr, *args):
    if attr == '':
        return obj
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)
    return reduce(_getattr, [obj] + attr.split('.'))

def quaternion_msg(rpy):
    q = Rotation.from_euler('xyz',(rpy['roll'], rpy['pitch'], rpy['yaw'])).as_quat()
    Q = Quaternion()
    Q.x = q[0]
    Q.y = q[1]
    Q.z = q[2]
    Q.w = q[3]
    return Q

def split_field(key):
    if '.' in key:
        return key.rsplit('.', 1)
    return '', key

def isRPY(key, msg):
    field, axis = split_field(key)
    if axis in ('roll', 'pitch', 'yaw'):
        return type(rgetattr(msg, field)) == Quaternion
    return False

def robust_eval(val):
    if type(val) in (list,tuple):
        return [robust_eval(v) for v in val]
    if type(val) == str:
        val_expr = val.strip().lower()
        
        # check for  Pi fractions 
        for sign, sign_rep in ((1, ''), (-1, '-')):

            if val_expr == sign_rep + 'pi':
                return sign*pi
            
            for denom in range(2, 13):
                if val_expr == sign_rep + 'pi/' + str(denom):
                    return sign * pi/denom
        return val
    
    return float(val)

def key_tag(topic, key):
    return topic + '/' + key

def get_interface(pkg, interface, name):
    pkg = __import__(pkg, globals(), locals(), [interface])
    return getattr(getattr(pkg,interface), name)

def get_type(msg, key):
    if '[' in key:
        key = ''.join(key.replace('[',']').split(']')[::2]).strip('.')
    types = msg.get_fields_and_field_types()
    if '.' in key:
        main, nested = key.split('.',1)
        pkg, name = types[main].split('/')
        return get_type(get_interface(pkg, 'msg', name), nested)
    
    # not nested -> raw types
    if key not in types and key in ('roll', 'pitch', 'yaw'):
        return 'double'
    out = types[key]
    if 'double' in out or 'float' in out:
        return 'double'
    if 'int' in out:
        return 'int'
    return 'bool'

class Control:
    def __init__(self, msg, info):
        
        self.type = get_type(msg, info['to'])
        
        if self.type in ('int', 'double'):
            
            self.min = self.conv(robust_eval(info['min']))
            self.max = self.conv(robust_eval(info['max']))                        
            self.range = 1000 if self.type == 'double' else self.max-self.min
            self.default = self.conv(robust_eval(info['default'])) if 'default' in info else self.conv((self.max+self.min)/2)
            self.value = self.default
        
        else:
            self.range = None
            self.default = self.conv(robust_eval(info['default'])) if 'default' in info else False
            self.value = self.default
                            
    def conv(self, v):
        if self.type == 'int':
            return int(v)
        if self.type == 'double':
            return float(v)
        if self.type == 'bool':
            return bool(v)
        print('Unknown type "{}"'.format(self.type))
        return None
    
    def reset(self):
        if self.range is None:
            self.box.setChecked(self.default)
            return
        elif self.type == 'double':
            slider_val = (self.default-self.min)/(self.max-self.min)*self.range            
        else:
            slider_val = self.default-self.min
        self.slider.setValue(int(slider_val))
        
    def refresh(self):
        if self.range is None:
            self.value = self.box.isChecked()
        else:
            slider_val = self.slider.value()
            if self.type == 'double':
                self.value = self.min + slider_val*(self.max - self.min)/self.range
                self.display.setText(f'{round(self.value, 2)}')
            else:
                self.value = self.min + slider_val
                self.display.setText(f'{self.value}')
    
    def init_slider(self):
        
        self.display = QLineEdit(f'{round(self.value, 2)}')
        self.display.setAlignment(Qt.AlignRight)
        self.display.setFont(font)
        self.display.setReadOnly(True)
                                    
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setFont(font)
        self.slider.setRange(0, self.range)
        self.reset()
        
    def init_box(self):
        
        self.box = QCheckBox()
        self.reset()
        
        
           
class Publisher:
    '''
    A publisher is either:
        - a classical publisher (topic / msg)
        - or a service client (topic is the service name, msg is the request)
            in this case the response is ignored
    '''
    node = None
    def __init__(self, topic, msg, info, is_msg = True):
        self.topic = topic
        
        if is_msg:
            self.msg = msg()
            self.pub = Publisher.node.create_publisher(msg, topic, 10)
            self.client = None
        else:
            self.msg = msg.Request()
            self.pub = None
            self.client = Publisher.node.create_client(msg, topic)
            Publisher.node.get_logger().info('Waiting for service ' + topic)
            self.client.wait_for_service()
                    
        self.rpy = {}
            
        # init map from GUI to message
        self.map = {}
        to_remove = []
        for key in info:
            tag = key_tag(topic, key)
            if type(info[key]) == dict:
                if isRPY(info[key]['to'], self.msg):
                    field, axis = split_field(info[key]['to'])
                    if field not in self.rpy:
                        self.rpy[field] = {'roll': 0, 'pitch': 0, 'yaw': 0}
                self.map[tag] = info[key]['to']
 
            else:
                if key != 'type':
                    # init non-zero defaults
                    if isRPY(key, self.msg):
                        field, axis = split_field(key)
                        if field not in self.rpy:
                            self.rpy[field] = {'roll': 0, 'pitch': 0, 'yaw': 0}
                        self.rpy[field][axis]  = robust_eval(info[key])
                    else:
                        self.write(key, robust_eval(info[key]))
                to_remove.append(key)
        for rm in to_remove:
            info.pop(rm)
                           
    def write(self, key, val):
        field, axis = split_field(key)
        if field in self.rpy:
            self.rpy[field][axis] = val
        elif '[' in key:
            field, idx = key[:-1].split('[')
            idx = int(idx)
            current = rgetattr(self.msg, field)
            if len(current) <= idx:
                default_val = 'name' in field and '' or 0
                for _ in range(idx +1 - len(current)):
                    current.append(default_val)
            current[idx] = val
            rsetattr(self.msg, field, current, False)
        else:
            rsetattr(self.msg, key, val)
        
    def update(self, values):
        for tag in self.map:
            self.write(self.map[tag], values[tag].value)
        # write RPY's to Quaternions
        for field in self.rpy:
            if field:
                rsetattr(self.msg, field, quaternion_msg(self.rpy[field]), False)
            else:
                self.msg = quaternion_msg(self.rpy[field])

        # update time if classical stamped msg
        if hasattr(self.msg, 'header'):
            self.write('header.stamp', Publisher.node.get_clock().now().to_msg())        
        elif hasattr(self.msg, 'stamp'):
            self.write('stamp', Publisher.node.get_clock().now().to_msg())
            
        if self.pub is not None:
            self.pub.publish(self.msg)
        else:
            # service call, dont care about the result
            self.client.call_async(self.msg)            

class SliderPublisher(QWidget):
    def __init__(self, node, content):
        super(SliderPublisher, self).__init__()
                
        content = content.replace('\t', '    ')
        
        self.running = True
        self.node = node
                        
        # get message types
        Publisher.node = node
        self.publishers = {}
        self.controls = {}
        
        self.timer = node.create_timer(0.1, self.publish)
        
        # to keep track of key ordering in the yaml file
        order = []
         
        for topic, info in yaml.safe_load(content).items():
                        
            msg = info.pop('type')
            if msg.count('/') == 2:
                pkg,interface,msg = msg.split('/')
            else:
                pkg,msg = msg.split('/')
                interface = None
            
            if interface is None:
                # guess msg or srv
                here = {}
                share = get_package_share_directory(pkg)                
                for key in ('msg','srv'):
                    here[key] = os.path.exists(f'{share}/{key}/{msg}.{key}')
                
                if here['msg'] and here['srv']:
                    node.get_logger().error(f'Specify srv/msg in the yaml file: both files exist for {msg}')
                    sys.exit(0)
                interface = 'msg' if here['msg'] else 'srv'
            
            msg = get_interface(pkg, interface, msg)
            self.publishers[topic] = Publisher(topic, msg, info, interface == 'msg')
            order.append((topic,[]))
            for key in info:
                tag = key_tag(topic,key)
                
                # identify key type -> slider (continuous or discrete) / checkbox
                self.controls[tag] = Control(self.publishers[topic].msg, info[key])
                
                order[-1][1].append((content.find(' ' + key + ':'), key))
                    
            order[-1][1].sort()
        order.sort(key = lambda x: x[1][0][0])
        # build sliders - thanks joint_state_publisher
        sliderUpdateTrigger = Signal()
        self.vlayout = QVBoxLayout(self)
        self.gridlayout = QGridLayout()
        
        y = 0
        for topic,keys in order:
            topic_layout = QVBoxLayout()
            label = QLabel('[{}] {}'.format('msg' if self.publishers[topic].pub is not None else 'srv',topic))
            label.setFont(topic_font)
            topic_layout.addWidget(label)
            self.gridlayout.addLayout(topic_layout, *(y, 0))
            y+=1
            for idx,key in keys:
                tag = key_tag(topic,key)
                key_layout = QVBoxLayout()
                row_layout = QHBoxLayout()
                label = QLabel(key)
                label.setFont(font)
                row_layout.addWidget(label)
                
                if self.controls[tag].range is not None:
                                
                    self.controls[tag].init_slider()
                
                    row_layout.addWidget(self.controls[tag].display)
                    key_layout.addLayout(row_layout)
                    key_layout.addWidget(self.controls[tag].slider)
                    self.controls[tag].slider.valueChanged.connect(self.onValueChanged)
                    
                else:
                    self.controls[tag].init_box()
                    row_layout.addWidget(self.controls[tag].box)
                    key_layout.addLayout(row_layout)
                    self.controls[tag].box.clicked.connect(self.onValueChanged)
                
                self.gridlayout.addLayout(key_layout, *(y,0))
                y+=1
            
        self.vlayout.addLayout(self.gridlayout)            
        
        self.reset_button = QPushButton('Reset', self)
        self.reset_button.clicked.connect(self.reset)
        self.vlayout.addWidget(self.reset_button)
                   

    def onValueChanged(self, event):
        for control in self.controls.values():
            control.refresh()
            
    def reset(self, event):
        for control in self.controls.values():
            control.reset()            
        self.onValueChanged(event)
                    
    def publish(self):      
        for pub in self.publishers:
            self.publishers[pub].update(self.controls)
            
    def closeEvent(self, event):
        self.running = False
        
    def loop(self):
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
        if self.running:
            self.node.destroy_node()
            rclpy.shutdown()        
        
def main(args=None):
           
    rclpy.init(args=args)
    node = rclpy.create_node('slider_publisher')
    
    # read passed param file
    filename = len(sys.argv) > 1 and sys.argv[1] or ''
    if not os.path.exists(filename):
        node.get_logger().error("did not get any configuration file, was '{}'".format(filename))
        sys.exit(0)
        
    # also get order from file
    with open(filename) as f:
        content = f.read()

    # build GUI
    full_namespace = '{}/{}'.format(node.get_namespace().strip('/'), node.get_name())
    app = QApplication([full_namespace])    
    sp = SliderPublisher(node, content)
    
    try:
        Thread(target=sp.loop).start()
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        sp.show()
        app.exec_()
        sp.running = False
    except:
        node.destroy_node()
        rclpy.shutdown()
    sp.running = False
            
            
if __name__ == "__main__":
    main()
 

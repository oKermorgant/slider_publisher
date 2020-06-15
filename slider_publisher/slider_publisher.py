#!/usr/bin/python
import rclpy
import yaml
import sys
import os
from python_qt_binding.QtWidgets import QApplication, QWidget, QVBoxLayout,QHBoxLayout,QGridLayout, QLabel, QSlider, QLineEdit, QPushButton
from python_qt_binding.QtCore import Signal, Qt,  pyqtSlot
from python_qt_binding.QtGui import QFont
from threading import Thread
import signal
from geometry_msgs.msg import Quaternion
from functools import reduce
from scipy.spatial.transform import Rotation

RANGE = 1000

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
    is_axis = False
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

class Publisher:
    def __init__(self, node, topic, msg, info):
        self.topic = topic
        self.msg = msg()
        self.pub = node.create_publisher(msg, topic, 10)
        self.rpy = {}
        self.node = node
            
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
                info[key].pop('to')
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
            self.write(self.map[tag], values[tag]['val'])
        # write RPY's to Quaternions
        for field in self.rpy:
            if field:
                rsetattr(self.msg, field, quaternion_msg(self.rpy[field]), False)
            else:
                self.msg = quaternion_msg(self.rpy[field])

        # update time if classical stamped msg
        if hasattr(self.msg, 'header'):
            self.write('header.stamp', self.node.get_clock().now().to_msg())        
        elif hasattr(self.msg, 'stamp'):
            self.write('stamp', self.node.get_clock().now().to_msg())  
        self.pub.publish(self.msg)

class SliderPublisher(QWidget):
    def __init__(self, node, content):
        super(SliderPublisher, self).__init__()
        
        content = content.replace('\t', '    ')
                        
        # get message types
        self.publishers = {}
        self.values = {}
        
        self.timer = node.create_timer(0.1, self.publish)
        
        # to keep track of key ordering in the yaml file
        order = []
        old = []
         
        for topic, info in yaml.safe_load(content).items():
            pkg,msg = info['type'].split('/')
            pkg = __import__(pkg, globals(), locals(), ['msg'])
            msg = getattr(pkg.msg, msg)
            self.publishers[topic] = Publisher(node, topic, msg, info)
            order.append((topic,[]))
            for key in info:
                tag = key_tag(topic,key)
                self.values[tag] = info[key]
                order[-1][1].append((content.find(' ' + key + ':'), key))
                old.append((content.find(' ' + key + ':'), key))
                for bound in ['min', 'max']:
                    self.values[tag][bound] = robust_eval(self.values[tag][bound])
                self.values[tag]['val'] = 0
            order[-1][1].sort()
        order.sort(key = lambda x: x[1][0][0])
        # build sliders - thanks joint_state_publisher
        sliderUpdateTrigger = Signal()
        self.vlayout = QVBoxLayout(self)
        self.gridlayout = QGridLayout()
        font = QFont("Helvetica", 9, QFont.Bold)
        topic_font = QFont("Helvetica", 10, QFont.Bold)
        
        sliders = []
        self.key_map = {}
        y = 0
        for topic,keys in order:
            topic_layout = QVBoxLayout()
            label = QLabel(topic)
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
                
                display = QLineEdit("0.00")
                display.setAlignment(Qt.AlignRight)
                display.setFont(font)
                display.setReadOnly(True)
                
                row_layout.addWidget(display)    
                key_layout.addLayout(row_layout)
                
                slider = QSlider(Qt.Horizontal)
                slider.setFont(font)
                slider.setRange(0, RANGE)
                slider.setValue(int(RANGE/2))
                
                key_layout.addWidget(slider)
            
                self.key_map[tag] = {'slidervalue': 0, 'display': display, 'slider': slider}
                slider.valueChanged.connect(self.onValueChanged)
                self.gridlayout.addLayout(key_layout, *(y,0))
                y+=1
                #sliders.append(key_layout)
        
            # Generate positions in grid and place sliders there
            #self.positions = [(y,0) for y in range(len(sliders))]
            #for item, pos in zip(sliders, self.positions):
            #    self.gridlayout.addLayout(item, *pos)
            
        self.vlayout.addLayout(self.gridlayout)            
        
        self.ctrbutton = QPushButton('Center', self)
        self.ctrbutton.clicked.connect(self.center)
        self.vlayout.addWidget(self.ctrbutton)
            
        self.center(1)
        
    def sliderToValue(self, slider, tag):
        val = self.values[tag]
        return val['min'] + slider*(val['max'] - val['min'])/RANGE            
        
    @pyqtSlot(int)
    def onValueChanged(self, event):
        # A slider value was changed, but we need to change the joint_info metadata.
        for key, key_info in self.key_map.items():
            key_info['slidervalue'] = key_info['slider'].value()
            # build corresponding value                        
            self.values[key]['val'] = self.sliderToValue(key_info['slidervalue'], key)    
            key_info['display'].setText("%.2f" % self.values[key]['val'])        
            
    def center(self, event):
        for key, key_info in self.key_map.items():
            key_info['slider'].setValue(int(RANGE/2))
        self.onValueChanged(event)
            
        
    def publish(self):      
        for pub in self.publishers:
            self.publishers[pub].update(self.values)
        
def main(args=None):
    
    print(sys.argv[1:])
       
    rclpy.init(args=args)
    node = rclpy.create_node('slider_publisher')
    
    # read passed param
    filename = len(sys.argv) > 1 and sys.argv[1] or ''
    if not os.path.exists(filename):
        if not node.has_parameter("file"):
            print("Pass a yaml file as argument")
            # TODO why can't parameter be read?
            #filename = '/home/olivier/code/ros2/src/slider_publisher/examples/TwistStamped.yaml'
            sys.exit(0)
        filename = node.get_parameter("file")
        
    # also get order from file
    with open(filename) as f:
        content = f.read()

    # build GUI
    title = node.get_name().split('/')[-1]
    app = QApplication([title.title()])    
    sp = SliderPublisher(node, content)
    #pause
    Thread(target=rclpy.spin, args=(node,)).start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sp.show()
    sys.exit(app.exec_())
            
            
if __name__ == "__main__":
    main()
 

#!/usr/bin/python3
import rclpy
import yaml
import sys
import os
from python_qt_binding.QtWidgets import QApplication, QWidget, QVBoxLayout,QHBoxLayout,QGridLayout, QLabel, QSlider, QLineEdit, QPushButton, QCheckBox, QComboBox
from python_qt_binding.QtCore import Signal, Qt
from python_qt_binding.QtGui import QFont
from threading import Thread
from copy import deepcopy
import signal
from geometry_msgs.msg import Quaternion
from math import pi, cos, sin
from ament_index_python import get_package_share_directory

font = QFont("Helvetica", 9, QFont.Bold)
topic_font = QFont("Helvetica", 10, QFont.Bold)


def rgetattr(obj, attr):
    if attr == '':
        return obj
    return eval(f'obj.{attr}')


def rsetattr(obj, attr, val, cast=True):
    pre, _, post = attr.rpartition('.')
    if pre:
        return setattr(rgetattr(obj, pre), post, val)
    if cast:
        return setattr(obj, post, type(getattr(obj, post))(val))
    else:
        return setattr(obj, post, val)


def quaternion_msg(rpy):

    def cossin(angle):
        return cos(angle), sin(angle)

    cr,sr = cossin(rpy['roll']/2)
    cp,sp = cossin(rpy['pitch']/2)
    cy,sy = cossin(rpy['yaw']/2)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def split_field(key):
    if '.' in key:
        return key.rsplit('.', 1)
    return '', key


def key_tag(topic, key):
    return topic + '/' + key


def get_interface(pkg, interface, name):
    from importlib import import_module
    pkg = import_module(f'{pkg}.{interface}')
    return getattr(pkg, name)


def get_type(msg, key):

    if not key:
        return msg

    if '[' in key:
        key = ''.join(key.replace('[',']').split(']')[::2]).strip('.')
    types = msg.get_fields_and_field_types()
    for field in types:
        if '<' in types[field]:
            types[field] = types[field].split('<')[1][:-1]

    if '.' in key:
        main, nested = key.split('.',1)
        pkg, submsg = types[main].split('/')
        return get_type(get_interface(pkg, 'msg', submsg), nested)

    if key not in types and key in ('roll', 'pitch', 'yaw'):
        return float

    if '/' in types[key]:
        pkg,submsg = types[key].split('/')
        return get_interface(pkg, 'msg', submsg)

    # basic types
    if 'int' in types[key]:
        return int
    elif 'bool' in types[key]:
        return bool
    elif 'string' in types[key]:
        return str
    return float


def isRPY(key, msg):
    field, axis = split_field(key)

    if axis not in ('roll', 'pitch', 'yaw'):
        return False
    elif field:
        return get_type(msg, field) == Quaternion

    return isinstance(msg, Quaternion)


def add_timestamp(msg, node):
    # update time if classical stamped msg
    if hasattr(msg, 'header'):
        rsetattr(msg, 'header.stamp', node.get_clock().now().to_msg())
    elif hasattr(msg, 'stamp'):
        rsetattr(msg, 'stamp', node.get_clock().now().to_msg())


def robust_eval(val, out_type = float):
    if isinstance(val, (list,tuple)):
        return [robust_eval(v, out_type) for v in val]
    if isinstance(val,str):
        val_expr = val.strip().lower()
        
        # check for  Pi fractions
        for sign, sign_rep in ((1, ''), (-1, '-')):

            if val_expr == f'{sign_rep}pi':
                return sign*pi
            
            for denom in range(2, 13):
                if val_expr == f'{sign_rep}pi/{denom}':
                    return sign * pi/denom
                elif val_expr == f'{sign_rep}{denom}pi' or val_expr == f'{sign_rep}{denom}*pi':
                    return sign * pi*denom
    
    return out_type(val)


class Control:

    def __init__(self, msg, info):
        
        self.type = get_type(msg, info['to'])

        if self.type not in (int,float,bool,str):
            msg = str(msg()).split('(')[0].replace('.','/')
            raise AttributeError(f'{msg}: {info["to"]} is not a basic type, cannot create control')

        self.choices = None
        if 'default' in info and isinstance(info['default'], list) and len(info['default']) > 1:
            self.choices = list(map(str,info['default']))
            self.default = self.value = self.eval(self.choices[0])
            return
        
        self.default = self.type()
        if self.type in (int,float):
            if 'min' in info and 'max' in info:
                self.min = self.eval(info['min'])
                self.max = self.eval(info['max'])
                self.range = 1000 if self.type == float else self.max-self.min

                self.default = self.type((self.max+self.min)/2)

                if self.type == float:
                    self.round = 2
                    for bound in ('min', 'max'):
                        val = getattr(self, bound)
                        if val == 0:
                            continue
                        from math import log10, ceil
                        self.round = max(self.round, int(ceil(-log10(abs(val)))+2))
            else:
                self.range = None

        self.default = self.value = self.eval(info['default']) if 'default' in info else self.default

    def eval(self, val):
        return robust_eval(val, self.type)
    
    def reset(self, value = None):
        if value is None:
            value = self.default

        if self.choices:
            self.cbox.setCurrentIndex(0)
            self.value = value
            return
        if self.type == bool:
            self.box.setChecked(value)
            return
        elif self.type == str:
            self.text.setText(value)
            return
        elif self.range is None:
            self.display.setText(str(value))
            self.value = value
            return
        elif self.type == float:
            slider_val = (value-self.min)/(self.max-self.min)*self.range
        else:
            slider_val = value-self.min
        self.slider.setValue(int(slider_val))

    def refresh(self, event = None):

        if self.choices:
            self.value = self.eval(self.choices[self.cbox.currentIndex()])
        elif self.type == bool:
            self.value = self.box.isChecked()
        elif self.type == str:
            self.value = self.text.text()
        elif event is None or self.range is None:
            # text was used, update slider
            try:
                new_val = self.type(float(self.display.text()))
                if self.range is None:
                    self.reset(new_val)
                else:
                    self.reset(min(self.max, max(self.min, new_val)))
            except ValueError:
                self.display.setText(f'{round(self.value, self.round)}')
        else:
            # slider was used, update text
            slider_val = self.slider.value()
            if self.type == float:
                self.value = self.min + slider_val*(self.max - self.min)/self.range
                self.display.setText(f'{round(self.value, self.round)}')
            else:
                self.value = self.min + slider_val
                self.display.setText(f'{self.value}')

    def init_layout(self, label):

        label = QLabel(label)
        label.setFont(font)
        key_layout = QVBoxLayout()
        row_layout = QHBoxLayout()
        row_layout.addWidget(label)

        if self.choices:
            self.cbox = QComboBox()
            self.cbox.currentIndexChanged.connect(self.refresh)
            self.cbox.setFont(font)
            self.cbox.addItems(self.choices)
            row_layout.addWidget(self.cbox)
            key_layout.addLayout(row_layout)

        elif self.type in (int,float):

            self.display = QLineEdit()
            self.display.setAlignment(Qt.AlignRight)
            self.display.setFont(font)
            self.display.returnPressed.connect(self.refresh)

            row_layout.addWidget(self.display)
            key_layout.addLayout(row_layout)
            if self.range is not None:
                self.slider = QSlider(Qt.Horizontal)
                self.slider.setFont(font)
                self.slider.setRange(0, self.range)
                self.slider.valueChanged.connect(self.refresh)
                key_layout.addWidget(self.slider)

        elif self.type == bool:
            self.box = QCheckBox()
            self.box.clicked.connect(self.refresh)
            row_layout.addWidget(self.box)
            key_layout.addLayout(row_layout)
        else:
            # text
            self.text = QLineEdit()
            self.text.setAlignment(Qt.AlignLeft)
            self.text.setFont(font)
            self.text.returnPressed.connect(self.refresh)
            row_layout.addWidget(self.text)
            key_layout.addLayout(row_layout)

        self.reset()
        return key_layout
    

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
            self.prev = msg.Request()
            self.pub = None
            self.client = Publisher.node.create_client(msg, topic)
            Publisher.node.get_logger().info('Waiting for service ' + topic)
            self.client.wait_for_service()
                    
        self.rpy = {}
            
        # init map from GUI to message
        self.map = {}
        to_remove = []
        for label in info:

            tag = key_tag(topic, label)
            if type(info[label]) == dict:

                if 'to' not in info[label]:
                    fields = tuple(self.msg.get_fields_and_field_types().keys())
                    if len(fields) == 1:
                        info[label]['to'] = fields[0]
                        Publisher.node.get_logger().info(f'Using "{fields[0]}" as destination only field for "{label}" on {topic}')
                    else:
                        info[label]['to'] = label
                        Publisher.node.get_logger().info(f'Using "{label}" as destination field for "{label}" on {topic}')

                if isRPY(info[label]['to'], self.msg):
                    field, axis = split_field(info[label]['to'])
                    if field not in self.rpy:
                        self.rpy[field] = {'roll': 0, 'pitch': 0, 'yaw': 0}
                self.map[tag] = info[label]['to']
 
            else:
                if label != 'type':
                    # init non-zero defaults
                    if isRPY(label, self.msg):
                        field, axis = split_field(label)
                        if field not in self.rpy:
                            self.rpy[field] = {'roll': 0, 'pitch': 0, 'yaw': 0}
                        self.rpy[field][axis] = robust_eval(info[label])
                    else:
                        self.write(label, robust_eval(info[label], get_type(self.msg, label)))
                to_remove.append(label)
        for rm in to_remove:
            info.pop(rm)

    def write(self, key, val):

        field, axis = split_field(key)
        if field in self.rpy:
            self.rpy[field][axis] = val
        elif '[' in key:
            field, idx = key.split('[')
            idx, subfield = idx.split(']')
            idx = int(idx)
            current = rgetattr(self.msg, field)

            if len(current) <= idx:
                # identify what should go there
                field_type = get_type(self.msg, field)
                while len(current) <= idx:
                    current.append(field_type())

            if subfield:
                rsetattr(current[idx], subfield[1:], val, False)

            else:
                current[idx] = val
            add_timestamp(current[idx], Publisher.node)
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
        add_timestamp(self.msg, Publisher.node)
                   
        if self.pub is not None:
            self.pub.publish(self.msg)
        elif self.msg != self.prev:
            # service call, dont care about the result
            self.client.call_async(self.msg)
            self.prev = deepcopy(self.msg)


class SliderPublisher(QWidget):
    def __init__(self, node, filename):
        
        from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
        
        super(SliderPublisher, self).__init__()
        
        if filename is None:
            # no raw argument, should be set as parameter
            filename = node.declare_parameter('config', '__').value
            if not os.path.exists(filename):
                node.get_logger().error('No configuration file given, give its path as an argument or use the `config` parameter')
                sys.exit(0)
                
        with open(filename) as f:
            content = f.read().replace('\t', '    ')
        
        self.running = True
        self.node = node
                        
        # get message types
        Publisher.node = node
        self.publishers = {}
        self.controls = {}
        
        rate_param = ParameterDescriptor(name = 'rate',
                    floating_point_range = [FloatingPointRange(
                        from_value = 0.01,
                        to_value = 100.)])
        rate = node.declare_parameter(rate_param.name, 10., rate_param).value
        
        self.timer = node.create_timer(1./rate, self.publish)
        
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
                
                # identify key type -> slider (continuous or discrete) / checkbox / text
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
            y += 1
            for idx,key in keys:
                self.gridlayout.addLayout(self.controls[key_tag(topic,key)].init_layout(key),
                                          *(y,0))
                y += 1
            
        self.vlayout.addLayout(self.gridlayout)
        
        self.reset_button = QPushButton('Reset', self)
        self.reset_button.clicked.connect(self.reset)
        self.vlayout.addWidget(self.reset_button)

    def reset(self, event):
        for control in self.controls.values():
            control.reset()
            control.refresh(event)
                    
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
        filename = None
        
    # build GUI
    full_namespace = '{}/{}'.format(node.get_namespace().strip('/'), node.get_name())
    app = QApplication([full_namespace])
    sp = SliderPublisher(node, filename)
    
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
 

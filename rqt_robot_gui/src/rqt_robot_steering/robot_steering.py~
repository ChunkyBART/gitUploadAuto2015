# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division
import os
import rospkg

from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import String
import rospy
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Slot
from python_qt_binding.QtGui import QKeySequence, QShortcut, QWidget
from rqt_gui_py.plugin import Plugin


class RobotSteering(Plugin):
    mode = True
    modesensor = True
    slider_factor = 1000.0
    qr_data = "A"
    def __init__(self, context):
        super(RobotSteering, self).__init__(context)
        self.setObjectName('RobotSteering')
      
        self._publisher = None
        self._publishersys = None
        self._widget = QWidget()

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_robot_steering'), 'resource', 'RobotSteering.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RobotSteeringUi')
        if context.serial_number() > 1:
           self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
        self._publisher = rospy.Publisher('speed', Int16,queue_size=10)
	self._publisherth = rospy.Publisher('set_heat', Int16,queue_size=10)
        self._publishersys = rospy.Publisher('syscommand',String,queue_size=10)
	self._publisherauto = rospy.Publisher('autocommand',String,queue_size=10)
        self._sub = rospy.Subscriber("temperature", Int16, self.heat_callback)
        self._subco = rospy.Subscriber("co2", Int16, self.co_callback)
        self._subbat = rospy.Subscriber("autocommand", String, self.auto_callback)
        self._subqrr = rospy.Subscriber("/qrdata", String, self.qrr_callback)
	self._subspeed = rospy.Subscriber("speedrobot",String,self.speed_callback)
	self._submotion = rospy.Subscriber("/image_motion",String,self.motion_callback)
	self._subhazard = rospy.Subscriber("/image_hazard",String,self.hazard_callback)
	self._substatus = rospy.Subscriber("status_robot",String,self.status_callback)
      #  self._widget.topic_line_edit.textChanged.connect(self._on_topic_changed)
       # self._widget.manual_push_button.pressed.connect(self._on_manual_pressed)

        self._widget.x_linear_slider.valueChanged.connect(self._on_x_linear_slider_changed)
	self._widget.heat_linear_slider.valueChanged.connect(self._on_heat_linear_slider_changed)
      #  self._widget.z_angular_slider.valueChanged.connect(self._on_z_angular_slider_changed)
  	
       # self._widget.increase_x_linear_push_button.pressed.connect(self._on_strong_increase_x_linear_pressed)
        #self._widget.reset_x_linear_push_button.pressed.connect(self._on_reset_x_linear_pressed)
        self._widget.reset_push_button.pressed.connect(self._on_reset_map_pressed)
	self._widget.start_push_button.pressed.connect(self._on_start_pressed)
	self._widget.mode_push_button.pressed.connect(self._on_mode_pressed)
	self._widget.radio_push_button.pressed.connect(self._on_radio_pressed)
	self._widget.comfirm_button.pressed.connect(self._on_confirm_pressed)
	self._widget.ignor_button.pressed.connect(self._on_ignor_pressed)
	self._widget.sensor_button.pressed.connect(self._on_sensor_pressed)
	self._widget.approach_victim_button.pressed.connect(self._on_approach_pressed)
	self._widget.check_sensor_button.pressed.connect(self._on_check_sensor_pressed)
        #self._widget.decrease_x_linear_push_button.pressed.connect(self._on_strong_decrease_x_linear_pressed)
        #self._widget.increase_z_angular_push_button.pressed.connect(self._on_strong_increase_z_angular_pressed)
        #self._widget.reset_z_angular_push_button.pressed.connect(self._on_reset_z_angular_pressed)
        #self._widget.decrease_z_angular_push_button.pressed.connect(self._on_strong_decrease_z_angular_pressed)

        #self._widget.max_x_linear_double_spin_box.valueChanged.connect(self._on_max_x_linear_changed)
        #self._widget.min_x_linear_double_spin_box.valueChanged.connect(self._on_min_x_linear_changed)
       # self._widget.max_z_angular_double_spin_box.valueChanged.connect(self._on_max_z_angular_changed)
        #self._widget.min_z_angular_double_spin_box.valueChanged.connect(self._on_min_z_angular_changed)

        #self.shortcut_w = QShortcut(QKeySequence(Qt.Key_W), self._widget)
        #self.shortcut_w.setContext(Qt.ApplicationShortcut)
        #self.shortcut_w.activated.connect(self._on_increase_x_linear_pressed)
        #self.shortcut_x = QShortcut(QKeySequence(Qt.Key_X), self._widget)
        ##self.shortcut_x.setContext(Qt.ApplicationShortcut)
        #self.shortcut_x.activated.connect(self._on_reset_x_linear_pressed)
        #self.shortcut_s = QShortcut(QKeySequence(Qt.Key_S), self._widget)
        #self.shortcut_s.setContext(Qt.ApplicationShortcut)
        #self.shortcut_s.activated.connect(self._on_decrease_x_linear_pressed)
        #self.shortcut_a = QShortcut(QKeySequence(Qt.Key_A), self._widget)
        #self.shortcut_a.setContext(Qt.ApplicationShortcut)
        #self.shortcut_a.activated.connect(self._on_increase_z_angular_pressed)
        #self.shortcut_z = QShortcut(QKeySequence(Qt.Key_Z), self._widget)
        #self.shortcut_z.setContext(Qt.ApplicationShortcut)
        #self.shortcut_z.activated.connect(self._on_reset_z_angular_pressed)
        #self.shortcut_d = QShortcut(QKeySequence(Qt.Key_D), self._widget)
        #self.shortcut_d.setContext(Qt.ApplicationShortcut)
        #self.shortcut_d.activated.connect(self._on_decrease_z_angular_pressed)

        #self.shortcut_shift_w = QShortcut(QKeySequence(Qt.SHIFT + Qt.Key_W), self._widget)
        ##self.shortcut_shift_w.setContext(Qt.ApplicationShortcut)
        #elf.shortcut_shift_w.activated.connect(self._on_strong_increase_x_linear_pressed)
        #self.shortcut_shift_x = QShortcut(QKeySequence(Qt.SHIFT + Qt.Key_X), self._widget)
        ##self.shortcut_shift_x.setContext(Qt.ApplicationShortcut)
        #self.shortcut_shift_x.activated.connect(self._on_reset_x_linear_pressed)
        #self.shortcut_shift_s = QShortcut(QKeySequence(Qt.SHIFT + Qt.Key_S), self._widget)
        #self.shortcut_shift_s.setContext(Qt.ApplicationShortcut)
        #self.shortcut_shift_s.activated.connect(self._on_strong_decrease_x_linear_pressed)
        #self.shortcut_shift_a = QShortcut(QKeySequence(Qt.SHIFT + Qt.Key_A), self._widget)
        #self.shortcut_shift_a.setContext(Qt.ApplicationShortcut)
        #self.shortcut_shift_a.activated.connect(self._on_strong_increase_z_angular_pressed)
        #self.shortcut_shift_z = QShortcut(QKeySequence(Qt.SHIFT + Qt.Key_Z), self._widget)
        #self.shortcut_shift_z.setContext(Qt.ApplicationShortcut)
        #self.shortcut_shift_z.activated.connect(self._on_reset_z_angular_pressed)
        #self.shortcut_shift_d = QShortcut(QKeySequence(Qt.SHIFT + Qt.Key_D), self._widget)
        #self.shortcut_shift_d.setContext(Qt.ApplicationShortcut)
        #self.shortcut_shift_d.activated.connect(self._on_strong_decrease_z_angular_pressed)

        self.shortcut_space = QShortcut(QKeySequence(Qt.Key_Space), self._widget)
        self.shortcut_space.setContext(Qt.ApplicationShortcut)
       # self.shortcut_space.activated.connect(self._on_manual_pressed)
        self.shortcut_space = QShortcut(QKeySequence(Qt.SHIFT + Qt.Key_Space), self._widget)
        self.shortcut_space.setContext(Qt.ApplicationShortcut)
      #  self.shortcut_space.activated.connect(self._on_manual_pressed)

    #    self._widget.stop_push_button.setToolTip(self._widget.stop_push_button.toolTip() + ' ' + self.tr('([Shift +] Space)'))
        #self._widget.increase_x_linear_push_button.setToolTip(self._widget.increase_x_linear_push_button.toolTip() + ' ' + self.tr('([Shift +] W)'))
        #self._widget.reset_x_linear_push_button.setToolTip(self._widget.reset_x_linear_push_button.toolTip() + ' ' + self.tr('([Shift +] X)'))
        #self._widget.decrease_x_linear_push_button.setToolTip(self._widget.decrease_x_linear_push_button.toolTip() + ' ' + self.tr('([Shift +] S)'))
      #  self._widget.increase_z_angular_push_button.setToolTip(self._widget.increase_z_angular_push_button.toolTip() + ' ' + self.tr('([Shift +] A)'))
      #  self._widget.reset_z_angular_push_button.setToolTip(self._widget.reset_z_angular_push_button.toolTip() + ' ' + self.tr('([Shift +] Z)'))
      #  self._widget.decrease_z_angular_push_button.setToolTip(self._widget.decrease_z_angular_push_button.toolTip() + ' ' + self.tr('([Shift +] D)'))

        # timer to consecutively send twist messages
       # self._update_parameter_timer = QTimer(self)
       # self._update_parameter_timer.timeout.connect(self._on_parameter_changed)
      #  self._update_parameter_timer.start(100)
        self.zero_cmd_sent = False

    @Slot(str)
   # def _on_topic_changed(self, topic):
       # topic = str(topic)
       # self._unregister_publisher()
       # self._publisher = rospy.Publisher(topic, Twist)
       # self._publisher = rospy.Publisher(topic, Int16)
       # self._sub = rospy.Subscriber("tempeture", Int16, self.heat_callback)
      #  self._subco = rospy.Subscriber("co", Int16, self.co_callback)

    def auto_callback(self, data):
	#self._widget.textEdit.append('autocall')  
	str = String() 
        if data.data == "detectC":
	   self._widget.victim_label.setText('Found Hole')
           self._widget.current_z_angular_label.setEnabled(True)
	   self._widget.label.setEnabled(True)
	  # self._widget.groupBox.setEnabled(True)
	   self._widget.comfirm_button.setEnabled(True)
	   self._widget.approach_victim_button.setEnabled(True)
	   self._widget.ignor_button.setEnabled(True)
           str.data = "stop"
           self._publishersys.publish(str)
   	if data.data == "detectT":
	   self._widget.victim_label.setText('Found Temp')
           self._widget.current_z_angular_label.setEnabled(True)
	   self._widget.label.setEnabled(True)
	  # self._widget.groupBox.setEnabled(True)
	   self._widget.comfirm_button.setEnabled(True)
	   self._widget.approach_victim_button.setEnabled(True)
	   self._widget.ignor_button.setEnabled(True)
	if data.data == "detectQ":
	   self._widget.victim_label.setText('QR Found')
           self._widget.current_z_angular_label.setEnabled(True)
	   self._widget.label.setEnabled(True)
    def speed_callback(self,data):
	self._widget.SP_R.setText('%s' % data.data)
    def status_callback(self,data):
	self._widget.robot_label.setText('%s' % data.data)
    def qrr_callback(self, data):
       #  self._widget.label_3.setText('%s' % data.data)
         self._widget.textEdit.append('%s' % data.data)     
    def motion_callback(self, data):
	self._widget.motion.setText('%s' % data.data)
	self._widget.motion.setEnabled(True)
    def hazard_callback(self, data):
	self._widget.hazard.setText('%s' % data.data)
	self._widget.hazard.setEnabled(True)
    def co_callback(self, data):
        self._widget.CO2.setText('%d ppm' % data.data)
	self._widget.CO2.setEnabled(True)
        self._on_parameter_changed()

    def heat_callback(self, data):
        self._widget.current_z_angular_label.setText('%d C' % data.data)
        self._on_parameter_changed()

    def _on_manual_pressed(self):
        self._widget.x_linear_slider.setValue(0)
      #  self._widget.z_angular_slider.setValue(0)

    def _on_x_linear_slider_changed(self):
        self._widget.current_x_linear_label.setText('%0.2f m/s' % self._widget.x_linear_slider.value() )
        self._on_parameter_changed()
    def _on_heat_linear_slider_changed(self):
        self._widget.current_heat_linear_label.setText('%0.2f C' % self._widget.heat_linear_slider.value() )
   	self._on_heat_changed()
    def _on_approach_pressed(self):
	str = String()
	str.data = "getposition"
	self._publishersys.publish(str)
	d = rospy.Duration(0.2,0)
	rospy.sleep(d)
	str.data = "approach_victim"
	self._publishersys.publish(str)
	self._widget.victim_label.setText('Approach Victim..')
    def _on_left_pressed(self):
	str = String()
	str.data = "left_path_gen"
	self._publishersys.publish(str)
    def _on_right_pressed(self):	
	str = String()
	str.data = "right_path_gen"
	self._publishersys.publish(str)
    def	_on_check_sensor_pressed(self):
	str = String()
	str.data = "check_sensor"
	self._publishersys.publish(str)
	self._widget.victim_label.setText('Check SS')
	self._widget.CO2.setText('0')
	self._widget.current_z_angular_label.setText('0')
    def _on_reset_map_pressed(self):
        str = String()
        str.data = "reset"
        self._publishersys.publish(str)
    def _on_start_pressed(self):
        str = String()
        str.data = "start_walk"
        self._publishersys.publish(str)
    def _on_mode_pressed(self):
	str = String()
	if RobotSteering.mode :
		self._widget.mode_push_button.setText('Auto_Mode')
		RobotSteering.mode = False
		str.data = "teleop_mode"
	else :
		self._widget.mode_push_button.setText('Teleop_Mode')
		RobotSteering.mode = True
		str.data = "auto_mode"
        self._publishersys.publish(str)
    def _on_radio_pressed(self):
	str = String()
	str.data="radio_mode"
	self._publishersys.publish(str)
        self._widget.robot_label.setText('RADIO GOGO GOODLUCK')
    def _on_confirm_pressed(self):
        str = String()
        str.data = "plot_map"
        self._publishersys.publish(str)
	d = rospy.Duration(0.2, 0)
    	rospy.sleep(d)
	str.data = "finish" ############# plottt mappp ##################
        self._publishersys.publish(str)
	d = rospy.Duration(0.2, 0)
    	rospy.sleep(d)
	str.data = "go"
        self._publishersys.publish(str)
	self._widget.motion.setEnabled(False)
	self._widget.eye_chart.setEnabled(False)
	self._widget.hazard.setEnabled(False)
	self._widget.CO2.setEnabled(False)
	#self._widget.groupBox.setEnabled(False)
	self._widget.comfirm_button.setEnabled(True)
	self._widget.approach_victim_button.setEnabled(True)
	self._widget.ignor_button.setEnabled(True)
	self._widget.current_z_angular_label.setEnabled(False)
	self._widget.label.setEnabled(False)
	self._widget.victim_label.setText('Searching...')
    def _on_ignor_pressed(self):
        str = String()
        str.data = "ignor"
        self._publishersys.publish(str)
	d = rospy.Duration(0.5,0)
	rospy.sleep(d)
	str.data = "go"
        self._publishersys.publish(str)
	#self._widget.groupBox.setEnabled(False)
	self._widget.comfirm_button.setEnabled(True)
	self._widget.approach_victim_button.setEnabled(True)
	self._widget.ignor_button.setEnabled(True)
	self._widget.current_z_angular_label.setEnabled(False)
	self._widget.label.setEnabled(False)
	self._widget.victim_label.setText('Searching...')
    def _on_sensor_pressed(self):
        str = String()
	if RobotSteering.modesensor :
		self._widget.sensor_button.setText('Stop_Sensor')
		RobotSteering.modesensor = False
		str.data = "start_sensor"
	else :
		self._widget.sensor_button.setText('Start_Sensor')
		RobotSteering.modesensor = True
		str.data = "stop_sensor"
        self._publishersys.publish(str)
#    def _on_increase_x_linear_pressed(self):
 #       self._widget.x_linear_slider.setValue(self._widget.x_linear_slider.value() + self._widget.x_linear_slider.singleStep())

  #  def _on_reset_x_linear_pressed(self):
   #     self._widget.x_linear_slider.setValue(50)

    #def _on_decrease_x_linear_pressed(self):
     #   self._widget.x_linear_slider.setValue(self._widget.x_linear_slider.value() - self._widget.x_linear_slider.singleStep())

    #def _on_increase_z_angular_pressed(self):
     #   self._widget.z_angular_slider.setValue(self._widget.z_angular_slider.value() + self._widget.z_angular_slider.singleStep())

#    def _on_reset_z_angular_pressed(self):
 #       self._widget.z_angular_slider.setValue(0)

   # def _on_decrease_z_angular_pressed(self):
   #     self._widget.z_angular_slider.setValue(self._widget.z_angular_slider.value() - self._widget.z_angular_slider.singleStep())

 #   def _on_max_x_linear_changed(self, value):
        #self._widget.x_linear_slider.setMaximum(value)
#
 #   def _on_min_x_linear_changed(self, value):
  #      self._widget.x_linear_slider.setMinimum(value)

 #   def _on_max_z_angular_changed(self, value):
  #      self._widget.z_angular_slider.setMaximum(value * RobotSteering.slider_factor)

   # def _on_min_z_angular_changed(self, value):
    #    self._widget.z_angular_slider.setMinimum(value * RobotSteering.slider_factor)

   # def _on_strong_increase_x_linear_pressed(self):
    #    self._widget.x_linear_slider.setValue(self._widget.x_linear_slider.value() + self._widget.x_linear_slider.pageStep())

   # def _on_strong_decrease_x_linear_pressed(self):
    #    self._widget.x_linear_slider.setValue(self._widget.x_linear_slider.value() - self._widget.x_linear_slider.pageStep())

  #  def _on_strong_increase_z_angular_pressed(self):
   #     self._widget.z_angular_slider.setValue(self._widget.z_angular_slider.value() + self._widget.z_angular_slider.pageStep())

    #def _on_strong_decrease_z_angular_pressed(self):
     #   self._widget.z_angular_slider.setValue(self._widget.z_angular_slider.value() - self._widget.z_angular_slider.pageStep())

    def _on_parameter_changed(self):
        self._send_twist(self._widget.x_linear_slider.value())
    def _on_heat_changed(self):
	self._send_heat(self._widget.heat_linear_slider.value())
    def _send_heat(self, heat):
	h = Int16()
        h.data = heat
	self._publisherth.publish(h)
	str = String()
        str.data = "setheat"
        self._publisherauto.publish(str)
    def _send_twist(self, x_linear):
        if self._publisher is None:
            return
       # twist = Twist()
        #twist.linear.x = x_linear
        #twist.linear.y = 0
        #twist.linear.z = 0
        #twist.angular.x = 0
        #twist.angular.y = 0
        #twist.angular.z = z_angular
        speed = Int16()
        speed.data = x_linear
        # Only send the zero command once so other devices can take control
        if x_linear == 0:
            if not self.zero_cmd_sent:
                self.zero_cmd_sent = True
#                self._publisher.publish(twist)
                self._publisher.publish(speed)
        else:
            self.zero_cmd_sent = False
       #     self._publisher.publish(twist)
        self._publisher.publish(speed)

    def _unregister_publisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

    def shutdown_plugin(self):
       # self._update_parameter_timer.stop()
        self._unregister_publisher()


        
        
        
        
        
        
        
        

#!/usr/bin/env python
import sys
import reflow_profile
import controller
import delay_model
import scipy
import time
from PyQt4 import QtCore, QtGui
import PyQt4.Qwt5 as Qwt
from reflow_gui import Ui_MainWindow 

TIMER_INTERVAL = 1000
SIMULATION_SPEEDUP = 1.0
PGAIN = 4.0
IGAIN = 0.0

class MyApp(QtGui.QMainWindow):

    def __init__(self, parent=None, device=None):
        self.state = 'stopped'
        self.t0 = None
        self.t_stop = None
        self.t_offset = 0.0

        # Set up device
        self.dev = device
        self.dev.set_power(0)
        self.dev.set_clock(self)
        temp = self.dev.get_therm_value()

        # Create set point function
        self.reflow_profile = reflow_profile.Reflow_Profile(start_T=temp)
        stop_t = self.reflow_profile.stop_time()

        # Setup controller
        self.model = delay_model.Therm_Delay_Model(T_amb=temp)
        self.model.load_param('model_param.pkl')
        self.pgain = PGAIN 
        self.igain = IGAIN 
        self.controller = controller.PI_Controller(
            self.reflow_profile.func, 
            self.pgain,
            self.igain,
            ff_func = self.model.ctl_for_steady_state
        )

        # Set Gui main window
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Connect buttons and checkbox
        self.connect(self.ui.startButton, QtCore.SIGNAL('clicked()'), self.on_start_clicked)
        self.connect(self.ui.stopButton, QtCore.SIGNAL('clicked()'), self.on_stop_clicked)
        self.connect(self.ui.resetButton, QtCore.SIGNAL('clicked()'), self.on_reset_clicked)
        self.connect(self.ui.controlCheckbox, QtCore.SIGNAL('clicked()'), self.on_control_clicked)

        # Set up event timer
        self.timer = QtCore.QTimer()
        self.connect(self.timer, QtCore.SIGNAL('timeout()'), self.on_timer_update)
        self.timer.setInterval(TIMER_INTERVAL / SIMULATION_SPEEDUP)
        self.timer.start()

        # Set lcd display
        self.update_lcd(temp)
        
        # Set up plot - may want to autoset x axis scale
        print stop_t
        self.ui.tempPlot.setCanvasBackground(QtCore.Qt.black)
        self.ui.tempPlot.setAxisTitle(Qwt.QwtPlot.xBottom, '(sec)')
        self.ui.tempPlot.setAxisTitle(Qwt.QwtPlot.yLeft, '(C)')
        self.ui.tempPlot.setAxisScale(Qwt.QwtPlot.xBottom, 0, 500, 100)
        self.ui.tempPlot.setAxisScale(Qwt.QwtPlot.yLeft, 0, 300, 100)

        # Setup setpoint curve
        self.setp_curve = Qwt.QwtPlotCurve('')
        self.setp_curve.setRenderHint(Qwt.QwtPlotItem.RenderAntialiased)
        self.pen = QtGui.QPen(QtGui.QColor('limegreen'))
        self.pen.setWidth(2)
        self.setp_curve.setPen(self.pen)
        self.setp_curve.attach(self.ui.tempPlot)
        t = scipy.linspace(0.0, stop_t, 1000)
        T = scipy.array([self.reflow_profile.func(tt-self.model.t_delay) for tt in t])
        self.setp_curve.setData(t, T)
        self.ui.tempPlot.replot()

        # Set data curve 
        self.data_curve = Qwt.QwtPlotCurve('')
        self.data_curve.setRenderHint(Qwt.QwtPlotItem.RenderAntialiased)
        self.pen = QtGui.QPen(QtGui.QColor('red'))
        self.pen.setWidth(2)
        self.data_curve.setPen(self.pen)
        self.data_curve.attach(self.ui.tempPlot)

        self.update_status()

        # Lists for time and temp samples
        self.temp_samples = []
        self.time_samples = []

    def on_control_clicked(self):
        print 'control', self.control_is_on()
        self.update_mode()

    def update_mode(self):
        if self.control_is_on()==True and self.state=='running':
            pass
        else:
            self.dev.set_power(0)

    def on_start_clicked(self):
        print 'start'
        if self.t0 == None:
            self.t0 = time.time()
        if self.t_stop != None:
            self.t_offset += time.time() - self.t_stop
        t = self.get_time()
        if t < self.reflow_profile.stop_time():
            self.state = 'running'
            self.update_status()
            self.update_mode()

    def on_stop_clicked(self):
        print 'stop'
        self.state = 'stopped'
        self.t_stop = time.time()
        self.update_status()
        self.update_mode()

    def on_reset_clicked(self):
        print 'reset'
        if self.state == 'stopped':
            self.t0 = None
        else:
            self.t0 = time.time() 
        self.t_stop = None
        self.t_offset = 0.0
        self.temp_samples = []
        self.time_samples = []
        self.update_plot()

    def on_timer_update(self):
        temp = self.dev.get_therm_value()
        self.update_lcd(temp)
        print 'state: %s, ctl: %s, temp: %f'%(self.state, self.control_is_on(),temp)
        if self.state == 'running':
            t = self.get_time()
            self.time_samples.append(t)
            self.temp_samples.append(temp)
            self.update_plot()
            if self.control_is_on():
                pwm_value = self.controller.func(t,temp)
                self.dev.set_power(pwm_value)
                print 'pwm_value:', pwm_value
            if t > self.reflow_profile.stop_time():
                self.state = 'stopped'
                self.update_mode()
                self.update_status()

    def update_plot(self):
        self.data_curve.setData(self.time_samples, self.temp_samples)
        self.ui.tempPlot.replot()

    def update_lcd(self,temp):
        self.ui.lcdTemp.display(num2tempstr(temp))

    def update_status(self):
        self.ui.statusbar.showMessage(self.state)

    def closeEvent(self,event):
        print 'closing device'
        self.dev.set_power(0)
        self.dev.close()

    def control_is_on(self):
        return self.ui.controlCheckbox.isChecked()

    def get_time(self):
        if self.t0 is None:
            return 0
        return (time.time() - self.t0 - self.t_offset) * SIMULATION_SPEEDUP


def num2tempstr(val):
    return '%1.0f'%(val,)


if __name__ == "__main__":

    app = QtGui.QApplication(sys.argv)
    dev_plugin = "reflow_device"
    if len(sys.argv) > 1:
        dev_plugin = sys.argv[1]
    reflow = __import__(dev_plugin)
    dev = reflow.ReflowDevice()
    myapp = MyApp(device=dev)
    myapp.show()
    sys.exit(app.exec_())

import sys
from PyQt4 import QtCore, QtGui
import PyQt4.Qwt5 as Qwt
from reflow_gui import Ui_MainWindow 

class MyApp(QtGui.QMainWindow):

    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.connect(self.ui.startButton, QtCore.SIGNAL('clicked()'), self.on_start_clicked)
        self.connect(self.ui.stopButton, QtCore.SIGNAL('clicked()'), self.on_stop_clicked)
        self.connect(self.ui.resetButton, QtCore.SIGNAL('clicked()'), self.on_reset_clicked)
        self.connect(self.ui.controlCheckbox, QtCore.SIGNAL('clicked()'), self.on_control_clicked)

        self.timer = QtCore.QTimer()
        self.connect(self.timer, QtCore.SIGNAL('timeout()'), self.on_timer_update)
        self.timer.setInterval(100)
        self.temp = 0
        self.ui.lcdTemp.display(num2tempstr(self.temp))
        self.state = 'up'
        self.ui.statusbar.showMessage('Stopped')
        
        # Set up plot
        self.ui.tempPlot.setCanvasBackground(QtCore.Qt.black)
        self.ui.tempPlot.setAxisTitle(Qwt.QwtPlot.xBottom, '(sec)')
        self.ui.tempPlot.setAxisTitle(Qwt.QwtPlot.yLeft, '(C)')
        self.ui.tempPlot.setAxisScale(Qwt.QwtPlot.xBottom, 0, 500, 100)
        self.ui.tempPlot.setAxisScale(Qwt.QwtPlot.yLeft, 0, 300, 100)

        # Set up line
        self.curve = Qwt.QwtPlotCurve('')
        self.curve.setRenderHint(Qwt.QwtPlotItem.RenderAntialiased)
        #self.pen = QtGui.QPen(QtGui.QColor('limegreen'))
        self.pen = QtGui.QPen(QtGui.QColor('red'))
        self.pen.setWidth(2)
        self.curve.setPen(self.pen)
        self.curve.attach(self.ui.tempPlot)

        self.temp_samples = []
        self.time_samples = []
        self.count = 0

    def on_control_clicked(self):
        print 'control', self.ui.controlCheckbox.isChecked()

    def on_start_clicked(self):
        self.timer.start()
        self.ui.statusbar.showMessage('Running')
        print 'start'

    def on_stop_clicked(self):
        self.ui.statusbar.showMessage('Stopped')
        self.timer.stop()
        print 'stop'

    def on_reset_clicked(self):
        print 'reset'
        self.temp = 0
        self.count = 0
        self.state = 'up'
        self.ui.lcdTemp.display(num2tempstr(self.temp))
        self.temp_samples = []
        self.time_samples = []
        self.update_plot()

    def on_timer_update(self):
        if self.state == 'up': 
            self.temp += 1
        else:
            self.temp -= 1

        if self.temp <= 0:
            self.state = 'up'
        if self.temp >= 300:
            self.state = 'down'

        self.ui.lcdTemp.display(num2tempstr(self.temp))

        self.count += 0.5
        self.time_samples.append(self.count)
        self.temp_samples.append(self.temp)
        self.update_plot()

    def update_plot(self):
        self.curve.setData(self.time_samples, self.temp_samples)
        self.ui.tempPlot.replot()





def num2tempstr(val):
    return '%1.0f'%(val,)

if __name__ == "__main__":

    app = QtGui.QApplication(sys.argv)
    myapp = MyApp()
    myapp.show()
    sys.exit(app.exec_())

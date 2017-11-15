#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

import sys
import glob
import csv
import numpy as np
import time

import serial
from serial import SerialException
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtGui import QPalette, QIcon
from opendaq import DAQ
from opendaq.models import DAQModel
from opendaq.common import LengthError

from . import daq_control
from . import config
from .widgets import NavigationToolbar
from . import axes_op

BUFFER_SIZE = 400


def list_serial_ports():
    #  Serial port names
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')
    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except SerialException:
            pass
    return result


class MyApp(QtWidgets.QMainWindow, daq_control.Ui_mainWindow):
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        self.names = ['AGND', 'A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7', 'A8', 'VREF']
        self.stop = 0
        self.cfg = QtCore.QSettings('opendaq')
        port_opendaq = str(self.cfg.value('port'))
        try:
            self.daq = DAQ(port_opendaq)
        except (LengthError, SerialException):
            port_opendaq = ''
        self.tabWidget.setEnabled(bool(port_opendaq))
        if port_opendaq:
            self.get_cb_values()
        #  Toolbar
        nav = NavigationToolbar(self.plotWidget.canvas, self.plotWidget.canvas)
        nav.setVisible(False)
        try:
            self.statusBar.showMessage("Hardware Version: %s   Firmware Version: %s" % (self.daq.hw_ver[1], self.daq.fw_ver))
        except AttributeError:
            pass
        icons = [":/resources/house.png", ":/resources/pan.png", ":/resources/zoom.png",
                 ":/resources/save.png"]
        actions_names = ['Home', 'Pan', 'Zoom', 'Save']
        for action in nav.actions():
            if action.text() in actions_names:
                self.toolBar.addAction(action)
        for i, action in enumerate(self.toolBar.actions()[4:8]):
            action.setIcon(QIcon(icons[i]))
        for s in [self.D1_in, self.D2_in, self.D3_in, self.D4_in, self.D5_in, self.D6_in]:
            s.setWindowIcon(QIcon(":/resources/led-off.png"))

        #  Axes configuration window
        self.dlg_axes = AxesConfiguration(self.plotWidget)

        self.page = self.tabWidget.currentIndex()
        self.tim_counter_index = self.cb.currentIndex()

        self.actionConfig.triggered.connect(self.get_port)
        self.actionCSV.triggered.connect(self.export_csv)
        self.Bstart_capture.clicked.connect(self.start_capture)
        self.Bstop_capture.clicked.connect(self.stop_capture)
        self.Bstart_counter.clicked.connect(self.start_counter)
        self.Bstop_counter.clicked.connect(self.stop_counter)
        self.Bstop_pwm.clicked.connect(self.stop_pwm)
        self.Bset_pwm.clicked.connect(self.set_pwm)
        self.Bstart_encoder.clicked.connect(self.start_encoder)
        self.Bstop_encoder.clicked.connect(self.stop_encoder)
        self.actionAxes.triggered.connect(self.change_axes)
        self.Bset_voltage.clicked.connect(self.set_dac)
        self.Bstop.clicked.connect(self.stop_read)
        self.Bplay.clicked.connect(self.play)
        self.tabWidget.currentChanged.connect(lambda: self.page_change(self.page))
        self.cb.currentIndexChanged.connect(lambda: self.tim_counter_change(self.tim_counter_index))
        self.mode_encoder.currentIndexChanged.connect(self.status_resolution)

    def set_dac(self):
        self.daq.set_analog(self.dac_value.value())

    def change_axes(self):
        self.dlg_axes.show()

    def stop_read(self):
        self.dlg_axes.configure_widgets()
        actions_names = ['Axes', 'Home', 'Pan', 'Zoom', 'Save']
        for action in self.toolBar.actions():
            if action.text() in actions_names:
                action.setEnabled(True)
        self.daq.stop()    

    def tim_counter_change(self, tim_counter_index):
        if tim_counter_index == 0 and self.Bstart_encoder.isChecked():
            self.Bstop_encoder.click()
        elif tim_counter_index == 1 and self.Bstart_capture.isChecked():
            self.Bstop_capture.click()
        elif tim_counter_index == 2 and self.Bstart_counter.isChecked():
            self.Bstart_counter.click()
        elif tim_counter_index == 3 and self.Bset_pwm.isChecked():
            self.Bstop_pwm.click()
        self.tim_counter_index = self.cb.currentIndex()

    def page_change(self, page):
        if page == 0:
            self.daq.set_analog(0)
            if self.Bplay.isChecked():
                self.Bstop.click()
        elif page == 2:
            self.tim_counter_change(self.tim_counter_index)
        self.page = self.tabWidget.currentIndex()
        for i, action in enumerate(self.toolBar.actions()[1:8]):
            action.setEnabled(not(bool(self.page)))
        if self.page == 1:
            self.digital_ports()

    def export_csv(self):
        fname = QtWidgets.QFileDialog.getSaveFileName(self, 'Export as CSV')[0]
        fieldnames = ['Time (ms)', 'Voltage (V)']
        with open('%s.csv' % fname, 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            j = 0
            while not(np.isnan(self.Y[j])):
                writer.writerow({'Time (ms)': self.X[j], 'Voltage (V)': self.Y[j]})
                j = j + 1

    def play(self):
        actions_names = ['Axes', 'Home', 'Pan', 'Zoom', 'Save']
        for action in self.toolBar.actions():
            if action.text() in actions_names:
                action.setEnabled(False)
        self.plotWidget.canvas.ax.cla()
        self.plotWidget.canvas.ax.grid(True)
        self.Y, self.X = [np.zeros(BUFFER_SIZE)] * 2
        self.Y[:] = np.nan
        self.X[:] = np.nan
        self.configure()
        self.update()

    def configure(self):
        ninput = self.names.index(self.neg_channel.currentText())
        if ninput > 8:
            ninput = 25
        pinput = self.names.index(self.pos_channel.currentText())
        if pinput > 8:
            pinput = 25
        self.period = self.sb_period.value()
        self.daq.conf_adc(pinput, ninput, gain=self.range.currentIndex())

    def update(self):
        if self.Bplay.isChecked() and not(self.stop):
            self.plot()
            timer = QtCore.QTimer()
            timer.timeout.connect(self.update)
            timer.start(self.period)
            QtCore.QTimer.singleShot(self.period*1000, self.update)

    def plot(self):
        self.plotWidget.canvas.ax.hold(not(self.cBosc.isChecked()))
        self.Y = np.roll(self.Y, 1)
        self.Y[0] = self.daq.read_analog()
        self.X = np.roll(self.X, 1)
        if np.isnan(self.X[1]):
            self.X[0] = 0
        else:
            self.X[0] = self.X[1] + self.period
        self.last_value.setText(str(self.daq.read_analog()))
        self.plotWidget.canvas.ax.plot(self.X, self.Y, color='#4d94ff', linewidth=0.7)
        self.plotWidget.canvas.ax.grid(True)
        self.plotWidget.canvas.draw()

    def get_cb_values(self):
        model = DAQModel.new(*self.daq.get_info())
        for ninput in model.adc.ninputs:
            if ninput < 9:
                self.neg_channel.addItem(self.names[ninput])
            else:
                self.neg_channel.addItem(self.names[9])
        for pinput in model.adc.pinputs:
            if pinput < 9:
                self.pos_channel.addItem(self.names[pinput])
            else:
                self.pos_channel.addItem(self.names[9])
        for gain in model.adc.pga_gains:
            self.range.addItem('x%s' % str(gain))
        self.dac_value.setMinimum(model.dac.vmin)
        self.dac_value.setMaximum(model.dac.vmax)

    def get_port(self):
        dlg = Configuration(self)
        if dlg.exec_() != '':
            port_opendaq = dlg.return_port() 
        try:
        	self.daq = DAQ(str(port_opendaq))
        except (LengthError, SerialException):
        	port_opendaq = ''
        self.tabWidget.setEnabled(bool(port_opendaq))
        if port_opendaq:
        	self.cfg.setValue('port', port_opendaq)
        	self.statusBar.showMessage("Hardware Version: %s   Firmware Version: %s" % (self.daq.hw_ver[1],
                                                                                        self.daq.fw_ver))
        	self.get_cb_values()

    def start_counter(self):
        self.daq.init_counter(0)
        self.daq.init_counter(0)
        while not(self.Bstop_counter.isChecked()) and not(self.stop):
            time.sleep(.1)
            counter = int(self.daq.get_counter(reset=False))
            self.counter_result.setText(str(counter))
            QtCore.QCoreApplication.processEvents()

    def stop_counter(self):
        self.daq.stop_capture()
        self.daq.stop()
        self.counter_result.setText(str(self.daq.get_counter(0)))

    def start_capture(self):
        self.daq.init_capture(int(1000 * self.reference_period.value()))
        while not(self.Bstop_capture.isChecked()) and not(self.stop):
            time.sleep(.1)
            modo = self.cbTime.currentIndex()
            result = self.daq.get_capture(modo)[1]
            frec = self.daq.get_capture(2)[1]
            self.lEPeriod.setText(str((result / 1000.0)))
            self.lEHz.setText(str(round(((1000000.0 / frec) if frec else 0), 3)))
            QtCore.QCoreApplication.processEvents()

    def stop_capture(self):
        self.daq.stop_capture()
        '''
        modo = self.cbTime.currentIndex()
        result = self.daq.get_capture(modo)[1]
        self.lEPeriod.setText(str((result / 1000.0)))
        self.lEHz.setText(str(round(((1000000.0 / result) if result else 0), 3)))
        '''

    def stop_pwm(self):
        self.daq.stop_capture()
        self.daq.stop_pwm()

    def set_pwm(self):
        self.period_PWM = self.periodPWM.value()
        self.duty = int(self.dutyPWM.value() * 1023 / 100)
        self.daq.init_pwm(self.duty, self.period_PWM)

    def status_resolution(self):
        self.resolution_encoder.setEnabled(False if self.mode_encoder.currentIndex() else True)

    def start_encoder(self):
        self.daq.init_encoder(self.resolution_encoder.value())
        result = 0
        while not(self.Bstop_encoder.isChecked()) and not(self.stop):
            time.sleep(.1)
            result = int(self.daq.get_encoder())
            self.result_encoder.setText(str(result))
            QtCore.QCoreApplication.processEvents()

    def stop_encoder(self):
        self.daq.stop_encoder()

    def digital_ports(self):
        ports_mode = [self.cbD1, self.cbD2, self.cbD3, self.cbD4, self.cbD5, self.cbD6]
        switchs = [self.switch1, self.switch2, self.switch3, self.switch4, self.switch5, self.switch6]
        DIs = [self.D1_in, self.D2_in, self.D3_in, self.D4_in, self.D5_in, self.D6_in]
        while (not(self.stop) and self.tabWidget.currentIndex() == 1):
            time.sleep(.3)
            for i in range(6):
                self.daq.set_pio_dir((i+1), ports_mode[i].currentIndex())
                if ports_mode[i].currentIndex():
                    self.daq.set_pio((i+1), int(switchs[i].isChecked()))
                else:
                    value = int(self.daq.read_pio(i+1))
                    DIs[i].setCurrentIndex((value))
                    QtCore.QCoreApplication.processEvents()

    def closeEvent(self, evnt):
        self.stop = 1


class AxesConfiguration(QtWidgets.QDialog, axes_op.Ui_MainWindow):
    def __init__(self, plot, parent=None):
        super(AxesConfiguration, self).__init__(parent)
        self.setupUi(self)
        self.plt = plot
        self.configure_widgets()
        self.ax_Bok.clicked.connect(self.config_axes)


    def configure_widgets(self):
        x_limits = self.plt.canvas.ax.get_xlim()
        self.ax_left.setText(str(x_limits[0]))
        self.ax_right.setText(str(x_limits[1]))
        y_limits = self.plt.canvas.ax.get_ylim()
        self.ax_bottom.setText(str(y_limits[0]))
        self.ax_top.setText(str(y_limits[1]))


    def config_axes(self):
        self.plt.canvas.ax.set_autoscaley_on(False)
        self.plt.canvas.ax.set_autoscalex_on(False)
        self.plt.canvas.ax.set_xlabel(self.ax_xlb.text(), fontsize=10)
        self.plt.canvas.ax.set_ylabel(self.ax_ylb.text(), fontsize=10)
        self.plt.canvas.ax.set_title(self.ax_title.text())
        self.plt.canvas.ax.set_xlim(float(self.ax_left.text()), float(self.ax_right.text()))
        self.plt.canvas.ax.set_ylim(float(self.ax_bottom.text()), float(self.ax_top.text()))
        scale_options = ['linear', 'log', 'logit']
        self.plt.canvas.ax.set_xscale(scale_options[self.ax_xscale.currentIndex()])
        self.plt.canvas.ax.set_yscale(scale_options[self.ax_yscale.currentIndex()])
        self.plt.canvas.draw()
        self.plt.show()
        self.hide()

    def closeEvent(self, evnt):
        evnt.ignore()
        self.hide()

class Configuration(QtWidgets.QDialog, config.Ui_MainWindow):
    def __init__(self, parent=None):
        super(Configuration, self).__init__(parent)
        self.setupUi(self)
        for portO in list_serial_ports():
            self.cbport.addItem(portO)
        self.connectButton.clicked.connect(self.return_port)

    def return_port(self):
        port = self.cbport.currentText()
        self.close()
        return port


def main():
    app = QtWidgets.QApplication(sys.argv)
    dlg = MyApp()
    dlg.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

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
import PyQt5
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPalette, QIcon
from opendaq import DAQ
from opendaq.models import DAQModel

from . import res_rc
from . import daq_control
from . import config
from . import widgets
from .widgets import NavigationToolbar

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
        except:
            pass
    return result


class MyApp(QtWidgets.QMainWindow, daq_control.Ui_mainWindow):
    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        self.names = ['AGND', 'A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7', 'A8', 'VREF']
        self.cfg = QtCore.QSettings('opendaq')
        port_opendaq = str(self.cfg.value('port'))
        try:
            self.daq = DAQ(port_opendaq)
        except SerialException:
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
                 ":/resources/customize.png", ":/resources/save.png"]
        for action in nav.actions()[:-1]:
            if action.text() != 'Subplots': 
                self.toolBar.addAction(action)
        for i, action in enumerate(self.toolBar.actions()[3:8]):
            action.setIcon(QIcon(icons[i]))
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
        self.Bupdate.clicked.connect(self.digital_ports)
        self.Bset_voltage.clicked.connect(self.set_dac)
        self.Bplay.clicked.connect(self.play)
        self.tabWidget.currentChanged.connect(lambda: self.page_change(self.page))
        self.cb.currentIndexChanged.connect(lambda: self.tim_counter_change(self.tim_counter_index))
        self.mode_encoder.currentIndexChanged.connect(self.status_resolution)

    def set_dac(self):
        self.daq.set_analog(self.dac_value.value())

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
        if self.Bplay.isChecked():
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
        self.cfg.setValue('port', port_opendaq)
        self.daq = DAQ(str(port_opendaq))
        self.tabWidget.setEnabled(bool(port_opendaq))
        self.statusBar.showMessage("Hardware Version: %s   Firmware Version: %s" % (self.daq.hw_ver[1], self.daq.fw_ver))
        self.get_cb_values()

    def start_counter(self):
        self.daq.init_counter(0)
        self.daq.init_counter(0)
        while not(self.Bstop_counter.isChecked()):
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

    def stop_capture(self):
        self.daq.stop_capture()
        modo = self.cbTime.currentIndex()
        result =  self.daq.get_capture(modo)[1]
        self.lEPeriod.setText(str((result / 1000.0)))
        self.lEHz.setText(str(round(((1000000.0/ result) if result else 0), 3)))

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
        while not(self.Bstop_encoder.isChecked()):
            time.sleep(.1)
            result = int(self.daq.get_encoder())
            self.result_encoder.setText(str(result))
            QtCore.QCoreApplication.processEvents()

    def stop_encoder(self):
        self.daq.stop_encoder()

    def digital_ports(self):
        ports_mode = [self.cbD1, self.cbD2, self.cbD3, self.cbD4, self.cbD5, self.cbD6]
        slider_out = [self.SDO1, self.SDO2, self.SDO3, self.SDO4, self.SDO5, self.SDO6]
        display_out = [self.DDI1, self.DDI2, self.DDI3, self.DDI4, self.DDI5, self.DDI6]

        for i in range(6):
            self.daq.set_pio_dir((i+1), ports_mode[i].currentIndex())
            if ports_mode[i].currentIndex():
                self.daq.set_pio((i+1), slider_out[i].value())
            else:
                value = 1 if self.daq.read_pio(i+1) else 0
                palette = display_out[i].palette()
                palette.setColor(QPalette.WindowText, QtCore.Qt.green if value else QtCore.Qt.red)
                display_out[i].setPalette(palette)
                display_out[i].display(value)


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

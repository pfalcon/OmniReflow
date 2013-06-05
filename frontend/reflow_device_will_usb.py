"""
-----------------------------------------------------------------------
reflow
Copyright (C) William Dickson, 2008.
  
wbd@caltech.edu
www.willdickson.com

Released under the LGPL Licence, Version 3

This file is part of reflow.

reflow is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
    
reflow is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with simple_step.  If not, see
<http://www.gnu.org/licenses/>.

------------------------------------------------------------------------

Purpose: . 

Author: William Dickson 

------------------------------------------------------------------------
"""
import sys
import usb_device
import ctypes
import time

def swap_dict(in_dict):
    """
    Swap key and values in dictionary
    
    Arguments:
      in_dict = input dictionary
      
    Return: dictionary w/ keys and values swapped.
    """
    return dict([(v,k) for k,v in in_dict.iteritems()])

# USB Command IDs
USB_CMD_SET_MODE = ctypes.c_uint8(1)
USB_CMD_GET_MODE = ctypes.c_uint8(2)
USB_CMD_SET_PWM_VAL = ctypes.c_uint8(3)
USB_CMD_GET_PWM_VAL = ctypes.c_uint8(4)
USB_CMD_GET_THERM_VAL = ctypes.c_uint8(5)
USB_CMD_SET_PWM_PERIOD = ctypes.c_uint8(6)
USB_CMD_GET_PWM_PERIOD = ctypes.c_uint8(7)
USB_CMD_AVR_RESET = ctypes.c_uint8(200)
USB_CMD_AVR_DFU_MODE = ctypes.c_uint8(201)

MODE2STR_DICT = {0:'off', 1:'on', 2:'pwm'}
STR2MODE_DICT = swap_dict(MODE2STR_DICT) 

PWM_PERIOD = 0.5
TIMER_TOP_MIN = 625      
TIMER_TOP_MAX = 65535   
F_CPU = 16.0e6

# Reflow device class
class Reflow(usb_device.USB_Device):

    def __init__(self):
        usb_device.USB_Device.__init__(self)
        self.set_pwm_period(PWM_PERIOD)

    def get_mode(self):
        cmd_id_sent = USB_CMD_GET_MODE
        outdata = [cmd_id_sent]
        intypes = [ctypes.c_uint8, ctypes.c_uint8]
        val_list = self.usb_cmd(outdata,intypes)
        cmd_id_recv = val_list[0]
        mode_int = val_list[1].value
        check_cmd_id(cmd_id_sent, cmd_id_recv)
        mode_str = MODE2STR_DICT[mode_int]
        return mode_str

    def get_pwm_period(self):
        cmd_id_sent = USB_CMD_GET_PWM_PERIOD
        outdata = [cmd_id_sent]
        intypes = [ctypes.c_uint8, ctypes.c_float]
        val_list = self.usb_cmd(outdata,intypes)
        cmd_id_recv = val_list[0]
        pwm_period = val_list[1].value
        check_cmd_id(cmd_id_sent, cmd_id_recv)
        return pwm_period

    def get_pwm_value(self):
        cmd_id_sent = USB_CMD_GET_PWM_VAL
        outdata = [cmd_id_sent]
        intypes = [ctypes.c_uint8, ctypes.c_uint8]
        val_list = self.usb_cmd(outdata,intypes)
        cmd_id_recv = val_list[0]
        pwm_value = val_list[1].value
        check_cmd_id(cmd_id_sent, cmd_id_recv)
        return pwm_value

    def get_therm_value(self,convert=True):
        cmd_id_sent = USB_CMD_GET_THERM_VAL
        outdata = [cmd_id_sent]
        intypes = [ctypes.c_uint8, ctypes.c_uint16]
        val_list = self.usb_cmd(outdata,intypes)
        cmd_id_recv = val_list[0]
        check_cmd_id(cmd_id_sent, cmd_id_recv)
        therm_val = 2.56*val_list[1].value/1023.0
        if convert == True:
            therm_val = therm_val*98.457273 + 0.741686 
        return therm_val

    def set_mode(self,mode):
        mode_int = STR2MODE_DICT[mode]
        mode_ctypes = ctypes.c_uint8(mode_int)
        cmd_id_sent = USB_CMD_SET_MODE
        outdata = [cmd_id_sent, mode_ctypes]
        intypes = [ctypes.c_uint8, ctypes.c_uint8]
        val_list = self.usb_cmd(outdata,intypes)
        cmd_id_recv = val_list[0]
        mode_int_recv = val_list[1].value
        check_cmd_id(cmd_id_sent, cmd_id_recv)
        if mode_int_recv != mode_int:
            raise IOError, 'mode int value sent (%d) not equal to mode int received (%d)'%(mode_int, mode_int_recv)

    def set_power(self, value):
        if value == 0:
            set_mode('off')
        else:
            self.set_pwm_value(value)

    def set_pwm_value(self, value):
        value = int(value)
        value_ctypes = ctypes.c_uint8(value)
        cmd_id_sent =   USB_CMD_SET_PWM_VAL
        outdata = [cmd_id_sent, value_ctypes]
        intypes = [ctypes.c_uint8, ctypes.c_uint8]
        val_list = self.usb_cmd(outdata,intypes)
        cmd_id_recv = val_list[0]
        value_recv = val_list[1].value
        check_cmd_id(cmd_id_sent, cmd_id_recv)
        if value != value_recv:
            raise IOError, 'pwm value sent (%d) not equal to pwm value received (%d)'%(value, value_recv)

    def set_pwm_period(self,period):
        max_period = timertop_2_period(TIMER_TOP_MAX)
        min_period = timertop_2_period(TIMER_TOP_MIN)
        if period > max_period:
            raise ValueError, 'period > max period'
        if period < min_period:
            raise ValueError, 'period < min period'
        period_ctypes = ctypes.c_float(period)
        cmd_id_sent = USB_CMD_SET_PWM_PERIOD
        outdata = [cmd_id_sent, period_ctypes]
        intypes = [ctypes.c_uint8, ctypes.c_float] 
        val_list = self.usb_cmd(outdata,intypes)
        cmd_id_recv = val_list[0]
        period_recv = val_list[1].value
        check_cmd_id(cmd_id_sent, cmd_id_recv)
        return period_recv

    def step_test(self, ctl_val=0.75*255, t_on=100.0, t_off=1100.0, t_stop = 2100.0, dt=1.0, pwm_period=0.5):
        """
        Step response test. 
        """

        mode = 'off'
        self.set_mode('off')
        self.set_pwm_period(pwm_period)
        temp_list = []
        time_list = []
        ctl_list = []
        t0 = time.time()
        t = 0.0 

        while 1:

            # Record time and temperature.
            time_list.append(t)
            temp = self.get_therm_value()
            temp_list.append(temp)
            if mode == 'pwm':
                ctl_list.append(ctl_val)
            else:
                ctl_list.append(0.0)

            # Display time, temperature and mode
            print 't = %1.2f, T = %1.2f, mode = %s'%(t,temp,mode)
            sys.stdout.flush()

            # Set mode based on current time.
            if t >= t_stop:
                self.set_mode('off')
                break
            elif (mode=='off') and (t >= t_on) and (t < t_off):
                self.set_pwm_value(ctl_val)
                self.set_mode('pwm')
                mode = 'pwm'
            elif (mode=='pwm') and (t>=t_off):
                self.set_mode('off')
                mode = 'off'

            # Sleep for dt seconds and then get new current time.
            time.sleep(dt)
            t = time.time() - t0

        # Pack step response data into dictionary
        data = {
            'ctl_val'    : ctl_val,
            't_on'       : t_on,
            't_off'      : t_off, 
            't_stop'     : t_stop,
            'dt'         : dt,
            'time_list'  : time_list,
            'temp_list'  : temp_list,
            'ctl_list'   : ctl_list,
            'pwm_period' : pwm_period,
        }

        return data


def timertop_2_period(timertop):
    freq = 256.0*timertop/F_CPU
    return freq

def period_2_timertop(freq):
    timertop = int(pwm_period*F_CPU/256.0);
    return timertop

def check_cmd_id(idout, idin):
    if idout.value != idin.value:
        raise IOError, 'cmd ID received (%d) does not match cmd ID sent (%d)'%(idin.value, idout.value)

#-------------------------------------------------------------------------------------
if __name__ == '__main__':

    import cPickle as pickle
    import pylab

    dev = Reflow()
    dev.print_values()
    print

    if 1:
        data = dev.step_test(ctl_val=1.00*255, t_on=100.0, t_off=300.0, t_stop = 600.0, dt=1.0, pwm_period=1.0)
        #fid = open('temp_data_u100_period_1p0.pkl','w')
        #pickle.dump(data,fid)
        #fid.close()

    if 0:

        fid = open('temp_data_u100_period_1p0.pkl','r')
        data = pickle.load(fid)
        fid.close()
        pylab.subplot(211)
        pylab.plot(data['time_list'], data['temp_list'],'.')
        pylab.subplot(212)
        pylab.plot(data['time_list'], data['ctl_list'],'r')
        pylab.show()

    if 0:
        t_on = 600.0
        t_cool = 600.0
        dev.set_mode('off')
        dev.set_pwm_period(0.5)
        dev.set_pwm_value(256/2)
        fid = open('temp_data_2.txt','w')
        t0 = time.time()
        dev.set_mode('pwm')
        while time.time() - t0 < (t_on + t_cool):
            temp = dev.get_therm_value()
            t = time.time()-t0
            print t, temp,
            fid.write('%f %f\n'%(t,temp))
            time.sleep(0.5)
            if time.time() - t0 > t_on:
                dev.set_mode('off')
                print 'off'
            else:
                print 'on'

        fid.close()


    if 0:
        dev.set_pwm_period(0.5)
        dev.set_pwm_value(256/8)
        dev.set_mode('pwm')
        time.sleep(10.0)
        dev.set_mode('off')

    if 0:
        mode = dev.get_mode()
        print 'mode:', mode
        pwm_period = dev.get_pwm_period()
        print 'pwm period:', pwm_period
        pwm_value = dev.get_pwm_value()
        print 'pwm value:', pwm_value

    if 0:
        for i in range(0,20):
            temp = dev.get_therm_value()
            print 'T = %f'%(temp,)
            time.sleep(0.25)

    if 0:
        dev.set_mode('on')
        mode = dev.get_mode()
        print 'mode:', mode
        time.sleep(2.0)
        dev.set_mode('off')
        mode = dev.get_mode()
        print 'mode:', mode

    if 0:
        dev.set_mode('pwm')
        dev.set_pwm_value(256/2)
        pwm_value = dev.get_pwm_value()
        print 'pwm value:', pwm_value
        time.sleep(5.0)
        dev.set_pwm_value(0)
        pwm_value = dev.get_pwm_value()
        print 'pwm value:', pwm_value
        dev.set_mode('off')

    if 0:
        dev.set_pwm_period(0.5)
        pwm_period = dev.get_pwm_period()
        print 'pwm period:', pwm_period





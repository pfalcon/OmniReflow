"""
-----------------------------------------------------------------------
reflow
Copyright (C) 2008 William Dickson <wbd@caltech.edu>
Copyright (C) 2013 Paul Sokolovsky

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

Purpose: Abstract interface of a reflow device. A reflow device has
two methods: 1) to query current heater temperature (in SI units);
2) to set relative heater power.

Author: Paul Sokolovsky

------------------------------------------------------------------------
"""
import reflow_device
from delay_model import Therm_Delay_Model


class ReflowDevice(reflow_device.ReflowDevice):

    def __init__(self):
        self.model = Therm_Delay_Model()
        self.model.load_param('model_param.pkl')
        self.first = True
        self.last_temp = self.model.T_amb
        self.ctrl = []

    def set_clock(self, clock):
        self.clock = clock

    def get_therm_value(self):
        if self.clock.state == "stopped":
            return self.last_temp
        t = self.clock.get_time()
        i = int(t - self.model.t_delay)
        if i < 0:
            ctrl = 0
        else:
            ctrl = self.ctrl[i]

        v = self.model.diff(self.last_temp, t, ctrl)
        self.last_temp += v
        return self.last_temp

    def set_power(self, value):
        """Set relative power of heater, in the range 0.0-255.0
        (note - floating point)"""
        self.ctrl.append(value)

    def close(self):
        pass

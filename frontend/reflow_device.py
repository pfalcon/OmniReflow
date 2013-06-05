"""
-----------------------------------------------------------------------
reflow
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
class ReflowDevice(object):

    def __init__(self):
        pass

    def get_therm_value(self):
        return 0

    def set_power(self, value):
        pass

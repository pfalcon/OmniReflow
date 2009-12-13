DEFAULT_PARAM_1 = {
        'preheat_T_end'         : 170.0,
        'preheat_C_per_sec'     : 1.5, 
        'soak_delta_t'          : 80.0,
        'soak_T_end'            : 180.0,
        'reflow_delta_t'        : 100.0,
        'reflow_T_peak'         : 220.0,
        'cool_C_per_sec'        : -1.0,
        'cool_T_end'            : 50.0
        }

DEFAULT_PARAM = {
        'preheat_T'      : 150, 
        'preheat_rate'   : 1.5, 
        'soak_T'         : 180,
        'soak_dt'        : 80.0,
        'reflow_T'       : 220.0,
        'reflow_dt'      : 75.0,
        }

class Reflow_Profile_new:

    def __init__(self, start_T, param=DEFAULT_PARAM):
        self.preheat_T = param['preheat_T']
        self.preheat_rate = param['preheat_rate']
        self.soak_T = param['soak_T']
        self.soak_dt = param['soak_dt']
        self.reflow_T = param['reflow_T']
        self.reflow_dt = param['reflow_dt']

    def get_times(self):
        pass


class Reflow_Profile:

    def __init__(self,start_T,param=DEFAULT_PARAM_1):
        self.start_T =  start_T
        self.preheat_T_end = param['preheat_T_end']
        self.preheat_C_per_sec = param['preheat_C_per_sec']
        self.soak_delta_t = param['soak_delta_t']
        self.soak_T_end = param['soak_T_end']
        self.reflow_delta_t = param['reflow_delta_t']
        self.reflow_T_peak = param['reflow_T_peak']
        self.cool_C_per_sec = param['cool_C_per_sec']
        self.cool_T_end = param['cool_T_end']

    def get_times(self):
        preheat_t = (self.preheat_T_end - self.start_T)/self.preheat_C_per_sec
        soak_t = preheat_t + self.soak_delta_t
        reflow_rise_t = soak_t + 2.0*self.reflow_delta_t/3.0
        reflow_cons_t = soak_t + 2.0*self.reflow_delta_t/3.0
        reflow_fall_t = soak_t + 3.0*self.reflow_delta_t/3.0
        cool_t = (self.cool_T_end - self.soak_T_end + self.cool_C_per_sec*reflow_fall_t)
        cool_t = cool_t/self.cool_C_per_sec
        return preheat_t, soak_t, reflow_rise_t, reflow_cons_t, reflow_fall_t, cool_t

    def stop_time(self):
        times = self.get_times()
        return times[-1]

    def func(self,t):

        times = self.get_times()
        preheat_t, soak_t, reflow_rise_t, reflow_cons_t, reflow_fall_t, cool_t = times

        if t < 0:
            value = self.start_T
        elif t >= 0 and t < preheat_t:
            # Preheat temperature profile
            value = self.preheat_C_per_sec*t + self.start_T
        elif t >= preheat_t and t < soak_t:
            # Soak temperature profile
            a = (self.soak_T_end - self.preheat_T_end)/self.soak_delta_t
            b = self.preheat_T_end - a*preheat_t
            value = a*t + b
        elif t >= soak_t and t < reflow_rise_t:
            # Rise to peak reflow temperature
            a = (self.reflow_T_peak - self.soak_T_end)/(reflow_rise_t - soak_t)
            b = self.soak_T_end - a*soak_t
            value = a*t + b
        elif t >= reflow_rise_t and t < reflow_cons_t:
            # Hold peak reflow temperature constant
            value = self.reflow_T_peak
        elif t >= reflow_cons_t and t < reflow_fall_t:
            # Fall to peak soak temperature
            a = (self.soak_T_end - self.reflow_T_peak)/(reflow_fall_t - reflow_cons_t)
            b = self.reflow_T_peak - a*reflow_cons_t
            value = a*t + b
        elif t >= reflow_fall_t and t <= cool_t:
            # Cooling phase
            b = self.soak_T_end - self.cool_C_per_sec*reflow_fall_t
            value = self.cool_C_per_sec*t + b
        else:
            value = self.cool_T_end

        return value

# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import scipy
    import pylab

    profile = Reflow_Profile(start_T = 20.0)
    stop_t = profile.stop_time()
    t = scipy.linspace(0.0, stop_t, 1000)
    T = scipy.array([profile.func(tt) for tt in t])
    pylab.plot(t,T)
    pylab.show()


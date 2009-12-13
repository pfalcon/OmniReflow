DEFAULT_PARAM = {
        'preheat_T'      : 180, 
        'preheat_rate'   : 1.5, 
        'soak_T'         : 180,
        'soak_dt'        : 90.0,
        'reflow_T'       : 220.0,
        'reflow_dt'      : 90.0,
        'cool_dt'        : 300.0
        }

class Reflow_Profile:

    def __init__(self, start_T, param=DEFAULT_PARAM):
        self.start_T = start_T
        self.preheat_T = param['preheat_T']
        self.preheat_rate = param['preheat_rate']
        self.soak_T = param['soak_T']
        self.soak_dt = param['soak_dt']
        self.reflow_T = param['reflow_T']
        self.reflow_dt = param['reflow_dt']
        self.cool_dt = param['cool_dt']

        # compute times
        self.preheat_t = (self.preheat_T - self.start_T)/self.preheat_rate
        self.soak_t = self.preheat_t + self.soak_dt
        self.reflow_t = self.soak_t + self.reflow_dt
        self.stop_t = self.reflow_t + self.cool_dt

    def stop_time(self):
        return self.stop_t

    def func(self,t):
        if t < 0:
            T = self.start_T
        elif t >=0 and t <= self.preheat_t:
            T = self.preheat_rate*t + self.start_T
        elif t > self.preheat_t and t <= self.soak_t:
            T = self.soak_T
        elif t > self.soak_t and t <= self.reflow_t:
            T = self.reflow_T
        else:
            T = self.start_T 
        return T 
            

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


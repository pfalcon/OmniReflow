class PI_Controller:
    """
    Proportional integral controller w/ feed forward term
    """

    def __init__(self,setpt_func,pgain,igain,ff_func=None,max_value=255.0, min_value=0.0):
        self.setpt_func = setpt_func
        self.pgain = pgain
        self.igain = igain
        self.max_value = max_value
        self.min_value = min_value
        self.integrator = Integrator()
        if ff_func == None:
            self.ff_func = self.zero_ff_func
        else:
            self.ff_func = ff_func

    def func(self,t,x):
        setpt = self.setpt_func(t)
        err = setpt - x
        self.integrator.update(t,err)
        ierr = self.integrator.value
        value = self.pgain*err + self.igain*ierr + self.ff_func(setpt)
        if value > self.max_value:
            value = self.max_value 
        if value < self.min_value:
            value = self.min_value 
        return value

    def zero_ff_func(self,T):
        return 0.0

class Integrator:

    def __init__(self, clamp=200.0):
        self.value = 0.0
        self.last_t = 0.0 
        self.clamp = clamp

    def update(self,t,x):
        dt = t - self.last_t
        self.last_t = t
        self.value += x*dt
        if self.value > self.clamp:
            self.value = self.clamp
        if self.value < -self.clamp:
            self.value = -self.clamp

    def get_value(self):
        return self.value 
            

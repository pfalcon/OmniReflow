import scipy
import scipy.integrate
import scipy.interpolate
import scipy.optimize
import pylab
import cPickle as pickle

class Therm_Delay_Model:
    """
    First order thermal system w/ delay where the control coefficient is a function
    of temperature.

    dT/dt = (ctl_coef[1] + ctl_coef[0]*T)*ctl_func(t - t_delay, T) - temp_coef*(T - T_amb)

    ctl_coef  = control coefficients 
    temp_coef = temperature coefficients 
    ctl_func  = control function
    t_delay   = transport tme delay
    T_amb     = ambient temperature
    """

    def __init__(self, ctl_coef=(0.0,0.0), temp_coef=(0.0,0.0), ctl_func=None, t_delay=0.0, T_amb=0.0): 
        self.ctl_coef = ctl_coef
        self.temp_coef = temp_coef
        if ctl_func == None:
            self.ctl_func = self.zero_func
        else:
            self.ctl_func = ctl_func
        self.t_delay = t_delay
        self.T_amb = T_amb

    def diff_eq(self,T,t):
        val = (self.ctl_coef[1] + self.ctl_coef[0]*T)*self.ctl_func(t-self.t_delay,T)
        val -= self.temp_coef*(T - self.T_amb) 
        return val

    def diff_eq_fixed_step(self,T,t):
        t_w_delay, T_w_delay = self.delay.get_values()
        val = (self.ctl_coef[1] + self.ctl_coef[0]*T)*self.ctl_func(t_w_delay,T_w_delay)
        val -= self.temp_coef*(T - self.T_amb) 
        return val

    def solve(self, T0, t):
        T_sol = scipy.integrate.odeint(self.diff_eq, T0, t)
        return T_sol

    def solve_fixed_step(self,T0,t):
        # Set t_delay to zero - store value in temporary variable for restoring to this
        # value later 
        t_delay_temp = self.t_delay
        self.t_delay = 0.0

        # Create time delay object
        self.delay = Time_Delay(t[0],T0,t_delay_temp)

        # Integrate system
        T_sol = scipy.zeros(t.shape)
        T_sol[0] = T0
        T_cur = T0
        for i in range(1,t.shape[0]):
            t0 = t[i-1]
            t1 = t[i]
            tvals = scipy.linspace(t0,t1,5)
            T_cur = scipy.integrate.odeint(self.diff_eq_fixed_step, T_cur, tvals)[-1][0]
            T_sol[i] = T_cur
            self.delay.update(t1,T_cur)
            
        # Restore value of t_delay
        self.t_delay = t_delay_temp
        return T_sol
            
    def zero_func(self, t, T):
        return 0.0

    def load_param(self,filename):
        fid = open('model_param.pkl', 'r')
        param = pickle.load(fid)
        fid.close()
        self.ctl_coef = param['ctl_coef']
        self.temp_coef = param['temp_coef']
        self.t_delay = param['t_delay']
        self.T_amb = param['T_amb']

    def steady_state(self, ctl):
        pass

    def ctl_for_steady_state(self, T):
        ctl = self.temp_coef*(T - self.T_amb)/(self.ctl_coef[1] + self.ctl_coef[0]*T)
        return ctl

class Time_Delay:
    """
    Simple time delay function for use with the fixed step solver in Therm_Delay_Model.
    """
    def __init__(self,t0,T0,t_delay):
        self.t0 = t0
        self.T0 = T0
        self.t_delay = t_delay
        self.vlist = []
        self.tlist = []
        self.vlist.append(t0)
        self.tlist.append(T0)

    def update(self,t,v):
        self.tlist.append(t)
        self.vlist.append(v)

    def get_values(self):
        t = self.tlist[-1]
        t_w_delay = t - self.t_delay
        if t_w_delay < self.tlist[0]:
            value_w_delay = self.T0
        else:
            interp_func = scipy.interpolate.interp1d(self.tlist,self.vlist,'linear')
            value_w_delay = interp_func(t_w_delay)
        return t_w_delay, value_w_delay


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

    def __init__(self,clamp=1000000.0):
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
            


class Pulse:

    def __init__(self, loval, hival, t0, t1):
        self.loval = loval
        self.hival = hival
        self.t0 = t0
        self.t1 = t1
        
    def func(self, t, *args):
        if (t >= self.t0) and (t <= self.t1):
            val = self.hival
        else:
            val = self.loval
        return val

class Pulse_Dataset:

    def __init__(self, data_dict):
        self.ctl_val = data_dict['ctl_val']
        self.t_on = data_dict['t_on']
        self.t_off = data_dict['t_off']
        self.t_stop = data_dict['t_stop']
        self.dt = data_dict['dt']
        self.t = scipy.array(data_dict['time_list'])
        self.T = scipy.array(data_dict['temp_list'])
        self.ctl = scipy.array(data_dict['ctl_list'])
        self.pwm_period = data_dict['pwm_period']
        self.interp = scipy.interpolate.interp1d(self.t,self.T,'linear')

    def plot(self,show=True):
        ax1 = pylab.subplot(211)
        pylab.plot(self.t, self.T, '.')
        pylab.ylabel('Temperature (C)')
        ax2 = pylab.subplot(212,sharex=ax1)
        pylab.plot(self.t, self.ctl, 'r')
        pylab.ylabel('Control')
        pylab.xlabel('Time (sec)')
        if show==True:
            pylab.show()

def load_pulse_dataset(filename):
    """
    Load dataset from file. 
    """
    fid = open(filename,'r')
    data = pickle.load(fid)
    fid.close()
    dataset = Pulse_Dataset(data)
    return dataset


def estimate_model_param(dataset): 
    """ 
    Estimate model parameters from pulse dataset. 

    * First, a rough estimate of the time constant and the ambient temperature
    are made using data from the fall (t > t_off).

    * Second, a rough estimate of the control coefficient, a linear function of
    the temperature, is make using data from the rise (t_on < t < t_off).

    * Third, the time constant is estimated using a 1D optimization with respect
    to the delay, t_delay, using the rough estimate of the time constant, ambient
    temperature, and control coefficient.

    * Finally, the parameters are refined using a hill climbing optimization on the 
    whole dataset. The rough parameters as used as an initial guess for the hill 
    climber.
    """
    # Temperature region for slope estimate as fraction of temperature change
    slope_est_T_region = 0.1,0.9
    slope_est_N = 10 

    # -------------------------------------------------------------------------
    # Step 1. Estimate ambient temperature form pre pulse data
    mask = dataset.t < dataset.t_on
    t_pre = dataset.t[mask]
    T_pre = dataset.T[mask]
    T_a = T_pre.mean()

    # -------------------------------------------------------------------------
    # Step 2. Estimate the time constant and ambient temperature.

    # Slice out fall by time
    mask = dataset.t > dataset.t_off
    t_fall = dataset.t[mask]
    T_fall = dataset.T[mask]
    # Get subregion of fall for estimates
    delta_T_fall = T_fall.max() - T_fall.min()
    mask0 = T_fall >= (T_fall.min() + slope_est_T_region[0]*delta_T_fall)
    mask1 = T_fall <= (T_fall.min() + slope_est_T_region[1]*delta_T_fall)
    mask = scipy.logical_and(mask0, mask1)
    # Estimate linear fit to dT/dt vs T
    t_fall = t_fall[mask]
    T_fall = T_fall[mask]
    T_fall = T_fall - T_a
    slope_fall, offset_fall = estimate_rate_func(t_fall, T_fall, slope_est_N, method='central diff')
    temp_coef = slope_fall

    # -------------------------------------------------------------------------
    # Step 3. Estimate the control coefficient - a linear function of T 
    # ctl_coef of the form A*T + B

    # Slice out rise by time
    mask0 = dataset.t > dataset.t_on
    mask1 = dataset.t < dataset.t_off
    mask = scipy.logical_and(mask0, mask1)
    t_rise = dataset.t[mask]
    T_rise = dataset.T[mask]
    # Get subregion of rise for estimates
    delta_T_rise = T_rise.max() - T_rise.min()
    mask0 = T_rise >= (T_rise.min() + slope_est_T_region[0]*delta_T_rise)
    mask1 = T_rise <= (T_rise.min() + slope_est_T_region[1]*delta_T_rise)
    mask = scipy.logical_and(mask0, mask1)
    # Estimate linear fit to dT/dt vs T
    t_rise = t_rise[mask]
    T_rise = T_rise[mask]
    slope_rise, offset_rise = estimate_rate_func(t_rise, T_rise, slope_est_N, method='central diff')
    # Calculate ctl_coef
    ctl_coef_slope = (slope_rise - slope_fall)/dataset.ctl_val
    ctl_coef_offset = (offset_rise - offset_fall)/dataset.ctl_val
    ctl_coef = ctl_coef_slope, ctl_coef_offset

    # -------------------------------------------------------------------------
    # Step 4. Estimate transport time delay

    def delay_cost_func(t_delay):
        pulse = Pulse(0.0, dataset.ctl_val, dataset.t_on, dataset.t_off)
        model = Therm_Delay_Model(ctl_coef=ctl_coef, temp_coef = temp_coef, 
                                  ctl_func=pulse.func, t_delay=t_delay, T_amb=T_a)
        t_sim = scipy.linspace(dataset.t.min(), 0.5*dataset.t_off, 1000)
        T_sim = model.solve(T_a, t_sim)
        T_sim = T_sim.reshape((T_sim.shape[0],))
        T_interp = dataset.interp(t_sim)
        cost = ((T_sim - T_interp)**2).sum()
        return cost

    if 0:
        # Debugging plot
        t_delay_array = scipy.linspace(0.0,100.0,30)
        cost_array = scipy.zeros(t_delay_array.shape)
        for i, t_delay in enumerate(t_delay_array):
            cost_array[i] = delay_cost_func(t_delay)
        pylab.plot(t_delay_array, cost_array,'o')
        pylab.show()

    t_delay = scipy.optimize.brent(delay_cost_func)

    # -------------------------------------------------------------------------
    # Step 5. Refine parameters using hill climber.

    def param_cost_func(param):
        ctl_coef = param[0], param[1]
        temp_coef = param[2]
        t_delay = param[3]
        T_a = param[4]

        pulse = Pulse(0.0, dataset.ctl_val, dataset.t_on, dataset.t_off)
        model = Therm_Delay_Model(ctl_coef=ctl_coef, temp_coef=temp_coef, 
                                  ctl_func=pulse.func, t_delay=t_delay, T_amb=T_a)
        delta_t_fall = dataset.t[-1] - dataset.t_off
        t_sim_start = dataset.t.min()
        t_sim_stop = dataset.t.max()
        t_sim = scipy.linspace(t_sim_start, t_sim_stop, 1000)
        T_sim = model.solve(T_a, t_sim)
        T_sim = T_sim.reshape((T_sim.shape[0],))
        T_interp = dataset.interp(t_sim)
        cost = ((T_sim - T_interp)**2).sum()
        return cost

    param_guess = (ctl_coef[0], ctl_coef[1], temp_coef, t_delay, T_a)
    param_sol = scipy.optimize.fmin(param_cost_func, param_guess,disp=0)

    ctl_coef = param_sol[0],param_sol[1]
    temp_coef = param_sol[2]
    t_delay = param_sol[3]
    T_a = param_sol[4]
    
    if 1:
        print 'ctl_coef:', ctl_coef
        print 'temp_coef:', temp_coef 
        print 't_delay:', t_delay
        print 'T_a:', T_a

        pulse = Pulse(0.0, dataset.ctl_val, dataset.t_on, dataset.t_off)
        model = Therm_Delay_Model(ctl_coef=ctl_coef, temp_coef=temp_coef, 
                                  ctl_func=pulse.func, t_delay=t_delay, T_amb=T_a)
        t_sim = scipy.linspace(dataset.t.min(), dataset.t.max(), 1000)
        T_sim = model.solve(T_a, t_sim)

        pylab.plot(dataset.t, dataset.T, '.b')
        pylab.plot(t_sim, T_sim, 'r')
        pylab.show()

    model_param = {
        'ctl_coef'  : ctl_coef,
        'temp_coef' : temp_coef,
        't_delay'   : t_delay,
        'T_amb'     : T_a,
    }

    return model_param 


def estimate_rate_func(t, T, N, plot_flag=False, method='central diff'):

    t_est_pts = scipy.linspace(t.min(), t.max(), N+2) 
    interp_func = scipy.interpolate.interp1d(t,T,'linear')
    T_est_pts = interp_func(t_est_pts)
 
    if plot_flag == True:
        pylab.figure()
        pylab.subplot(211)
        pylab.plot(t_est_pts, T_est_pts,'or')

    # Estimate slopes
    slope_pts = scipy.zeros((N,)) 
    T_slope_pts = scipy.zeros((N,))  

    if method == 'local fit':
        for i in range(1,(N+1)):
            mask0 = t > 0.5*(t_est_pts[i-1] + t_est_pts[i])
            mask1 = t < 0.5*(t_est_pts[i+1] + t_est_pts[i])
            mask = scipy.logical_and(mask0, mask1)
            t_slope_est = t[mask]
            T_slope_est = T[mask]
            local_fit = scipy.polyfit(t_slope_est,T_slope_est,2)
            dlocal_fit = scipy.polyder(local_fit)
            slope_pts[i-1] = scipy.polyval(dlocal_fit,t_est_pts[i]) 
            T_slope_pts[i-1] = scipy.polyval(local_fit,t_est_pts[i])
            if plot_flag == True:
                t_slope_fit = scipy.linspace(t_slope_est[0], t_slope_est[-1], 100)
                T_slope_fit = scipy.polyval(local_fit,t_slope_fit)
                pylab.plot(t_slope_fit, T_slope_fit,'g')
    elif method == 'central diff':
        dt = t_est_pts[1] - t_est_pts[0]
        slope_pts = (T_est_pts[2:] - T_est_pts[:-2])/(2.0*dt)
        T_slope_pts = T_est_pts[1:-1]
    else:
        raise ValueError, 'unkown method %s'%(method,)
        
    # Fit line to slope estimates
    fit = scipy.polyfit(T_slope_pts, slope_pts,1)

    if plot_flag == True:
        T_slope_fit = scipy.linspace(T_slope_pts.min(), T_slope_pts.max(),100)
        slope_fit = scipy.polyval(fit,T_slope_fit)
        pylab.subplot(212)
        pylab.plot(T_slope_fit, slope_fit,'r')
        pylab.plot(T_slope_pts, slope_pts,'o')
        pylab.show()

    rate_slope = fit[0]
    rate_offset = fit[1]
    return rate_slope, rate_offset 


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    if 0:
        dataset = load_pulse_dataset('temp_data_u75_period_1p0.pkl')
        model_param = estimate_model_param(dataset)
        #fid = open('model_param.pkl','w')
        #pickle.dump(model_param,fid)
        #fid.close()

    if 0:
        model = Therm_Delay_Model()
        model.load_param('model_param.pkl')

        # test delay model
        pulse_lo = 0.0 
        pulse_hi = 255.0 
        pulse_t0 = 100.0
        pulse_t1 = 300.0
        t_max = 600.0
        n_t = 1000

        pulse = Pulse(pulse_lo, pulse_hi, pulse_t0, pulse_t1)
        model.ctl_func = pulse.func
        
        t = scipy.linspace(0.0, t_max, n_t)
        T_sol = model.solve(model.T_amb, t)
        T_sol_fixed = model.solve_fixed_step(model.T_amb, t)
        pylab.plot(t,T_sol)
        pylab.plot(t,T_sol_fixed,'r')
        pylab.show()


    if 0:
        model = Therm_Delay_Model()
        model.load_param('model_param.pkl')
        
        # test delay model
        pulse_lo = 0.0 
        pulse_hi = 200.0 
        pulse_t0 = 100.0
        pulse_t1 = 1500.0
        pulse = Pulse(pulse_lo, pulse_hi, pulse_t0, pulse_t1)

        def test_func(t):
            T1 = 200.0
            T2 = 220.0
            t0 = 200.0 
            t1 = 400.0
            t2 = 500.0
            t3 = 800.0
            if t < t0:
                return t*T1/t0 
            elif t >= t0 and t < t1:
                return T1 
            elif t >= t1 and t < t2:
                a = (T1-T2)/(t1-t2)
                b = T1 - a*t1
                return a*t + b
            elif t >= t2 and t < t3:
                return T2
            else:
                return 0.0

        def steps(t):
            if t < 400:
                return 200
            if t >= 400 and t < 600:
                return 180
            else:
                return 150

        #control = PI_Controller(pulse.func, 3.8, 0.015)
        #control = PI_Controller(pulse.func, 3.8, 0.005,ff_func = model.ctl_for_steady_state)
        #control = PI_Controller(pulse.func, 4.0, 0.001, ff_func = model.ctl_for_steady_state)
        control = PI_Controller(pulse.func, 3.0, 0.0, ff_func = model.ctl_for_steady_state)
        model.ctl_func = control.func 

        t = scipy.linspace(0.0, 1500.0, 1000.0)
        T_sol = model.solve_fixed_step(model.T_amb, t)
        pylab.plot(t,T_sol)
        pylab.show()

    if 1:
        import reflow_profile

        model = Therm_Delay_Model()
        model.load_param('model_param.pkl')
        profile = reflow_profile.Reflow_Profile(model.T_amb)
        stop_t = profile.stop_time()
        

        #control = PI_Controller(profile.func, 3.8, 0.015)
        #control = PI_Controller(profile.func, 3.8, 0.005,ff_func = model.ctl_for_steady_state)
        control = PI_Controller(profile.func, 4.0, 0.0, ff_func = model.ctl_for_steady_state)
        model.ctl_func = control.func 

        t = scipy.linspace(0.0, stop_t, 1000.0)
        T_profile = [profile.func(tt-model.t_delay) for tt in t]
        T_sol = model.solve_fixed_step(model.T_amb, t)
        pylab.plot(t,T_profile,'r')
        pylab.plot(t,T_sol,'b')
        pylab.show()


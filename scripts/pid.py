class PID:
    def __init__(self, p=1, i=1, d=1, minI=-1, maxI=1, minOut = -2, maxOut = 2):
        self.P = p
        self.I = i
        self.D = d
        self.minI = minI
        self.maxI = maxI
        self.minOut = minOut
        self.maxOut = maxOut

        self.input = 0
        self.error = 0
        self.setpoint = 0
        self.integral = 0

        self.output = 0

    
    def updateSetpoint(self, sp):
        self.setpoint = sp
        self.integral = 0

    def update(self, input):
        self.input = input

        error = self.setpoint - self.input
        D = self.D * (self.error - error)

        self.error = error

        self.integral += self.error

        P = self.P * self.error
        I = self.I * self.integral
        I = min(self.maxI, max(self.minI, I))
        

        print(self.setpoint, self.input, self.error, P, I, D)
        
        self.output = P + I + D
        self.output = min(self.maxOut, max(self.minOut, self.output))
        return self.output
        
# coding:utf-8

import time


class PID:
    '''
    PID算法实现类
    '''

    def __init__(self):
        # 初始化增益
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.Initialize()

    def SetKp(self, invar):
        '''设置比例增益'''
        self.Kp = invar

    def SetKi(self, invar):
        '''设置积分增益'''
        self.Ki = invar

    def SetKd(self, invar):
        '''设置微分增益'''
        self.Kd = invar

    def SetPrevErr(self, preverr):
        '''设置上一个偏差'''
        self.prev_err = preverr

    def Initialize(self):
        '''设置时间变量'''
        self.currtm = time.time()
        self.prevtm = self.currtm

        self.prev_err = 0

        # 各项结果
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0

    def GetOut(self, error):
        '''进行PID运算并根据时间的增量（dt）和偏差值（参数error）生成控制变量'''
        self.currtm = time.time()
        dt = self.currtm - self.prevtm
        de = error - self.prev_err

        self.Cp = self.Kp * error
        self.Ci += error * dt

        self.Cd = 0
        if dt > 0:
            self.Cd = de / dt

        self.prevtm = self.currtm
        self.prev_err = error
        return self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)


# unit testcase
Kp = 1
Ki = 2
Kd = 3
pid = PID()
pid.SetKp(Kp)
pid.SetKi(Ki)
pid.SetKd(Kd)
print pid.GetOut(10)
#!/usr/bin/env python

import skfuzzy as fuzz
from skfuzzy import control as ctrl
import numpy as np
import matplotlib.pyplot as plt

autonomy = 0

def inicializaFuzzy():
    global autonomy

    #INPUTS
    MyoValue = ctrl.Antecedent(np.arange(20,160,0.005),'MyoValue')
    JoyLinear = ctrl.Antecedent(np.arange(-1,1,0.005),'JoyLinear')
    JoyAngular = ctrl.Antecedent(np.arange(-2,2,0.005),'JoyTheta')
    WeldPos = ctrl.Antecedent(np.arange(-1,1,0.005),'WeldPos')

    #OUTPUTS
    LoA = ctrl.Consequent(np.arange(1,4,0.1),'LoA')


    #MEMBERSHIP FUNCTIONS
    #myo emg rms mean value
    MyoValue['Low'] = fuzz.trimf(MyoValue.universe,[20, 20, 40])
    MyoValue['MedLow'] = fuzz.trimf(MyoValue.universe,[20, 40, 60])
    MyoValue['Medium'] = fuzz.trimf(MyoValue.universe,[40, 60, 80])
    MyoValue['MedHigh'] = fuzz.trimf(MyoValue.universe,[60, 80, 100])
    MyoValue['High'] = fuzz.trimf(MyoValue.universe,[80, 160, 160])

    #velocity linear - joystick
    JoyLinear['NegHigh'] = fuzz.trimf(JoyLinear.universe,[-1, -1, -0.5])    
    JoyLinear['NegLow'] = fuzz.trimf(JoyLinear.universe,[-0.5, -0.1, 0])
    JoyLinear['Zero'] = fuzz.trimf(JoyLinear.universe,[-0.1, 0, 0.1])
    JoyLinear['PosLow'] = fuzz.trimf(JoyLinear.universe,[0, 0.1, 0.5])
    JoyLinear['PosHigh'] = fuzz.trimf(JoyLinear.universe,[0.5, 1, 1])

    #velocity angular - joystick
    JoyAngular['LeftHigh'] = fuzz.trimf(JoyAngular.universe,[-2, -2, -1])    
    JoyAngular['LeftLow'] = fuzz.trimf(JoyAngular.universe,[-1, -0.5, 0])
    JoyAngular['Center'] = fuzz.trimf(JoyAngular.universe,[-0.5, 0, 0.5])
    JoyAngular['RightLow'] = fuzz.trimf(JoyAngular.universe,[0, 0.5, 1])
    JoyAngular['RightHigh'] = fuzz.trimf(JoyAngular.universe,[1, 2, 2])

    #weld position
    WeldPos['LeftHigh'] = fuzz.trimf(WeldPos.universe,[-1, -1, -0.5])    
    WeldPos['LeftLow'] = fuzz.trimf(WeldPos.universe,[-0.5, -0.1, 0])
    WeldPos['Center'] = fuzz.trimf(WeldPos.universe,[-0.5, 0, 0.5])
    WeldPos['RightLow'] = fuzz.trimf(WeldPos.universe,[0, 0.1, 0.5])
    WeldPos['RightHigh'] = fuzz.trimf(WeldPos.universe,[0.5, 1, 1])

    #Level of Autonomy - LoA
    LoA['Manual'] = fuzz.trimf(LoA.universe,[1, 1, 2])
    LoA['Shared'] = fuzz.trimf(LoA.universe,[1, 2, 3])
    LoA['Supervisory'] = fuzz.trimf(LoA.universe,[2, 3, 4])
    LoA['Autonomous'] = fuzz.trimf(LoA.universe,[3, 4, 4])


    #RULES

    rule1 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftHigh'],LoA['Manual'])
    rule2 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftLow'],LoA['Shared'])
    rule3 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['Center'],LoA['Supervisory'])
    rule4 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightLow'],LoA['Autonomous'])
    rule5 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightHigh'],LoA['Autonomous'])

    rule6 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftHigh'],LoA['Shared'])
    rule7 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftLow'],LoA['Manual'])
    rule8 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['Center'],LoA['Shared'])
    rule9 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightLow'],LoA['Supervisory'])
    rule10 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightHigh'],LoA['Autonomous'])

    rule11 = ctrl.Rule(JoyAngular['Center'] & WeldPos['LeftHigh'],LoA['Supervisory'])
    rule12 = ctrl.Rule(JoyAngular['Center'] & WeldPos['LeftLow'],LoA['Shared'])
    rule13 = ctrl.Rule(JoyAngular['Center'] & WeldPos['Center'],LoA['Manual'])
    rule14 = ctrl.Rule(JoyAngular['Center'] & WeldPos['RightLow'],LoA['Shared'])
    rule15 = ctrl.Rule(JoyAngular['Center'] & WeldPos['RightHigh'],LoA['Supervisory'])

    rule16 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['LeftHigh'],LoA['Autonomous'])
    rule17 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['LeftLow'],LoA['Supervisory'])
    rule18 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['Center'],LoA['Shared'])
    rule19 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['RightLow'],LoA['Manual'])
    rule20 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['RightHigh'],LoA['Shared'])

    rule21 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['LeftHigh'],LoA['Autonomous'])
    rule22 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['LeftLow'],LoA['Autonomous'])
    rule23 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['Center'],LoA['Supervisory'])
    rule24 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['RightLow'],LoA['Shared'])
    rule25 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['RightHigh'],LoA['Manual'])

    rule26 = ctrl.Rule(MyoValue['Low'],LoA['Manual'])
    rule27 = ctrl.Rule(MyoValue['MedLow'],LoA['Manual'])
    rule28 = ctrl.Rule(MyoValue['Medium'],LoA['Shared'])
    rule29 = ctrl.Rule(MyoValue['MedHigh'],LoA['Supervisory'])
    rule30 = ctrl.Rule(MyoValue['High'],LoA['Autonomous'])


    #CONTROL
    autonomy_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10,
                rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20,
                rule21, rule22, rule23, rule24, rule25, rule26, rule27, rule28, rule29, rule30
                ])
    autonomy = ctrl.ControlSystemSimulation(autonomy_ctrl)

    return

def calculateAutonomy(myo,jlinear,jangular,wpos):
    global autonomy

    autonomy.input['MyoValue'] = myo
    autonomy.input['JoyLinear'] = jlinear
    autonomy.input['JoyAngular'] = jangular
    autonomy.input['WeldPos'] = wpos
    autonomy.compute()

    autonomy_level = autonomy.output['LoA']

    return autonomy_level
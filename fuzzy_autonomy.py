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
    LoA = ctrl.Consequent(np.arange(1,4,1),'LoA')


    #MEMBERSHIP FUNCTIONS
    #myo emg rms mean value
    MyoValue['Low'] = fuzz.trimf(MyoValue.universe,[20, 20, 40])
    MyoValue['MedLow'] = fuzz.trimf(MyoValue.universe,[20, 40, 60])
    MyoValue['Medium'] = fuzz.trimf(MyoValue.universe,[40, 60, 80])
    MyoValue['MedHigh'] = fuzz.trimf(MyoValue.universe,[60, 80, 100])
    MyoValue['High'] = fuzz.trimf(MyoValue.universe,[80, 160, 160])

    #velocity linear - joystick
    JoyLinear['NegHigh'] = fuzz.trimf(JoyLinear.universe,[-1, -1, -0.5])    
    JoyLinear['NegMedium'] = fuzz.trimf(JoyLinear.universe,[-1, -0.5, -0.1])
    JoyLinear['NegLow'] = fuzz.trimf(JoyLinear.universe,[-0.5, -0.1, 0])
    JoyLinear['Zero'] = fuzz.trimf(JoyLinear.universe,[-0.1, 0, 0.1])
    JoyLinear['PosLow'] = fuzz.trimf(JoyLinear.universe,[0, 0.1, 0.5])
    JoyLinear['PosMedium'] = fuzz.trimf(JoyLinear.universe,[0.1, 0.5, 1])
    JoyLinear['PosHigh'] = fuzz.trimf(JoyLinear.universe,[0.5, 1, 1])

    #velocity angular - joystick
    JoyAngular['LeftHigh'] = fuzz.trimf(JoyAngular.universe,[-2, -2, -1])    
    JoyAngular['LeftMedium'] = fuzz.trimf(JoyAngular.universe,[-2, -1, -0.5])
    JoyAngular['LeftLow'] = fuzz.trimf(JoyAngular.universe,[-1, -0.5, 0])
    JoyAngular['Center'] = fuzz.trimf(JoyAngular.universe,[-0.5, 0, 0.5])
    JoyAngular['RightLow'] = fuzz.trimf(JoyAngular.universe,[0, 0.5, 1])
    JoyAngular['RightMedium'] = fuzz.trimf(JoyAngular.universe,[0.5, 1, 2])
    JoyAngular['RightHigh'] = fuzz.trimf(JoyAngular.universe,[1, 2, 2])

    #weld position
    WeldPos['LeftHigh'] = fuzz.trimf(WeldPos.universe,[-1, -1, -0.5])    
    WeldPos['LeftMedium'] = fuzz.trimf(WeldPos.universe,[-1, -0.5, -0.1])
    WeldPos['LeftLow'] = fuzz.trimf(WeldPos.universe,[-0.5, -0.1, 0])
    WeldPos['Center'] = fuzz.trimf(WeldPos.universe,[-0.5, 0, 0.5])
    WeldPos['RightLow'] = fuzz.trimf(WeldPos.universe,[0, 0.1, 0.5])
    WeldPos['RightMedium'] = fuzz.trimf(WeldPos.universe,[0.1, 0.5, 1])
    WeldPos['RightHigh'] = fuzz.trimf(WeldPos.universe,[0.5, 1, 1])

    #Level of Autonomy - LoA
    LoA['Manual'] = fuzz.trimf(LoA.universe,[1, 1, 2])
    LoA['Shared'] = fuzz.trimf(LoA.universe,[1, 2, 3])
    LoA['Supervisory'] = fuzz.trimf(LoA.universe,[2, 3, 4])
    LoA['Autonomous'] = fuzz.trimf(LoA.universe,[3, 4, 4])


    #RULES

    rule1 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftHigh'] & MyoValue['Low'],LoA['Manual'])
    rule2 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftHigh'] & MyoValue['MedLow'],LoA['Manual'])
    rule3 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftHigh'] & MyoValue['Medium'],LoA['Manual'])
    rule4 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftHigh'] & MyoValue['MedHigh'],LoA['Shared'])
    rule5 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftHigh'] & MyoValue['High'],LoA['Supervisory'])
    rule6 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftMedium'] & MyoValue['Low'],LoA['Manual'])
    rule7 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftMedium'] & MyoValue['MedLow'],LoA['Manual'])
    rule8 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftMedium'] & MyoValue['Medium'],LoA['Manual'])
    rule9 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftMedium'] & MyoValue['MedHigh'],LoA['Shared'])
    rule10 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftMedium'] & MyoValue['High'],LoA['Supervisory'])
    rule11 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftLow'] & MyoValue['Low'],LoA['Shared'])
    rule12 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftLow'] & MyoValue['MedLow'],LoA['Shared'])
    rule13 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftLow'] & MyoValue['Medium'],LoA['Shared'])
    rule14 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftLow'] & MyoValue['MedHigh'],LoA['Shared'])
    rule15 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftLow'] & MyoValue['High'],LoA['Shared'])
    rule16 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['Center'] & MyoValue['Low'],LoA['Shared'])
    rule17 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['Center'] & MyoValue['MedLow'],LoA['Shared'])
    rule18 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['Center'] & MyoValue['Medium'],LoA['Shared'])
    rule19 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['Center'] & MyoValue['MedHigh'],LoA['Shared'])
    rule20 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['Center'] & MyoValue['High'],LoA['Shared'])
    rule21 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightLow'] & MyoValue['Low'],LoA['Supervisory'])
    rule22 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightLow'] & MyoValue['MedLow'],LoA['Supervisory'])
    rule23 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightLow'] & MyoValue['Medium'],LoA['Supervisory'])
    rule24 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightLow'] & MyoValue['MedHigh'],LoA['Supervisory'])
    rule25 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightLow'] & MyoValue['High'],LoA['Supervisory'])
    rule26 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightMedium'] & MyoValue['Low'],LoA['Supervisory'])
    rule27 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightMedium'] & MyoValue['MedLow'],LoA['Supervisory'])
    rule28 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightMedium'] & MyoValue['Medium'],LoA['Supervisory'])
    rule29 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightMedium'] & MyoValue['MedHigh'],LoA['Supervisory'])
    rule30 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightMedium'] & MyoValue['High'],LoA['Supervisory'])
    rule31 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightHigh'] & MyoValue['Low'],LoA['Autonomous'])
    rule32 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightHigh'] & MyoValue['MedLow'],LoA['Autonomous'])
    rule33 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightHigh'] & MyoValue['Medium'],LoA['Autonomous'])
    rule34 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightHigh'] & MyoValue['MedHigh'],LoA['Autonomous'])
    rule35 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightHigh'] & MyoValue['High'],LoA['Autonomous'])

    rule36 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftHigh'] & MyoValue['Low'],LoA['Manual'])
    rule37 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftHigh'] & MyoValue['MedLow'],LoA['Manual'])
    rule38 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftHigh'] & MyoValue['Medium'],LoA['Manual'])
    rule39 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftHigh'] & MyoValue['MedHigh'],LoA['Shared'])
    rule40 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftHigh'] & MyoValue['High'],LoA['Supervisory'])
    rule41 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftMedium'] & MyoValue['Low'],LoA['Manual'])
    rule42 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftMedium'] & MyoValue['MedLow'],LoA['Manual'])
    rule43 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftMedium'] & MyoValue['Medium'],LoA['Manual'])
    rule44 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftMedium'] & MyoValue['MedHigh'],LoA['Shared'])
    rule45 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftMedium'] & MyoValue['High'],LoA['Supervisory'])
    rule46 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftLow'] & MyoValue['Low'],LoA['Shared'])
    rule47 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftLow'] & MyoValue['MedLow'],LoA['Shared'])
    rule48 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftLow'] & MyoValue['Medium'],LoA['Shared'])
    rule49 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftLow'] & MyoValue['MedHigh'],LoA['Shared'])
    rule50 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftLow'] & MyoValue['High'],LoA['Shared'])
    rule51 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['Center'] & MyoValue['Low'],LoA['Shared'])
    rule52 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['Center'] & MyoValue['MedLow'],LoA['Shared'])
    rule53 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['Center'] & MyoValue['Medium'],LoA['Shared'])
    rule54 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['Center'] & MyoValue['MedHigh'],LoA['Shared'])
    rule55 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['Center'] & MyoValue['High'],LoA['Shared'])
    rule56 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightLow'] & MyoValue['Low'],LoA['Supervisory'])
    rule57 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightLow'] & MyoValue['MedLow'],LoA['Supervisory'])
    rule58 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightLow'] & MyoValue['Medium'],LoA['Supervisory'])
    rule59 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightLow'] & MyoValue['MedHigh'],LoA['Supervisory'])
    rule60 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightLow'] & MyoValue['High'],LoA['Supervisory'])
    rule61 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightMedium'] & MyoValue['Low'],LoA['Supervisory'])
    rule62 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightMedium'] & MyoValue['MedLow'],LoA['Supervisory'])
    rule63 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightMedium'] & MyoValue['Medium'],LoA['Supervisory'])
    rule64 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightMedium'] & MyoValue['MedHigh'],LoA['Supervisory'])
    rule65 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightMedium'] & MyoValue['High'],LoA['Supervisory'])
    rule66 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightHigh'] & MyoValue['Low'],LoA['Autonomous'])
    rule67 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightHigh'] & MyoValue['MedLow'],LoA['Autonomous'])
    rule68 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightHigh'] & MyoValue['Medium'],LoA['Autonomous'])
    rule69 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightHigh'] & MyoValue['MedHigh'],LoA['Autonomous'])
    rule70 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightHigh'] & MyoValue['High'],LoA['Autonomous'])

    rule71 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftHigh'] & MyoValue['Low'],LoA['Manual'])
    rule72 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftHigh'] & MyoValue['MedLow'],LoA['Manual'])
    rule73 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftHigh'] & MyoValue['Medium'],LoA['Manual'])
    rule74 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftHigh'] & MyoValue['MedHigh'],LoA['Shared'])
    rule75 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftHigh'] & MyoValue['High'],LoA['Supervisory'])
    rule76 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftMedium'] & MyoValue['Low'],LoA['Manual'])
    rule77 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftMedium'] & MyoValue['MedLow'],LoA['Manual'])
    rule78 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftMedium'] & MyoValue['Medium'],LoA['Manual'])
    rule79 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftMedium'] & MyoValue['MedHigh'],LoA['Shared'])
    rule80 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftMedium'] & MyoValue['High'],LoA['Supervisory'])
    rule81 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftLow'] & MyoValue['Low'],LoA['Shared'])
    rule82 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftLow'] & MyoValue['MedLow'],LoA['Shared'])
    rule83 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftLow'] & MyoValue['Medium'],LoA['Shared'])
    rule84 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftLow'] & MyoValue['MedHigh'],LoA['Shared'])
    rule85 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftLow'] & MyoValue['High'],LoA['Shared'])
    rule86 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['Center'] & MyoValue['Low'],LoA['Shared'])
    rule87 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['Center'] & MyoValue['MedLow'],LoA['Shared'])
    rule88 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['Center'] & MyoValue['Medium'],LoA['Shared'])
    rule89 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['Center'] & MyoValue['MedHigh'],LoA['Shared'])
    rule90 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['Center'] & MyoValue['High'],LoA['Shared'])
    rule91 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightLow'] & MyoValue['Low'],LoA['Supervisory'])
    rule92 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightLow'] & MyoValue['MedLow'],LoA['Supervisory'])
    rule93 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightLow'] & MyoValue['Medium'],LoA['Supervisory'])
    rule94 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightLow'] & MyoValue['MedHigh'],LoA['Supervisory'])
    rule95 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightLow'] & MyoValue['High'],LoA['Supervisory'])
    rule96 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightMedium'] & MyoValue['Low'],LoA['Supervisory'])
    rule97 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightMedium'] & MyoValue['MedLow'],LoA['Supervisory'])
    rule98 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightMedium'] & MyoValue['Medium'],LoA['Supervisory'])
    rule99 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightMedium'] & MyoValue['MedHigh'],LoA['Supervisory'])
    rule100 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightMedium'] & MyoValue['High'],LoA['Supervisory'])
    rule101 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightHigh'] & MyoValue['Low'],LoA['Autonomous'])
    rule102 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightHigh'] & MyoValue['MedLow'],LoA['Autonomous'])
    rule103 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightHigh'] & MyoValue['Medium'],LoA['Autonomous'])
    rule104 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightHigh'] & MyoValue['MedHigh'],LoA['Autonomous'])
    rule105 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightHigh'] & MyoValue['High'],LoA['Autonomous'])

    rule22 = ctrl.Rule(JoyAngular['Center'] & WeldPos['LeftHigh'],LoA['Autonomous'])
    rule23 = ctrl.Rule(JoyAngular['Center'] & WeldPos['LeftMedium'],LoA['Supervisory'])
    rule24 = ctrl.Rule(JoyAngular['Center'] & WeldPos['LeftLow'],LoA['Shared'])
    rule25 = ctrl.Rule(JoyAngular['Center'] & WeldPos['Center'],LoA['Manual'])
    rule26 = ctrl.Rule(JoyAngular['Center'] & WeldPos['RightLow'],LoA['Shared'])
    rule27 = ctrl.Rule(JoyAngular['Center'] & WeldPos['RightMedium'],LoA['Supervisory'])
    rule28 = ctrl.Rule(JoyAngular['Center'] & WeldPos['RightHigh'],LoA['Autonomous'])

    rule29 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['LeftHigh'],LoA['Autonomous'])
    rule30 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['LeftMedium'],LoA['Supervisory'])
    rule31 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['LeftLow'],LoA['Shared'])
    rule32 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['Center'],LoA['Manual'])
    rule33 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['RightLow'],LoA['Manual'])
    rule34 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['RightMedium'],LoA['Manual'])
    rule35 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['RightHigh'],LoA['Shared'])

    rule36 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['LeftHigh'],LoA['Autonomous'])
    rule37 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['LeftMedium'],LoA['Supervisory'])
    rule38 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['LeftLow'],LoA['Shared'])
    rule39 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['Center'],LoA['Shared'])
    rule40 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['RightLow'],LoA['Manual'])
    rule41 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['RightMedium'],LoA['Manual'])
    rule42 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['RightHigh'],LoA['Manual'])

    rule43 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['LeftHigh'],LoA['Autonomous'])
    rule44 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['LeftMedium'],LoA['Supervisory'])
    rule45 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['LeftLow'],LoA['Supervisory'])
    rule46 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['Center'],LoA['Shared'])
    rule47 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['RightLow'],LoA['Shared'])
    rule48 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['RightMedium'],LoA['Manual'])
    rule49 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['RightHigh'],LoA['Manual'])


    #CONTROL
    autonomy_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10,
                rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20,
                rule21, rule22, rule23, rule24, rule25, rule26, rule27, rule28, rule29, rule30,
                rule31, rule32, rule33, rule34, rule35, rule36, rule37, rule38, rule39, rule40,
                rule41, rule42, rule43, rule44, rule45, rule46, rule47, rule48, rule49, rule50,
                rule51, rule52, rule53, rule54, rule55, rule56, rule57, rule58, rule59, rule60,
                rule61, rule62, rule63, rule64, rule65, rule66, rule67, rule68, rule69, rule70,
                rule71, rule72, rule73, rule74, rule75, rule76, rule77, rule78, rule79, rule80,
                rule81, rule82, rule83, rule84, rule85, rule86, rule87, rule88, rule89, rule90,
                rule91, rule92, rule93, rule94, rule95, rule96, rule97, rule98, rule99, rule100,
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
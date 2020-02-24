#!/usr/bin/env python

import skfuzzy as fuzz
from skfuzzy import control as ctrl
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Required for 3D plotting

#INPUTS
MyoValue = ctrl.Antecedent(np.arange(20,180,0.005),'MyoValue')
MyoRoll = ctrl.Antecedent(np.arange(-2,2,0.005),'MyoRoll')
JoyAngular = ctrl.Antecedent(np.arange(-5,5,0.005),'JoyAngular')
WeldPos = ctrl.Antecedent(np.arange(-1,1,0.005),'WeldPos')

#OUTPUTS
LoA = ctrl.Consequent(np.arange(0,4,0.1),'LoA')

#MEMBERSHIP FUNCTIONS
#myo emg rms mean value
MyoValue['Low'] = fuzz.trimf(MyoValue.universe,[20.000, 20.000, 60.000])
MyoValue['MedLow'] = fuzz.trimf(MyoValue.universe,[20.000, 60.000, 100.000])
MyoValue['Medium'] = fuzz.trimf(MyoValue.universe,[60.000, 100.000, 140.000])
MyoValue['MedHigh'] = fuzz.trimf(MyoValue.universe,[100.000, 140.000, 180.000])
MyoValue['High'] = fuzz.trimf(MyoValue.universe,[140.000, 180.000, 180.000])

#myo roll
MyoRoll['ACWHigh'] = fuzz.trapmf(MyoRoll.universe,[-2.000, -2.000, -0.500, -0.250])
MyoRoll['ACWLow'] = fuzz.trimf(MyoRoll.universe,[-0.500, -0.250, 0.000])
MyoRoll['Zero'] = fuzz.trimf(MyoRoll.universe,[-0.250, 0.000, 0.250])
MyoRoll['CWLow'] = fuzz.trimf(MyoRoll.universe,[0.000, 0.250, 0.500])
MyoRoll['CWHigh'] = fuzz.trapmf(MyoRoll.universe,[0.250, 0.500, 2.000, 2.000])

#velocity angular - joystick
JoyAngular['LeftHigh'] = fuzz.trapmf(JoyAngular.universe,[-5.000, -5.000, -2.000, -1.000])
JoyAngular['LeftLow'] = fuzz.trimf(JoyAngular.universe,[-2.000, -1.000, 0.000])
JoyAngular['Center'] = fuzz.trimf(JoyAngular.universe,[-1.000, 0.000, 1.000])
JoyAngular['RightLow'] = fuzz.trimf(JoyAngular.universe,[0.000, 1.000, 2.000])
JoyAngular['RightHigh'] = fuzz.trapmf(JoyAngular.universe,[1.000, 2.000, 5.000, 5.000])

#weld position
WeldPos['LeftHigh'] = fuzz.trimf(WeldPos.universe,[-1.000, -1.000, -0.500])
WeldPos['LeftLow'] = fuzz.trimf(WeldPos.universe,[-1.000, -0.500, 0.000])
WeldPos['Center'] = fuzz.trimf(WeldPos.universe,[-0.500, 0.000, 0.500])
WeldPos['RightLow'] = fuzz.trimf(WeldPos.universe,[0.000, 0.500, 1.000])
WeldPos['RightHigh'] = fuzz.trimf(WeldPos.universe,[0.500, 1.000, 1.000])

#Level of Autonomy - LoA
LoA['Manual'] = fuzz.trapmf(LoA.universe,[0.000, 0.000, 0.500, 1.500])
LoA['Shared'] = fuzz.trimf(LoA.universe,[0.500, 1.500, 2.500])
LoA['Supervisory'] = fuzz.trimf(LoA.universe,[1.500, 2.500, 3.500])
LoA['Autonomous'] = fuzz.trapmf(LoA.universe,[2.500, 3.500, 4.000, 4.000])

#MyoValue.view()
#MyoRoll.view()
#JoyAngular.view()
#WeldPos.view()
#LoA.view()

#plt.show()


#RULES
#Joy Angular e Weld Position

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

# Myo Roll e Weld Position

rule26 = ctrl.Rule(MyoRoll['ACWHigh'] & WeldPos['LeftHigh'],LoA['Manual'])
rule27 = ctrl.Rule(MyoRoll['ACWHigh'] & WeldPos['LeftLow'],LoA['Shared'])
rule28 = ctrl.Rule(MyoRoll['ACWHigh'] & WeldPos['Center'],LoA['Supervisory'])
rule29 = ctrl.Rule(MyoRoll['ACWHigh'] & WeldPos['RightLow'],LoA['Autonomous'])
rule30 = ctrl.Rule(MyoRoll['ACWHigh'] & WeldPos['RightHigh'],LoA['Autonomous'])

rule31 = ctrl.Rule(MyoRoll['ACWLow'] & WeldPos['LeftHigh'],LoA['Shared'])
rule32 = ctrl.Rule(MyoRoll['ACWLow'] & WeldPos['LeftLow'],LoA['Manual'])
rule33 = ctrl.Rule(MyoRoll['ACWLow'] & WeldPos['Center'],LoA['Shared'])
rule34 = ctrl.Rule(MyoRoll['ACWLow'] & WeldPos['RightLow'],LoA['Supervisory'])
rule35 = ctrl.Rule(MyoRoll['ACWLow'] & WeldPos['RightHigh'],LoA['Autonomous'])

rule36 = ctrl.Rule(MyoRoll['Zero'] & WeldPos['LeftHigh'],LoA['Supervisory'])
rule37 = ctrl.Rule(MyoRoll['Zero'] & WeldPos['LeftLow'],LoA['Shared'])
rule38 = ctrl.Rule(MyoRoll['Zero'] & WeldPos['Center'],LoA['Manual'])
rule39 = ctrl.Rule(MyoRoll['Zero'] & WeldPos['RightLow'],LoA['Shared'])
rule40 = ctrl.Rule(MyoRoll['Zero'] & WeldPos['RightHigh'],LoA['Supervisory'])

rule41 = ctrl.Rule(MyoRoll['CWLow'] & WeldPos['LeftHigh'],LoA['Autonomous'])
rule42 = ctrl.Rule(MyoRoll['CWLow'] & WeldPos['LeftLow'],LoA['Supervisory'])
rule43 = ctrl.Rule(MyoRoll['CWLow'] & WeldPos['Center'],LoA['Shared'])
rule44 = ctrl.Rule(MyoRoll['CWLow'] & WeldPos['RightLow'],LoA['Manual'])
rule45 = ctrl.Rule(MyoRoll['CWLow'] & WeldPos['RightHigh'],LoA['Shared'])

rule46 = ctrl.Rule(MyoRoll['CWHigh'] & WeldPos['LeftHigh'],LoA['Autonomous'])
rule47 = ctrl.Rule(MyoRoll['CWHigh'] & WeldPos['LeftLow'],LoA['Autonomous'])
rule48 = ctrl.Rule(MyoRoll['CWHigh'] & WeldPos['Center'],LoA['Supervisory'])
rule49 = ctrl.Rule(MyoRoll['CWHigh'] & WeldPos['RightLow'],LoA['Shared'])
rule50 = ctrl.Rule(MyoRoll['CWHigh'] & WeldPos['RightHigh'],LoA['Manual'])

#rule51 = ctrl.Rule(MyoValue['Low'],LoA['Manual'])
#rule52 = ctrl.Rule(MyoValue['MedLow'],LoA['Manual'])
#rule53 = ctrl.Rule(MyoValue['Medium'],LoA['Shared'])
rule54 = ctrl.Rule(MyoValue['MedHigh'],LoA['Autonomous'])
rule55 = ctrl.Rule(MyoValue['High'],LoA['Autonomous'])


#CONTROL
autonomy_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10,
                                    rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20,
                                    rule21, rule22, rule23, rule24, rule25, rule26, rule27, rule28, rule29, rule30,
                                    rule31, rule32, rule33, rule34, rule35, rule36, rule37, rule38, rule39, rule40,
                                    rule41, rule42, rule43, rule44, rule45, rule46, rule47, rule48, rule49, rule50,
                                    rule53, rule54, rule55
                                    ])
autonomy = ctrl.ControlSystemSimulation(autonomy_ctrl)

def calculateAutonomy(myo_rms,joy_angular,weld_pos, myo_roll):
    global autonomy

    autonomy.input['MyoValue'] = myo_rms
    autonomy.input['JoyAngular'] = joy_angular
    autonomy.input['WeldPos'] = weld_pos
    autonomy.input['MyoRoll'] = myo_roll
    autonomy.compute()

    #para visualizar dados manualmente
    print autonomy.output['LoA']
    LoA.view(sim=autonomy)
    plt.show()

    autonomy_level = autonomy.output['LoA']

    return autonomy_level

calculateAutonomy(50,-4.7, 0.931, -1.92)
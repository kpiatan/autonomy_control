#!/usr/bin/env python

import skfuzzy as fuzz
from skfuzzy import control as ctrl
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # Required for 3D plotting

#INPUTS
MyoRMS = ctrl.Antecedent(np.arange(20,180,0.005),'MyoRMS')
MyoRoll = ctrl.Antecedent(np.arange(-2,2,0.005),'MyoRoll')
JoyAngular = ctrl.Antecedent(np.arange(-5,5,0.005),'JoyAngular')
WeldPos = ctrl.Antecedent(np.arange(-1,1,0.005),'WeldPos')

#OUTPUTS
LoA = ctrl.Consequent(np.arange(0,4,0.1),'LoA')

#MEMBERSHIP FUNCTIONS
#myo emg rms mean value
MyoRMS['Baixo'] = fuzz.trimf(MyoRMS.universe,[20.000, 20.000, 60.000])
MyoRMS['MedBaixo'] = fuzz.trimf(MyoRMS.universe,[20.000, 60.000, 100.000])
MyoRMS['Medio'] = fuzz.trimf(MyoRMS.universe,[60.000, 100.000, 140.000])
MyoRMS['MedAlto'] = fuzz.trimf(MyoRMS.universe,[100.000, 140.000, 180.000])
MyoRMS['Alto'] = fuzz.trimf(MyoRMS.universe,[140.000, 180.000, 180.000])

#myo roll
MyoRoll['AntiHorarioAlto'] = fuzz.trapmf(MyoRoll.universe,[-2.000, -2.000, -0.500, -0.250])
MyoRoll['AntiHorarioBaixo'] = fuzz.trimf(MyoRoll.universe,[-0.500, -0.250, 0.000])
MyoRoll['Zero'] = fuzz.trimf(MyoRoll.universe,[-0.250, 0.000, 0.250])
MyoRoll['HorarioBaixo'] = fuzz.trimf(MyoRoll.universe,[0.000, 0.250, 0.500])
MyoRoll['HorarioAlto'] = fuzz.trapmf(MyoRoll.universe,[0.250, 0.500, 2.000, 2.000])

#velocity angular - joystick
JoyAngular['EsquerdaAlto'] = fuzz.trapmf(JoyAngular.universe,[-5.000, -5.000, -2.000, -1.000])
JoyAngular['EsquerdaBaixo'] = fuzz.trimf(JoyAngular.universe,[-2.000, -1.000, 0.000])
JoyAngular['Centro'] = fuzz.trimf(JoyAngular.universe,[-1.000, 0.000, 1.000])
JoyAngular['DireitaBaixo'] = fuzz.trimf(JoyAngular.universe,[0.000, 1.000, 2.000])
JoyAngular['DireitaAlto'] = fuzz.trapmf(JoyAngular.universe,[1.000, 2.000, 5.000, 5.000])

#weld position
WeldPos['EsquerdaAlto'] = fuzz.trimf(WeldPos.universe,[-1.000, -1.000, -0.500])
WeldPos['EsquerdaBaixo'] = fuzz.trimf(WeldPos.universe,[-1.000, -0.500, 0.000])
WeldPos['Centro'] = fuzz.trimf(WeldPos.universe,[-0.500, 0.000, 0.500])
WeldPos['DireitaBaixo'] = fuzz.trimf(WeldPos.universe,[0.000, 0.500, 1.000])
WeldPos['DireitaAlto'] = fuzz.trimf(WeldPos.universe,[0.500, 1.000, 1.000])

#Level of Autonomy - LoA
LoA['Manual'] = fuzz.trapmf(LoA.universe,[0.000, 0.000, 0.500, 1.500])
LoA['Compartilhado'] = fuzz.trimf(LoA.universe,[0.500, 1.500, 2.500])
LoA['Supervisorio'] = fuzz.trimf(LoA.universe,[1.500, 2.500, 3.500])
LoA['Autonomo'] = fuzz.trapmf(LoA.universe,[2.500, 3.500, 4.000, 4.000])

#MyoRMS.view()
#MyoRoll.view()
#JoyAngular.view()
#WeldPos.view()
#LoA.view()

#plt.show()


#RULES
#Joy Angular e Weld Position

rule1 = ctrl.Rule(JoyAngular['EsquerdaAlto'] & WeldPos['EsquerdaAlto'],LoA['Manual'])
rule2 = ctrl.Rule(JoyAngular['EsquerdaAlto'] & WeldPos['EsquerdaBaixo'],LoA['Compartilhado'])
rule3 = ctrl.Rule(JoyAngular['EsquerdaAlto'] & WeldPos['Centro'],LoA['Supervisorio'])
rule4 = ctrl.Rule(JoyAngular['EsquerdaAlto'] & WeldPos['DireitaBaixo'],LoA['Autonomo'])
rule5 = ctrl.Rule(JoyAngular['EsquerdaAlto'] & WeldPos['DireitaAlto'],LoA['Autonomo'])

rule6 = ctrl.Rule(JoyAngular['EsquerdaBaixo'] & WeldPos['EsquerdaAlto'],LoA['Compartilhado'])
rule7 = ctrl.Rule(JoyAngular['EsquerdaBaixo'] & WeldPos['EsquerdaBaixo'],LoA['Manual'])
rule8 = ctrl.Rule(JoyAngular['EsquerdaBaixo'] & WeldPos['Centro'],LoA['Compartilhado'])
rule9 = ctrl.Rule(JoyAngular['EsquerdaBaixo'] & WeldPos['DireitaBaixo'],LoA['Supervisorio'])
rule10 = ctrl.Rule(JoyAngular['EsquerdaBaixo'] & WeldPos['DireitaAlto'],LoA['Autonomo'])

rule11 = ctrl.Rule(JoyAngular['Centro'] & WeldPos['EsquerdaAlto'],LoA['Supervisorio'])
rule12 = ctrl.Rule(JoyAngular['Centro'] & WeldPos['EsquerdaBaixo'],LoA['Compartilhado'])
rule13 = ctrl.Rule(JoyAngular['Centro'] & WeldPos['Centro'],LoA['Manual'])
rule14 = ctrl.Rule(JoyAngular['Centro'] & WeldPos['DireitaBaixo'],LoA['Compartilhado'])
rule15 = ctrl.Rule(JoyAngular['Centro'] & WeldPos['DireitaAlto'],LoA['Supervisorio'])

rule16 = ctrl.Rule(JoyAngular['DireitaBaixo'] & WeldPos['EsquerdaAlto'],LoA['Autonomo'])
rule17 = ctrl.Rule(JoyAngular['DireitaBaixo'] & WeldPos['EsquerdaBaixo'],LoA['Supervisorio'])
rule18 = ctrl.Rule(JoyAngular['DireitaBaixo'] & WeldPos['Centro'],LoA['Compartilhado'])
rule19 = ctrl.Rule(JoyAngular['DireitaBaixo'] & WeldPos['DireitaBaixo'],LoA['Manual'])
rule20 = ctrl.Rule(JoyAngular['DireitaBaixo'] & WeldPos['DireitaAlto'],LoA['Compartilhado'])

rule21 = ctrl.Rule(JoyAngular['DireitaAlto'] & WeldPos['EsquerdaAlto'],LoA['Autonomo'])
rule22 = ctrl.Rule(JoyAngular['DireitaAlto'] & WeldPos['EsquerdaBaixo'],LoA['Autonomo'])
rule23 = ctrl.Rule(JoyAngular['DireitaAlto'] & WeldPos['Centro'],LoA['Supervisorio'])
rule24 = ctrl.Rule(JoyAngular['DireitaAlto'] & WeldPos['DireitaBaixo'],LoA['Compartilhado'])
rule25 = ctrl.Rule(JoyAngular['DireitaAlto'] & WeldPos['DireitaAlto'],LoA['Manual'])

# Myo Roll e Weld Position

rule26 = ctrl.Rule(MyoRoll['AntiHorarioAlto'] & WeldPos['EsquerdaAlto'],LoA['Manual'])
rule27 = ctrl.Rule(MyoRoll['AntiHorarioAlto'] & WeldPos['EsquerdaBaixo'],LoA['Compartilhado'])
rule28 = ctrl.Rule(MyoRoll['AntiHorarioAlto'] & WeldPos['Centro'],LoA['Supervisorio'])
rule29 = ctrl.Rule(MyoRoll['AntiHorarioAlto'] & WeldPos['DireitaBaixo'],LoA['Autonomo'])
rule30 = ctrl.Rule(MyoRoll['AntiHorarioAlto'] & WeldPos['DireitaAlto'],LoA['Autonomo'])

rule31 = ctrl.Rule(MyoRoll['AntiHorarioBaixo'] & WeldPos['EsquerdaAlto'],LoA['Compartilhado'])
rule32 = ctrl.Rule(MyoRoll['AntiHorarioBaixo'] & WeldPos['EsquerdaBaixo'],LoA['Manual'])
rule33 = ctrl.Rule(MyoRoll['AntiHorarioBaixo'] & WeldPos['Centro'],LoA['Compartilhado'])
rule34 = ctrl.Rule(MyoRoll['AntiHorarioBaixo'] & WeldPos['DireitaBaixo'],LoA['Supervisorio'])
rule35 = ctrl.Rule(MyoRoll['AntiHorarioBaixo'] & WeldPos['DireitaAlto'],LoA['Autonomo'])

rule36 = ctrl.Rule(MyoRoll['Zero'] & WeldPos['EsquerdaAlto'],LoA['Supervisorio'])
rule37 = ctrl.Rule(MyoRoll['Zero'] & WeldPos['EsquerdaBaixo'],LoA['Compartilhado'])
rule38 = ctrl.Rule(MyoRoll['Zero'] & WeldPos['Centro'],LoA['Manual'])
rule39 = ctrl.Rule(MyoRoll['Zero'] & WeldPos['DireitaBaixo'],LoA['Compartilhado'])
rule40 = ctrl.Rule(MyoRoll['Zero'] & WeldPos['DireitaAlto'],LoA['Supervisorio'])

rule41 = ctrl.Rule(MyoRoll['HorarioBaixo'] & WeldPos['EsquerdaAlto'],LoA['Autonomo'])
rule42 = ctrl.Rule(MyoRoll['HorarioBaixo'] & WeldPos['EsquerdaBaixo'],LoA['Supervisorio'])
rule43 = ctrl.Rule(MyoRoll['HorarioBaixo'] & WeldPos['Centro'],LoA['Compartilhado'])
rule44 = ctrl.Rule(MyoRoll['HorarioBaixo'] & WeldPos['DireitaBaixo'],LoA['Manual'])
rule45 = ctrl.Rule(MyoRoll['HorarioBaixo'] & WeldPos['DireitaAlto'],LoA['Compartilhado'])

rule46 = ctrl.Rule(MyoRoll['HorarioAlto'] & WeldPos['EsquerdaAlto'],LoA['Autonomo'])
rule47 = ctrl.Rule(MyoRoll['HorarioAlto'] & WeldPos['EsquerdaBaixo'],LoA['Autonomo'])
rule48 = ctrl.Rule(MyoRoll['HorarioAlto'] & WeldPos['Centro'],LoA['Supervisorio'])
rule49 = ctrl.Rule(MyoRoll['HorarioAlto'] & WeldPos['DireitaBaixo'],LoA['Compartilhado'])
rule50 = ctrl.Rule(MyoRoll['HorarioAlto'] & WeldPos['DireitaAlto'],LoA['Manual'])

#rule51 = ctrl.Rule(MyoRMS['Baixo'],LoA['Manual'])
#rule52 = ctrl.Rule(MyoRMS['MedBaixo'],LoA['Manual'])
#rule53 = ctrl.Rule(MyoRMS['Medio'],LoA['Compartilhado'])
rule54 = ctrl.Rule(MyoRMS['MedAlto'],LoA['Supervisorio'])
rule55 = ctrl.Rule(MyoRMS['Alto'],LoA['Autonomo'])


#CONTROL
autonomy_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10,
                                    rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20,
                                    rule21, rule22, rule23, rule24, rule25, rule26, rule27, rule28, rule29, rule30,
                                    rule31, rule32, rule33, rule34, rule35, rule36, rule37, rule38, rule39, rule40,
                                    rule41, rule42, rule43, rule44, rule45, rule46, rule47, rule48, rule49, rule50,
                                    rule54, rule55
                                    ])
autonomy = ctrl.ControlSystemSimulation(autonomy_ctrl)

def calculateAutonomy(myo_rms,joy_angular,weld_pos, myo_roll):
    global autonomy

    autonomy.input['MyoRMS'] = myo_rms
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

calculateAutonomy(180, -2, 0.8, -1)
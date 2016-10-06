#! /usr/bin/env python

"""






"""

from inverseKinematics2DOF import ik2DOF
import hubo_ach as ha
import ach
import ach_py
import sys
import time
from ctypes import *
import numpy as np

class makeHuboBend(object):

    def __init__(self):


        increment = 15
        self.l1 = .30038
        self.l2 = .30003
        self.l3 = .18247


        s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
        r = ach.Channel(ha.HUBO_CHAN_REF_NAME)

        state = ha.HUBO_STATE()
        ref = ha.HUBO_REF()

        [status , framesize] = s.get(state,wait=False,last=True)
        currentStartingValues = self._returnDOFValues( state, s)

        print "\n\n\nWAITING TO COLLECT STATIONARY DATA"
        self.simulationDelay(1, state , s)

        print "\n\n\nINITIAL SQUAT"
        out = ik2DOF(.6, 0., self.l1, self.l2)
        upTheta1 = out.theta1
        upTheta2 = out.theta2
        upTheta3 = 0 - upTheta1 - upTheta2


        #
        # out = ik2DOF(.40, 0., self.l1, self.l2)
        # theta1 = out.theta1
        # theta2 = out.theta2
        # theta3 = 0 - theta1 - theta2
        #
        #
        # self._sendInitialSquatCommand( s, r, ref, state, theta1, theta2, theta3, increment)
        #
        #
        # print "\n\n\nFINISHED... STARTING TASKS:"
        # print "\n\n\nHIPS MOVING"
        #
        #
        # self._sendHipCommand(s, r, ref, state, .15, .15, -.15, -.15, 10, 'RIGHT')
        # self.simulationDelay(3,state,s)
        #
        #
        #
        #
        #
        # print "\n\n\nSQUATING"
        #
        # currentValues = self._returnDOFValues( state, s)
        #
        # out = ik2DOF(.45, 0., self.l1, self.l2)
        #
        # theta1 = out.theta1
        # theta2 = out.theta2
        # theta3 = 0 - theta1 - theta2
        #
        # self._sendSquatCommand( s, r, ref, state, currentValues, upTheta1, upTheta2, upTheta3, 20, "UP" , "RIGHT")
        # self.simulationDelay(1,state,s)
        #
        # self._sendSquatCommand(s, r, ref, state, currentValues, theta1, theta2, theta3, 20, "DOWN", "RIGHT")
        # self.simulationDelay(1, state, s)
        #
        # self._sendSquatCommand(s, r, ref, state, currentValues, upTheta1, upTheta2, upTheta3 , 20, "UP", "RIGHT")
        # self.simulationDelay(1, state, s)
        #
        # self._sendSquatCommand(s, r, ref, state, currentValues, theta1, theta2, theta3, 20, "DOWN", "RIGHT")
        # self.simulationDelay(1, state, s)
        #
        # self._sendSquatCommand(s, r, ref, state, currentValues, upTheta1, upTheta2, upTheta3 , 20, "UP", "RIGHT")
        # self.simulationDelay(1, state, s)
        #
        # print "\n\n\nMOVING TO CENTER"
        #
        # self._movingToCenter( currentStartingValues, ref, r, state, s, increment)
        # self.simulationDelay(3,state,s)

        print "\n\n\nHIPS MOVING"

        self._sendHipCommand(s, r, ref, state, -.15, -.15, .15, .15, 10, 'LEFT')
        self.simulationDelay(3,state,s)

        print "\n\n\nINITIAL SQUAT"

        out = ik2DOF(.58, 0., self.l1, self.l2)
        upTheta1 = out.theta1
        upTheta2 = out.theta2
        upTheta3 = 0 - upTheta1 - upTheta2



        out = ik2DOF(.48, 0., self.l1, self.l2)
        theta1 = out.theta1
        theta2 = out.theta2
        theta3 = 0 - theta1 - theta2


        self._sendInitialSquatCommand( s, r, ref, state, theta1, theta2, theta3, 20)


        print "\n\n\nGOING UP AND BENDING AT WAIST"



        currentValues = self._returnDOFValues( state, s)

        hipTheta = -.3




        out = ik2DOF(.5, 0., self.l1, self.l2)

        theta1 = out.theta1
        theta2 = out.theta2
        theta3 = 0 - theta1 - theta2

        self._sendSquatCommand( s, r, ref, state, currentValues, upTheta1, upTheta2, upTheta3, 20, "UP" , "LEFT")
        self.simulationDelay(1,state,s)

        print "\n\t\t\tBENDING {} RADIANS\n".format(hipTheta)
        self._sendBendCommand( s , r , ref , state , hipTheta , 'LEFT',30)
        self.simulationDelay(1, state, s)

        print "\n\n\nSQUATING"


        self._sendSquatCommand(s, r, ref, state, currentValues, theta1, theta2, hipTheta-theta1 - theta2, 20, "DOWN", "LEFT" , "YES")
        self.simulationDelay(1, state, s)

        self._sendSquatCommand(s, r, ref, state, currentValues, upTheta1, upTheta2, hipTheta -theta1 - theta2, 20, "UP", "LEFT" , "YES")
        self.simulationDelay(1, state, s)

        self._sendSquatCommand(s, r, ref, state, currentValues, theta1, theta2, hipTheta - theta1 - theta2, 20, "DOWN", "LEFT" , "YES")
        self.simulationDelay(1, state, s)

        self._sendSquatCommand(s, r, ref, state, currentValues, upTheta1, upTheta2, hipTheta -theta1 - theta2, 20, "UP", "LEFT" , "YES")
        self.simulationDelay(1, state, s)




        print "\n\n\nCENTERING"

        self._movingToCenter(currentStartingValues, ref, r, state, s, 20)

    def _sendBendCommand(self , s, r, ref, state, theta, leg , increment):
        currentValues = self._returnDOFValues(state , s)
        lHipInc = np.linspace(currentValues[15], theta, increment)
        rHipInc = np.linspace(0., theta, increment)
        moving=True
        inc = 0
        while moving:

            self._announceHipLocation(state, s, 'LEFT')


            ref.ref[ha.RHP] = rHipInc[inc]
            ref.ref[ha.LHP] = lHipInc[inc]


            r.put(ref)

            self.simulationDelay(.2,state,s)
            inc += 1
            if inc >= increment:
                moving = False

    def _sendInitialSquatCommand(self,s,r,ref,state , theta1 , theta2 , theta3 , increment):

        currentValues = self._returnDOFValues(state,s)
        lKneeInc = np.linspace(currentValues[22] , theta2 , increment)
        rKneeInc = np.linspace(currentValues[16] , theta2 , increment)
        lHipInc = np.linspace(currentValues[21], theta3, increment)
        rHipInc = np.linspace(currentValues[15],theta3,increment)
        lAnkleInc = np.linspace(currentValues[23] , theta1 , increment)
        rAnkleInc = np.linspace(currentValues[17],theta1,increment)


        moving=True
        inc = 0
        while moving:

            self._announceHipLocation(state, s, 'RIGHT')

            ref.ref[ha.RAP] = rAnkleInc[inc]
            ref.ref[ha.RKN] = rKneeInc[inc]
            ref.ref[ha.RHP] = rHipInc[inc]
            ref.ref[ha.LAP] = lAnkleInc[inc]
            ref.ref[ha.LKN] = lKneeInc[inc]
            ref.ref[ha.LHP] = lHipInc[inc]

            r.put(ref)

            self.simulationDelay(.2,state,s)
            inc += 1
            if inc >= increment:
                moving = False

    def _movingToCenter(self,values,ref,r,state,s,increment):

        current = self._returnDOFValues(state , s)

        RSP = np.linspace(current[0],values[0],increment)
        RSR = np.linspace(current[1],values[1],increment)
        RSY = np.linspace(current[2],values[2],increment)
        REB = np.linspace(current[3],values[3],increment)
        RWY = np.linspace(current[4],values[4],increment)
        RWP = np.linspace(current[5],values[5],increment)
        LSP = np.linspace(current[6],values[6],increment)
        LSR = np.linspace(current[7],values[7],increment)
        LSY = np.linspace(current[8],values[8],increment)
        LEB = np.linspace(current[9],values[9],increment)
        LWY = np.linspace(current[10],values[10],increment)
        LWP = np.linspace(current[11],values[11],increment)
        WST = np.linspace(current[12],values[12],increment)
        RHY = np.linspace(current[13],values[13],increment)
        RHR = np.linspace(current[14],values[14],increment)
        RHP = np.linspace(current[15],values[15],increment)
        RKN = np.linspace(current[16],values[16],increment)
        RAP = np.linspace(current[17],values[17],increment)
        RAR = np.linspace(current[18],values[18],increment)
        LHY = np.linspace(current[19],values[19],increment)
        LHR = np.linspace(current[20],values[20],increment)
        LHP = np.linspace(current[21],values[21],increment)
        LKN = np.linspace(current[22],values[22],increment)
        LAP = np.linspace(current[23],values[23],increment)
        LAR = np.linspace(current[24],values[24],increment)


        moving = True
        inc = 0
        while moving:
            ref.ref[ha.RSP] = RSP[inc]
            ref.ref[ha.RSR] = RSR[inc]
            ref.ref[ha.RSY] = RSY[inc]
            ref.ref[ha.REB] = REB[inc]
            ref.ref[ha.RWY] = RWY[inc]
            ref.ref[ha.RWP] = RWP[inc]
            ref.ref[ha.LSP] = LSP[inc]
            ref.ref[ha.LSR] = LSR[inc]
            ref.ref[ha.LSY] = LSY[inc]
            ref.ref[ha.LEB] = LEB[inc]
            ref.ref[ha.LWY] = LWY[inc]
            ref.ref[ha.LWP] = LWP[inc]
            ref.ref[ha.WST] = WST[inc]
            ref.ref[ha.RHY] = RHY[inc]
            ref.ref[ha.RHR] = RHR[inc]
            ref.ref[ha.RHP] = RHP[inc]
            ref.ref[ha.RKN] = RKN[inc]
            ref.ref[ha.RAP] = RAP[inc]
            ref.ref[ha.RAR] = RAR[inc]
            ref.ref[ha.LHY] = LHY[inc]
            ref.ref[ha.LHR] = LHR[inc]
            ref.ref[ha.LHP] = LHP[inc]
            ref.ref[ha.LKN] = LKN[inc]
            ref.ref[ha.LAP] = LAP[inc]
            ref.ref[ha.LAR] = LAR[inc]



            r.put(ref)

            self.simulationDelay(.2,state,s)
            inc += 1
            if inc >= increment:
                moving = False


    def _announceHipLocation(self,state,s,leg):

        currentValues = self._returnDOFValues(state , s)

        if leg=='RIGHT':
            height = self.l1*np.cos(currentValues[17]) + self.l2*np.cos(currentValues[17] + currentValues[16]) + self.l3*np.cos(currentValues[17] + currentValues[16] + currentValues[15])
        elif leg=='LEFT':
            height = self.l1 * np.cos(currentValues[23]) + self.l2 * np.cos(currentValues[22] + currentValues[23]) + self.l3 * np.cos(currentValues[21] + currentValues[23] + currentValues[22])

        print "\t\t\tCURRENT HIP HEIGHT IS {}".format(height)

    def _sendRaiseLegCommand(self,s,r,ref,state,theta1,theta2,theta3,increment,direction):
        currentValues = self._returnDOFValues( state, s)

        if direction == 'LEFT':


            lKneeInc = np.linspace(currentValues[22] , theta2 , increment)
            rKneeInc = np.squeeze(np.tile(currentValues[16],[1,increment]))
            lHipInc = np.linspace(currentValues[21], theta3, increment)
            rHipInc = np.squeeze(np.tile(currentValues[15],[1,increment]))
            # leg = 'RIGHT'


        elif direction == 'RIGHT':


            lKneeInc = np.squeeze(np.tile(currentValues[22] , [1 , increment]))
            rKneeInc = np.linspace(currentValues[16] , theta2 , increment)
            lHipInc = np.squeeze(np.tile(currentValues[21] , [1 , increment]))
            rHipInc = np.linspace(currentValues[15] , theta3 , increment)
            # leg = 'LEFT'

        print "\t\t{} LEG SELECTED".format(direction)

        moving = True
        inc = 0

        # print lKneeInc
        # print rKneeInc

        while moving:

            # self._announceHipLocation(state,s,leg)

            ref.ref[ha.RKN] = rKneeInc[inc]
            ref.ref[ha.RHP] = rHipInc[inc]
            ref.ref[ha.LKN] = lKneeInc[inc]
            ref.ref[ha.LHP] = lHipInc[inc]

            r.put(ref)

            self.simulationDelay(.2,state,s)
            inc += 1
            if inc >= increment:
                moving = False



    def _sendHipCommand(self, s , r , ref , state , RHR , LHR , RAR , LAR , increment , direction):

        currentValues = self._returnDOFValues( state, s)

        if direction == 'RIGHT':

            print "\t\t{} SIDE SELECTED".format(direction)

            hipRollRInc =np.linspace(currentValues[14] , RHR , increment)
            hipRollLInc = np.linspace(currentValues[20] , LHR , increment)
            ankleRollRInc = np.linspace(currentValues[18] , RAR , increment)
            ankleRollLInc = np.linspace(currentValues[24] , LAR , increment)
        if direction == 'LEFT':

            print "\t\t{} SIDE SELECTED".format(direction)

            hipRollRInc =np.linspace(currentValues[14] , RHR , increment)
            hipRollLInc = np.linspace(currentValues[20] , LHR , increment)
            ankleRollRInc = np.linspace(currentValues[18] , RAR , increment)
            ankleRollLInc = np.linspace(currentValues[24] , LAR , increment)
        moving = True
        inc = 0
        while moving:


            ref.ref[ha.RHR] = hipRollRInc[inc]
            ref.ref[ha.LHR] = hipRollLInc[inc]
            ref.ref[ha.RAR] = ankleRollRInc[inc]
            ref.ref[ha.LAR] = ankleRollLInc[inc]
            r.put(ref)

            self.simulationDelay(.2,state,s)
            inc += 1
            if inc >= increment:
                moving = False

    def _sendSquatCommand(self, s , r , ref , state , currentValues , theta1 , theta2 , theta3 , increment , direction , leg , bent="NO"):

        currentValues = self._returnDOFValues(state,s)

        if leg == 'RIGHT':

            print currentValues[15] , currentValues[16] , currentValues[17] , currentValues[21] , currentValues[22] , currentValues[23]

            rKneeInc = np.linspace(currentValues[16], theta2, increment)
            lKneeInc = np.squeeze(np.tile(currentValues[22], [1, increment]))
            rHipInc = np.linspace(currentValues[15], theta3, increment)
            lHipInc = np.squeeze(np.tile(currentValues[21], [1, increment]))
            rAnkleInc = np.linspace(currentValues[17], theta1, increment)
            lAnkleInc = np.squeeze(np.tile(currentValues[23], [1, increment]))


        elif leg == 'LEFT':

            if direction == 'DOWN' and bent=='NO':
                lKneeInc = np.linspace(currentValues[22], theta2, increment)
                rKneeInc = np.squeeze(np.tile(currentValues[16], [1, increment]))
                lHipInc = np.linspace(currentValues[21], theta3, increment)
                rHipInc = np.squeeze(np.tile(currentValues[15], [1, increment]))
                lAnkleInc = np.linspace(currentValues[23], theta1, increment)
                rAnkleInc = np.squeeze(np.tile(currentValues[17], [1, increment]))

            if direction == 'DOWN' and bent=='YES':
                lKneeInc = np.linspace(currentValues[22], theta2, increment)
                rKneeInc = np.squeeze(np.tile(currentValues[16], [1, increment]))
                lHipInc = np.linspace(currentValues[21], theta3, increment)
                rHipInc = np.squeeze(np.tile(0., [1, increment]))
                lAnkleInc = np.linspace(currentValues[23], theta1, increment)
                rAnkleInc = np.squeeze(np.tile(currentValues[17], [1, increment]))

            if direction == 'UP' and bent =='NO':
                lKneeInc = np.linspace(currentValues[22], theta2, increment)
                rKneeInc = np.squeeze(np.tile(currentValues[16], [1, increment]))
                lHipInc = np.linspace(currentValues[21], theta3, increment)
                rHipInc = np.squeeze(np.tile(currentValues[15], [1, increment]))
                lAnkleInc = np.linspace(currentValues[23], theta1, increment)
                rAnkleInc = np.squeeze(np.tile(currentValues[17], [1, increment]))

            if direction == 'UP' and bent == 'YES':
                lKneeInc = np.linspace(currentValues[22], theta2, increment)
                rKneeInc = np.squeeze(np.tile(currentValues[16], [1, increment]))
                lHipInc = np.linspace(currentValues[21], theta3, increment)
                rHipInc = np.squeeze(np.tile(0., [1, increment]))
                lAnkleInc = np.linspace(currentValues[23], theta1, increment)
                rAnkleInc = np.squeeze(np.tile(currentValues[17], [1, increment]))



        else:
            print "ERROR"


        print "\t\tMOVING {} ON THE {} LEG".format(direction,leg)



        # print "\t\t\t\tHIP HEIGHT IS {} METERS".format(hipHeight)

        moving = True
        inc = 0
        while moving:

            self._announceHipLocation(state,s,leg)

            ref.ref[ha.RAP] = rAnkleInc[inc]
            ref.ref[ha.RKN] = rKneeInc[inc]
            ref.ref[ha.RHP] = rHipInc[inc]
            ref.ref[ha.LAP] = lAnkleInc[inc]
            ref.ref[ha.LKN] = lKneeInc[inc]
            ref.ref[ha.LHP] = lHipInc[inc]
            # ref.ref[ha.LSP] = LAP[inc]

            r.put(ref)

            self.simulationDelay(.2,state,s)
            inc += 1
            if inc >= increment:
                moving = False

    def simulationDelay( self, waitTime , state , s ):
        [statuss, framesizes] = s.get(state, wait=False, last=False)
        current = state.time
        while (True):
            [statuss, framesizes] = s.get(state, wait=True, last=False)

            if ((state.time - current) > waitTime):
                break

    def _returnDOFValues(self , state , s):
        [statuss, framesizes] = s.get(state, wait=False, last=False)
        return np.array([
        [state.joint[ha.RSP].pos],
        [state.joint[ha.RSR].pos],
        [state.joint[ha.RSY].pos],
        [state.joint[ha.REB].pos],
        [state.joint[ha.RWY].pos],
        [state.joint[ha.RWP].pos],
        [state.joint[ha.LSP].pos],
        [state.joint[ha.LSR].pos],
        [state.joint[ha.LSY].pos],
        [state.joint[ha.LEB].pos],
        [state.joint[ha.LWY].pos],
        [state.joint[ha.LWP].pos],
        [state.joint[ha.WST].pos],
        [state.joint[ha.RHY].pos],
        [state.joint[ha.RHR].pos],
        [state.joint[ha.RHP].pos],
        [state.joint[ha.RKN].pos],
        [state.joint[ha.RAP].pos],
        [state.joint[ha.RAR].pos],
        [state.joint[ha.LHY].pos],
        [state.joint[ha.LHR].pos],
        [state.joint[ha.LHP].pos],
        [state.joint[ha.LKN].pos],
        [state.joint[ha.LAP].pos],
        [state.joint[ha.LAR].pos]
        ])



if __name__ == "__main__":
    makeHuboBend()
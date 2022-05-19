import serial
import time
from pynq import PL
from pynq import Overlay, MMIO, Clocks, allocate
import os, warnings
from IPython import get_ipython
import numpy as np
import matplotlib.pyplot as plt

# Initialize Q table

# Python :

    # request the status
    # send the command
    # receive the status

# Arduino :

    # Receive the serial
    # Decode
    # Execute

# Learning :

    # Count step
    # Check status = comm
    # Get state = comm
    # Get action
    # Start action = comm
    # Check status = comm

print("entry")

def main():

    Q_table = main_learning()
    ser = serial.Serial('/dev/ttyACM0', baudrate = 9600, timeout=1)
    # ser = serial.Serial('COM4', baudrate = 9600, timeout=1)
    time.sleep(3)
    step = 0

    rl_init_state = Data()
    rl_init_state.message = "0";
    status = setValues(ser,"state",rl_init_state)
    print(status.strip(),rl_init_state.message.strip())
    print("=======================")

    while(1):
        if(step>=25):
            break
        step += 1


        internal_state = Data()
        rl_state = Data()
        rl_action = Data()
        internal_state = Data()


        status = getValues(ser,"internal",internal_state)
        print(status.strip(),internal_state.message.strip())


        status = getValues(ser,"state",rl_state)
        print(status.strip(),rl_state.message.strip())

        s = int(rl_state.message)
        M = max(Q_table[s][0], Q_table[s][1], Q_table[s][2], Q_table[s][3])
        S = 0
        while Q_table[s][S] != M:
            S += 1
        rl_action.message = f"{S}"


        status = setValues(ser,"action",rl_action)
        print(status.strip(),rl_action.message.strip())


        status = getValues(ser,"internal",internal_state)
        print(status.strip(),internal_state.message.strip())

        print("=======================")



def setValues(ser, command, input):
    ser.write(b'set ')
    ser.write(command.encode(encoding='ascii'))
    ser.write(b' ')
    ser.write(input.message.encode(encoding='ascii'))
    ser.write(b'\0')
    arduinoData = ser.readline().decode('ascii')
    print("[success]",arduinoData.strip())

    ser.write(b'g_status\0')
    arduinoData = ser.readline().decode('ascii')
    return arduinoData



def getValues(ser, command, output):
    ser.write(b'get ')
    ser.write(command.encode(encoding='ascii'))
    ser.write(b'\0')
    arduinoData = ser.readline().decode('ascii')
    output.message = arduinoData

    ser.write(b'g_status\0')
    arduinoData = ser.readline().decode('ascii')
    return arduinoData



class Data:
    message = ""

################################################





def main_learning():

    print("One")
    ol = Overlay("/home/xilinx/jupyter_notebooks/RTL/design.bit")
    print("two")

    Action = 0x46000000
    Nextstate = 0x42000000
    Valid = 0x44000000
    Reward = 0x40000000
    Control = 0x43C00000

    ADDRESS_RANGE_8 = 0x2000
    ADDRESS_RANGE_64 = 0x10000



    act_bram = MMIO(Action, ADDRESS_RANGE_8)
    nxt_bram = MMIO(Nextstate, ADDRESS_RANGE_8)
    vld_bram = MMIO(Valid, ADDRESS_RANGE_8)
    rwd_bram = MMIO(Reward, ADDRESS_RANGE_8)
    ctrl_bram = MMIO(Control, ADDRESS_RANGE_64)



    #reset
    ctrl_bram.write(0, 1)
    ctrl_bram.write(0, 0)
    #start
    ctrl_bram.write(1*4, 0)

    #alpha
    ctrl_bram.write(5*4, 0x4000FFFF)
    #gamma
    ctrl_bram.write(6*4, 0x7000FFFF)

    #max eps
    ctrl_bram.write(7*4, 1000)
    #max step
    ctrl_bram.write(8*4, 15)
    #start state
    ctrl_bram.write(9*4, 0)
    #end state
    ctrl_bram.write(10*4, 24)
    #invalid penalty
    ctrl_bram.write(11*4, 50 * 2**16)
    #number of state
    ctrl_bram.write(25*4, 25)

    #read seed
    ctrl_bram.write(2*4, 0)
    #epsilon
    ctrl_bram.write(4*4, 8)
    #rng seed
    ctrl_bram.write(3*4, 675)
    ctrl_bram.write(2*4, 1)
    ctrl_bram.write(2*4, 0)
    ctrl_bram.write(3*4, 0)

    #reward
    #next statet
    #valid action
    #initialization



    for i in range(0,25*4,4):
        rwd_bram.write(i, 0)

    rwd_bram.write(4*4, -100 * 2**16)
    rwd_bram.write(6*4, -100 * 2**16)
    rwd_bram.write(7*4, -100 * 2**16)
    rwd_bram.write(13*4, -100 * 2**16)
    rwd_bram.write(16*4, -100 * 2**16)
    rwd_bram.write(18*4, -100 * 2**16)
    rwd_bram.write(19*4, -100 * 2**16)
    rwd_bram.write(21*4, -100 * 2**16)

    rwd_bram.write(24*4, 100 * 2**16)



    for i in range(25):
        for j in range(4):
            address = i*16+j*4
            if (j == 0):
                nextstate = i - 5;
            elif (j == 1):
                nextstate = i + 1;
            elif (j == 2):
                nextstate = i + 5;
            else:
                nextstate = i - 1;

            if (nextstate < 0):
                isValid = 0;
            elif (nextstate > 24):
                isValid = 0;
            elif ((i % 5 == 4) and (j == 1)):
                isValid = 0;
            elif ((i % 5 == 0) and (j == 3)):
                isValid = 0;
            else:
                isValid = 1;

            if (isValid):
                nxt_bram.write(address, nextstate)
                vld_bram.write(address, 1)
            else:
                nxt_bram.write(address, i)
                vld_bram.write(address, 0)


    for i in range(0,25*4*4,4):
        act_bram.write(i, 0)

    start = time.time()
    ctrl_bram.write(1*4, 1)
    while(not ctrl_bram.read(12*4)):
        pass
    end = time.time()

    time1 = (end - start) * 1000
    print("Time : %.2f ms" % (time1))


    Q_Table = list();
    for i in range(0,25*4,4):
        print(act_bram.read(i*4), act_bram.read(i*4+4), act_bram.read(i*4+8), act_bram.read(i*4+12))
        Q_Table.append([act_bram.read(i*4), act_bram.read(i*4+4), act_bram.read(i*4+8), act_bram.read(i*4+12)])


    print("[")
    for i,j in enumerate(Q_Table):
        for k,l in enumerate(j):
            if(Q_Table[i][k] & 0x80000000):
                Q_Table[i][k] = -0x100000000 + Q_Table[i][k]
            Q_Table[i][k] = (Q_Table[i][k] / 2**16)
        print(Q_Table[i])
    print("]")


    print(Q_Table)
    return Q_Table

##############################################


main()


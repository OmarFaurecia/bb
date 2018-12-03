
import RPi.GPIO as GPIO
import picamera
import datetime as dt
import time
import contextlib
import os
import sys
from xmlrpc.server import SimpleXMLRPCServer
import _thread
from pathlib import Path
import can
import socket
import time
import serial
import os



import bluepy
from bluepy.btle import Peripheral, ADDR_TYPE_RANDOM, ADDR_TYPE_PUBLIC,AssignedNumbers

#set up pin 17 input

BtnPin = 22
RecordOn = 0
ConvertOn = 0
Blebpm = [0,0]

Entetes=["Time",
         "Sequence",
         "Year",
         "Month",
         "Day",
         "Hour",
         "Minutes",
         "Seconds",
         "Heart_Rate",
         "Respiration_Rate",
         "Engine_Load_[%]",
         "Engine_Speed_[RPM]",
         "Vehicule_Speed_[km/h]",
         "Humidity1",
         "Temperature1",
         "Humidity2",
         "Temperature2",
         "Humidity3",
         "Temperature3",
         "fHR",
         "fRR",
         "fHRV",
         "Sensor_1",
         "Sensor_2",
         "Sensor_3",
         "Sensor_4",
         "Sensor_5",
         "Sensor_6",
         "Sensor_7",
         "Sensor_8",]


colors = [0xFF0000, 0x00FF00, 0x0000FF, 0x7F7F00, 0xFF00FF, 0x00FFFF, 0x007F00]

R = 17
G = 18
B = 27


AmbienteAirTemperaure = 0
EngineRPM = 0
Speed = 0
ActualEngineTorque = 0



def GetData(pid):
    global bus
    global AmbienteAirTemperaure
    global EngineRPM
    global Speed
    global ActualEngineTorque


    msg = can.Message(arbitration_id=0x7df,data=[2,1,pid,0,0,0,0,0],extended_id=False)
    bus.send(msg)
    i = 0
    while (i < 10):
        message = bus.recv(1.0)
        if message is None :
                print("timeout occur, no message")
        else:
                #print(message)
                if (message.arbitration_id == 2024):
                        #print("ID : ", message.arbitration_id, "dlc:",message.dlc,"data:",message.data[2])
                        if (message.data[2] == 70):
                                AmbienteAirTemperature = message.data[3] -40
                                #print("Ambiente Air Temperaure:",AmbienteAirTemperature)
                        if (message.data[2] == 12):
                                EngineRPM = int((message.data[3] * 256 + message.data[4]) / 4)
                                #print("EngineRPM:",EngineRPM)
                        if (message.data[2] == 13):
                                Speed = message.data[3]
                                #print("Speed:",Speed)
                        if (message.data[2] == 17):
                                ActualEngineTorque = message.data[3]
                                #print("ActualEngineTorque:",ActualEngineTorque)
        i = i + 1

def ThreadCAN():

    global bus
    global AmbienteAirTemperaure
    global EngineRPM
    global Speed
    global ActualEngineTorque

    os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
    pid = [12,13,70,17]
    pidmax = 4
    bus = can.interface.Bus(channel='can0',bustype='socketcan_native')
    i = 0
    while (True):
        GetData(pid[i])
        i = i + 1
        if (i==pidmax):
            i=0
        #    print("Speed:",Speed," RPM:",EngineRPM," Torque:",ActualEngineTorque)



class HRM(Peripheral):
    def __init__(self, addr):
        Peripheral.__init__(self, addr)#, addrType=ADDR_TYPE_RANDOM)
        #Peripheral.__init__(self, addrType=ADDR_TYPE_RANDOM)


def detect(chn):
        #Led(GPIO.input(BtnPin))
        Print(GPIO.input(BtnPin))

def SetCamera(mode):
        global RecordOn
        global ConvertOn

        RecordOn = mode
        if (mode == 1):
                print('     *   Start Camera    *')
                #setColor(colors[2])
        elif (mode == 0):
                print ('    *   Stop  Camera!   *')
                #if ConvertOn:
                #        setColor(colors[3])
                #else:
                #        setColor(colors[1])
        else:
                print (' error call ')
        return 0


def Print(x):
        global RecordOn
        global ConvertOn
        if x == 0:
                RecordOn = not RecordOn
                if RecordOn :
                        print ('    *   Launch Camera!   *')
                        #setColor(colors[2])
                else :
                        print ('    *   Stop  Camera!   *')
                        #if ConvertOn:
                        #        setColor(colors[3])
                        #else:
                        #        setColor(colors[1])
                print ('    ***********************')


def map(x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def off():
        for i in pins:
                GPIO.output(pins[i], GPIO.HIGH)    # Turn off all leds

def setColor(col):   # For example : col = 0x112233
        R_val = (col & 0xff0000) >> 16
        G_val = (col & 0x00ff00) >> 8
        B_val = (col & 0x0000ff) >> 0

        R_val = map(R_val, 0, 255, 0, 100)
        G_val = map(G_val, 0, 255, 0, 100)
        B_val = map(B_val, 0, 255, 0, 100)


def detect(chn):
        # Led(GPIO.input(BtnPin))
        Print(GPIO.input(BtnPin))


#def RechercheUSB( nom ) :
#  with contextlib.closing(open('/etc/mtab')) as fp:
 #  for m in fp:
 #   fs_spec, fs_file, fs_vfstype, fs_mntops, fs_freq, fs_passno = m.split()
 #   if fs_spec.startswith('/'):
  #    r = os.statvfs(fs_file)
  #    if "CAM_VOITURE" in fs_file:
   #     return fs_file
  #return 0

#0 == Cardio
#1 == AW
def LaunchBLE(AddressBLE,Id,Type):
    global Blebpm

    while True:
        cccid = AssignedNumbers.client_characteristic_configuration
        hrmid = AssignedNumbers.heart_rate
        hrmmid = AssignedNumbers.heart_rate_measurement

        hrm = None
        try:
            #00:0B:57:34:49:11   Aw1
            #00:0B:57:34:49:0B   aw6
            #1C:BA:8C:1E:00:03  Geonote
            #hrm = HRM("00:0B:57:34:49:11")

            print("Waiting connection with ",AddressBLE)
            hrm = HRM(None)
            hrm.connect(AddressBLE,Type)
            #print(hrm)
            service, = [s for s in hrm.getServices() if s.uuid==hrmid]
            ccc, = service.getCharacteristics(forUUID=str(hrmmid))

            if 0: # This doesn't work
                ccc.write(bytes('\1\0','UTF-8'))

            else:
                desc = hrm.getDescriptors(service.hndStart,
                                          service.hndEnd)
                d, = [d for d in desc if d.uuid==cccid]

                hrm.writeCharacteristic(d.handle, bytes('\1\0','UTF-8'))

            t0=time.time()
            print("Device connected",AddressBLE)
            def print_hr(cHandle, data):
                global Blebpm

                Blebpm[Id] = data[1]
                #print (AddressBLE,":",Blebpm[Id],"%.2f"%(time.time()-t0))

            hrm.delegate.handleNotification = print_hr

            #for x in range(100):
            while True:
                hrm.waitForNotifications(3.)
        except bluepy.btle.BTLEException :
            print("Device disconnected",AddressBLE)


        finally:
            if hrm:
                hrm.disconnect()


        Blebpm[Id] = -1

    return 0







def InitialiseSerial():
    SerialInitialised = False
    while (not SerialInitialised):
        try:
            ser = serial.Serial(
             port='/dev/ttyUSB0',
             baudrate = 115200,
             parity = serial.PARITY_NONE,
             stopbits=serial.STOPBITS_ONE,
             bytesize=serial.EIGHTBITS,
             timeout =1
            )
            SerialInitialised = True;
            print("UART Initialized")
        except:
            print("error waiting UART Initialization")
            pass
    return ser

def WaitingSynchro(ser):
    Synchro= False
    while ( Synchro != True):
        try:
            if(ord(ser.read(1)))== 0x55:
                if(ord(ser.read(1)))== 0xaa:
                    if(ord(ser.read(1)))== 0x00:
                        Synchro = True
                        print("Synchro Found")
            else:
                Synchro = False
        except:
            print("Erreur UART")
            Synchro = False



def ManageSerial():
        global RecordOn
        global ConvertOn
        global FolderVideo
        global CurrentVideoName
        global Blebpm
        global AmbienteAirTemperaure
        global EngineRPM
        global Speed
        global ConvertOn

        blink=0

        ser = InitialiseSerial()

        counter = 0
        counter_frame =0
        index = 0
        Cor5 = 0
        Cor10 = 0
        Sequence = 0

        Time = ""
        Year = 0
        Month = 0
        Day = 0
        Hour = 0
        Minute = 0
        Second = 0
        Heart_Rate = 0
        EngineLoad = 0
        Engine_RPM = 0
        Speed = 0
        Humidity1 = 0
        Temperature1 = 0
        Humidity2 = 0
        Temperature2 = 0
        Humidity3 = 0
        Temperature3 = 0
        fHR = 0
        fRR = 0
        fHRV = 0
        erreur = False


      #  NameVideo = FolderVideo + dt.datetime.now().strftime('/%Y-%m-%d_%H-%M-%S')
       # NameVideotxt = NameVideo + '-DCS360.csv'
       # hrm_file = open(NameVideotxt,'wt')

     #   i = 0
     #   while i<30 :
     ##       hrm_file.write(Entetes[i])
      #      hrm_file.write(";")
      #      i = i + 1

        #hrm_file.write("\n\r")
   #     position = hrm_file.tell()
        startSpeed0 = dt.datetime.now()
        start = dt.datetime.now()
        sample = 1
        Correlation5bpm = 1
        Correlation10bpm = 1
        WaitingSynchro(ser)
        buffer = ser.read(48)
        #Lost the first Frame
        i = 0
        j = 0
        counter_frame = counter_frame + 1
        Time = str(int(time.clock()*1000000))
        Year = dt.datetime.now().strftime('%Y')
        Month = dt.datetime.now().strftime('%m')
        Day = dt.datetime.now().strftime('%d')
        Hour = dt.datetime.now().strftime('%H')
        Minute = dt.datetime.now().strftime('%M')
        Second = dt.datetime.now().strftime('%S')
        print ("Start Measuring")

        while i < 48 :
            cbyte1 = buffer[i]
            cbyte2 = buffer[i+1]
            word = cbyte1 << 8 | cbyte2
            i = i + 2
            j = j + 1
            if j == 8 :
                j =0

        bAff = False

        while ((dt.datetime.now() - start).seconds < 2700 ):

            if(counter_frame == 120):
                buffer = ser.read(39)
                #print("Last Frame")
                i=3
                counter_frame = 0
                while i < 39 :
                    byte1 = buffer[i]
                    byte2 = buffer[i+1]
                    word = byte1 << 8 | byte2
                    i = i + 2
                    if ( i == 7 ) :
                        Blebpm[1] = word
                        fHR = Blebpm[1]
                        Heart_Rate = Blebpm[0]
                    if ( i == 11 ) :
                        #print("RR:",str(word))
                        fRR = word
                    if ( i == 15 ) :
                        fHRV = word
                    if ( i == 19 ) :
                        #print("Speed:",str(word))
                        Speed = word
                    if ( i == 23 ) :
                        #print("RPM:",str(word))
                        Engine_RPM =  word
                    if ( i == 27 ) :
                        EngineLoad = word
                    if ( i == 29 ) :
                        Humidity1 = word
                    if ( i == 31 ) :
                        Temperature1 = word
                    if ( i == 33 ) :
                        Humidity2 = word
                    if ( i == 35 ) :
                        Temperature2 = word
                    if ( i == 37 ) :
                        Humidity3 = word
                    if ( i == 39 ) :
                        Temperature3 = word
                        Sequence = Sequence + 1
                        print("HR:",fHR," RR:",fRR)
                        soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        try:
                            soc.connect(("192.168.7.7",9000))
                            clients_input ='{\"event-name\": \"active-wellness\", \"heart-rate\" :' + str(fHR) + ',\"breath-rate\" : ' + str(fRR) + ', \"heart-rate-variability\" : 0} \n'


                            if not clients_input :
                               print("not a valid input")
                               break
                            print("sending")
                            soc.send(clients_input.encode('utf-8'))
                            print("sent & closing")
                            soc.close()
                            print("closed")
                        #Last Frame
                        except OSError as e :
                            print(e)


            Time = str(int(time.clock()*1000000))
            Year = dt.datetime.now().strftime('%Y')
            Month = dt.datetime.now().strftime('%m')
            Day = dt.datetime.now().strftime('%d')
            Hour = dt.datetime.now().strftime('%H')
            Minute = dt.datetime.now().strftime('%M')
            Second = dt.datetime.now().strftime('%S')

            buffer = ser.read(51)
            counter_frame = counter_frame + 1
            i = 3
            j = 0

            while (i < 51) : #and (Heart_Rate > 0) and (fHR > 0) :
                byte1 = buffer[i]
                byte2 = buffer[i+1]
                word = byte1 << 8 | byte2
                if (word > 5000):
                    erreur = True
                i = i + 2
                j = j + 1
              #  if (bAff):
              #     hrm_file.write(str(word))
              #     hrm_file.write(';')
                if j == 8 :
                    bAff = True
                  #  if (counter_frame == 1):
                    #     position = hrm_file.tell()
               #     hrm_file.write("\n")
                    j = 0
                #    hrm_file.write(Time)
                 #   hrm_file.write(";")
                 #   hrm_file.write(str(Sequence))
                 #   hrm_file.write(";")
                 #   hrm_file.write(Year)
                  #  hrm_file.write(";")
                  #  hrm_file.write(Month)
                  #  hrm_file.write(";")
                  #  hrm_file.write(Day)
                  #  hrm_file.write(";")
                   # hrm_file.write(Hour)
                 #   hrm_file.write(";")
                #    hrm_file.write(Minute)
                 #   hrm_file.write(";")
                 #   hrm_file.write(Second)
               #     hrm_file.write(";")
                #    hrm_file.write(str(Heart_Rate))
                ##    hrm_file.write(";0;") # Ref Respiration
                 #   hrm_file.write(str(EngineLoad))
                 #   hrm_file.write(";")
                 #   hrm_file.write(str(Engine_RPM))
                 #   hrm_file.write(";")
                 #   hrm_file.write(str(Speed))
                #    hrm_file.write(";")
                 #   hrm_file.write(str(Humidity1))
                 #   hrm_file.write(";")
                 #   hrm_file.write(str(Temperature1))
                 #   hrm_file.write(";")
                 #   hrm_file.write(str(Humidity2))
                  #  hrm_file.write(str(Temperature2))
                #    hrm_file.write(";")
                 #   hrm_file.write(str(Humidity3))
                 #   hrm_file.write(";")
                 #   hrm_file.write(str(Temperature3))
                 #   hrm_file.write(";")
                 #   hrm_file.write(str(fHR))
                 #   hrm_file.write(";")
                 #   hrm_file.write(str(fRR))
                 #   hrm_file.write(";")
                 #   hrm_file.write(str(fHRV))
                 #   hrm_file.write(";")


            if (Blebpm[0] > 0) and (Blebpm[1]> 1):
                if (abs(Blebpm[0] - Blebpm[1]) < 6) :
                    Correlation5bpm = Correlation5bpm + 1
                if abs(Blebpm[0] - Blebpm[1]) < 11:
                    Correlation10bpm = Correlation10bpm + 1
                sample = sample + 1
                Cor5 = int(Correlation5bpm/sample*100)
                Cor10 = int(Correlation10bpm/sample*100)

            if ((Blebpm[1]>250) or (Speed>250) or (fHR>300) or (EngineLoad>200) or (erreur==True)):
                WaitingSynchro(ser)
                buffer = ser.read(48)
                i = 0
                j = 0
                counter_frame = 1
                print ("Re-Start Measuring")
                hrm_file.write("\n")
                while i < 48 :
                    cbyte1 = buffer[i]
                    cbyte2 = buffer[i+1]
                    word = cbyte1 << 8 | cbyte2
                    i = i + 2
                    j = j + 1
                    #hrm_file.write(str(word))
                    #hrm_file.write(',')
                    if j == 8 :
                        #hrm_file.write("\n\r")
                        j =0
                Blebpm[1] = 0
                Speed = 0
                fHR = 0
                EngineLoad = 0
                Engine_RPM = 0
                position = hrm_file.seek(position)
                bAff = False
                erreur = False


            recordtxt = dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S') + ".\n\r"
            recordtxt = recordtxt + "Ref:" + str(Blebpm[0]) + ".\n\r"
            recordtxt = recordtxt + "AW:" + str(Blebpm[1])  + ".\n\r"
            recordtxt = recordtxt + "5bpm:" + str(Cor5) + ".\n\r"
            recordtxt = recordtxt + "10bpm:" + str(Cor10) + ".\n\r"
            recordtxt = recordtxt + "Speed:" + str(Speed) + ".\n\r"
            php_file = open("/var/www/html/aw.txt",'wt')
            php_file.write(recordtxt)
            php_file.close()

        recordtxt = "\t5bpm:\t" + str(Cor5) + "%\t10bpm:\t" + str(Cor10) + "%\tItem:\t" + str(sample)
        hrm_file.write(recordtxt)
        hrm_file.close()
        CurrentVideoName = ""
        ser.close()

def loop():
        global RecordOn
        global ConvertOn
        global FolderVideo
        global CurrentVideoName
        global Blebpm
        global Speed

        Blebpm[1] = 0
        Blebpm[0] = 0
        Cor5 = 50
        Cor10 = 89
        Speed = 0


        RecordOn = 0
        bContinue = True
        setColor(colors[0])

#        while bContinue:
 #             FolderVideo=RechercheUSB("CAM_VOITURE")
 #             if FolderVideo == 0 :
  #               bContinue = True
  #            else:
  #               bContinue = False
  #            pass
  #      CurrentVideoName = FolderVideo
        #os.system("sudo service motioneye stop")
        #better tu crontab and logrotate but no time on this raspberry
        os.system("sudo rm -f /var/log/kern.log")
        os.system("sudo rm -f /var/log/syslog")
        #setColor(colors[1])
        #try:
                #_thread.start_new_thread( ThreadLedManagment,() )
                # no led
                #_thread.start_new_thread( ThreadCAN,() )
        #except:
        #        print ("Error: unable to start ThreadCAN")
        # Launch batch conversion

   #     try:
                #_thread.start_new_thread( LaunchBLE,("1C:BA:8C:1E:00:03",0,ADDR_TYPE_PUBLIC) )
                # Old Geonote to use with ADDR_TYPE_PUBLIC

                #_thread.start_new_thread( LaunchBLE,("EF:DC:02:5C:CB:4C",0,ADDR_TYPE_RANDOM) )  #Michel
                #_thread.start_new_thread( LaunchBLE,("F1:B8:5E:F3:16:9B",0,ADDR_TYPE_RANDOM) )  #polar
   #             _thread.start_new_thread( LaunchBLE,("A0:E6:F8:4A:A6:D5",0,ADDR_TYPE_PUBLIC) )  #Zephyr
                #_thread.start_new_thread( LaunchBLE,("EF:F7:AE:33:3C:0C",0,ADDR_TYPE_RANDOM) )  #ionic
                #_thread.start_new_thread( LaunchBLE,("DC:E0:24:77:9A:37",0,ADDR_TYPE_RANDOM))
  #      except:
   #             print ("Error: unable to start LaunchBLE 1")
        #try:
        #       _thread.start_new_thread( LaunchBLE,("1C:BA:8C:1E:00:03",1,ADDR_TYPE_PUBLIC) )
        #except:
        #        print ("Error: unable to start thread LaunchBLE 2")


        php_file = open("/var/www/html/aw.txt",'wt')
        recordtxt = dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S') + ".\n\r"
        recordtxt = recordtxt + "Ref:" + str(Blebpm[0]) + ".\n\r"
        recordtxt = recordtxt + "AW:" + str(Blebpm[1])  + ".\n\r"
        recordtxt = recordtxt + "5bpm:" + str(Cor5) + ".\n\r"
        recordtxt = recordtxt + "10bpm:" + str(Cor10) + ".\n\r"
        recordtxt = recordtxt + "Speed:" + str(Speed) + ".\n\r"
        php_file.write(recordtxt)
        php_file.close()

        RecordOn = True
        #RecordOn = False
        while True:
                ManageSerial()
                time.sleep(1)
                print(Blebpm[0])
                pass

        #no end

def destroy():
        #GPIO.output(Gpin, GPIO.HIGH)       # Green led off
        #GPIO.output(Rpin, GPIO.HIGH)       # Red led off
        #p_R.stop()
        #p_G.stop()
        #p_B.stop()
        #off()
        #GPIO.cleanup()
        print("End Program")

if __name__ == '__main__':     # Program start from here
        # not use with CAN
        #setup(R,G,B)
        try:
                loop()
        except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
                destroy()

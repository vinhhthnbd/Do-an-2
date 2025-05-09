from tkinter import *
import serial.tools.list_ports
from tkinter import messagebox
from tkinter import PhotoImage
import csv
import threading 
import time


class RootGui:
    def __init__(self):
        self.root=Tk()
        self.root.title('Do an 2 - Dieu khien bluetooth ')
        self.root.geometry("800x800")
      



#Create frames
class CommunicationGui:
    def __init__(self,root,serial,control,measurement):
        #Frame
        #bring main root
        self.root=root
        #create frame
        self.frame=LabelFrame(root,text="Comport management",padx=8,pady=8,bg="white")

        self.label_com=Label(self.frame,text="Available ports:",bg="white",width=15,anchor="w")

        self.label_baudrate=Label(self.frame,text="Baudrate",bg="white",width=15,anchor="w")
        self.control=control
        self.serial=serial
        self.measurement=measurement


        #create object
        self.ComportOption()
        self.BaudrateOption()

        #define the button with the button callback funtion in command=
        self.buttonRefresh=Button(self.frame,text="Refesh",width=10,command=self.refreshCommand)#define the button with the button callback funtion in command=
        self.buttonConnect=Button(self.frame,text="Connect",width=10,state="disabled",command=self.connectCommand)

        #For publishing
        self.padx=20
        self.pady=5
        #show components on root
        self.publish()

    def ComportOption(self):
        self.serial.getComList()
        self.clicked_com=StringVar()
        self.clicked_com.set(self.serial.com_list[0])#this is the variable that store value we pick from Option menu
        self.drop_com=OptionMenu(self.frame,self.clicked_com,*self.serial.com_list,command=self.connectControl)
        self.drop_com.config(width=10)

    def BaudrateOption(self):
        bauds=["-","2400","4800","9600","14400","19200","28800","38400","56000","57600","115200","128000","256000"]
        self.clicked_baud=StringVar()
        self.clicked_baud.set(bauds[0])#this is the variable that store value we pick from Option menu
        self.drop_baud=OptionMenu(self.frame,self.clicked_baud,*bauds,command=self.connectControl)
        self.drop_baud.config(width=10)

    def connectControl(self,other):
        if "-" in self.clicked_com.get() or "-" in self.clicked_baud.get():
            self.buttonConnect["state"]="disable"
        else:
            self.buttonConnect["state"]="active"

    def refreshCommand(self):
        self.drop_com.destroy()
        self.ComportOption()
        self.drop_com.grid(column=2,row=2,padx=self.padx,pady=self.pady)
        self.drop_baud.destroy()
        self.BaudrateOption()
        self.drop_baud.grid(column=2,row=3,padx=self.padx,pady=self.pady)
        self.buttonConnect["state"]="disable"
  
       

    def connectCommand(self):
        '''
        Method that Updates the GUI during connect / disconnect status
        Manage serials and data flows during connect / disconnect status
        '''
        if self.buttonConnect["text"] in "Connect":
            # Start the serial communication
            self.serial.SerialOpen(self)

            # If connection established move on
            if self.serial.status==1:
                # Update the COM manager
                self.buttonConnect["text"] = "Disconnect"
                self.buttonRefresh["state"] = "disable"
                self.drop_baud["state"] = "disable"
                self.drop_com["state"] = "disable"
                InfoMsg = f"Successful UART connection using {self.clicked_com.get()}"
                messagebox.showinfo("showinfo", InfoMsg)
                self.control.AllowControl()
                self.measurement.AllowGetMeasurement()

            else:
                ErrorMsg = f"Failure to estabish UART connection using {self.clicked_com.get()} "
                messagebox.showerror("showerror", ErrorMsg)
                self.control.BlockControl()
                self.measurement.BlockMeasurement()
        else:

            # Closing the Serial COM
            # Close the serial communication
            self.serial.SerialClose(self)
            self.control.BlockControl()
            self.measurement.BlockMeasurement()
            self.buttonConnect["text"] = "Connect"
            self.buttonRefresh["state"] = "active"
            self.drop_baud["state"] = "active"
            self.drop_com["state"] = "active"
         

    

    def publish(self):
        #span 3 column, spend 3 row
        self.frame.grid(row=0,column=0,rowspan=3,columnspan=3,padx=8,pady=8)
        self.label_com.grid(column=1,row=2)
        self.drop_com.grid(column=2,row=2,padx=self.padx,pady=self.pady)
        self.label_baudrate.grid(column=1,row=3) ##create space 
        self.drop_baud.grid(column=2,row=3,padx=self.padx,pady=self.pady)
        self.buttonRefresh.grid(column=3,row=2)
        self.buttonConnect.grid(column=3,row=3)


#for serial port
class SerialCtrl:
    def __init__(self):
        self.com_list=[]
        self.status=False

    def getComList(self):
        ports=serial.tools.list_ports.comports()
        self.com_list=[com[0] for com in ports]
        self.com_list.insert(0,"-")

    def SerialOpen(self,gui):
        try:
            self.ser.is_open
        except:
            PORT=gui.clicked_com.get()
            BAUD=gui.clicked_baud.get()
            self.ser=serial.Serial()
            self.ser.baudrate=BAUD
            self.ser.port=PORT
            self.ser.timeout=0.1
            self.ser.open()


        try :
            if self.ser.is_open:
                self.status=True
            else:
                PORT=gui.clicked_com.get()
                BAUD=gui.clicked_baud.get()
                self.ser=serial.Serial()
                self.ser.baudrate=BAUD
                self.ser.port=PORT
                self.ser.timeout=0.1
                self.ser.open()
                self.staus=True
        except:
            self.status=False

    def SerialClose(self, ComGUI):
        '''
        Method used to close the UART communication
        '''
        try:
            self.ser.is_open
            self.ser.close()
            self.status = False
        except:
            self.status = True
    def SerialWrite(self,string):
        try:
           self.ser.is_open
           self.ser.write(string.encode('utf-8'))
        except:
            pass
            ErrorMsg = f"Failure to transmit data "
            messagebox.showerror("showerror", ErrorMsg)
    
    def SerialReadline(self):
        try:
            reading=self.ser.readline().decode('utf-8')
        except:
            ErrorMsg = f"Failure to receive data using {self.clicked_com.get()} "
            messagebox.showerror("showerror", ErrorMsg)
            reading="NULL/"
        return reading


#for control part
class Control:
    def __init__(self,root,serial):
        #Frame
        #bring main root
        self.root=root
        #create frame
        self.frame=LabelFrame(root,text="Controlling part",padx=5,pady=5,bg="white")
        self.serial=serial

        self.buttonForward=Button(self.frame,text="Forward",width=10,state="disabled")#define the button with the button callback funtion in command=
        self.buttonBackward=Button(self.frame,text="Backward",width=10,state="disabled")
        self.buttonLeft=Button(self.frame,text="Left",width=10,state="disabled")
        self.buttonRight=Button(self.frame,text="Right",width=10,state="disabled")

        self.buttonForward.bind(sequence="<ButtonPress>",func=self.Forward)
        self.buttonForward.bind(sequence="<ButtonRelease>",func=self.Stop)

        self.buttonBackward.bind(sequence="<ButtonPress>",func=self.Backward)
        self.buttonBackward.bind(sequence="<ButtonRelease>",func=self.Stop)

        self.buttonLeft.bind(sequence="<ButtonPress>",func=self.Left)
        self.buttonLeft.bind(sequence="<ButtonRelease>",func=self.Stop)

        self.buttonRight.bind(sequence="<ButtonPress>",func=self.Right)
        self.buttonRight.bind(sequence="<ButtonRelease>",func=self.Stop)

        self.padx=20
        self.pady=20
        self.publish()
  
    def Forward(self,event):
        self.serial.SerialWrite('f')

    def Backward(self,event):
        self.serial.SerialWrite('b')

    def Left(self,event):
        self.serial.SerialWrite('l')

    def Right(self,event):
        self.serial.SerialWrite('r')
        
    def Stop(self,event):
        self.serial.SerialWrite('s')
    
    
    def AllowControl(self):
        self.buttonForward["state"]="active"
        self.buttonBackward["state"]="active"
        self.buttonLeft["state"]="active"
        self.buttonRight["state"]="active"

    def BlockControl(self):
        self.buttonForward["state"]="disabled"
        self.buttonBackward["state"]="disabled"
        self.buttonLeft["state"]="disabled"
        self.buttonRight["state"]="disabled"

    def publish(self):
        self.frame.place(x=0,y=130)
        self.buttonForward.grid(column=1,row=0,padx=20,pady=20)
        self.buttonBackward.grid(column=1,row=2,padx=20,pady=20)
        self.buttonLeft.grid(column=0,row=1,padx=20,pady=20)
        self.buttonRight.grid(column=2,row=1,padx=20,pady=0)

class Measurement:
    def __init__(self,root,serial):
        self.root=root
        self.serial=serial
        self.frame=LabelFrame(root,text="Measurement part",padx=5,pady=5,bg="white")

        self.buttonMeasurement=Button(self.frame,text="Start getting correction data",width=25,height=1,state="disabled")
        self.buttonMeasurement.bind(sequence="<ButtonPress>",func=self.StartGetMearsurement)

        self.x_correction=Label(self.frame,text="X:",padx=20,pady=5,bg="white",anchor="w")
        self.y_correction=Label(self.frame,text="Y:",padx=20,pady=5,bg="white",anchor="w")
        self.theta_correction=Label(self.frame,text="Theta:",padx=20,pady=5,bg="white",anchor="w")

        self.x_txt=Text(self.frame,width=15,height=1,fg="black")
        self.y_txt=Text(self.frame,width=15,height=1,fg="black")
        self.theta_txt=Text(self.frame,width=15,height=1,fg="black")

        self.x_txt.insert(END,'NULL')
        self.y_txt.insert(END,'NULL')
        self.theta_txt.insert(END,'NULL')

        self.x_str=[]
        self.y_str=[]
        self.theta_str=[]
        
        self.filepath='correction.csv'

        with open(self.filepath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            data=[['x','y','theta']]
            writer.writerows(data)

        self.flag=0
        self.thread=threading.Thread(target=self.loop)
        self.thread.daemon=True

        self.thread.start()

        self.publish()

    def AllowGetMeasurement(self):
        self.buttonMeasurement["state"]="active"

    def BlockMeasurement(self):
        self.flag=0
        self.buttonMeasurement["text"]="Start getting correction data"
        self.buttonMeasurement["state"]="disabled"
        with open(self.filepath, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            data=[['x','y','theta']]
            writer.writerows(data)


        

    def StartGetMearsurement(self,event):
        self.flag^=1
        self.Measurement_action()
        # if self.flag==1:
        #     self.buttonMeasurement["text"]="Stop getting correction data"
        # else:
        #     self.buttonMeasurement["text"]="Start getting correction data"

    def Measurement_action(self):
        self.x_txt.delete("1.0","end")
        self.y_txt.delete("1.0","end")
        self.theta_txt.delete("1.0","end")
        self.serial.SerialWrite('c')
        try:
            reading=self.serial.SerialReadline()
            data_package=reading.split("/")
            cleaned_data = [item.replace('\x00', '').strip() for item in data_package]
            self.x_str=cleaned_data[0]
            self.y_str=cleaned_data[1]
            self.theta_str=cleaned_data[2]
            with open(self.filepath, 'a', newline='') as csvfile:
                 writer = csv.writer(csvfile)
                 data=[[self.x_str,self.y_str,self.theta_str]]
                 writer.writerows(data)

            self.x_txt.insert(END,cleaned_data[0])
            self.y_txt.insert(END,cleaned_data[1])
            self.theta_txt.insert(END,cleaned_data[2])
        except:
            self.x_txt.delete("1.0","end")
            self.y_txt.delete("1.0","end")
            self.theta_txt.delete("1.0","end")
            self.x_txt.insert(END,'Error reading')
            self.y_txt.insert(END,'Error reading')
            self.theta_txt.insert(END,'Error reading')

    def loop(self):
        while True:
            time.sleep(0.5)

            if self.flag == 1:
                #self.Measurement_action()
                pass
            

    def publish(self):
        self.frame.place(x=0,y=380)
        self.buttonMeasurement.grid(column=2,row=1,padx=20,pady=5)
        self.x_correction.grid(column=1,row=2,padx=20,pady=5)
        self.y_correction.grid(column=1,row=3,padx=20,pady=5)
        self.theta_correction.grid(column=1,row=4,padx=20,pady=5)
        self.x_txt.grid(column=2,row=2,padx=20,pady=5)
        self.y_txt.grid(column=2,row=3,padx=20,pady=5)
        self.theta_txt.grid(column=2,row=4,padx=20,pady=5)
        
        

class MoreObject:
    def __init__(self,root):
        self.root=root
        self.frame=LabelFrame(root,text="Logo",padx=5,pady=5,bg="white")
        self.image=PhotoImage(file="GUI/hcmut.png")
        self.image_label=Label(self.root,image=self.image)
        self.frame2=LabelFrame(root,text="Author",padx=5,pady=5,bg="white")
        self.name1=Label(self.frame2,text="Đào Anh Phi :2111996",bg="white",width=20,anchor="w")
        self.name2=Label(self.frame2,text="Vũ Quốc Khánh :2111503",bg="white",width=20,anchor="w")
        self.name3=Label(self.frame2,text="Đặng Văng Vinh :2051209",bg="white",width=20,anchor="w")
        self.publish()

    def publish(self):
        self.frame.grid(rowspan=1,columnspan=1,padx=5,pady=5)
        self.image_label.place(x=700,y=700)
        self.frame2.grid(row=0,column=4,padx=5,pady=5,rowspan=3,columnspan=1)
        self.name1.grid(row=0,column=0)
        self.name2.grid(row=1,column=0)
        self.name3.grid(row=2,column=0)
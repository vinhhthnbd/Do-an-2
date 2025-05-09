from tkinter import *
import serial.tools.list_ports
from tkinter import messagebox
from tkinter import PhotoImage
from PIL import Image, ImageTk
import threading 
import time


class RootGui:
    def __init__(self):
        self.root=Tk()
        self.root.title('Do an 2 - Dieu khien bluetooth ')
        self.root.geometry("600x600")
      



#Create frames
class CommunicationGui:
    def __init__(self,root,serial,control):
        #Frame
        #bring main root
        self.root=root
        #create frame
        self.frame=LabelFrame(root,text="Comport management",padx=8,pady=8,bg="white")

        self.label_com=Label(self.frame,text="Available ports:",bg="white",width=15,anchor="w")

        self.label_baudrate=Label(self.frame,text="Baudrate",bg="white",width=15,anchor="w")
        self.control=control
        self.serial=serial


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
                

            else:
                ErrorMsg = f"Failure to estabish UART connection using {self.clicked_com.get()} "
                messagebox.showerror("showerror", ErrorMsg)
                self.control.BlockControl()
        else:

            # Closing the Serial COM
            # Close the serial communication
            self.serial.SerialClose(self)
            self.control.BlockControl()
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
                self.status=True
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
            # ErrorMsg = f"Failure to transmit data using {self.clicked_com.get()} "
            # messagebox.showerror("showerror", ErrorMsg)
    


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
        self.frame.grid(row=3,column=1,rowspan=6,columnspan=6,padx=5,pady=5)
        self.buttonForward.grid(column=1,row=0,padx=20,pady=20)
        self.buttonBackward.grid(column=1,row=2,padx=20,pady=20)
        self.buttonLeft.grid(column=0,row=1,padx=20,pady=20)
        self.buttonRight.grid(column=2,row=1,padx=20,pady=0)


class MoreObject:
    def __init__(self,root):
        self.root=root
        self.frame=LabelFrame(root,text="Logo",padx=5,pady=5,bg="white")
        self.image=PhotoImage(file="hcmut.png")
        self.image_label=Label(self.root,image=self.image)
        self.frame2=LabelFrame(root,text="Author",padx=5,pady=5,bg="white")
        self.name1=Label(self.frame2,text="Đào Anh Phi :2111996",bg="white",width=20,anchor="w")
        self.name2=Label(self.frame2,text="Vũ Quốc Khánh :2111503",bg="white",width=20,anchor="w")
        self.name3=Label(self.frame2,text="Đặng Văng Vinh :2051209",bg="white",width=20,anchor="w")
        self.publish()

    def publish(self):
        self.frame.grid(rowspan=1,columnspan=1,padx=5,pady=5)
        self.image_label.place(x=500,y=500)
        self.frame2.grid(row=0,column=4,padx=5,pady=5,rowspan=3,columnspan=1)
        self.name1.grid(row=0,column=0)
        self.name2.grid(row=1,column=0)
        self.name3.grid(row=2,column=0)
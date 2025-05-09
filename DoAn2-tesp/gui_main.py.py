from bluetooth_gui_master import *


MySerial=SerialCtrl()
mainGui=RootGui()
img=MoreObject(mainGui.root)
control=Control(mainGui.root,MySerial)
com=CommunicationGui(mainGui.root,MySerial,control)

mainGui.root.mainloop()
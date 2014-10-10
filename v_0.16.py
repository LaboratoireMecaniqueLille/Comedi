# -*- coding: utf-8 -*-
import threading
import comedi as c
import time
import numpy as np
import scipy.interpolate as scipy_interpolate
import matplotlib.pyplot as plt 
import sys
import random
from multiprocessing import Process, Queue, Array, Value
import copy
import math
from xlwt import Workbook
import datetime
np.set_printoptions(threshold='nan', linewidth=500)

class Path:
    def __init__(self,path_file,step):
      self.path_file=path_file
      self.step=step
      self.xp=np.arange(0,60,0.05)
      self.yp=0.5*np.sin(frequence*self.xp)+0.6
      self.degp=0.2*np.sin(frequence*self.xp)+0.6
      
      #with open(self.path_file,'r') as path:
	#self.coordinate=np.array([[float(x) for x in ln.split()] for ln in path]) # transforms raw data in an array of float
	#self.xp=self.coordinate[:,0]
	#self.yp=self.coordinate[:,1]
	#self.degp=self.coordinate[:,2]

      
      self.x=np.arange(np.min(self.xp),np.max(self.xp)+self.step,self.step) # create an array for the x parameter
	
    def interpolate_linear(self):
      self.y=np.interp(self.x,self.xp,self.yp)
      self.deg=np.interp(self.x,self.xp,self.degp)
     
    def interpolate_spline(self):
      spline=scipy_interpolate.InterpolatedUnivariateSpline(self.xp,self.yp)
      self.y=spline(self.x)
      spline_deg=scipy_interpolate.InterpolatedUnivariateSpline(self.xp,self.degp)
      self.deg=spline_deg(self.x)

class Out:
  def __init__(self, device='/dev/comedi0',subdevice=1,channel=0,range_num=1,gain=1,offset=0):
    self.subdevice=subdevice
    self.channel=channel
    self.range_num=range_num
    self.device0=c.comedi_open(device)
    self.maxdata=c.comedi_get_maxdata(self.device0,self.subdevice,self.channel)
    self.range_ds=c.comedi_get_range(self.device0,self.subdevice,self.channel,self.range_num)
    self.out=0
    self.gain=gain
    self.offset=offset
    self.I_term=0
    self.last_sensor_input=0
    self.K=K
    self.Ki=Ki
    self.Kd=Kd
    self.last_time=t0
    self.out_min=0
    self.out_max=4.095
    self.last_output=0
    
  def command(self,wanted_position):
      self.out=(wanted_position-self.offset)/self.gain
      out_a=c.comedi_from_phys(self.out,self.range_ds,self.maxdata) # convert the wanted_position 
      c.comedi_data_write(self.device0,self.subdevice,self.channel,self.range_num,c.AREF_GROUND,out_a) # send the signal to the controler
      t_=datetime.datetime.now() 
      t=(((((((t_.year*12)+t_.month)*30+t_.day)*24+t_.hour)*60+t_.minute)*60+t_.second)*1000000)+t_.microsecond
      return (t-t0,self.out)
      
  def command_PID(self,wanted_position,sensor_input):
      self.time= time.time()
      self.out=(wanted_position-self.offset)/self.gain
      #print "sensor=%s" %sensor_input

      self.error=self.out-sensor_input
      self.I_term += self.Ki*self.error*(self.last_time-self.time)
      
      if self.I_term>self.out_max:
	self.I_term=self.out_max
      elif self.I_term<self.out_min:
	self.I_term=self.out_min
      
      self.out_PID=self.last_output+self.K*self.error+self.I_term-self.Kd*(sensor_input-self.last_sensor_input)/(self.last_time-self.time)
      
      if self.out_PID>self.out_max:
	self.out_PID=self.out_max
      elif self.out_PID<self.out_min:
	self.out_PID=self.out_min
	
      self.last_time=copy.copy(self.time)
      self.last_sensor_input=copy.copy(sensor_input)
      self.last_output=copy.copy(self.out_PID)
      #self.t.append(time.time()-t0)
      #print "I_term= %s, out_PID=%s" %(self.I_term, self.out_PID)
      out_a=c.comedi_from_phys(self.out_PID,self.range_ds,self.maxdata) # convert the wanted_position 
      c.comedi_data_write(self.device0,self.subdevice,self.channel,self.range_num,c.AREF_GROUND,out_a) # send the signal to the controler
      t_=datetime.datetime.now() 
      t=(((((((t_.year*12)+t_.month)*30+t_.day)*24+t_.hour)*60+t_.minute)*60+t_.second)*1000000)+t_.microsecond
      return (t-t0,self.out_PID)

class In:
    def __init__(self,device='/dev/comedi0',subdevice=0,channel=1,range_num=0,gain=1,offset=0): 
      self.subdevice=subdevice
      self.channel=channel
      self.range_num=range_num
      self.device0=c.comedi_open(device)
      self.maxdata=c.comedi_get_maxdata(self.device0,self.subdevice,self.channel)
      self.range_ds=c.comedi_get_range(self.device0,self.subdevice,self.channel,self.range_num)
      self.gain=gain
      self.offset=offset
      #self.y=[]
      #self.t=[]

    def get_position(self):
      data = c.comedi_data_read(self.device0,self.subdevice,self.channel,self.range_num, c.AREF_GROUND)
      self.position=(c.comedi_to_phys(data[1],self.range_ds,self.maxdata)*self.gain+self.offset)
      t_=datetime.datetime.now() 
      t=(((((((t_.year*12)+t_.month)*30+t_.day)*24+t_.hour)*60+t_.minute)*60+t_.second)*1000000)+t_.microsecond
      return ((t-t0), self.position)

if __name__ == '__main__':
  
#THIS SECTION IS FOR INIT AND PARAMETERS
    step = 0.0011 # choose step here
    # set PID parameters below :
    K=1.0
    Ki=0.
    Kd=0.0
    
    frequence=5 # "frequency" of the sinus used for tests in the Path class.
    
    acquisition_step=1000#Define how many points the scripts waits before saving them
    saving_step=1 # Allows you to save 1 point every "saving step": use this parameter for high frequencies and long durations.
    
    Path=Path(path_file='/home/martel/Bureau/chemin', step=step) # open file 'chemin', wich has to be in the same directory
    
    Path.interpolate_linear()    #choose here for interpolation method
    #Path.interpolate_spline()   #

    
    t0_=datetime.datetime.now() # set a common time for all channels
    t0=(((((((t0_.year*12)+t0_.month)*30+t0_.day)*24+t0_.hour)*60+t0_.minute)*60+t0_.second)*1000000)+t0_.microsecond
    
# START YOUR PROGRAM FORM HERE:

#Initialise your classes here:
    # set subdevice to 1 for outputs and 1 for inputs.
    # set channel
    # range_num define the range for the A/D and D/A converters, depending if your input/out is bipolar or not. See the documentation for values.
    Rotate=Out(device='/dev/comedi0',subdevice=1,channel=1,range_num=1,gain=1,offset=0)
    Move=Out(device='/dev/comedi0',subdevice=1,channel=0,range_num=1,gain=1,offset=0) 
    
    Angle=In(device='/dev/comedi0',subdevice=0,channel=1,range_num=0,gain=1,offset=0)
    Position=In(device='/dev/comedi0',subdevice=0,channel=0,range_num=0,gain=1,offset=0)


    
#Variables used to share the in and out data. You need one for each variable in each input/output
    traction_values1=Array('d',acquisition_step) #I.t
    traction_values2=Array('d',acquisition_step)  #I.y
    traction_values3=Array('d',acquisition_step)  #O.t
    traction_values4=Array('d',acquisition_step)  #O.y
    torsion_values1=Array('d',acquisition_step)
    torsion_values2=Array('d',acquisition_step)
    torsion_values3=Array('d',acquisition_step)
    torsion_values4=Array('d',acquisition_step)
    signal=Array('i',2) # This one is used to communicate between processes and secure the saving.
    
    
    # Below are variables used to measure times of execution
    t_com_t=Array('d',1)
    t_pos_t=Array('d',1)
    t_com=Array('d',1)
    t_pos=Array('d',1)
    t_sav=Array('d',1)
    
    
    def f(I,O,path_x,path_y,a,b,c,d,t_com,t_pos,signal_key): # Main function, allows you to control one actuator.
      run=True
      save_time_tot=0
      main_time=0
      wait=0
      #print "top boucle"
      while run==True:
	signal[signal_key]=0 #signal to put the saving process on hold
	#signal[1]=0
	for i in range(len(path_x)):
	  save_time_0=time.time()
	  if i%acquisition_step==0 and i>1: #initiate the saving every "acquisition_step" number of points.
	    t_save=time.time()
	    #print "SAVING for i =%s" %i
	    signal[signal_key]=1
	    #signal[1]=1
	    while signal[signal_key]==1: # wait until the data are safely saved in the log process
	      indent=True  
	    a[0:acquisition_step]=np.zeros(acquisition_step)  # Resets the values of the shared variables
	    b[0:acquisition_step]=np.zeros(acquisition_step)
	    c[0:acquisition_step]=np.zeros(acquisition_step)
	    d[0:acquisition_step]=np.zeros(acquisition_step)
	    t_sav[0]=t_sav[0]+time.time()-t_save
	  save_time_tot+=time.time()-save_time_0
	  k=i%acquisition_step
	  wait0=time.time()
	  t1_=datetime.datetime.now() # set a common time for all channels
	  t1=(((((((t1_.year*12)+t1_.month)*30+t1_.day)*24+t1_.hour)*60+t1_.minute)*60+t1_.second)*1000000)+t1_.microsecond
	  #print "t1=%s" %t1
	  #print "t0=%s" %t0
	  while t1<(t0+(path_x[i]*1000000)):  # Waits for the time set in the path file
	    t1_=datetime.datetime.now() 
	    t1=(((((((t1_.year*12)+t1_.month)*30+t1_.day)*24+t1_.hour)*60+t1_.minute)*60+t1_.second)*1000000)+t1_.microsecond
	    #print "t1_while=%s" %t1
	  wait+=time.time()-wait0
	  #print"top "
	  t_position = time.time()
	  a[k],b[k]=(I.get_position()) # measuring the position and saving it in the shared variables
	  t_pos[0]=t_pos[0]+time.time()-t_position
	  t_command = time.time()  
	  c[k],d[k]=(O.command_PID(path_y[i], b[k])) # setting the position to a new value and saving it in the saherd variables
	  t_com[0]=t_com[0]+time.time()-t_command
	  main_time+=time.time()-save_time_0
	#print "Final SAVE"    
	signal[signal_key]=1 # signal for the final save
	while signal[signal_key]==1: # wait until the last data are safely saved in the log process
	  indent=True
	signal[signal_key]=2 #signal to close the log file
	print "save_time_tot= %s" %(save_time_tot/len(path_x))
	print "main time=%s" %(main_time/len(path_x))
	print "wait time=%s" %(wait/len(path_x))
	#signal[1]=2
	run=False

    def save():# This function saves data in a file.
      condition=True
      save_number=0
      save_time=0
      fig=plt.figure()
      ax=fig.add_subplot(111)
      li,= ax.plot(np.arange(5000),np.zeros(5000))
      lo,= ax.plot(np.arange(5000),np.zeros(5000))
      # draw and show it
      fig.canvas.draw()
      plt.show(block=False)
      x_1=np.empty(1)
      y_1=np.empty(1)
      x_2=np.empty(1)
      y_2=np.empty(1)
      #sheet1.write(x,i("I.t I.y O.t O.y\n")  # initialise the file with the titles of the column on the first line
      while condition==True:
	if signal[0]==1 and signal[1]==1: # signal that the data are ready to be saved
	  t_save=time.time()
	  fo=open("log.txt","a") # "a" for appending
	  fo.seek(0,2) #place the "cursor" at the end of the file, so every writing will not erase the previous ones
	  data_to_save=""
	  values=[traction_values1,traction_values2,traction_values3,traction_values4,torsion_values1,torsion_values2,torsion_values3,torsion_values4]
	  #data_to_save="" # initialise the variable used in the loop below, do not modify
	  data=[]
	  for i in range(np.shape(np.array(values))[0]):# copy the shared data to avoid blocking the acquisition process
	    data.append(copy.copy(np.array(values)[i]))  
	  signal[0]=0 # signal that the data are safe 
	  signal[1]=0
	  data1=np.empty((np.shape(np.array(values))[0],int(math.ceil(len(data[0])//saving_step))))
	  #for i in range(np.shape(np.array(values))[0]):   #This loop cut the end of the last saving array, to remove the "0"'s put on reset
	    #if 0 in data[i] and saving_step>1: 
	      #data[i]=data[i][:list(data[i]).index(0)]
	  if save_number>0:                              # this loop is used for the continous plotting
	    #print "data=%s, x_1=%s" %(len(data[0]),len(x_1))
	    if save_number<6: # This integer define the size of the plot: it plots "x" times the data. 
	      #print data[0]
	      #print x_1
	      x_1=copy.copy(np.concatenate((x_1,(data[0])),axis=1))
	      y_1=copy.copy(np.concatenate((y_1,(data[1])),axis=1))
	      x_2=copy.copy(np.concatenate((x_2,(data[4])),axis=1))
	      y_2=copy.copy(np.concatenate((y_2,(data[5])),axis=1))
	      #print "loop1, size_x: %s, size_y: %s" %(np.shape(x_1),len(y_1))
	    else :   # this loop delete the first values of the plot and add new value at the end to create a continuous plot
	      x_1[:-np.shape(np.array(values))[1]] = x_1[np.shape(np.array(values))[1]:]
	      x_1[-np.shape(np.array(values))[1]:]=data[0]
	      y_1[:-np.shape(np.array(values))[1]] = y_1[np.shape(np.array(values))[1]:]
	      y_1[-np.shape(np.array(values))[1]:]=data[1]
	      x_2[:-np.shape(np.array(values))[1]] = x_2[np.shape(np.array(values))[1]:]
	      x_2[-np.shape(np.array(values))[1]:]=data[4]
	      y_2[:-np.shape(np.array(values))[1]] = y_2[np.shape(np.array(values))[1]:]
	      y_2[-np.shape(np.array(values))[1]:]=data[5]
	      #print"loop2"
	    li.set_ydata(y_1)
	    li.set_xdata(x_1)
	    lo.set_ydata(y_2)
	    lo.set_xdata(x_2)
	    # draw and show it
	    ax.relim()
	    ax.autoscale_view(True,True,True)
	    fig.canvas.draw()
	  if saving_step>1:
	    for x in range(int(math.ceil(len(data[0])//saving_step))): # euclidian division here
	      for i in range(np.shape(np.array(values))[0]):
		if x<(len(data[0])//saving_step):
		  data1[i][x]=(np.mean(data[i][x*saving_step:(x+1)*saving_step]))
		else:
		  data1[i][x]=(np.mean(data[i][x*saving_step:]))
	    #print data1
	    data_to_save=str(np.transpose(data1))+"\n"
	  else:
	    data_to_save=str(np.transpose(data))+"\n"
	    #for x in range(len(data[0])):
	      #for i in range(np.shape(np.array(values))[0]):
		#data_to_save+=str(data[i][x])+" " # the espace here will be use as a separator on excel
	      #data_to_save+="\n"
	  fo.write(data_to_save)
	  fo.close()
	  save_number+=1
	  save_time+=time.time()-t_save
	  #print"writing done!"
	  
	if signal[0]==2 and signal[1]==2: # signal that the acquisition is complete
	  fo.close()
	  plt.ioff()
	  plt.close('all')
	  print "save_time= %s, save_number=%s" %(save_time/save_number, save_number)
	  condition=False    

    def test_signal():
      while signal[0]!=2:
	print "signal[0]=%s , signal[1]=%s" %(signal[0],signal[1])
	time.sleep(0.1)
    
	

#while True:
    #try:
        #y[:-10] = y[10:]
        #y[-10:] = np.random.randn(10)

        ## set the new data
        #li.set_ydata(y)

        #fig.canvas.draw()

        #time.sleep(0.01)
    #except KeyboardInterrupt:
        #break
      
# This part is used to define the processes:
    Traction=Process(target=f,args=(Position,Move,Path.x,Path.y,traction_values1,traction_values2,traction_values3,traction_values4,t_com_t,t_pos_t,0))
    Rotation=Process(target=f,args=(Angle,Rotate,Path.x,Path.deg,torsion_values1,torsion_values2,torsion_values3,torsion_values4,t_com,t_pos,1))
    Save=Process(target=save,args=())
    #Test=Process(target=test_signal, args='')



# This part is used to start the processes
    Save.start()
    Traction.start()
    Rotation.start()
    #Test.start()


    Save.join()
    Traction.join()
    Rotation.join()
    #Test.join()
    

# This part is used to terminate the processes once they are done. DO NOT FORGET this part or the process will keep running on your computer.
    Save.terminate()
    print "save terminated"
    #time.sleep(2)
    Traction.terminate()
    print "traction terminated"
    #time.sleep(2)
    Rotation.terminate()
    print "torsion terminated"
    #Test.terminate()



    print "t_com= %s" %(t_com[0]/len(Path.x))
    print "t_pos= %s" %(t_pos[0]/len(Path.x))
    #print "t_com_t= %s" %(t_com_t[0]/len(Path.x))
    #print "t_pos_t= %s" %(t_pos_t[0]/len(Path.x))
    #print "t_sav= %s" %(t_sav[0]/len(Path.x))
    
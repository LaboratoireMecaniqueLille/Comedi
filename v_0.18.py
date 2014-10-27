# -*- coding: utf-8 -*-
#import threading
import comedi as c
import time
import numpy as np
import scipy.interpolate as scipy_interpolate
import matplotlib.pyplot as plt 
#import sys
#import random
from multiprocessing import Process, Pipe#, Array, Value
import copy
import math
#import datetime
np.set_printoptions(threshold='nan', linewidth=500)
from matplotlib import rcParams
rcParams['font.family'] = 'serif'

class Path:
    def __init__(self,path_file,step):
      self.path_file=path_file
      self.step=step
      self.xp=np.arange(0,6,0.05)
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
    
  def set_(self,wanted_position):
      self.out=(wanted_position-self.offset)/self.gain
      out_a=c.comedi_from_phys(self.out,self.range_ds,self.maxdata) # convert the wanted_position 
      c.comedi_data_write(self.device0,self.subdevice,self.channel,self.range_num,c.AREF_GROUND,out_a) # send the signal to the controler
      #t_=datetime.datetime.now() 
      #t=(((((((t_.year*12)+t_.month)*30+t_.day)*24+t_.hour)*60+t_.minute)*60+t_.second)*1000000)+t_.microsecond
      t=time.time()
      #return (t-t0,self.out)
      
  def set_PID(self,wanted_position,sensor_input):
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
      #t_=datetime.datetime.now() 
      #t=(((((((t_.year*12)+t_.month)*30+t_.day)*24+t_.hour)*60+t_.minute)*60+t_.second)*1000000)+t_.microsecond
      t=time.time()
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

    def get(self):
      data = c.comedi_data_read(self.device0,self.subdevice,self.channel,self.range_num, c.AREF_GROUND)
      self.position=(c.comedi_to_phys(data[1],self.range_ds,self.maxdata)*self.gain+self.offset)
      t=time.time()
      #t_=datetime.datetime.now() 
      #t=(((((((t_.year*12)+t_.month)*30+t_.day)*24+t_.hour)*60+t_.minute)*60+t_.second)*1000000)+t_.microsecond
      return ((t-t0), self.position)

if __name__ == '__main__':
  
################ THIS SECTION IS FOR INIT AND PARAMETERS ################
    step = 0.002 # choose step here
    # set PID parameters below :
    K=1.0
    Ki=0.
    Kd=0.0
    
    frequence=5 # "frequency" of the sinus used for tests in the Path class.
    
    acquisition_step=500#Define how many points the scripts waits before saving them
    saving_step=1 # Allows you to save 1 point every "saving step": use this parameter for high frequencies and long durations.
    
    Path=Path(path_file='/media/corentin/data/Git/Comedi/Chemin', step=step) # open file 'chemin', wich has to be in the same directory
    
    Path.interpolate_linear()    #choose here for interpolation method
    #Path.interpolate_spline()   #

    
    #t0_=datetime.datetime.now() # set a common time for all channels
    #t0=(((((((t0_.year*12)+t0_.month)*30+t0_.day)*24+t0_.hour)*60+t0_.minute)*60+t0_.second)*1000000)+t0_.microsecond
    t0=time.time()
    
################ START YOUR PROGRAM FORM HERE: ################

################ Initialise your classes here:
    # set subdevice to 1 for outputs and 0 for inputs.
    # set channel according to your connection to the camedi card
    # range_num define the range for the A/D and D/A converters, depending if your input/out is bipolar or not. See the documentation for values.
    Rotate=Out(device='/dev/comedi0',subdevice=1,channel=1,range_num=1,gain=1,offset=0)
    Move=Out(device='/dev/comedi0',subdevice=1,channel=0,range_num=1,gain=1,offset=0) 
    
    Angle=In(device='/dev/comedi0',subdevice=0,channel=1,range_num=0,gain=1,offset=0)
    Position=In(device='/dev/comedi0',subdevice=0,channel=0,range_num=0,gain=1,offset=0)


    

################ Variables used to share the in and out data. You need one for each variable in each input/output

    traction_sensor_send, traction_sensor_recv = Pipe()
    traction_time_send, traction_time_recv = Pipe()
    torsion_sensor_send, torsion_sensor_recv = Pipe()
    torsion_time_send, torsion_time_recv = Pipe()
    
    pipe_send,pipe_recv=Pipe() # This one transfert data from save to graph
    
################ Functions:

    def f(I,O,path_x,path_y,time_pipe,sensor_pipe): # Main function, allows you to control one actuator.
      for i in range(len(path_x)):
	t1=time.time()
	while t1<(t0+(path_x[i])):  # Waits for the time set in the path file
	  t1=time.time() 
	a,b=(I()) # measuring the position and saving it in the shared variables
	c,d=(O(path_y[i], b)) # setting the position to a new value and saving it in the saherd variables
	time_pipe.send(a) # send data to the save function
	sensor_pipe.send(b)
      time_pipe.send(0.0) # signal that the acquisition is over
      sensor_pipe.send(0.0)
	  
	  ### This is an alternative to interpolate on the go
	  #t1=time.time()-t0
	#if t1<= path_x[-1]:
	  #y=np.interp(t1,path_x,path_y)
	  ##while t1<(t0+(path_x[i])):  # Waits for the time set in the path file
	    ##t1=time.time() 
	  #a,b=(I()) # measuring the position and saving it in the shared variables
	  #c,d=(O(y, b)) # setting the position to a new value and saving it in the saherd variables
	  #time_pipe.send(a) # send data to the save function
	  #sensor_pipe.send(b)


    def save(*args):# This function saves data in a file and display it in an animated plot
      nbr=len(args)
### INIT
      condition=True
      save_number=0
### Main loop
      while condition==True:
## init data matrixes
	data=[[0 for x in xrange(acquisition_step)] for x in xrange(nbr)] 
	i=0
## This loop fill the data matrix up to "acquisition step" number of values
	while i<acquisition_step and condition==True:
	  for z in range (nbr):
	    data[z][i]=args[z].recv()
	  if data[0][i]==0.0: # if acquisiton is over, save remaining data
	    condition=False
	  i+=1
## send data to plot	
	pipe_send.send(data)
## The following loops are used to save the data
	fo=open("log.txt","a") # "a" for appending
	fo.seek(0,2) #place the "cursor" at the end of the file, so every writing will not erase the previous ones
	data_to_save=""
	data1=np.empty((np.shape(np.array(data))[0],int(math.ceil(len(data[0])//saving_step))))
	if saving_step>1:  # This loop means the data to decrease the number of points to save
	  for x in range(int(math.ceil(len(data[0])//saving_step))): # euclidian division here
	    for i in range(np.shape(np.array(data))[0]):
	      if x<(len(data[0])//saving_step):
		data1[i][x]=(np.mean(data[i][x*saving_step:(x+1)*saving_step]))
	      else:
		data1[i][x]=(np.mean(data[i][x*saving_step:]))
	  data_to_save=str(np.transpose(data1))+"\n"
	else:  # this loop save all data
	  data_to_save=str(np.transpose(data))+"\n"
	fo.write(data_to_save)
	fo.close()
	save_number+=1



    def plot(graph_recv_n,nbr_graphs): # plot up to 3 differents graphs in one figure, and keep a fixed abscisse range. On update, old plots are erased and new ones are added. No memory overload, this plot is safe even for long plots.
      condition=True
      save_number=0
## init the plot
      fig=plt.figure()
      ax=fig.add_subplot(111)
      li,= ax.plot(np.arange(5000),np.zeros(5000))
      if nbr_graphs ==2: # add a 2nd graph in the same plot
	lo,= ax.plot(np.arange(5000),np.zeros(5000))
      if nbr_graphs ==3: # add a 3rd graph
	la,= ax.plot(np.arange(5000),np.zeros(5000))
      #ax.set_ylim(0,1.2)
      fig.canvas.draw()     # draw and show it
      plt.show(block=False)
      nbr=nbr_graphs*2
      var=[[]]*nbr
      while condition==True:
	data=graph_recv_n.recv()
## this loop is used for the continous plotting
	if save_number>0:                              
	  if save_number==1: # this loop init the first round of data
	    for z in range(nbr):
	      var[z]=copy.copy(data[z])
	  if save_number<6 and save_number>1: # This integer define the size of the plot: it plots "x" times the data. 
	    for z in range(nbr):
	      var[z]=copy.copy(np.concatenate((var[z],(data[z])),axis=1))
	  else :   # this loop delete the first values of the plot and add new value at the end to create a continuous plot
	    for z in range(nbr):
	      var[z][:-np.shape(np.array(data))[1]] = var[z][np.shape(np.array(data))[1]:]
	      var[z][-np.shape(np.array(data))[1]:]= data[z]
	  li.set_xdata(var[0])
	  li.set_ydata(var[1])  # update the graph values
	  if nbr_graphs ==2:
	    lo.set_xdata(var[2])
	    lo.set_ydata(var[3])
	  if nbr_graphs ==3:
	    la.set_xdata(var[4])
	    la.set_ydata(var[5])
	  ax.relim()
	  ax.autoscale_view(True,True,True)
	  fig.canvas.draw() 
	save_number+=1

    def plot_value_value(graph_recv_n,order): # This function plot one or 2 graph of  y=f(x) , and you can choose y and x in the order variable. Autoscale, but doesn't reset. BEWARE, long plots may cause data losses.
      condition=True
      nbr=len(order) # number of variables 
      plt.ion()
      fig=plt.figure()
      while condition==True:
	data=graph_recv_n.recv()  # receive data from main graph process
	plt.plot(data[order[0]],data[order[1]],'b-')
	plt.xlabel(order[-1][0])
	plt.ylabel(order[-1][1])
	if nbr ==4:
	  plt.plot(data[order[2]],data[order[3]],'r-')
	plt.draw()
	
	
    def graph(*args): # This function as to be called in a process. It create the desired plots and updates the data in link with the save function.
      condition=True
      graph_send={}
      graph_recv={}
      graph_n={}
      data=pipe_recv.recv() # this pipe receive data from the save function
      nbr_graphs=len(args)
      for i in range(nbr_graphs): 
	graph_type=args[i][0] # the first value of args[i] is the graph type
	graph_args=args[i][1:] # other values depend on the graph type
	graph_send[i],graph_recv[i]=Pipe() #creating pipes to communicate with the graphs to be created
	if graph_type=='values':
	  graph_send[i].send(data) #init the pipe
	  graph_n[i]=Process(target=plot_value_value,args=(graph_recv[i],graph_args)) # creating a new process for each graph
	if graph_type=='time':
	  graph_send[i].send(data)#init the pipe
	  graph_n[i]=Process(target=plot,args=(graph_recv[i],graph_args[0]))# creating a new process for each graph
	graph_n[i].start() #start graphs processes
      while condition==True: # this loop will feed the pipes with new data received from the save process.
	data=pipe_recv.recv()
	for i in range(nbr_graphs):
	  graph_send[i].send(data)


	
	
	

    def test_signal(): ## debug function, deprecated
      while signal[0]!=2:
	print "signal[0]=%s , signal[1]=%s" %(signal[0],signal[1])
	time.sleep(0.1)
    
    try:
      
  ################ This part is used to define the processes:
      Traction=Process(target=f,args=(Position.get,Move.set_PID,Path.x,Path.y,traction_time_send,traction_sensor_send))
      Rotation=Process(target=f,args=(Angle.get,Rotate.set_PID,Path.x,Path.deg,torsion_time_send,torsion_sensor_send))
      Save=Process(target=save,args=(traction_time_recv,traction_sensor_recv,torsion_time_recv,torsion_sensor_recv))
      Graph=Process(target=graph,args=([['time',2],['values',0,1,2,3,['Time (s)','Value (unit)']]]))


  ################ This part is used to start the processes
      Save.start()
      Traction.start()
      Rotation.start()
      Graph.start()


      Save.join()
      Traction.join()
      Rotation.join()
      Graph.join()

      
    except (KeyboardInterrupt):
  ################ This part is used to terminate the processes once they are done. DO NOT FORGET this part or the process will keep running on your computer.
      Save.terminate()
      print "save terminated"
      Traction.terminate()
      print "traction terminated"
      Rotation.terminate()
      print "torsion terminated"
      Graph.terminate()
      print "graph terminated"


    
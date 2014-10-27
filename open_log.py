import numpy as np
import scipy.ndimage as nd


### basic commands to open a log file 
def get_data(file_path,columns):
  a=np.loadtxt(file_path,dtype=str,usecols=(columns)) # load the file in str format ### put here the column you need 
  b=np.char.rstrip(a, ']')  # remove the useless ]
  c=b.astype(np.float64)  # convert to float
  d=c[0:(list(c[:,0])).index(0.00000000e+00),:]  # remove all the useless 0's at the end 
  data=np.transpose(d) # allow to use data as column


get_data('log.txt',(1,2,3,4))
### specific loop for calculating the mean value
#mean=[]
#path="/home/corentin/Bureau/Temperature_thermocouple/"
#for i in range(0,18,1):
  #file_name='T'+str(i)+'.txt'
  #a=np.loadtxt(path+file_name,dtype=str,usecols=(1,2)) # load the file in str format
  #b=np.char.rstrip(a, ']')  # remove the useless ]
  #c=b.astype(np.float64)  # convert to float
  #d=c[0:(list(c[:,0])).index(0.00000000e+00),:]  # remove all the useless 0's at the end 
  #mean.append(np.mean(d[:,1]))
  
  
### specific loop for calculating the T values during traction
path="/home/corentin/Bureau/Temperature_thermocouple/"
d=[]
for i in range(13,17,1):
  file_name='T'+str(i)+'-'+str(i+1)+'_traction'+'.txt'
  a=np.loadtxt(path+file_name,dtype=str,usecols=(1,2)) # load the file in str format
  b=np.char.rstrip(a, ']')  # remove the useless ]
  c=b.astype(np.float64)  # convert to float
  d.append(nd.gaussian_filter(c[:,1],10))  
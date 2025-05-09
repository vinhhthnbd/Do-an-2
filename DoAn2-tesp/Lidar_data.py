import numpy as np

import matplotlib.pyplot as plt


class lidarData:
    def __init__(self) :
        self.pose=[]
        self.z=[]
        self.a=[]
        self.c=[]


def ransacAlgorithm(z,N,S,D,threshold,C):
    # pose :3*1 matrix (x,y,theta),from odometry
    # z: observation maxtrix (data which gets from lidar) (2*n matrix) :z[0,:] is range; z[1,:] is phi
    # N: The number of loops to do the ransac algorithm
    # S: Number of samples to compute initial line.
    # D: Degrees from initial reading to sample from
    # threshold: the maxmimum distance from landmarks to the ransac line in order to be recognized as a point in a line
    # C: the minimum number of points lie on best fit line so it can count as a perfect line in order to calcutate a best fit lines base on every single points in z matrix
        #(C> 5 samples)
    #find the position of lidar lankmark
    n=z.shape[1]
    [xp,yp]=[0,0]
    #xlm=np.zeros((n))#temporary landmark coordinates
    #ylm=np.zeros((n))
    #for i in range(n):
    #    xlm[i]=z[0][i]
    #    ylm[i]=z[1][i]
    
    a=[]# ax-y+c=0 -> b=-1
    c=[]
    dis=np.zeros((n))#distances between the points and the line
    
    
    for i in range(N):
        xbf=[]
        ybf=[]
        #if the the number of unasociated point is too low, then escape
        h=z.shape[1]
        if h<C:
            break
        datax=[]
        datay=[]
        sample_point=np.random.randint(h)
        #get S point in an area of D radians
        for j in range(h):
            if len(datax)==S:
                break
            ## Handling -pi->pi angle 
            if (np.abs(z[3][j]-z[3][sample_point]))<=D or (np.abs(2*np.pi-np.abs(z[3][j]-z[3][sample_point]))):
                datax.append(z[0][j])
                datay.append(z[1][j])
         
        if len(datax)==0:
            break
        #compute the best fit lines based on sample points
        ak,ck=np.polyfit(np.array(datax),np.array(datay),1)
        
        #compute the distances between the points and the line and determine find how many points that lies on the best fil line
        num_points=0
        delete_matrix=[]
        for j in range(z.shape[1]):
            dis[j]=(np.abs(ak*z[0][j]-z[1][j]+ck))/(np.sqrt(np.abs(np.power(ak,2)+1)))#distance=|a*x+b*y+c|/sqrt(a^2+b^2)
            if dis[j]<=threshold:
                num_points=num_points+1
                #store the points that inline
                xbf.append(z[0][j])
                ybf.append(z[1][j])
                delete_matrix.append(j)
        #if there are any lines that satisfy the concesus condition, we calulate the best fit lines base on the points that lie on the upper best fit line
        if num_points>=C:
            #calculate new least squares best fit line based on all the laser readings determined to lie on the old best fit line
            xbf=np.array(xbf)
            ybf=np.array(ybf)
            at,ct=np.polyfit(xbf,ybf,1)
            #Add this best fit line to the lines we have extracted
            a.append(at)
            c.append(ct)
            #Remove the number of readings lying on the line from the total set of unassociated readings
            z=np.delete(z,delete_matrix,axis=1)
        #########

    #extract the landmark
    a=np.array(a)
    c=np.array(c)
    m=a.shape[0]
    lm=np.zeros((2,m))
    for i in range(m):
        cn=1*(-xp)+a[i]*(-yp)
        mat_a=np.array([[a[i],-1],[1,a[i]]])# matrix |a  b| |x|=|-c1| -> find  x,y 
        mat_b=np.array([[-c[i]],[-cn]])#              |-b a| |y| |-c2|
        ld=np.linalg.inv(mat_a)@mat_b
        lm[0][i]=ld[0][0]
        lm[1][i]=ld[1][0]

    ##########
    ##change output to 1 matrix:
    return a,c,lm
    

####Test part, if you want to test, then uncomment it

# pose=np.array([0,0,0])

# z=np.array([[1,1.005,0.998,1.002,1,1],[2,3,4,5,6,7],[np.sqrt(5),np.sqrt(10.005),np.sqrt(16.998),np.sqrt(25.002),np.sqrt(37),np.sqrt(50)],[np.atan2(2,1),np.atan2(3,1.005),np.atan2(4,0.998),np.atan2(5,1.002),np.atan2
#                                                                                                                   (6,1),np.atan2(7,1)]])
# a,c,p=RansacAlgorithm(z,N=10,S=3,D=np.pi/10,threshold=0.6,C=5)
# plt.plot(pose[0],pose[1],'ob')
# plt.plot(z[0,:],z[1,:],'or')

# xn=np.arange(-10,10,0.5)
# for i in range(a.shape[0]):
#     yn=np.polyval([a[i],c[i]],xn)
#     plt.plot(xn,yn)
# plt.plot(p[0,:],p[1,:],'og')
# print('a matrix='+str(a))
# print('c matrix='+str(c))
# print('xc matrix='+str(p[0,:]))
# print('yc matrix='+str(p[1,:]))
# plt.show()



import numpy as np
#this module was based on SLAM for Dummies and Thrun et al_2005_Probabilistic robotics
#beside we also use some formula, sign, constain from Giorgio Grisetti, Cyrill Stachniss,
#Kai Arras, Wolfram Burgard slide

class EKF_SLAM():
    def __init__(self,maximum_landmark,odo_c,mea_mm,mea_rad,alpha=0):
        #bd : nosie constant of Rt matrix
        # N: user define -> how many landmarks
        # mean : an (3+2N)*1 matrix, first 3 elements are x,y, theta of robot
        # covariance: an (3+2N)*(3+2N) matrix 
        # Rtx : 3*3 matrix : which uncertainty we add in motion (x,y,theta), it doesn't the uncertainty of the landmark, only change uncertainty of pose
        # Qt: user define (2*2 matrix) the noise of measurement
        self.maximum_landmark=maximum_landmark
        self.Nt=int(0)
        self.mean=np.zeros((2*self.maximum_landmark+3,1))#init mean matrix
        self.covariance=np.zeros((2*self.maximum_landmark+3,2*self.maximum_landmark+3))#init covariance matrix
        self.delta_x=0# get from encoder(microcontrollers)
        self.delta_y=0# get from encoder(microcontrollers)
        self.delta_theta=0# get from encoder(microcontrollers)
        self.odo_c=odo_c#odometry error constant 
        self.mea_mm=mea_mm#error of range measurment (%)
        self.mea_rad=mea_rad#error of angle measurment (radian)
        self.Rt=[]#3*3 matrix
        self.Qt=[]#2*2 matrix
        self.Mahanobis_threshold=5.991#we have this threshold constant define from Chi square distribution table (which 2 dregee off freedom and 5%) 
        self.Fx=np.hstack([np.eye(3,3),np.zeros((3,2*self.maximum_landmark))])#3*(2N+3) matrix
        self.zt=[]# init observation model
        for i in range(3,2*self.maximum_landmark+3):
            self.covariance[i][i]=1e7

    #motion update
    def prediction(self):
    
        
        #line 3 in report
        #predict robot’s pose
        self.mean[0][0]=self.mean[0][0]+self.delta_x
        self.mean[1][0]=self.mean[1][0]+self.delta_y
        self.mean[2][0]=self.mean[0][0]+self.delta_x

        #line 4 in report
        Gtx=np.array([[1,0,-self.delta_y],[0,1,self.delta_x],[0,0,1]]) #3*3 matrix
        Gt=np.eye(self.maximum_landmark+3,self.maximum_landmark+3) + (self.Fx.T).dot(Gtx).dot(self.Fx)
        ##
        W=np.array([[self.delta_x],[self.delta_y],[self.delta_theta]])#3*1 matrix
        self.Rt=W@self.odo_c@(W.T)

        #line 5 in report
        #predict robot’s pose covariance
        self.covariance=Gt.dot(self.covariance).dot(Gt.T)+(self.Fx.T).dot(self.Rtx).dot(self.Fx)#(2N+3)*(2N+3) matrix
        
    #
    def correction(self,z):
        # z:this matrix contains information about the landmark we obsered from sensor, each time we read data from sensor, the osb matrix needs to be added, this matrix needs 2 element[range, theta]
        #z :2*x matrices(x: number of extracted landmarks)(this is a matrix contains coordinates of landmark after we use RANSAC algorithm)
        H=[]# a list to store a bunch of 2*(2N+3) matrix 
        si=[]#a list to store a bunch of 2*2 matrix (landmark covariance)
        z_hat=[]#2*n matrix
       
        #line 6 in report
        for k in range(self.Nt):
            delta_x=self.mean[2*k+3][0]-self.mean[0][0]
            delta_y=self.mean[2*k+4][0]-self.mean[1][0]
            delta=np.array([[delta_x],[delta_y]])
            q=(delta.T)@delta

            #find saved landmarks’ range and bearing
            z_hat_t=np.array([[np.sqrt(q)],[np.atan2(delta_y,delta_x)-self.mean[2][0]]])
            z_hat.append(z_hat_t)


            Fx_j=np.hstack([[np.eye(5,3)],[np.zeros(5,2*(k+1)-2)],[np.vstack([[np.zeros((3,2))],[np.eye(2,2)]])],[np.zeros((5,self.maximum_landmark-2*(k+1)))]])
            Ht=(1/q)*np.array([[-np.sqrt(q)*delta_x,-np.sqrt(q)*delta_y,0,np.sqrt(q)*delta_x,np.sqrt(q)*delta_y],
                        [delta_y,-delta_x,-1,-delta_y,delta_x]])@Fx_j
            #Qt
            self.Qt=np.array([[np.power(z_hat_t*self.mea_mm,2),0],[0,np.power(self.mea_rad,2)]])

            #find saved landmarks’ covariance1
            si_t=Ht@self.covariance@(Ht.T)+self.Qt#landmark covariance
            si.append(si_t)
            H.append(Ht)
        #endfor

        #line 7 in report
        for i in range(z.shape[1]):
            #assume that new observation is an unobserved landmark
            self.mean[2*self.Nt+3][0]=z[0][i]# (Nt+1) x landmark
            self.mean[2*self.Nt+4][0]=z[1][i]# (Nt+1) y landmark
            ######
            delta_x=self.mean[2*self.Nt+3][0]-self.mean[0][0]# (Nt+1) x landmark-x pose
            delta_y=self.mean[2*self.Nt+4][0]-self.mean[1][0]# (Nt+1) y landmark-y pose
            delta=np.array([[delta_x],[delta_y]])
            q=(delta.T)@delta


            #find observation’s range, theta
            z_hat_t=np.array([[np.sqrt(q)],[np.atan2(delta_y,delta_x)-self.mean[2][0]]])
            try:
                z_hat[self.Nt]=z_hat_t
            except:
                z_hat.append(z_hat_t)

            Fx_j=np.hstack([[np.eye(5,3)],[np.zeros(5,2*(self.Nt+1)-2)],[np.vstack([[np.zeros((3,2))],[np.eye(2,2)]])],[np.zeros((5,self.maximum_landmark*2-2*(self.Nt+1)))]])# this is a 5*(2*N+3) matrix
            Ht=(1/q)*np.array([[-np.sqrt(q)*delta_x,-np.sqrt(q)*delta_y,0,np.sqrt(q)*delta_x,np.sqrt(q)*delta_y],
                        [delta_y,-delta_x,-1,-delta_y,delta_x]])@Fx_j# this is a 2*(2N+3) matrix
            
            #Qt:from z_hat which is z_hat[self.Nt](get the estimated landmark we just add)
            self.Qt=np.array([[np.power(z_hat_t*self.mea_mm,2),0],[0,np.power(self.mea_rad,2)]])
       
            si_t=Ht@self.covariance@(Ht.T)+self.Qt#landmark covariance

            try:
                si[self.Nt]=si_t
            except:
                si.append(si_t)

            try:
                H[self.Nt]=Ht
            except:
                H.append(Ht)
            
            #calculate mahalanobis distance
            maha_pi=[]#a list to store float variables (mahalanobis distance)
            for k in range(self.Nt):
                maha_pi_k=((z_hat_t-z_hat[:,k]).T)@np.linalg.inv(si[k])@(z_hat_t-z_hat[:,k])
                
                maha_pi.append(maha_pi_k)
            #endfor
      
            maha_pi.append(self.Mahanobis_threshold)
            
            j=int(np.argmin(maha_pi))
          
            if j>(self.Nt-1):#j is index, meanwhile self.Nt is size, and we put the index into the aray
                self.Nt+=1

            
            K=self.covariance@(H[j].T)@(np.linalg.inv(si[j]))
          
            #### change
            self.mean=self.mean+K@(z_hat_t-z_hat[:,j])
                                                            ###can be replaced by self.maxmimum_landmark
            self.covariance=(np.eye(self.covariance.shape[0],self.covariance.shape[1])-K@H[j])@self.covariance
        #end

            



        
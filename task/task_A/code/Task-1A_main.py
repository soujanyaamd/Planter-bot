#classes and subclasses to import
import cv2
import numpy as np
import os 

filename = 'result1A_3759.csv'

#subroutine to write results to a csv
def writecsv(color,shape,(cx,cy)):
    global filename
    #open csv file in append mode
    filep = open(filename,'a')
    # create string data to write per image
    datastr = "," + color + "-" + shape + "-" + str(cx) + "-" + str(cy)
    #write to csv
    filep.write(datastr)
    filep.close()

def main(path):
    img=cv2.imread(path)
    colMask(img)
    path1=path.replace('.png','output.png')
    cv2.imwrite(path1,img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
def shapeC(img,cimg,color):
    shape,cx,cy='',0,0
    gray=cv2.cvtColor(cimg,cv2.COLOR_BGR2GRAY)
    ret,thresh=cv2.threshold(gray,20,255,0)
    im2,contours,hierarchy=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    i=0
    while(i<len(contours)):
         M = cv2.moments(contours[i])
         if M['m00']!=0 :
             cx = int(M['m10']/M['m00'])
             cy = int(M['m01']/M['m00'])
         cnt=contours[i]
         approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
         if len(approx)==6:
            shape='Hexagon'
         elif len(approx)==5  :
            shape='Pentagon'
         elif len(approx)==3:
            shape='Triangle'
         elif len(approx)==4:
             h,w,c=cimg.shape
             img1=np.zeros((h,w,c),np.float32)
             img1[:,:,:]=255
             cv2.drawContours(img1,[cnt],0,(0,255,0),-1)
             gray1=cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
             corners = cv2.goodFeaturesToTrack(gray1,4,0.01,10)
             flag,n,a,b,slope=0,0,[0]*4,[0]*4,[0]*6
             for j in range(0,4):
                 a[j],b[j]=corners[(j,0)]
             for j in range(0,4):
                 for k in range(j+1,4):
                     slope[n]=round((b[k]-b[j])/(a[k]-a[j]),1)
                     n+=1
             for j in range(0,6):
                 for k in range(0,6):
                     if(slope[k]==slope[j]and j!=k):
                         flag+=1
             if flag>=4:
                 shape='Rhombus'
             else:
                 shape='Trapezium'
         else:                          
            shape='Circle'
         #considering the 6 sample shapes only, else shape is taken as circle
         centroid='('+str(cx)+','+str(cy)+')'
         text='     ' +'\n  '+ color+ '\n   ' +shape+ '\n   ' +centroid
         writecsv(color.lower(),shape.lower(),(cx,cy))
         y0,dy=cx-60,4
         for j,line in enumerate(text.split('\n')):
            y = y0+j*dy
            cv2.putText(img,line,(cx-85+20*j,cy-40+20*j),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),2)
         i=i+1
         

def colMask(img):
    #ranges of values for each color are taken from sample images
    l1=np.array([100,0,0])
    u1=np.array([255,0,0])
    mask=cv2.inRange(img,l1, u1)
    blue=cv2.bitwise_and(img, img, mask= mask)
    shapeC(img,blue,'Blue')
    l2=np.array([0,100,0])
    u2=np.array([0,255,0])
    mask=cv2.inRange(img,l2, u2)
    green=cv2.bitwise_and(img, img, mask= mask)
    shapeC(img,green,'Green')
    l3=np.array([0,0,100])
    u3=np.array([0,0,255])
    mask=cv2.inRange(img,l3, u3)
    red=cv2.bitwise_and(img, img, mask= mask)
    shapeC(img,red,'Red')

#main where the path is set for the directory containing the test images
if __name__ == "__main__":
    global filename
    mypath = '.'
    #getting all files in the directory
    onlyfiles = [os.path.join(mypath,f) for f in os.listdir(mypath) if f.endswith(".png")]
    #iterate over each file in the directory
    for fp in onlyfiles:
        #Open the csv to write in append mode
        filep = open(filename,'a')
        #this csv will later be used to save processed data, thus write the file name of the image
        fp=fp[2:len(fp)]
        filep.write(fp)
        #close the file so that it can be reopened again later
        filep.close()
        #process the image
        data = main(fp)
        print data
        #open the csv
        filep = open(filename,'a')
        #make a newline entry so that the next image data is written on a newline
        filep.write('\n')
        #close the file
        filep.close()

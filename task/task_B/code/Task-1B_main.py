#classes and subclasses to import
import cv2
import numpy as np
import os
import datetime

filename = 'result1B_3759.csv'

#subroutine to write results to a csv
def writecsv(color,shape,(cx,cy)):
    global filename
    #open csv file in append mode
    filep = open(filename,'a')
    # create string data to write per image
    datastr = "," + color + "-" + shape + "-" + str(cx) + "-" + str(cy)
    #write to csv
    filep.write(datastr)

def blend_transparent(face_img, overlay_t_img):
    # Split out the transparency mask from the colour info
    overlay_img = overlay_t_img[:,:,:3] # Grab the BRG planes
    overlay_mask = overlay_t_img[:,:,3:]  # And the alpha plane

    # Again calculate the inverse mask
    background_mask = 255 - overlay_mask

    # Turn the masks into three channel, so we can use them as weights
    overlay_mask = cv2.cvtColor(overlay_mask, cv2.COLOR_GRAY2BGR)
    background_mask = cv2.cvtColor(background_mask, cv2.COLOR_GRAY2BGR)

    # Create a masked out face image, and masked out overlay
    # We convert the images to floating point in range 0.0 - 1.0
    face_part = (face_img * (1 / 255.0)) * (background_mask * (1 / 255.0))
    overlay_part = (overlay_img * (1 / 255.0)) * (overlay_mask * (1 / 255.0))

    # And finally just add them together, and rescale it back to an 8bit integer image    
    return np.uint8(cv2.addWeighted(face_part, 255.0, overlay_part, 255.0, 0.0))


def main(video_file_with_path):
    cap = cv2.VideoCapture(video_file_with_path)
    image_red = cv2.imread("Overlay_Images\\yellow_flower.png",-1)
    image_blue = cv2.imread("Overlay_Images\\pink_flower.png",-1)
    image_green = cv2.imread("Overlay_Images\\red_flower.png",-1)
    
    if(not(cap.isOpened())):
        print 'Video cannot be opened'
        return
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    fps=cap.get(cv2.CAP_PROP_FPS)
    w=(int)(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h=(int)(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out = cv2.VideoWriter('video_output.mp4', fourcc, fps, (w,h))
    cap.set(cv2.CAP_PROP_POS_FRAMES,1)
    cap.grab()
    ret, img = cap.read()
    start_time = datetime.datetime.now()
    count,j,n=0,0,50
    acx,acy,acolor,ashape,ov=[0]*n,[0]*n,['']*n,['']*n,[0]*n
    while(ret):
        ret2, img = cap.read()
        count+=1
        oimg=img
        if(ret2):
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            ret2,thresh = cv2.threshold(gray,127,255,0)
            image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            imgblue,imggreen,imgred=mask(img)
            flower,shape,color,cx,cy='','','',0,0
            i = 1
            while(i < len(contours)):
                ov[j]=0
                cnt = contours[i]
                i=i+1
                M = cv2.moments(cnt)
                if(M['m00']!=0):
                   cx = int(M['m10']/M['m00'])
                   cy = int(M['m01']/M['m00'])
                if (240 < img.item(cy,cx,0) <  255) and (240 < img.item(cy,cx,2) < 255) and (240 < img.item(cy,cx,1) < 255):
                   continue
                shape=cntshape(cnt,img)
                if (imggreen.item(cy,cx,1) > 0 ) :
                    color,flower='Green',image_green
                elif (imgblue.item(cy,cx,0) > 0 ) :
                    color,flower='Blue',image_blue
                elif (imgred.item(cy,cx,2) > 0 ) :
                    color,flower='Red',image_red
                k,flag=0,0
                while k<j:
                    if(cx==acx[k] and cy==acy[k] and color==acolor[k] and shape==ashape[k]):
                       flag=1
                       ov[k]+=1
                       break 
                    k=k+1
                if(flag==1 and ov[k]>2):
                    oimg=img1
                    continue
                if(flag==0):
                    acolor[j],ashape[j],acx[j],acy[j],ov[j]=color,shape,cx,cy,1
                    j=j+1
                if(ov[k]==2):
                    print color+'-'+shape+'-'+str(cx)+'-'+str(cy)
                    writecsv(color,shape,(cx,cy))
                    img1=img
                    x,y,w,h = cv2.boundingRect(cnt)
                    overlay_image = cv2.resize(flower,(w,h))
                    img1[y:y+h,x:x+w,:] = blend_transparent(img[y:y+h,x:x+w,:], overlay_image)
                    oimg=img1                
            out.write(oimg)
            cv2.waitKey(40)
        else:
            break
    print count
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    end_time = datetime.datetime.now()
    tp = end_time-start_time
    print tp
   
def cntshape(cnt,cimg):
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
        a=[0]*4
        b=[0]*4
        for j in range(0,4):
            a[j],b[j]=corners[(j,0)]
        dist=[0]*6
        n=0
        for j in range(0,4):
            for k in range(j+1,4):
                dist[n]=np.sqrt( ((a[k] - a[j])**2) + ((b[k] - b[j])**2) )
                n+=1
        for j in range(0,6):
            flag=0
            for k in range(0,6):
                if j!=k:
                    if ((dist[j]<=(dist[k]+3))and (dist[j]>(dist[k]+3)))or ((dist[j]>=(dist[k]-3))and (dist[j]<(dist[k]+3))):
                        flag+=1
            if flag>=3:
                break            
        if flag>=3:
            shape='Rhombus'
        else:
            shape='Trapezium'       
    else:
       shape='Circle'
    return shape

def mask(img):
    l1=np.array([100,0,0])
    u1=np.array([255,0,0])
    mask=cv2.inRange(img,l1, u1)
    blue=cv2.bitwise_and(img, img, mask= mask)
    l2=np.array([0,100,0])
    u2=np.array([0,255,0])
    mask=cv2.inRange(img,l2, u2)
    green=cv2.bitwise_and(img, img, mask= mask)
    l3=np.array([0,0,100])
    u3=np.array([0,0,255])
    mask=cv2.inRange(img,l3, u3)
    red=cv2.bitwise_and(img, img, mask= mask)
    return blue,green,red

#main where the path is set for the directory containing the test images
if __name__ == "__main__":
    main('video2.mp4')

import numpy as np
import matplotlib.pyplot as plt

from skimage import data
import cv2
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from matplotlib import cm #colormap

# if os.path.isfile('sample.jpg'): im = array(Image.open('sample.jpg'))
img_ref = cv2.imread('marked-images/reference.png',0)
img_act = cv2.imread('marked-images/actual.png',0)

blur_ref = cv2.GaussianBlur(img_ref,(5,5),0)
ret,thresh_ref = cv2.threshold(blur_ref,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

blur_act = cv2.GaussianBlur(img_act,(5,5),0)
ret,thresh_act = cv2.threshold(blur_act,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)


titles = ['Reference Image','Actual Cut line']
images = [thresh_ref, thresh_act]

for i in xrange(len(images)):
    plt.subplot(1,len(images),i+1),plt.imshow(images[i],'gray')
#     plt.subplot(2,3,i+1),plt.imshow(images[i])
    plt.title(titles[i])
    plt.xticks([]),plt.yticks([])
plt.show()

contours_ref, hierarchy = cv2.findContours(thresh_ref,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
contours_act, hierarchy = cv2.findContours(thresh_act,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

boundary_ref = np.vstack(contours_ref[1]).squeeze()
np.savetxt('boundary_files/boundary_ref.out', boundary_ref)

boundary_act = np.vstack(contours_act[1]).squeeze()
np.savetxt('boundary_files/boundary_act.out', boundary_act)

cv2.drawContours(img_ref,contours_ref,-1,(255,255,255),1)
cv2.imshow("Contour",img_ref)
cv2.waitKey(0)

cv2.drawContours(img_act,contours_act,-1,(255,255,255),1)
cv2.imshow("Contour",img_act)
cv2.waitKey(0)
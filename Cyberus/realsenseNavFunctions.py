import cv2
import numpy as np
from scipy import ndimage
from scipy.ndimage import morphology as scmorph

class NavData:
    # common class structure for handling navigation tasks
    def __init__(self):
        self.coeff0 = 1
        self.coeff1 = 0.5
        self.thresh = 20
        self.yaw_error = 0
        self.mean_disp = 0
        self.cX = 0
        self.cY = 0
        self.est_dist = 100
        

    # Segments the depth map and identifies objects based on contour information
    def depthmap_seg_nav(self, depth_im_col, imwidth):
        
        # initialize
        mean = 0
        found_direction = 0
        cont_min_size = 1500
        
        # to segment convert depth array to greyscale, then threshold
        depth_im_gray = cv2.cvtColor(depth_im_col, cv2.COLOR_BGR2GRAY)
        depth_thresh = cv2.adaptiveThreshold(depth_im_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 3, 5)
        depth_thresh[depth_im_gray < 5] = 0
    
        # remove white noise by eroding thresholded function
        kernel = np.ones((2, 2), np.uint8)
        opening = cv2.morphologyEx(depth_thresh, cv2.MORPH_OPEN, kernel, iterations=2)
        dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)

        _, sure_fg = cv2.threshold(dist_transform, 0.12 * dist_transform.max(), 255, 0)
        sure_fg = np.uint8(sure_fg)
        
        _, contours, hierarchy = cv2.findContours(sure_fg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
  
        for cntr in contours:
            if cont_min_size < cv2.contourArea(cntr):
                cv2.drawContours(depth_im_col, [cntr], 0, (0, 255, 0), 2)
                mask = np.zeros(depth_im_gray.shape, np.uint8)
                cv2.drawContours(mask, [cntr], 0, (255, 255, 255), -1)
                mean_new = cv2.mean(depth_im_gray, mask)
            
                if mean_new > mean:
                    mean = mean_new
                    cntr_direction = cntr
                    found_direction = 1

        # we can attempt to use segmentation for navigation, for example by finding
        # the centroid of a certain region (and navigating towards it)
        if found_direction:
            M = cv2.moments(cntr_direction)
            self.cX = int(M["m10"] / M["m00"])
            self.cY = int(M["m01"] / M["m00"])
            self.mean_disp = mean[0]
        
            self.yaw_error = self.cX - (imwidth/2)
    
        return True



    def depthmap_flow_nav(self, d_im, imwidth):
        # calculates a crude depth flow field and identifies a likely clear path
        # by template matching to a gaussian distance function

        # create appropriate kernel
        kernlen = imwidth/2 + 1
        dirac_im = np.zeros((kernlen, kernlen))
        dirac_im[kernlen//2, kernlen//2] = 1
    
        gauss_template = cv2.GaussianBlur(dirac_im, (kernlen, kernlen), 0)
        max_g = max(gauss_template.max(axis=1))
        gauss_display = np.array(255*gauss_template/max_g, dtype=np.uint8)
    
        # filter the distance output to remove discontinuities and approximate a flow field
        d_im_filt = scmorph.grey_closing(d_im, size=(7, 7))
        blur = cv2.GaussianBlur(d_im_filt, (71, 71), 0)
        blur = np.array(blur, dtype=np.uint8)
    
        # Cross correlate a gaussian peaked function of size < image:
        template_match = cv2.matchTemplate(blur, gauss_display, cv2.TM_CCORR_NORMED)
        template_match = cv2.normalize(template_match, 0, 1, cv2.NORM_MINMAX)
    
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(template_match)
        
        # identify location of template
        self.cX = max_loc[0] + kernlen // 2
        self.cY = max_loc[1] + kernlen // 2
    
        # low-pass filter, estimate average distance around centroid of target region
        vis_cent = blur[(self.cX-8):(self.cX+8), (self.cY-8):(self.cY+8)]
        vis_cent = vis_cent.astype('float64')

        # set low-value (close) regions to NaN, ignore
        vis_cent[vis_cent < 5 ] = np.nan
        self.est_dist = np.nanmean(vis_cent)
    
        self.yaw_error = self.cX - imwidth/2

        return True

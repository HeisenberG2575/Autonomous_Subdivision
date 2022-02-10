import numpy as np
import cv2
import os

#This can be used to test the arrow detection without ROS

def preprocess(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _,img_thres =  cv2.threshold(img_gray, 70, 255, cv2.THRESH_TOZERO)
    # img_blur = cv2.GaussianBlur(img_thres, (5, 5), 1)
    img_blur = cv2.bilateralFilter(img_thres,5,75,75)
    img_canny = cv2.Canny(img_blur, 50, 50)
    kernel = np.ones((3, 3))
    img_dilate = cv2.dilate(img_canny, kernel, iterations=1)
    img_erode = cv2.erode(img_dilate, kernel, iterations=1)
    return img_erode

def find_tip(points, convex_hull):
    length = len(points)
    indices = np.setdiff1d(range(length), convex_hull)
    #print(indices, "convex_hull:",convex_hull,"points:", points)
    for i in range(2):
        j = indices[i] + 2
        #if j > length - 1:
        #    j = length - j
        if np.all(points[j%length] == points[indices[i - 1] - 2]):
            return tuple(points[j%length]), j%length
    return None, None
def find_tail_rect(points, convex_hull):
    length = len(points)
    indices = np.setdiff1d(range(length), convex_hull)
    direction = None
    for i in range(2):
        j = (indices[i] + 2)%length
        # if j > length - 1:
        #     j = length - j
        if np.all(points[j] == points[indices[i - 1] - 2]):
            sides = []#length of sides of the tail rectangle
            prev_pt = points[(indices[i-1]+1)%length]
            for pt in (points[indices[i]-1], points[indices[i]], points[indices[i-1]], points[(indices[i-1]+1)%length]):
                sides.append(np.linalg.norm(pt - prev_pt))
                prev_pt = pt
            # print(sides)
            print(abs(sides[0]-sides[2])/float(sides[2]))
            print(abs(sides[1]-sides[3])/float(sides[1]))
            # print( "diff: "+ str( abs(abs(points[(indices[i-1]+1)%length]- points[indices[i-1]]) - abs(points[indices[i]]- points[indices[i]-1])) ))#/abs(points[(indices[i-1]+1)%length]- points[indices[i-1]])
            # print( "diff: "+ str( abs(abs(points[(indices[i-1]+1)%length]- points[indices[i-1]]) - abs(points[indices[i]]- points[indices[i]-1]))/abs((points[(indices[i-1]+1)%length]- points[indices[i]]).astype(np.float32)) ))#

            if abs(sides[0]-sides[2])/float(max(sides[2], sides[0])) < 0.5 and abs(sides[1]-sides[3])/float(sides[1]) < 0.15 :
            #if np.all(abs(abs(points[(indices[i-1]+1)%length]- points[indices[i-1]]) - abs(points[indices[i]]- points[indices[i]-1])) < 5):#Check if tails is nearly a rectangle#TODO change 5 to something relative to area
                if points[indices[i]-1][0] < points[indices[i]][0]:
                    print("Right")
                    direction = 1#TODO : Add respective rect pts in order
                else:
                    print("Left")
                    direction = 0
                if points[indices[i-1]][1] < points[indices[i]][1]:
                    # print("here")
                    return np.array((points[indices[i]-1], points[indices[i]], points[indices[i-1]], points[(indices[i-1]+1)%length])), direction
                return np.array((points[(indices[i-1]+1)%length], points[indices[i-1]], points[indices[i]], points[indices[i]-1])), direction
    return None, None

def correct_corners(points, corners):
    new_points = []
    for n, pt in enumerate(points):
        err = 5 if not n in [3,4] else 7
        new_pt = corners[np.argmin([np.linalg.norm(corner- pt) for corner in corners])]
        # print(np.linalg.norm(new_pt - pt))
        new_pt = new_pt if np.linalg.norm(new_pt - pt) < err else pt
        new_points.append(new_pt)
    return np.array(new_points)

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 3)
    img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 3)
    img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 3)
    return img

def get_arrow_arr(img, debug = True):
    if debug:
        cv2.imshow("Image", img)
        cv2.waitKey(0)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _,img_thres =  cv2.threshold(img_gray, 120, 255, cv2.THRESH_OTSU)
    img_blur = cv2.GaussianBlur(img_thres, (3, 3), 1)
    img = cv2.bilateralFilter(img_thres,5,75,75)
    contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    if debug:
        cv2.imshow("Image", img)
        cv2.waitKey(0)
    # tmp = cv2.Sobel(img,cv2.CV_64F,1,0,ksize=3)
    # tmp = np.uint8(np.abs(tmp))
    # cv2.imshow("sobel", np.absolute(tmp))
    # cv2.waitKey(0)
    for cnt in contours:
        if cv2.contourArea(cnt) < 200:
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
        hull = cv2.convexHull(approx, returnPoints=False)
        sides = len(hull)

        if (sides == 5 or sides == 4) and sides + 2 == len(approx):
            if debug:
                img_tmp = img.copy()
                # cv2.drawContours(img_tmp, [cnt], -1, (0, 25, 0), 1)
                cv2.drawContours(img_tmp, [approx], -1, (100), 1)
                cv2.imshow("contour", img_tmp)
                cv2.waitKey(0)
            arrow_tip, tip_idx = find_tip(approx[:,0,:], hull.squeeze())
            if arrow_tip is None:
                continue
            # rect, dirct = find_tail_rect(approx[:,0,:], hull.squeeze())
            dst = cv2.cornerHarris(img_blur,5,5,0.05)
            ret, dst = cv2.threshold(dst,0.05*dst.max(),255,0)
            dst = np.uint8(dst)
            ret, labels, stats, centroids = cv2.connectedComponentsWithStats(dst)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
            corners = cv2.cornerSubPix(img_blur,np.float32(centroids),(3,3),(-1,-1),criteria)
            # corners = centroids
            # print(corners)
            corners = np.uint8(corners)
            if debug:
                img_tmp = img.copy()
                for corner in corners:
                    cv2.circle(img_tmp, tuple(corner), 3, (125), cv2.FILLED)
                cv2.imshow("corners", img_tmp)
                cv2.waitKey(0)
            points = np.roll(approx[:,0,:], -tip_idx)
            points = correct_corners(points, corners)
            # points[3] = corners[np.argmin([np.linalg.norm(corner- points[3]) for corner in corners])]
            # points[4] = corners[np.argmin([np.linalg.norm(corner- points[4]) for corner in corners])]

            points = np.concatenate([points, [(points[2]+points[3])/2], [(points[-2]+points[-3])/2]])
            # print(points)
            if debug:
                img_tmp = img.copy()    
                for n,i in enumerate(points):
                    cv2.circle(img_tmp, tuple(i), 3, (125), cv2.FILLED)
                cv2.imshow(str(n)+"th point", img_tmp)
                cv2.waitKey(0)

            return points

def cameraPoseFromHomography(H):
    H1 = H[:, 0]
    H2 = H[:, 1]
    H3 = np.cross(H1, H2)

    norm1 = np.linalg.norm(H1)
    norm2 = np.linalg.norm(H2)
    tnorm = (norm1 + norm2) / 2.0;

    T = H[:, 2] / tnorm
    return np.mat([H1, H2, H3, T])

# def find_pose_from_homography(H, K):
#     '''
#     function for pose prediction of the camera from the homography matrix, given the intrinsics 
    
#     :param H(np.array): size(3x3) homography matrix
#     :param K(np.array): size(3x3) intrinsics of camera
#     :Return t: size (3 x 1) vector of the translation of the transformation
#     :Return R: size (3 x 3) matrix of the rotation of the transformation (orthogonal matrix)
#     '''
    
#     #to disambiguate two rotation marices corresponding to the translation matrices (t and -t), 
#     #multiply H by the sign of the z-comp on the t-matrix to enforce the contraint that z-compoment of point
#     #in-front must be positive and thus obtain a unique rotational matrix
#     H=H*np.sign(H[2,2])

#     h1,h2,h3 = H[:,0].reshape(-1,1), H[:,1].reshape(-1,1) , H[:,2].reshape(-1,1)
    
#     R_ = np.hstack((h1,h2,np.cross(h1,h2,axis=0))).reshape(3,3)
    
#     U, S, V = np.linalg.svd(R_)
    
#     R = U@np.array([[1,0,0],
#                    [0,1,0],
#                     [0,0,np.linalg.det(U@V.T)]])@V.T
    
#     t = (h3/np.linalg.norm(h1)).reshape(-1,1)
    
#     return R,t

def arrow_detect(img):
            #Arrow detection
            #img = self.frame.copy()
            orig_img = img.copy()
            found = False
            theta = None
            orient = None
            direction = None
            bounding_box = None
            contours, _ = cv2.findContours(preprocess(img), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[-2:]
            #cv2.imshow("Image", preprocess(img))
            #cv2.waitKey(0)
            template = cv2.imread("frame4.jpg")
            for cnt in contours:
                  if cv2.contourArea(cnt) < 300:
                        continue
                  peri = cv2.arcLength(cnt, True)
                  approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
                  hull = cv2.convexHull(approx, returnPoints=False)
                  sides = len(hull)

                  if (sides == 5 or sides == 4) and sides + 2 == len(approx):
                        arrow_tip, _ = find_tip(approx[:,0,:], hull.squeeze())
                        rect, dirct = find_tail_rect(approx[:,0,:], hull.squeeze())
                        if arrow_tip and rect is not None:
                              #cv2.polylines(img, [rect],  True, (0, 0, 255), 2)
                              arrow_tail = tuple(np.average([rect[0], rect[3]], axis = 0).astype(int))
                              if arrow_tail[0]-arrow_tip[0]==0:#to avoid division by 0 in next step
                                  continue
                              print("tip-tail tan angle: ",abs(float(arrow_tail[1]-arrow_tip[1])/(arrow_tail[0]-arrow_tip[0])) )
                              #Check that tan of angle of the arrow in the image from horizontal is less than 0.2(we are expecting nearly horizontal arrows)(atan(0.2) = 11.31)
                              if abs(float(arrow_tail[1]-arrow_tip[1])/(arrow_tail[0]-arrow_tip[0])) > 0.2:
                                  continue#Discard it, not a horizontal arrow
                              #cv2.circle(img, arrow_tail, 3, (0, 0, 255), cv2.FILLED)
                              #cv2.circle(img, tuple(np.average([arrow_tail, arrow_tip], axis=0).astype(int)), 3, (0, 0, 255), cv2.FILLED)#arrow centre
                              theta = -(np.average([arrow_tail[0], arrow_tip[0]])/(np.shape(img)[0]) - 0.5)*45*2#linear estimate, assuming camera horizontal range from -45 to 45
                              direction = dirct#TODO multiple arrow case
                              found = True
                              bounding_box = cv2.boundingRect(cnt)
                              cv2.drawContours(img, [cnt], -1, (0, 255, 0), 2)
                              cv2.drawContours(img, [approx], -1, (0, 150, 155), 2)
                              cv2.circle(img, arrow_tip, 3, (0, 0, 255), cv2.FILLED)
                              print("arrow_x_img: "+str(np.average(rect, axis=0)[0] ))
            
            if direction is not None: #TODO: Improve upon this naive orientation
                print(bounding_box)#new_img may go out bounds
                new_img = orig_img[
                 int(bounding_box[1])-10: int(bounding_box[1]+bounding_box[3]+10), 
                 int(bounding_box[0])-10:int(bounding_box[0]+bounding_box[2]+10)]
                train_pts = get_arrow_arr(new_img)
                # print(train_pts)
                new_train_pts = []
                for i, pt in enumerate(train_pts):
                    new_pt = [pt[0] + int(bounding_box[0])-10, pt[1] + int(bounding_box[1])-10]
                    new_train_pts.append(new_pt)
                train_pts = np.array(new_train_pts)
                # img_tmp = orig_img.copy()    
                # for n,i in enumerate(train_pts):
                #     cv2.circle(img_tmp, tuple(i), 3, (125), cv2.FILLED)
                # cv2.imshow(str(n)+"th point", img_tmp)
                # cv2.waitKey(0)
                new_img = orig_img.copy()
                query_pts = get_arrow_arr(template[223: 305, 320: 480], False)
                new_query_pts = []
                for i, pt in enumerate(query_pts):
                    new_pt = [pt[0] + 320, pt[1] + 223]
                    new_query_pts.append(new_pt)
                query_pts = np.array(new_query_pts)
                # img_tmp = template.copy()    
                # for n,i in enumerate(query_pts):
                #     cv2.circle(img_tmp, tuple(i), 3, (125), cv2.FILLED)
                # cv2.imshow(str(n)+"th point", img_tmp)
                # cv2.waitKey(0)
                if train_pts is None:
                    print("not found in close up")
                    return False, None, None, None, img
                matrix, mask = cv2.findHomography(query_pts, train_pts, 0, 5.0)
                # print(matrix)
                mat_inv = np.linalg.inv(matrix) 
                # matches_mask = mask.ravel().tolist()
                warped = np.array([])
                img_tmp = orig_img.copy()
                print(tuple(img_tmp.shape[:2]))
                warped = cv2.warpPerspective(img_tmp, mat_inv, tuple(img_tmp.shape[:2]))
                cv2.imshow("warped", warped)
                cv2.waitKey(0)
                h,w,d = template[223: 305, 320: 480].shape
                pts = np.float32([ [10,10],[10,h-10],[w-10,h-10],[w-10,10] ]).reshape(-1,1,2) + [[320, 223]]
                # print(pts)
                dst = cv2.perspectiveTransform(pts, matrix)
                homography = cv2.polylines(new_img, [np.int32(dst)], True, (255, 0, 0), 3)
                cam_mat = np.array([[476.701438, 0, 400], 
                                    [0, 476.701438, 400],
                                    [0, 0, 1]])
                num, Rs, Ts, Ns  = cv2.decomposeHomographyMat(mat_inv, cam_mat)
                print("Ns: ", Ns)
                # print("Rs: ",Rs,'\n Ts: ', Ts,"\n Ns: ", Ns)
                        # Find the rotation and translation vectors.
                axis = np.float32([[1,0,0], [0,1,0], [0,0,-1]]).reshape(-1,3)/10
                # print(query_pts)
                # ps = cameraPoseFromHomography(matrix)
                # ps = cam_mat.dot(ps)
                # print(ps)
                # imgt = new_img.copy()
                # imgpts, jac = cv2.projectPoints(axis, ps[0], ps[1], cam_mat.astype(np.float32), 0)
                # imgt = draw(imgt,train_pts[2:],imgpts)
                # cv2.imshow('axes img',imgt)
                # k = cv2.waitKey(0) & 0xFF
                for i in range(int(np.ceil(len(Ns)/2.0))):
                    pm = cam_mat.dot(np.c_[Rs[i*2], Ts[i*2]])#dot product doesn't affect euler angle though
                    _, _, tvec, _, _, _, ea =  cv2.decomposeProjectionMatrix(pm)
                    # print(train_pts[0])
                    # tmp_arr = pm.dot(np.r_[train_pts[0], -0.5e3, 1.0])
                    # print(tmp_arr)
                    # print(tmp_arr[0]/tmp_arr[2], tmp_arr[1]/tmp_arr[2])
                    print("euler angles: ", ea)
                    # print( "translation: ", -np.matrix(Rs[i*2]).T*np.matrix(Ts[i*2]) )
                    # print("euler angles: ", cv2.decomposeProjectionMatrix(cam_mat.dot(pm))[-1])
                    # orient = cv2.decomposeProjectionMatrix(pm)[-1][1]#-1 gets euler angles(deg), 1 gets the pitch
                    # if orient >= 0:#we need positive orient angle
                    #     break
                    imgt = new_img.copy()
                    # ret,rvecs, tvecs = cv2.solvePnP(np.c_[query_pts, np.zeros(9)].astype(np.float32), train_pts.astype(np.float32), cam_mat, 0)
                    # print(rvecs, tvecs)# project 3D points to image plane
                    imgpts, jac = cv2.projectPoints(axis, Rs[i*2], Ts[i*2], cam_mat.astype(np.float32), 0)
                    imgt = draw(imgt,train_pts[2:],imgpts)
                    cv2.imshow('axes img',imgt)
                    k = cv2.waitKey(0) & 0xFF
                orient = 0
                assert orient >= 0
                cv2.imshow("Homography", homography)
                cv2.waitKey(0)
                if direction == 1:  #Right
                    orient = -90 - orient
                elif direction == 0:#Left
                    orient = 90 - orient
                else:
                    print("error: direction not found and not None, "+str(direction))
                    found = False
            return found, theta, orient, direction, img


if __name__ == '__main__':
    print("Starting arrow detection script")
    ''''''
    for i in range(11,1,-1):
        if i == 4:
            continue
        sample_img=cv2.imread('frame' + str(i) + '.jpg')
        found, theta, orient, direction, output = arrow_detect(sample_img)
        if direction == 1:
            direction = 'Right'
        else:
            direction = 'Left'
        output = cv2.putText(output, direction + " \n"+ str(orient), (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.imshow("Result", output)
        cv2.waitKey(0)
    
    #Uncomment what you need

    capture = cv2.VideoCapture(0)
    while True:
        ret_val, frame = capture.read()
        if ret_val == False:
            print("image/video error")
            time.sleep(1)
            continue
        found, theta, orient, direction, output = arrow_detect(frame)
        if found == False:
            continue
        if direction == 1:
            direction = 'Right'
        else:
            direction = 'Left'
        # font
        font = cv2.FONT_HERSHEY_SIMPLEX

        # org
        org = (50, 50)

        # fontScale
        fontScale = 1

        # Blue color in BGR
        color = (255, 0, 0)

        # Line thickness of 2 px
        thickness = 2

        output = cv2.putText(output, direction + " \n"+ str(orient), org, font,
                            fontScale, color, thickness, cv2.LINE_AA)

        cv2.imshow("Arrow", output)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        pass


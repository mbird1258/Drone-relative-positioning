import numpy as np
from ServerReceiver import get
import cv2
import copy
from matplotlib import pyplot as plt
import utils

## Hyperparameters
CameraOffset = 10 # cm

## Classes
class camera:
    def __init__(self, ZYRatio):
        self.ZYRatio = ZYRatio

class MainDrone:
    def __init__(self, cameras, FollowCameras):
        self.cameras = cameras
        self.FollowCameras = FollowCameras
    
    def capture(self, frames, orientations):
        self.imgs = frames
        self.imgs = [cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) for img in self.imgs]
        # self.imgs = [[cv2.GaussianBlur(img,(3,3),0) for img in DroneImgs] for DroneImgs in self.imgs]
        
        self.KeyIndexes, self.thetas, self.descriptors = [], [], []
        for img in self.imgs:
            out = utils.GetDescriptors(img, debug=False)
            self.KeyIndexes.append(out[0])
            self.thetas.append(out[1])
            self.descriptors.append(out[2])
        
        self.AccelReadings = orientations # [[Pitch, Roll], ...]
    
    def landmark(self, threshold=1):
        DescriptorArr1 = self.descriptors[0]
        IndexArr1 = self.KeyIndexes[0]
        self.indexes1 = np.array([range(len(IndexArr1)), IndexArr1[:, 0], IndexArr1[:, 1]], dtype = np.float_).T
        
        DescriptorArrs = self.descriptors[1:]
        IndexArrs = self.KeyIndexes[1:]
        self.indexes = []
        
        for ind, (DescriptorArr2, IndexArr2) in enumerate(zip(DescriptorArrs, IndexArrs)):
            MSEArr = np.sum((DescriptorArr1[:, np.newaxis, :] - DescriptorArr2[np.newaxis, :, :])**2, axis = 2)
            
            thetas = []
            out = []
            
            TopTwo = np.argsort(MSEArr, axis = 1)[:, [0,1]]
            order = np.array([range(len(MSEArr)), TopTwo[:, 0], MSEArr[np.arange(MSEArr.shape[0]), TopTwo[:, 0]], MSEArr[np.arange(MSEArr.shape[0]), TopTwo[:, 1]]])
            order = order.T[np.argsort(order[2])]
            while True:
                ind1, ind2, cost1, cost2 = order[0]
                order = order[~(order[:, 1] == ind2)]
                if cost2/cost1 > threshold:
                    thetas.append(self.thetas[0][int(ind1)] - self.thetas[ind+1][int(ind2)])
                    out.append([ind1, IndexArr2[int(ind2), 0], IndexArr2[int(ind2), 1]])
                
                if len(order) == 0: break
            
            out = np.array(out, dtype = np.float_)
            
            # compensate for yaw offset(around our z axis)
            if ind == 0:
                def InlierFunction(input, points, threshold):
                    model = 0
                    inliers = input[np.abs(input[:, -1]-model) <= threshold]
                    
                    return inliers, model
                
                inliers, _ = utils.ransac(np.hstack((out, np.array([thetas]).T)), InlierFunction, threshold=np.pi/16, iterations=1000, samples=1, axes=[0])
                out = inliers[:, :-1]
            else:
                def InlierFunction(input, points, threshold):
                    model = np.average(points[:, -1])
                    inliers = input[np.abs(input[:, -1]-model) <= threshold]
                    
                    return inliers, model

                inliers, theta = utils.ransac(np.hstack((out, np.array([thetas]).T)), InlierFunction, threshold=np.pi/16, iterations=1000, samples=min(5, len(thetas)), axes=[0])
                out = inliers[:, :-1]
                out[:, [1,2]] -= (np.array(self.imgs[ind+1].shape)-1)/2
                out[:, [1,2]] = (utils.rotateMatrix(-theta) @ out[:, [1,2]].T).T
                out[:, [1,2]] += (np.array(self.imgs[ind+1].shape)-1)/2
            self.indexes.append(out)
    
    def triangulate(self):
        coordinates = [] # [[x, y, z], ...]
        for ind, (IndexArr2, camera2) in enumerate(zip(self.indexes, [self.cameras[1]]+self.FollowCameras)):
            IndexArr2 = copy.deepcopy(IndexArr2)
            IndexArr2[:, [1,2]] -= (np.array(self.imgs[ind+1].shape)-1)/2
            
            IndexArr1 = copy.deepcopy(self.indexes1)
            IndexArr1[:, [1,2]] -= (np.array(self.imgs[0].shape)-1)/2
            
            depth = self.imgs[0].shape[0]*camera2.ZYRatio
            
            if ind == 0:
                ObsIndexes = IndexArr2[:, 0]
                
                ObsX, ObsY, ObsZ = [], [], []
                for ind in range(len(IndexArr2)):
                    _, y1, x1 = IndexArr1[int(IndexArr2[ind, 0])]
                    _, y2, x2 = IndexArr2[ind]
                    x, y, z = utils.intersection([0, 0, 0], [x1, y1, depth], [CameraOffset, 0, 0], [CameraOffset+x2, y2, depth])
                    ObsX.append(x)
                    ObsY.append(y)
                    ObsZ.append(z)
                ObsX, ObsY, ObsZ = np.array(ObsX), np.array(ObsY), np.array(ObsZ)
                
                ObsX, ObsY, ObsZ = utils.rotateMatrix3D(-self.AccelReadings[0][0], 'x') @ utils.rotateMatrix3D(self.AccelReadings[0][1], 'y') @ np.array([ObsX, ObsY, ObsZ])
                ObsX *= -1
                
                # https://www.geogebra.org/calculator/tvsyvsqd
                # IndexArr2 = IndexArr2[IndexArr1[IndexArr2[:, 0].astype(np.int_)][:, 2] != IndexArr2[:, 2]]
                # ObsZ = depth*CameraOffset/(IndexArr1[IndexArr2[:, 0].astype(np.int_)][:, 2] - IndexArr2[:, 2])
                # ObsX = ObsZ*IndexArr1[IndexArr2[:, 0].astype(np.int_)][:, 2]/depth
                # ObsY = ObsZ*IndexArr1[IndexArr2[:, 0].astype(np.int_)][:, 1]/depth
                
                continue
            
            
            CamX, CamY, CamZ = [], [], []
            
            IndexArr2 = IndexArr2[np.isin(IndexArr2[:, 0], ObsIndexes)]
            
            for ind_, (ind1, Y2_, X2_) in enumerate(IndexArr2):
                for ind2, Y4_, X4_ in IndexArr2[ind_+1:]:
                    X1, Y1, Z1 = ObsX[ObsIndexes == ind1][0], ObsY[ObsIndexes == ind1][0], ObsZ[ObsIndexes == ind1][0]
                    X3, Y3, Z3 = ObsX[ObsIndexes == ind2][0], ObsY[ObsIndexes == ind2][0], ObsZ[ObsIndexes == ind2][0]
                    
                    X2, Y2, Z2 = utils.rotateMatrix3D(-self.AccelReadings[ind][0], 'x') @ utils.rotateMatrix3D(self.AccelReadings[ind][1], 'y') @ np.array([X2_, Y2_, depth]) # [X2_, Y2_, depth] # 
                    X4, Y4, Z4 = utils.rotateMatrix3D(-self.AccelReadings[ind][0], 'x') @ utils.rotateMatrix3D(self.AccelReadings[ind][1], 'y') @ np.array([X4_, Y4_, depth]) # [X4_, Y4_, depth] # 
                    
                    Y2 *= -1
                    Z2 *= -1
                    Y4 *= -1
                    Z4 *= -1
                    
                    x, y, z = utils.intersection([X1, Y1, Z1], [X1+X2, Y1+Y2, Z1+Z2], [X3, Y3, Z3], [X3+X4, Y3+Y4, Z3+Z4])
                    CamX.append(x)
                    CamY.append(y)
                    CamZ.append(z)
                    
                    # CamX.append((depth*X1/IndexArr2[ind1ind, 2]+Z2-depth*X2/IndexArr2[ind2ind, 2]-Z1)/(depth/IndexArr2[ind1ind, 2]-depth/IndexArr2[ind2ind, 2]))
                    # CamZ.append(depth/IndexArr2[ind1ind, 2]*(CamX[-1]-X1)+Z1)
                    # CamY.append(((Y1 - IndexArr2[ind1ind, 1]/depth*(Z1-CamZ[-1]))+(Y2 - IndexArr2[ind2ind, 1]/depth*(Z2-CamZ[-1])))/2)
            
            CamX, CamY, CamZ = np.array(CamX), np.array(CamY), np.array(CamZ)
            CamX *= -1
            
            def InlierFunction(input, points, threshold):
                model = np.array([np.average(points, axis=0)])
                inliers = input[np.mean(np.abs(input-model), axis=1) <= threshold]
                
                return inliers, model
            
            inliers, CamXYZ = utils.ransac(np.array([CamX, CamY, CamZ]).T, InlierFunction, threshold=1, iterations=1000, samples=min(5, len(CamX)), axes=[0])
            
            coordinates.append(CamXYZ)
        
        return np.array(coordinates)
## Main
a = MainDrone([camera(1.11607142857), camera(1.11607142857)], [camera(1.11607142857)])

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(0, 0, 0, s=10, c="red")
ax.scatter(CameraOffset, 0, 0, s=8, c="gray")
last = ax.scatter(0, 0, 0, s=10, c="blue", alpha=0)

ax.set_xlim(-50, 50)
ax.set_ylim(-30, 30)
ax.set_zlim(-1000, 1000)

while True:
    # try:
    data = get()
    
    frames = []
    orientations = []

    for img, orientation, index in data:
        while len(frames)-1 < int(index[0]):
            frames.append([])
            orientations.append([])
        
        while len(frames[int(index[0])])-1 < int(index[1]):
            frames[int(index[0])].append(None)
        
        frames[int(index[0])][int(index[1])] = img
        
        if int(index[1]) == 0:
            orientations[int(index[0])] = orientation

    a.capture([i for j in frames for i in j], orientations)
    a.landmark()
    utils.drawMatches(a.imgs[0], a.indexes1, a.imgs[1], a.indexes[0], '0')
    utils.drawMatches(a.imgs[0], a.indexes1, a.imgs[2], a.indexes[1], '1')
    x, y, z = a.triangulate().T
    print(x, y, z)
    
    last.remove()    
    last = ax.scatter(x, y, z, s=10, c="blue")
    plt.pause(0.01)
    # except Exception as e:
    #     print(f"\n{e}\n")
    #     continue
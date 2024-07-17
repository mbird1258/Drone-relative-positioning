import numpy as np
from scipy import signal, ndimage, stats
import cv2
import copy


## Hyperparameters
UpdateRate = 5
CameraOffset = 10 # cm

BlockSize = 30 # scale up with dimensions
dimensions = np.array([300, 300]) # keep both values the same
CorrelationMin = 0.95
MaxLandmarks = 20

ExtraHighResCorrelationLeeway = 5
MaxCorrelations = 50
MinDistance = 0 # 30 # Use rectangle not Circle because easier; scale up with dimensions
AcceptedProximity = 0 # 2 # scale up with dimensions

ProximityMiddle = 0.7
ProximitySteepness = 15
ConfidenceDegree = 2
RefPointWindow = 20 # scale up with dimensions(to a lesser extent)

VectorsTaken = 10000
MagnitudeLeeway = 0.01
ThetaLeeway = 0.2 * np.pi/180
MaxVectors = 1000


## Classes
class filters:
    sobel = np.array([[[-1, 0, 1],
                       [-2, 0, 2],
                       [-1, 0, 1]], 

                      [[1, 2, 1], 
                       [0, 0, 0], 
                       [-1, -2, -1]]])#/(8*2**0.5)


class ScoreFunctions:
    def CentralityFunction(arr, UpperLim):
        # https://www.desmos.com/calculator/udvdzwpoqw
        global ProximityMiddle, ProximitySteepness

        return 1/(1+np.exp(ProximitySteepness*(arr/UpperLim-ProximityMiddle)))


    def ConfidenceFunction(arr, UpperLim):
        # https://www.desmos.com/calculator/9u7a4yqnia
        global ConfidenceDegree
        
        return (arr/UpperLim)**ConfidenceDegree


class camera:
    def __init__(self, XYRatio, ZYRatio):
        self.XYRatio = XYRatio
        self.ZYRatio = ZYRatio
    
    
    def get(self):
        pass


class NetworkDevice:
    pass


class Flagship(NetworkDevice):
    def __init__(self, communicator, cameras):
        super().__init__()
        
        self.communicator = communicator
        self.cameras = [cameras]
        
        self.FollowerCommunicators = []
        self.FollowCameras = [[camera(4/3, 1.11607142857)]] # [[cam 1, cam 2, ...], [cam 1], ...]

        self.CamXEMAs = [[None, None]] + [[{} for i in j] for j in self.FollowCameras]
        self.CamOffsetXEMAs = [[None, None]] + [[{} for i in j] for j in self.FollowCameras]
        self.CamYEMAs = [[None, None]] + [[{} for i in j] for j in self.FollowCameras]
        self.CamOffsetYEMAs = [[None, None]] + [[{} for i in j] for j in self.FollowCameras]
        self.CamZEMAs = [[None, None]] + [[{} for i in j] for j in self.FollowCameras]
        self.CamOffsetZEMAs = [[None, None]] + [[{} for i in j] for j in self.FollowCameras]

        # EMAHistoryLength = 10
        # self.CamXEMAHistory = [np.array([]) for _ in range(EMAHistoryLength)]
        # self.CamYEMAHistory = [np.array([]) for _ in range(EMAHistoryLength)]
        # self.CamZEMAHistory = [np.array([]) for _ in range(EMAHistoryLength)]
        # self.EMAHistoryWeight = [0.8 ** i for i in range(EMAHistoryLength)]
    
    
    def capture(self, frames):
        # self.imgs = [[cv2.imread(rf"/Users/matthewbird/Documents/Python Code/Getting Landmarks From Image/Images/{i}.jpg") for i in (1,2)], [frame]]
        self.imgs = frames
        self.imgs = [[cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) for img in DroneImgs] for DroneImgs in self.imgs]
        self.imgs = [[cv2.resize(img, (500,)*2) for img in DroneImgs] for DroneImgs in self.imgs]
        self.imgs = [[cv2.GaussianBlur(img,(5,5),0) for img in DroneImgs] for DroneImgs in self.imgs]
    
    
    # @profile
    def landmark(self):
        out = [[GetDescriptors(*fast(img), False) for img in DroneImgs] for DroneImgs in self.imgs]

        DescriptorArrs = [[i[0] for i in j] for j in out]
        IndexArrs = [[i[2] for i in j] for j in out]

        DescriptorArr1, IndexArr1 = DescriptorArrs[0][0], IndexArrs[0][0]

        self.AllIndexes = []

        self.ReferenceIndexes = np.array([range(len(IndexArr1)), IndexArr1[:, 0], IndexArr1[:, 1]], dtype = np.float_).T

        for DescriptorArr, IndexArr in zip([DescriptorArrs[0][1:]]+DescriptorArrs[1:], [IndexArrs[0][1:]]+IndexArrs[1:]):
            self.AllIndexes.append([])

            for DescriptorArr2, IndexArr2 in zip(DescriptorArr, IndexArr):
                MSEArr = np.sum((DescriptorArr1[:, :, np.newaxis] - DescriptorArr2[:, np.newaxis, :])**2, axis = 0)

                self.AllIndexes[-1].append([])

                thetas1 = []
                thetas2 = []
                
                order = np.array([range(len(MSEArr)), np.argsort(MSEArr, axis = 1)[:, 0]])
                order = order.T[np.argsort(MSEArr[*order])]
                for _ in range(int(min(MSEArr.shape)/5)):
                    ind1, ind2 = order[0]
                    order = order[~(order[:, 1] == ind2)]
                    self.AllIndexes[-1][-1].append([ind1, IndexArr2[ind2, 0], IndexArr2[ind2, 1]])

                    if len(order) == 0: break

                self.AllIndexes[-1][-1] = np.array(self.AllIndexes[-1][-1], dtype = np.float_)
    
    
    # @profile
    def triangulate(self):
        coordinates = [] # [[[x, y, z], cam 2 rel coords, ...], drone 2, ...] relative to flagship drone cam 1
        
        for DroneInd, (DroneIndexArr, DroneCameraArr) in enumerate(zip(self.AllIndexes, self.cameras+self.FollowCameras)):
            if DroneInd != 0: coordinates.append([])
            
            for CamInd, (IndexArr, camera) in enumerate(zip(DroneIndexArr, DroneCameraArr)):
                IndexArr = IndexArr.astype(np.float_)
                
                ScaleFactor = self.imgs[DroneInd][CamInd].shape[0]/dimensions[0]
                
                center = (self.imgs[0][0].shape[0]+1)/2
                IndexArr[:, [1,2]] -= center
                IndexArr[:, 2] *= camera.XYRatio
                
                CamReferenceIndexes = copy.deepcopy(self.ReferenceIndexes)
                CamReferenceIndexes[:, [1,2]] -= center
                CamReferenceIndexes[:, 2] *= camera.XYRatio
                
                depth = self.imgs[DroneInd][CamInd].shape[0]*camera.ZYRatio
                
                if DroneInd == 0:
                    # https://www.geogebra.org/calculator/tvsyvsqd
                    IndexArr = IndexArr[CamReferenceIndexes[IndexArr[:, 0].astype(np.int_)][:, 2] != IndexArr[:, 2]]
                    ObsIndexes = IndexArr[:, 0]
                    ObsZ = depth*CameraOffset/(CamReferenceIndexes[IndexArr[:, 0].astype(np.int_)][:, 2] - IndexArr[:, 2])
                    ObsX = ObsZ*CamReferenceIndexes[IndexArr[:, 0].astype(np.int_)][:, 2]/depth
                    ObsY = ObsZ*CamReferenceIndexes[IndexArr[:, 0].astype(np.int_)][:, 1]/depth
                
                else:
                    # https://www.geogebra.org/3d/grs7udc6
                    CamX = []
                    CamZ = []
                    CamY = []
                    
                    IndexArr = IndexArr[np.isin(IndexArr[:, 0], ObsIndexes)]
                    
                    for ind1ind, ind1 in enumerate(IndexArr[:, 0]):
                        for ind2ind, ind2 in enumerate(IndexArr[ind1ind+1:, 0]):
                            ind2ind += ind1ind + 1
                            
                            if IndexArr[ind1ind, 2] == IndexArr[ind2ind, 2]:
                                continue # fix with logic using x instead of y next time if there is one because I'm a lazy lazy bad boi
                            
                            X1, Y1, Z1 = ObsX[ObsIndexes == ind1][0], ObsY[ObsIndexes == ind1][0], ObsZ[ObsIndexes == ind1][0]
                            X2, Y2, Z2 = ObsX[ObsIndexes == ind2][0], ObsY[ObsIndexes == ind2][0], ObsZ[ObsIndexes == ind2][0]
                            
                            CamX.append((depth*X1/IndexArr[ind1ind, 2]+Z2-depth*X2/IndexArr[ind2ind, 2]-Z1)/(depth/IndexArr[ind1ind, 2]-depth/IndexArr[ind2ind, 2]))
                            CamZ.append(depth/IndexArr[ind1ind, 2]*(CamX[-1]-X1)+Z1)
                            CamY.append(((Y1 - IndexArr[ind1ind, 1]/depth*(Z1-CamZ[-1]))+(Y2 - IndexArr[ind2ind, 1]/depth*(Z2-CamZ[-1])))/2)
                    
                    CamX, CamY, CamZ = np.array(CamX), np.array(CamY), np.array(CamZ)

                    precision = 0.2
                    zPrecision = 2
                    EMAFactor = 0.5

                    # self.CamXEMAHistory = [CamX] + self.CamXEMAHistory[:-1]
                    # self.CamYEMAHistory = [CamY] + self.CamYEMAHistory[:-1]
                    # self.CamZEMAHistory = [CamZ] + self.CamZEMAHistory[:-1]

                    # XMode = stats.mode(np.round(CamX/precision)*precision, keepdims = False)
                    # YMode = stats.mode(np.round(CamY/precision)*precision, keepdims = False)
                    # ZMode = stats.mode(np.round(CamZ/zPrecision)*zPrecision, keepdims = False)
                    # OffsetXMode = stats.mode(np.floor(CamX/precision)*precision, keepdims = False)
                    # OffsetYMode = stats.mode(np.floor(CamY/precision)*precision, keepdims = False)
                    # OffsetZMode = stats.mode(np.floor(CamZ/zPrecision)*zPrecision, keepdims = False)

                    values, counts = np.unique(np.round(CamX/precision)*precision, return_counts=True)
                    for value, count in zip(values, counts):
                        if value not in self.CamXEMAs[DroneInd][CamInd]:
                            self.CamXEMAs[DroneInd][CamInd][value] = count
                            continue
                        self.CamXEMAs[DroneInd][CamInd][value] = self.CamXEMAs[DroneInd][CamInd][value] * EMAFactor + count * (1-EMAFactor)
                    XMode = max(self.CamXEMAs[DroneInd][CamInd], key = self.CamXEMAs[DroneInd][CamInd].get)
                    XModeCount = self.CamXEMAs[DroneInd][CamInd][XMode]

                    values, counts = np.unique(np.floor(CamX/precision)*precision, return_counts=True)
                    for value, count in zip(values, counts):
                        if value not in self.CamOffsetXEMAs[DroneInd][CamInd]:
                            self.CamOffsetXEMAs[DroneInd][CamInd][value] = count
                            continue
                        self.CamOffsetXEMAs[DroneInd][CamInd][value] = self.CamOffsetXEMAs[DroneInd][CamInd][value] * EMAFactor + count * (1-EMAFactor)
                    OffsetXMode = max(self.CamOffsetXEMAs[DroneInd][CamInd], key = self.CamOffsetXEMAs[DroneInd][CamInd].get)
                    OffsetXModeCount = self.CamOffsetXEMAs[DroneInd][CamInd][OffsetXMode]

                    values, counts = np.unique(np.round(CamY/precision)*precision, return_counts=True)
                    for value, count in zip(values, counts):
                        if value not in self.CamYEMAs[DroneInd][CamInd]:
                            self.CamYEMAs[DroneInd][CamInd][value] = count
                            continue
                        self.CamYEMAs[DroneInd][CamInd][value] = self.CamYEMAs[DroneInd][CamInd][value] * EMAFactor + count * (1-EMAFactor)
                    YMode = max(self.CamYEMAs[DroneInd][CamInd], key = self.CamYEMAs[DroneInd][CamInd].get)
                    YModeCount = self.CamYEMAs[DroneInd][CamInd][YMode]

                    values, counts = np.unique(np.floor(CamY/precision)*precision, return_counts=True)
                    for value, count in zip(values, counts):
                        if value not in self.CamOffsetYEMAs[DroneInd][CamInd]:
                            self.CamOffsetYEMAs[DroneInd][CamInd][value] = count
                            continue
                        self.CamOffsetYEMAs[DroneInd][CamInd][value] = self.CamOffsetYEMAs[DroneInd][CamInd][value] * EMAFactor + count * (1-EMAFactor)
                    OffsetYMode = max(self.CamOffsetYEMAs[DroneInd][CamInd], key = self.CamOffsetYEMAs[DroneInd][CamInd].get)
                    OffsetYModeCount = self.CamOffsetYEMAs[DroneInd][CamInd][OffsetYMode]

                    values, counts = np.unique(np.round(CamZ/zPrecision)*zPrecision, return_counts=True)
                    for value, count in zip(values, counts):
                        if value not in self.CamZEMAs[DroneInd][CamInd]:
                            self.CamZEMAs[DroneInd][CamInd][value] = count
                            continue
                        self.CamZEMAs[DroneInd][CamInd][value] = self.CamZEMAs[DroneInd][CamInd][value] * EMAFactor + count * (1-EMAFactor)
                    ZMode = max(self.CamZEMAs[DroneInd][CamInd], key = self.CamZEMAs[DroneInd][CamInd].get)
                    ZModeCount = self.CamZEMAs[DroneInd][CamInd][ZMode]

                    values, counts = np.unique(np.floor(CamZ/zPrecision)*zPrecision, return_counts=True)
                    for value, count in zip(values, counts):
                        if value not in self.CamOffsetZEMAs[DroneInd][CamInd]:
                            self.CamOffsetZEMAs[DroneInd][CamInd][value] = count
                            continue
                        self.CamOffsetZEMAs[DroneInd][CamInd][value] = self.CamOffsetZEMAs[DroneInd][CamInd][value] * EMAFactor + count * (1-EMAFactor)
                    OffsetZMode = max(self.CamOffsetZEMAs[DroneInd][CamInd], key = self.CamOffsetZEMAs[DroneInd][CamInd].get)
                    OffsetZModeCount = self.CamOffsetZEMAs[DroneInd][CamInd][OffsetZMode]

                    CamXYZ = [XMode if XModeCount > OffsetXModeCount else OffsetXMode + precision/2, YMode if YModeCount > OffsetYModeCount else OffsetYMode + precision/2, ZMode if ZModeCount > OffsetZModeCount else OffsetZMode + zPrecision/2]

                    import matplotlib.pyplot as plt
                    plt.ion()
                    plt.clf()
                    plt.bar(0.5 + np.array([int(k) for k in self.CamOffsetYEMAs[DroneInd][CamInd].keys()])[np.logical_and(np.array([int(k) for k in self.CamOffsetYEMAs[DroneInd][CamInd].keys()]) < 10, np.array([int(k) for k in self.CamOffsetYEMAs[DroneInd][CamInd].keys()]) > -10)], np.array([v for v in self.CamOffsetYEMAs[DroneInd][CamInd].values()])[np.logical_and(np.array([int(k) for k in self.CamOffsetYEMAs[DroneInd][CamInd].keys()]) < 10, np.array([int(k) for k in self.CamOffsetYEMAs[DroneInd][CamInd].keys()]) > -10)], 1)
                    plt.bar(np.array([int(k) for k in self.CamYEMAs[DroneInd][CamInd].keys()])[np.logical_and(np.array([int(k) for k in self.CamYEMAs[DroneInd][CamInd].keys()]) < 10, np.array([int(k) for k in self.CamYEMAs[DroneInd][CamInd].keys()]) > -10)], np.array([v for v in self.CamYEMAs[DroneInd][CamInd].values()])[np.logical_and(np.array([int(k) for k in self.CamYEMAs[DroneInd][CamInd].keys()]) < 10, np.array([int(k) for k in self.CamYEMAs[DroneInd][CamInd].keys()]) > -10)], 1)


                    # precision = 15
                    # zPrecision = 40

                    # if XModeCount > OffsetXModeCount:
                    #     mask = [np.all([i >= XMode - precision/2, i <= XMode + precision/2], axis = 0) for i in self.CamXEMAHistory]
                    # else:
                    #     mask = [np.all([i >= OffsetXMode, i <= OffsetXMode + precision], axis = 0) for i in self.CamXEMAHistory]

                    # if YModeCount > OffsetYModeCount:
                    #     mask = [np.all([mask[ind], i >= YMode - precision/2, i <= YMode + precision/2], axis = 0) for ind, i in enumerate(self.CamYEMAHistory)]
                    # else:
                    #     mask = [np.all([mask[ind], i >= OffsetYMode, i <= OffsetYMode + precision], axis = 0) for ind, i in enumerate(self.CamYEMAHistory)]

                    # if ZModeCount > OffsetZModeCount:
                    #     mask = [np.all([mask[ind], i >= ZMode - zPrecision/2, i <= ZMode + zPrecision/2], axis = 0) for ind, i in enumerate(self.CamZEMAHistory)]
                    # else:
                    #     mask = [np.all([mask[ind], i >= OffsetZMode, i <= OffsetZMode + (precision*10)], axis = 0) for ind, i in enumerate(self.CamZEMAHistory)]

                    # NaNAverage = lambda x, w: np.nansum(x*w)/((~np.isnan(x))*w).sum()
                    # CamX = NaNAverage(np.array([np.average(i[mask[ind]]) for ind, i in enumerate(self.CamXEMAHistory)]), self.EMAHistoryWeight)
                    # CamY = NaNAverage(np.array([np.average(i[mask[ind]]) for ind, i in enumerate(self.CamYEMAHistory)]), self.EMAHistoryWeight)
                    # CamZ = NaNAverage(np.array([np.average(i[mask[ind]]) for ind, i in enumerate(self.CamZEMAHistory)]), self.EMAHistoryWeight)

                    # CamXYZ = [CamX, CamY, CamZ]
                    
                    coordinates[-1].append(CamXYZ)
        
        print(coordinates)


## Functions
def canny(img, filter = filters.sobel):
    out = []
    
    for arr in filter:
        out.append(ndimage.convolve(img.astype(np.int16), arr, mode = 'constant', origin = -1))
    
    out = np.array(out)
    
    theta = np.arctan2(out[1], out[0])
    NormalDirection = np.round(theta/np.pi*4)%4
    theta[theta < 0] += 2*np.pi
    
    out = np.hypot(*out)
    
    for direction in range(4):
        match direction:
            case 0:
                a = out - np.pad(out[:, :-1], [[0, 0], [1, 0]])
                b = out - np.pad(out[:, 1:], [[0, 0], [0, 1]])

            case 1:
                a = out - np.pad(out[1:, :-1], [[0, 1], [1, 0]])
                b = out - np.pad(out[:-1, 1:], [[1, 0], [0, 1]])

            case 2:
                a = out - np.pad(out[1:, :], [[0, 1], [0, 0]])
                b = out - np.pad(out[:-1, :], [[1, 0], [0, 0]])

            case 3:
                a = out - np.pad(out[1:, 1:], [[0, 1], [0, 1]])
                b = out - np.pad(out[1:, :-1], [[1, 0], [1, 0]])

        out[np.logical_and(np.min([a, b], axis = 0) < 0, NormalDirection == direction)] = 0
    
    indexes = np.array(np.nonzero(out[1:-1, 1:-1] >= 0.3*np.max(out)))+1

    AlphaMatrix = np.zeros_like(theta)
    theta[out == 0] = 20 # arbitrary big value
    
    for index in indexes.T:
        match NormalDirection[*index]:
            case 0:
                matrix = theta[index[0] + np.array([-1, -1, -1]), index[1] + np.array([-1, 0, 1])][:, np.newaxis] - theta[index[0] + np.array([1, 1, 1]), index[1] + np.array([-1, 0, 1])][np.newaxis, :]
            
            case 1:
                matrix = theta[index[0] + np.array([-1, -1, 0]), index[1] + np.array([-1, 0, -1])][:, np.newaxis] - theta[index[0] + np.array([0, 1, 1]), index[1] + np.array([1, 0, 1])][np.newaxis, :]
            
            case 2:
                matrix = theta[index[0] + np.array([-1, 0, 1]), index[1] + np.array([-1, -1, -1])][:, np.newaxis] - theta[index[0] + np.array([-1, 0, 1]), index[1] + np.array([1, 1, 1])][np.newaxis, :]
            
            case 3:
                matrix = theta[index[0] + np.array([-1, -1, 0]), index[1] + np.array([0, 1, 1])][:, np.newaxis] - theta[index[0] + np.array([0, 1, 1]), index[1] + np.array([-1, -1, 0])][np.newaxis, :]
        
        AlphaMatrix[*index] = np.abs(matrix[*np.unravel_index(np.argmin(np.abs(np.abs(matrix)-np.pi/2)), matrix.shape)])

    theta[theta == 20] = 0
    
    return out, AlphaMatrix, theta


def fast(img, threshold = 10):
    img = cv2.GaussianBlur(img,(5,5),0)
    circles = np.array([img[ :-6, 3:-3],
                        img[ :-6, 4:-2],
                        img[1:-5, 5:-1],
                        img[2:-4, 6:  ],
                        img[3:-3, 6:  ],
                        img[4:-2, 6:  ],
                        img[5:-1, 5:-1],
                        img[6:  , 4:-2],
                        img[6:  , 3:-3],
                        img[6:  , 2:-4],
                        img[5:-1, 1:-5],
                        img[4:-2,  :-6],
                        img[3:-3,  :-6],
                        img[2:-4,  :-6],
                        img[1:-5, 1:-5],
                        img[ :-6, 2:-4]])
 
    centers = img[3:-3, 3:-3] # final output shape 6 smaller in both dimensions

    mask = np.sum(circles[[0,4,8,12]] <= centers.astype(np.int32) - threshold, axis = 0) >= 2
    MaskBrighter = np.zeros_like(centers)
    MaskBrighter[mask] = np.sum(circles[:, mask] <= centers[mask].astype(np.int32) - threshold, axis = 0) >= 9

    mask = np.sum(circles[[0,4,8,12]] >= centers.astype(np.int32) + threshold, axis = 0) >= 2
    MaskDarker = np.zeros_like(centers)
    MaskDarker[mask] = np.sum(circles[:, mask] >= centers[mask].astype(np.int32) + threshold, axis = 0) >= 9

    CornerMask = np.any([MaskBrighter, MaskDarker], axis = 0)
    scores = np.zeros_like(centers)
    scores[CornerMask] = np.sum(np.abs(circles - centers), axis = 0)[CornerMask]
    NMSCornerMask = np.zeros_like(CornerMask)

    for index in np.array(np.nonzero(CornerMask)).T:
        clip = lambda x, dim: max(min(x, CornerMask.shape[dim]), 0)
        if scores[*index] == np.max(scores[clip(index[0]-1,0):clip(index[0]+2,0),clip(index[1]-1,1):clip(index[1]+2,1)]):
            NMSCornerMask[*index] = True

    # temp = cv2.dilate(NMSCornerMask.astype(np.uint8), np.ones((2, 2), dtype='uint8'))
    # while True:
    #     cv2.imshow("", temp/np.max(temp))
    #     cv2.waitKey(0)
    #     cv2.imshow("", img[3:-3]/np.max(img[3:-3]))
    #     cv2.waitKey(0)

    return centers, circles, NMSCornerMask


def GetDescriptors(centers, circles, mask, RotationInvariant):
    # vector --> polar coords
    UnitVectors = np.array([[0, 1],
                            [0.316227766017, 0.948683298051],
                            [0.707106781187, 0.707106781187],
                            [0.948683298051, 0.316227766017],
                            [1, 0],
                            [0.948683298051, -0.316227766017],
                            [0.707106781187, -0.707106781187],
                            [0.316227766017, -0.948683298051],
                            [0, -1],
                            [-0.316227766017, -0.948683298051],
                            [-0.707106781187, -0.707106781187],
                            [-0.948683298051, -0.316227766017],
                            [-1, 0],
                            [-0.948683298051, 0.316227766017],
                            [-0.707106781187, 0.707106781187],
                            [-0.316227766017, 0.948683298051]])

    intensities = circles[:, mask].astype(np.int32) - centers[mask].astype(np.int32)
    
    LightVectorArr = copy.deepcopy(intensities)
    LightVectorArr[LightVectorArr > 0] = 0
    LightVectorArr = np.sum(LightVectorArr[:, :, np.newaxis] * UnitVectors[:, np.newaxis, :], axis = 0)
    LightVectorArr = np.array([np.hypot(LightVectorArr[:, 0], LightVectorArr[:, 1]), np.arctan2(LightVectorArr[:, 1], LightVectorArr[:, 0])]).T
    
    DarkVectorArr = copy.deepcopy(intensities)
    DarkVectorArr[DarkVectorArr < 0] = 0
    DarkVectorArr = np.sum(DarkVectorArr[:, :, np.newaxis] * UnitVectors[:, np.newaxis, :], axis = 0)
    DarkVectorArr = np.array([np.hypot(DarkVectorArr[:, 0], DarkVectorArr[:, 1]), np.arctan2(DarkVectorArr[:, 1], DarkVectorArr[:, 0])]).T

    # RelativeThetaArr = np.abs(LightVectorArr[:, 1] - DarkVectorArr[:, 1])

    cornerness = np.sum(np.abs(intensities), axis = 0)

    EdgeVectors = []
    for index in np.array(np.nonzero(mask)).T:
        window = np.pad(centers, 1)[index[0]:index[0]+3,index[1]:index[1]+3]
        
        HGradient = np.sum(window * filters.sobel[0])
        VGradient = np.sum(window * filters.sobel[1])

        EdgeVectors.append([np.hypot(HGradient,VGradient), np.arctan2(VGradient,HGradient)])
    EdgeVectors = np.array(EdgeVectors)

    ThetaArr = np.array([LightVectorArr[:, 1], DarkVectorArr[:, 1]])
    if RotationInvariant:
        DescriptorArr = np.array([LightVectorArr[:, 0]/np.max(LightVectorArr[:, 0]), DarkVectorArr[:, 0]/np.max(DarkVectorArr[:, 0]), cornerness/np.max(cornerness), EdgeVectors[:, 0]/np.max(EdgeVectors[:, 0]), centers[mask]/np.max(centers[mask])])
    else:
        DescriptorArr = np.array([LightVectorArr[:, 0]/np.max(LightVectorArr[:, 0]), LightVectorArr[:, 1]/np.max(LightVectorArr[:, 1]), DarkVectorArr[:, 0]/np.max(DarkVectorArr[:, 0]), DarkVectorArr[:, 1]/np.max(DarkVectorArr[:, 1]), cornerness/np.max(cornerness), EdgeVectors[:, 0]/np.max(EdgeVectors[:, 0]), centers[mask]/np.max(centers[mask])])
    return DescriptorArr, ThetaArr, np.array(np.nonzero(mask)).T


def GetRotation(DescriptorArrs, ThetaArrs):
    '''
    First index of descriptor arrs and theta arrs should always be flagship
    '''
    DescriptorArr1, ThetaArr1 = DescriptorArrs[0], ThetaArrs[0]

    out = []

    for DescriptorArr2, ThetaArr2 in zip(DescriptorArrs[1:], ThetaArrs[1:]):
        MSEArr = np.sum((DescriptorArr1[:, :, np.newaxis] - DescriptorArr2[:, np.newaxis, :])**2, axis = 0)

        thetas1 = []
        thetas2 = []
        
        order = np.array([range(len(MSEArr)), np.argsort(MSEArr, axis = 1)[:, 0]])
        order = order.T[np.argsort(MSEArr[*order])]
        for _ in range(int(min(MSEArr.shape)/5)):
            ind1, ind2 = order[0]
            order = order[~(order[:, 1] == ind2)]
            thetas1.extend(ThetaArr1[:, ind1])
            thetas2.extend(ThetaArr2[:, ind2])

            if len(order) == 0: break
        
        thetas1 = np.array(thetas1)
        thetas2 = np.array(thetas2)
        thetas = thetas1-thetas2
        thetas = thetas[~np.all([thetas1 == 0, thetas2 == 0], axis = 0)]
        thetas[thetas < -np.pi] += 2*np.pi
        thetas[thetas > np.pi] -= 2*np.pi

        precision = 0.035
        
        ThetaMode = stats.mode(np.round(thetas/precision)*precision, keepdims = False)
        OffsetThetaMode = stats.mode(np.floor(thetas/precision)*precision, keepdims = False)

        if ThetaMode.count > OffsetThetaMode.count:
            theta = np.average(thetas[np.logical_and(thetas >= ThetaMode.mode - precision/2, thetas <= ThetaMode.mode + precision/2)])
        else:
            theta = np.average(thetas[np.logical_and(thetas >= OffsetThetaMode.mode, thetas <= OffsetThetaMode.mode + precision)])

        out.append(theta)

    return out


def blockshaped(arr, nrows, ncols):
    """
    Return an array of shape (n, nrows, ncols) where
    n * nrows * ncols = arr.size

    If arr is a 2D array, the returned array should look like n subblocks with
    each subblock preserving the "physical" layout of arr.
    """
    h, w = arr.shape
    assert h % nrows == 0, f"{h} rows is not evenly divisible by {nrows}"
    assert w % ncols == 0, f"{w} cols is not evenly divisible by {ncols}"
    return (arr.reshape(h//nrows, nrows, -1, ncols).swapaxes(1,2).reshape(-1, nrows, ncols))


def normalize(arr):
    mean = np.mean(arr)
    var = np.mean((arr - mean)**2)
    
    return (arr-mean)/(var+0.00001)**0.5


def PhaseCorrelation(a, b):
    pad_vert1  = (a.shape[0] - b.shape[0]) // 2
    pad_vert0  = (a.shape[0] - b.shape[0]) - pad_vert1
    pad_horiz1 = (a.shape[1] - b.shape[1]) // 2
    pad_horiz0 = (a.shape[1] - b.shape[1]) - pad_horiz1
    pad_shape = [[pad_vert0, pad_vert1], [pad_horiz0, pad_horiz1]]
    b = np.pad(b, pad_shape)
    
    assert b.shape == a.shape
    
    G_a = np.fft.fft2(a)
    G_b = np.fft.fft2(b)
    conj_b = np.ma.conjugate(G_b)
    R = G_a*conj_b
    R /= np.absolute(R)
    r = np.fft.ifft2(R).real
    return r


def correlate(arr1, arr2, blocksize = None, algorithm = 0):
    '''
    Returns [arr2 to arr1 offset, arr1 correlatead with arr2 split into blocks]
    '''
    match algorithm:
        case 0:
            return [signal.correlate(normalize(arr1), normalize(arr2[int(arr2.shape[0]/2-RefPointWindow/2):int(arr2.shape[0]/2+RefPointWindow/2+1), int(arr2.shape[1]/2-RefPointWindow/2):int(arr2.shape[1]/2+RefPointWindow/2+1)]), mode = "full")] + [signal.correlate(arr1, normalize(block), mode = "full") for block in blockshaped(arr2, blocksize, blocksize).reshape(-1, blocksize, blocksize)]
        case 1:
            return signal.correlate(normalize(arr1), normalize(arr2), mode = "valid")


def NeighbourDetection(arr, indexes, LowerBound):
    global MinDistance, CorrelationMin
    
    return np.any([(np.sum(arr[max(row - MinDistance, 0):min(row + MinDistance + 1, arr.shape[0]), max(col - MinDistance, 0):min(col + MinDistance + 1, arr.shape[1])] > np.max(arr)*CorrelationMin) - np.sum(arr[max(row - LowerBound, 0):min(row + LowerBound + 1, arr.shape[0]), max(col - LowerBound, 0):min(col + LowerBound + 1, arr.shape[1])] > np.max(arr)*CorrelationMin)) > 0 for row, col in indexes.T])


def GetOutliers(arr):
    arr = arr.T

    mask = []

    for subarr in arr:
        sorted_subbarr = np.sort(subarr)

        ind1 = len(sorted_subbarr) // 4
        if len(sorted_subbarr) % 4 < 2:
            Quartile1 = (sorted_subbarr[ind1-1] + sorted_subbarr[ind1])/2
        else:
            Quartile1 = sorted_subbarr[ind1]

        ind3 = 3 * len(sorted_subbarr) // 4
        if len(sorted_subbarr) % 4 < 2:
            Quartile3 = (sorted_subbarr[ind3-1] + sorted_subbarr[ind3])/2
        else:
            Quartile3 = sorted_subbarr[ind3]

        InterQuartileRange = Quartile3-Quartile1

        LowerLim = Quartile1 - 1.5*InterQuartileRange
        UpperLim = Quartile3 + 1.5*InterQuartileRange

        mask.extend([subarr > LowerLim, subarr < UpperLim])

    mask = np.all(mask, axis = 0)

    return mask


## Main
a = Flagship(None, [camera(4/3, 1.11607142857), camera(4/3, 1.11607142857)])

cap = cv2.VideoCapture(1)

frames = []

for _ in range(2):
    while not(cv2.waitKey(1) & 0xFF == ord('q')):
        frame = cap.read()[1]
        cv2.imshow('webcam', frame)

    frames.append(frame)

while cap.isOpened():
    try:
        ret, frame = cap.read()

        cv2.imshow('webcam', frame)

        a.capture([frames, [frame]])
        a.landmark()
        a.triangulate()
    except NotImplementedError as e:
        print(e)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()


# DescriptorArrs, ThetaArrs = [], []
# test = [cam[0] for cam in a.imgs]
# for img in test:
#     DescriptorArr, ThetaArr, _ = GetDescriptors(*fast(img), True)
#     DescriptorArrs.append(DescriptorArr)
#     ThetaArrs.append(ThetaArr)
# thetas = GetRotation(DescriptorArrs, ThetaArrs)
# print(thetas)

# a.landmark()
# a.triangulate()
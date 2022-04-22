import numpy as np
import math
def log(R):

    trR = R[0][0] + R[1][1] + R[2][2]
    if trR == 3:
        return np.array([0, 0, 0])

    elif trR == -1:
        w1 = np.array([R[0][2], R[1][2], 1 + R[2][2]]) / np.sqrt(2 * (1 + R[2][2]))

        w2 = np.array([R[0][1], 1 + R[1][1], R[2][1]]) / np.sqrt(2 * (1 + R[1][1]))

        w3 = np.array([1 + R[0][0], R[1][0], R[2][0]]) / np.sqrt(2 * (1 + R[0][0]))

        th = np.pi

        if w1[0] == 0 and w1[1] == 0 and w1[2] == 0:  # w1 是零向量
            if w2[0] == 0 and w2[1] == 0 and w2[2] == 0:  # w2是零向量
                return th * w3
            else:
                return th * w2
        else:
            return th * w1

    else:
        th = np.arccos((R[0][0] + R[1][1] + R[2][2] - 1) / 2)
        v1 = (R[2][1] - R[1][2]) / (2 * np.sin(th))
        v2 = (R[0][2] - R[2][0]) / (2 * np.sin(th))
        v3 = (R[1][0] - R[0][1]) / (2 * np.sin(th))
        return th * np.array([v1, v2, v3])


def exp(rv):
    th = np.linalg.norm(rv)
    if th == 0:
        return np.identity(3)
    u = rv / th

    R = np.array([[np.cos(th) + (u[0] ** 2) * (1 - np.cos(th)),
                   u[0] * u[1] * (1 - np.cos(th)) - u[2] * np.sin(th),
                   u[0] * u[2] * (1 - np.cos(th)) + u[1] * np.sin(th)],

                  [u[1] * u[0] * (1 - np.cos(th)) + u[2] * np.sin(th)
                      , np.cos(th) + (u[1] ** 2) * (1 - np.cos(th))
                      , u[1] * u[2] * (1 - np.cos(th)) - u[0] * np.sin(th)],

                  [u[2] * u[0] * (1 - np.cos(th)) - u[1] * np.sin(th)
                      , u[2] * u[1] * (1 - np.cos(th)) + u[0] * np.sin(th)
                      , np.cos(th) + (u[2] ** 2) * (1 - np.cos(th))]])
    return R

def lerp(v1, v2, t):
    return (1 - t) * v1 + t * v2

def slerp(R1, R2, t):
    return R1 @ exp(t * log(R1.T @ R2))

def postureInterpolation(p1, p2, t):
    l1 = len(p1)
    l2 = len(p2)
    if l1 != l2:
        print("posture interpolation error")
        return
    result = []
    for i in range(l1):
        m1 = p1[i]  # 4 * 4
        m2 = p2[i]
        r1 = m1[:3, :3]
        r2 = m2[:3, :3]
        rotation = slerp(r1, r2, t)
        I = np.identity(4)
        I[:3, :3] = rotation
        if i == 0:
            v1 = m1[:3, 3]
            v2 = m2[:3, 3]
            translation = lerp(v1, v2, t)
            I[:3, 3] = translation

        result.append(I)

    return result

def timeWarping(motion, func):
    length = len(motion)
    result = []
    t = 0
    while True:
        y = func(t)
        if y >= length - 1:
            return result

        yFloor = math.floor(y)
        difference = y - yFloor
        yPlusOne = yFloor + 1

        interpolatedPostrue = postureInterpolation(motion[yFloor], motion[yPlusOne], difference)
        result.append(interpolatedPostrue)
        t += 1

def computePostureDifference(p1, p2):   # p2 - p1
    l1 = len(p1)
    l2 = len(p2)
    if l1 != l2:
        print("compute posture difference error!")
        return

    result = []
    for i in range(l1):
        m1 = p1[i]  # 4 * 4
        m2 = p2[i]
        r1 = m1[:3, :3]
        r2 = m2[:3, :3]

        Rd = r1.T @ r2
        I = np.identity(4)
        I[:3, :3] = Rd
        if i == 0:
            pos1 = m1[:3, 3]
            pos2 = m2[:3, 3]

            Pd = pos2 - pos1
            I[:3, 3] = Pd
        result.append(I)

    return result

def computeRootAlignmentDifference(p1, p2):   # p2 - p1
    l1 = len(p1)
    l2 = len(p2)
    if l1 != l2:
        print("compute root alignment difference error!")
        return

    m1 = p1[0]  # 4 * 4
    m2 = p2[0]
    r1 = m1[:3, :3]
    r2 = m2[:3, :3]

    pos1 = m1[:3, 3]
    pos2 = m2[:3, 3]

    I = np.identity(4)

    PosAlignmentDifference = pos2 - pos1
    # PosAlignmentDifference[1] = 0
    RotationAlignmentDifference = r2 @ r1.T

    logResult = log(RotationAlignmentDifference)
    logResult[0] = 0
    logResult[2] = 0   #projected vector of log(R) on y axis

    expResult = exp(logResult)

    I[:3, :3] = expResult
    I[:3, 3] = PosAlignmentDifference

    return I

def scaleDifference(c, d):
    length = len(d)
    result = []
    for i in range(length):
        m = d[i]
        r = m[:3, :3]
        I = np.identity(4)
        r2 = exp(c * log(r))
        I[:3, :3] = r2
        if i == 0:
            t = m[:3, 3]
            t2 = c * t
            I[:3, 3] = t2

        result.append(I)

    return result


def motionWarping(motion, difference, numFrames, frameIndex, start, end):
    if start < 0 or start >= frameIndex:
        print("motion warping error")
        return

    if end <= frameIndex or end >= numFrames:
        print("motion warping error")
        return

    def func1(x):
        y = 1 - (x - frameIndex) / (start - frameIndex)
        return y

    def func2(x):
        y = 1 - (x - frameIndex) / (end - frameIndex)
        return y

    resultMotion = []
    lengthOfMotionList = len(motion)

    for idx in range(lengthOfMotionList):
        frame = motion[idx]
        if idx >= start and idx <= end:
            if idx < frameIndex:
                c = func1(idx)
            else:
                c = func2(idx)
            scaledDifference = scaleDifference(c, difference)

            length = len(frame)
            resultPosture = []
            for i in range(length):
                m1 = frame[i]
                m2 = scaledDifference[i]
                if i == 0:
                    temp = m1.copy()
                    resultPosture.append(temp)
                else:
                    r1 = m1[:3, :3]
                    r2 = m2[:3, :3]
                    rotation = r1 @ r2
                    I = np.identity(4)
                    I[:3, :3] = rotation
                    resultPosture.append(I)
            resultMotion.append(resultPosture)

        else:
            resultMotion.append(frame)

    return resultMotion


def motionStitiching(motion, difference, rootDifference, transitionLength):
    def func(x):
        y = 1 - (x / (transitionLength - 1))
        return y

    lengthOfMotionList = len(motion)
    resultMotion = []

    for idx in range(lengthOfMotionList):
        frame = motion[idx]
        if idx < transitionLength:
            c = func(idx)
            scaledDifference = scaleDifference(c, difference)

            length = len(frame)
            resultPosture = []
            for i in range(length):
                I = np.identity(4)
                m1 = frame[i]
                m2 = scaledDifference[i]
                if i == 0:
                    I[:4, :4] = m1[:4, :4]
                    resultPosture.append(I)
                else:
                    r1 = m1[:3, :3]
                    r2 = m2[:3, :3]
                    rotation = r1 @ r2

                    I[:3, :3] = rotation

                    resultPosture.append(I)
            resultMotion.append(resultPosture)

        else:
            temp = frame.copy()
            resultMotion.append(temp)

    rootStartPos = resultMotion[0][0][:3, 3].copy()

    for idx in range(lengthOfMotionList):
        frame = resultMotion[idx]
        m1 = frame[0]
        m2 = rootDifference
        r1 = m1[:3, :3]
        r2 = m2[:3, :3]
        rotation = r2 @ r1

        pos1 = m1[:3, 3]
        pos2 = m2[:3, 3]

        translation = r2 @ (pos1 - rootStartPos) + rootStartPos + pos2

        I = np.identity(4)
        I[:3, :3] = rotation
        I[:3, 3] = translation
        resultMotion[idx][0] = I

    return resultMotion

def motionAdd(motion1, motion2):
    return motion1 + motion2


def motionBlending(motion1, motion2):

    motionLength = len(motion1)
    resultMotion = []

    def func(x):
        y = (1 / (motionLength - 1)) * x
        return y

    for idx in range(motionLength):
        frame1 = motion1[idx]
        frame2 = motion2[idx]

        frameLength = len(frame1)
        resultPosture = []
        c = func(idx)
        for i in range(frameLength):
            m1 = frame1[i]
            m2 = frame2[i]

            r1 = m1[:3, :3]
            r2 = m2[:3, :3]

            I = np.identity(4)
            if i == 0:
                pos1 = m1[:3, 3]
                pos2 = m2[:3, 3]
                rotation = slerp(r1, r2, 0.5)
                translation = lerp(pos1, pos2, 0.5)

                I[:3, :3] = rotation
                I[:3, 3] = translation

            else:
                rotation = slerp(r1, r2, c)
                I[:3, :3] = rotation

            resultPosture.append(I)
        resultMotion.append(resultPosture)

    return resultMotion















# def main():
#     print(motionStitiching(1, 1, 10))
#
# main()


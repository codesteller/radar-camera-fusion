import serial
import time
import numpy as np
import time
import xlsxwriter
from pynput import keyboard
import threading
import cv2
import concurrent.futures



#get header
def getHeader(bytevec, idx):
    idx = idx + 8
    word = [1, 256, 65536, 16777216]
    header = {}
    # multiply by 256^0, 256^1, 256^2, 256^3 and add resulting in unit32
    header['version'] = int(np.frombuffer(bytevec[idx:idx+4], dtype='uint32'))
    idx = idx + 4
    header['totalPacketLen'] = int(np.frombuffer(bytevec[idx:idx+4], dtype='uint32'))
    idx = idx + 4
    header['platform'] = int(np.frombuffer(bytevec[idx:idx+4], dtype='uint32'))
    idx = idx + 4
    header['frameNumber'] = int(np.frombuffer(bytevec[idx:idx+4], dtype='uint32'))
    idx = idx + 4
    header['timeCpuCycles'] = int(np.frombuffer(bytevec[idx:idx+4], dtype='uint32'))
    idx = idx + 4
    header['numDetectedObj'] = int(np.frombuffer(bytevec[idx:idx+4], dtype='uint32'))
    idx = idx + 4
    header['numTLVs'] = int(np.frombuffer(bytevec[idx:idx+4], dtype='uint32'))
    idx = idx + 4
    header['subFrameNumber'] = int(np.frombuffer(bytevec[idx:idx+4], dtype='uint32'))
    idx = idx + 4
    return header, idx

#get tlv
def getTlv(bytevec, idx):
    tlv = {}
    tlv['type'] = int(np.frombuffer(bytevec[idx:idx+4], dtype='uint32'))
    idx = idx + 4
    tlv['length'] = int(np.frombuffer(bytevec[idx:idx+4], dtype='uint32'))
    idx = idx + 4
    return tlv, idx

def getDetObj(bytevec, idx, tlvlen, detObj):
    detObj['numObj'] = 0
    len_bytevec = len(bytevec)

    if len_bytevec < idx + 4:
        idx = len_bytevec
        return detObj, idx
    
    if tlvlen > 0:
        detObj['numObj'] = int(np.frombuffer(bytevec[idx:idx+2], dtype='int16'))
        idx = idx + 2
        xyzQFormat = 2**int(np.frombuffer(bytevec[idx:idx+2], dtype='int16'))
        idx = idx + 2
        invxyzQFormat = 1.0/float(xyzQFormat)
        if len_bytevec < idx + detObj['numObj']*OBJ_STRUCT_SIZE_BYTES:
            detObj['numObj'] = 0
            idx = len_bytevec
            return detObj, idx
        bytes = bytevec[idx:idx+detObj['numObj']*OBJ_STRUCT_SIZE_BYTES]
        idx = idx + detObj['numObj']*OBJ_STRUCT_SIZE_BYTES

        bytes = np.reshape(bytes, (detObj['numObj'], OBJ_STRUCT_SIZE_BYTES))
        detObj['doppler'] = bytes[:, 0] + bytes[:, 1]*256
        detObj['peakVal'] = bytes[:, 2] + bytes[:, 3]*256

        detObj['x'] = np.array(bytes[:, 4] + bytes[:, 5]*256, dtype='int16')
        detObj['y'] = np.array(bytes[:, 6] + bytes[:, 7]*256, dtype='int16')
        detObj['z'] = np.array(bytes[:, 8] + bytes[:, 9]*256, dtype='int16')

        
        
        detObj['x'][detObj['x'] > 32767] = detObj['x'][detObj['x'] > 32767] - 65536
        detObj['y'][detObj['y'] > 32767] = detObj['y'][detObj['y'] > 32767] - 65536
        detObj['z'][detObj['z'] > 32767] = detObj['z'][detObj['z'] > 32767] - 65536

        detObj['doppler'][detObj['doppler'] > 32767] = detObj['doppler'][detObj['doppler'] > 32767] - 65536

        detObj['x'] = detObj['x']*invxyzQFormat
        detObj['y'] = detObj['y']*invxyzQFormat
        detObj['z'] = -detObj['z']*invxyzQFormat
        detObj['doppler'] = detObj['doppler']*invxyzQFormat
        detObj['range'] = np.sqrt(detObj['x']**2 + detObj['y']**2 + detObj['z']**2)
    
    return detObj, idx

def getCluster(bytevec, idx, tlvlen, cluster):
    len_bytevec = len(bytevec)
    if len_bytevec < idx + 4:
        idx = len_bytevec
        return cluster, idx
    
    if tlvlen > 0:
        cluster['numObj'] = int(np.frombuffer(bytevec[idx:idx+2], dtype='int16'))
        idx = idx + 2
        onebyxyzqformat = 1.0/(2 ** int(np.frombuffer(bytevec[idx:idx+2], dtype='int16')))
        idx = idx + 2
        if len_bytevec < idx + cluster['numObj']*CLUSTER_STRUCT_SIZE_BYTES:
            cluster['numObj'] = 0
            idx = len_bytevec
            return cluster, idx
        
        bytes = bytevec[idx:idx+cluster['numObj']*CLUSTER_STRUCT_SIZE_BYTES]
        idx = idx + cluster['numObj']*CLUSTER_STRUCT_SIZE_BYTES

        bytes = np.reshape(bytes, (cluster['numObj'], CLUSTER_STRUCT_SIZE_BYTES))
        
        x = np.array(bytes[:, 0] + bytes[:, 1]*256, dtype='int16')
        y = np.array(bytes[:, 2] + bytes[:, 3]*256, dtype='int16')

        x[x > 32767] = x[x > 32767] - 65536
        y[y > 32767] = y[y > 32767] - 65536

        x = x*onebyxyzqformat
        y = y*onebyxyzqformat

        x_size = bytes[:, 4] + bytes[:, 5]*256
        y_size = bytes[:, 6] + bytes[:, 7]*256
        
        x_size = x_size*onebyxyzqformat
        y_size = y_size*onebyxyzqformat

        area = x_size*y_size * 4

        x_size[area > 20] = 99999

        x_loc = x + x_size * [-1, 1, 1, -1, -1, 99999]
        y_loc = y + y_size * [-1, -1, 1, 1, -1, 99999]

        cluster['x_loc'] = x_loc
        cluster['y_loc'] = y_loc

    return cluster, idx

def getTracker(bytevec, idx, tlvlen, tracker):

    tracker['numObj'] = 0
    len_bytevec = len(bytevec)

    if len_bytevec < idx + 4:
        idx = len_bytevec
        return tracker, idx
    
    if tlvlen > 0:
        tracker['numObj'] = int(np.frombuffer(bytevec[idx:idx+2], dtype='int16'))
        idx = idx + 2
        xyzQFormat = 2**int(np.frombuffer(bytevec[idx:idx+2], dtype='int16'))
        idx = idx + 2
        invxyzQFormat = 1.0/float(xyzQFormat)

        if len_bytevec < idx + tracker['numObj']*TRACKER_STRUCT_SIZE_BYTES:
            tracker['numObj'] = 0
            idx = len_bytevec
            return tracker, idx
        
        bytes = bytevec[idx:idx+tracker['numObj']*TRACKER_STRUCT_SIZE_BYTES]
        idx = idx + tracker['numObj']*TRACKER_STRUCT_SIZE_BYTES

        bytes = np.reshape(bytes, (tracker['numObj'], TRACKER_STRUCT_SIZE_BYTES))

        tracker['x'] = np.array(bytes[:, 0] + bytes[:, 1]*256, dtype='int16')
        tracker['y'] = np.array(bytes[:, 2] + bytes[:, 3]*256, dtype='int16')

        tracker['x'][tracker['x'] > 32767] = tracker['x'][tracker['x'] > 32767] - 65536
        tracker['y'][tracker['y'] > 32767] = tracker['y'][tracker['y'] > 32767] - 65536

        tracker['x'] = tracker['x']*invxyzQFormat
        tracker['y'] = tracker['y']*invxyzQFormat

        # tracker['vx'] = bytes[:, 4] + bytes[:, 5]*256
        # tracker['vy'] = bytes[:, 6] + bytes[:, 7]*256
        
        tracker['vx'] = np.array(bytes[:, 4] + bytes[:, 5]*256, dtype='int16')
        tracker['vy'] = np.array(bytes[:, 6] + bytes[:, 7]*256, dtype='int16')

        tracker['vx'][tracker['vx'] > 32767] = tracker['vx'][tracker['vx'] > 32767] - 65536
        tracker['vy'][tracker['vy'] > 32767] = tracker['vy'][tracker['vy'] > 32767] - 65536

        tracker['vx'] = tracker['vx']*invxyzQFormat
        tracker['vy'] = tracker['vy']*invxyzQFormat

        x_size = bytes[:, 8] + bytes[:, 9]*256
        y_size = bytes[:, 10] + bytes[:, 11]*256

        x_size = x_size*invxyzQFormat
        y_size = y_size*invxyzQFormat

        # convert x_size to a numpy array of 1 columns
        x_size = np.array(x_size).reshape(-1, 1)
        y_size = np.array(y_size).reshape(-1, 1)



        x_loc = np.array(tracker['x']).reshape(-1,1) + x_size * np.array([[-1, 1, 1, -1, -1, 99999]])
        y_loc = np.array(tracker['y']).reshape(-1,1) + y_size * np.array([[-1, 1, 1, -1, -1, 99999]])

        tracker['cluster_x_loc'] = x_loc
        tracker['cluster_y_loc'] = y_loc

        tracker['range'] = np.sqrt(tracker['x']**2 + tracker['y']**2)
        tracker['doppler'] = (tracker['x']*tracker['vx'] + tracker['y']*tracker['vy'])/tracker['range']
    return tracker, idx



# def on_press(key):
#     if key == keyboard.Key.q:
#         return False  

def radar_data():
    bytevec_cp_len = 0
    bytevecAccLen = 0
    bytevec_cp_max_len = 2**15
    bytevec_cp = np.zeros(bytevec_cp_max_len, dtype='uint8')

    bytebufferlength = 0

    magicok = 0
    data =[]
    # listener = keyboard.Listener(on_press=on_press)
    # listener.start()
    # while True:
    if ser.in_waiting > 0:
        ser_data = ser.read(ser.in_waiting)
        bytebufferlength = len(ser_data)
        if (bytevec_cp_len + bytebufferlength) < bytevec_cp_max_len:
            bytevec_cp[bytevec_cp_len:bytevec_cp_len+bytebufferlength] = np.frombuffer(ser_data, dtype='uint8')
            bytevec_cp_len += bytebufferlength
            bytebufferlength = 0
            # print("bytevec_cp_len: ", bytevec_cp_len)
        else:
            print("Error: bytevec_cp overflow")
            bytebufferlength = 0
            bytevec_cp_len = 0
            bytevec_cp = np.zeros(bytevec_cp_max_len, dtype='uint8')
    
    bytevecStr = bytevec_cp[0:bytevec_cp_len].tostring()
    magicok = 0
    if (bytevec_cp_len>72):
        startIdx = bytevecStr.find(b'\x02\x01\x04\x03\x06\x05\x08\x07')
    else:
        startIdx = -1
    
    if (startIdx != -1):
        if (startIdx >= 0):
            # print("startIdx: ", startIdx)
            magicok = 1
            countok = 0
            bytevecAccLen = bytevec_cp_len - startIdx
            bytevec_cp[0:bytevecAccLen] = bytevec_cp[startIdx:bytevec_cp_len]
            bytevec_cp_len = bytevecAccLen
        else:
            print("Error: startIdx < 0")
            magicok = 0
            countok = 0
            bytevec_cp_len = 0
            bytevec_cp = np.zeros(bytevec_cp_max_len, dtype='uint8')

    byteVecIdx = 0
    if (magicok == 1):
        #new frame
        # start time
        tStart = time.time()
        header, byteVecIdx = getHeader(bytevec_cp, byteVecIdx)
        sfIdx = header['subFrameNumber'] + 1
        if (sfIdx > 2) | (header['numDetectedObj'] > MAX_NUM_OBJECTS):
            # continue
            print("Error: sfIdx > 2 or numDetectedObj > MAX_NUM_OBJECTS")
        
        detObj = {}
        cluster = {}
        tracker = {}
        detObj['numObj'] = 0
        cluster['numObj'] = 0
        tracker['numObj'] = 0
        for tlvidx in range(header['numTLVs']):

            tlv, byteVecIdx = getTlv(bytevec_cp, byteVecIdx)
            # print("tlv['type']: ", tlv['type'])

            if tlv['type'] == MMWDEMO_UART_MSG_DETECTED_POINTS:
                if tlv['length'] >= OBJ_STRUCT_SIZE_BYTES:
                    detObj, byteVecIdx = getDetObj(bytevec_cp, byteVecIdx, tlv['length'], detObj)
            
            elif tlv['type'] == MMWDEMO_UART_MSG_CLUSTERS:
                if tlv['length'] >= CLUSTER_STRUCT_SIZE_BYTES:
                    cluster, byteVecIdx = getCluster(bytevec_cp, byteVecIdx, tlv['length'], cluster)

            elif tlv['type'] == MMWDEMO_UART_MSG_TRACKED_OBJ:
                if tlv['length'] >= TRACKER_STRUCT_SIZE_BYTES:
                    tracker, byteVecIdx = getTracker(bytevec_cp, byteVecIdx, tlv['length'], tracker)
        time.sleep(0.033)  # Introduce a delay of approximately 33 milliseconds (0.033 seconds)
        num = tracker['numObj']
        # num = detObj['numObj']
        if num > 0:
            for i in range(0,num):
                # num_objects = tracker['numObj']
                val_x = tracker['x'][i]
                val_y = tracker['y'][i]
                # range_val = tracker['range'][i]
                # doppler_val = tracker['doppler'][i]
                # print("Num Objects {} Object ID {} X {} Y {} Range {} Doppler {}".format(num_objects,byteVecIdx, val_x, val_y, range_val, doppler_val))
                # data.append([timestamp, i, val_x, val_y, range_val, doppler_val])-
                data.append([val_x, val_y,0])
                # num_objects = tracker['numObj']
                # val_x = detObj['x'][i]
                # val_y = detObj['y'][i]
                # data.append([val_x, val_y,0])


    return data


cap = cv2.VideoCapture(2)

def camera_data():
    ret, frame = cap.read()
    if ret:
        # cap.release()
        return frame
    else:
        # cap.release()
        return None

def rectify(image):
        D = [-0.418196, 0.154948, -0.002137, -0.001495, 0.000000]
        K = [1336.243344, 0.000000, 968.249812, 0.000000, 1337.843618, 556.135602, 0.000000, 0.000000, 1.000000]
        P = [996.403320, 0.000000, 963.695972, 0.000000, 0.000000, 1247.530151, 560.818595, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
        R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        #get the camera matrix
        K = np.array(K).reshape(3,3)
        #get the distortion matrix
        D = np.array(D).reshape(5,1)
        #get the rotation matrix
        R = np.array(R).reshape(3,3)
        #calculate the projection matrix assuming no skew and no translation
        # P = K.dot(np.hstack((R, np.zeros((3,1)))))
        P = np.array(P).reshape(3,4)
        # print P in standard format without scientific notation
        # np.set_printoptions(suppress=True)
        # print(P)
        
        
        #get the image size
        h, w = image.shape[:2]
        # print(K.shape)
        # print(D.shape)
        #get the map for remapping the image plum bob model
        map1, map2 = cv2.initUndistortRectifyMap(K, D, R, P, (w,h), cv2.CV_16SC2)
        #rectify the image
        dst = cv2.remap(image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # print(dst.shape)
        return dst        

if __name__ == '__main__':
    ser = serial.Serial()
    ser.baudrate = 921600
    ser.port = '/dev/ttyACM1'
    ser.open()

    MAX_NUM_OBJECTS = 200;
    OBJ_STRUCT_SIZE_BYTES = 10;
    MAX_NUM_CLUSTERS = 24;
    CLUSTER_STRUCT_SIZE_BYTES = 8;
    MAX_NUM_TRACKERS = 24;
    TRACKER_STRUCT_SIZE_BYTES = 12;
    STATS_SIZE_BYTES = 16;
    MMWDEMO_UART_MSG_DETECTED_POINTS    = 1;
    MMWDEMO_UART_MSG_CLUSTERS           = 2;
    MMWDEMO_UART_MSG_TRACKED_OBJ        = 3;
    MMWDEMO_UART_MSG_PARKING_ASSIST     = 4;

    with concurrent.futures.ThreadPoolExecutor() as executor:
        while True:
            radar = executor.submit(radar_data)
            camera = executor.submit(camera_data)
            radar_frames = radar.result()
            camera_frames = camera.result()
            if camera_frames is not None:
                camera_frames = rectify(camera_frames)
            projection_matrix = np.array([[ 9.54488857e+02, -1.33958331e+03, -1.32369146e+02,  1.92046470e+03],
                                        [ 3.68015683e+02, -1.28462229e+00, -1.40131192e+03,  1.01862215e+03],
                                        [ 9.90605116e-01, -3.45787825e-03, -1.36709705e-01,  2.00000000e+00]])
            for frame in radar_frames:
                x = frame[1]
                y = frame[0]
                z = frame[2]
                if x < 70.0:
                    radar_point = np.asarray([-x,y,z,1])
                else:
                    radar_point = np.asarray([1,1,0,1])
                # print("X {} Y {} Z {}".format(x,y,z))
                image_point = projection_matrix @ radar_point
                image_point = image_point/image_point[2]
                image_point = image_point[:2]
                image_point = image_point.astype(int)
                camera_frames = cv2.resize(camera_frames, (1920, 1020))
                cv2.circle(camera_frames, (image_point[0], image_point[1]), 10, (0, 0, 255), -1)
                #put point coordinates along with circle
                cv2.putText(camera_frames, "X: " + str(round(x,1)) + " Y: " + str(round(y,1)) , (image_point[0] - 50, image_point[1] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.imshow('frame', camera_frames)
                if cv2.waitKey(1) == 27:  # Break the loop when 'Esc' key is pressed
                    break
        cv2.destroyAllWindows()
        
cap.release()
        
            

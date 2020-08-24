#python3
"""
Mask R-CNN for instant
The implement of pretrained model, using socket communication to transform in/output images

Copyright (c) 2017 Matterport, Inc.
Licensed under the MIT License (see LICENSE for details)
Written by wangzile/an zheng
"""

###
### NOTE: download the pre-trained model(address below), and modify the model path on line 44:
### https://drive.google.com/file/d/1HZ9i7IvWW7uSMMODVvPw7pHgKGjK5lKD/view?usp=sharing
###

print('start import socket modules...')
#socket modules
import socket
import time
import skimage.io
import matplotlib.pyplot as plt

print('start import mrcnn modules...')
#mrcnn modules
import os
import sys
import random
import math
import numpy as np
import matplotlib


# socket works

def socket_process():
    #IP地址'0.0.0.0'为等待客户端连接
    address = ('127.0.0.1', 6666)
    #建立socket对象，参数意义见https://blog.csdn.net/rebelqsp/article/details/22109925
    #socket.AF_INET：服务器之间网络通信 
    #socket.SOCK_STREAM：流式socket , for TCP
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #将套接字绑定到地址, 在AF_INET下,以元组（host,port）的形式表示地址.
    s.bind(address)
    #开始监听TCP传入连接。参数指定在拒绝连接之前，操作系统可以挂起的最大连接数量。该值至少为1，大部分应用程序设为5就可以了。
    s.listen(1)

    def recvall(sock, count):
        buf = b''#buf是一个byte类型
        while count:
            try:
                #接受TCP套接字的数据。数据以字符串形式返回，count指定要接收的最大数据量.
                newbuf = sock.recv(count)
                if not newbuf: return None
                buf += newbuf
                count -= len(newbuf)
            except socket.error as msg:
                print('socket connection break!')
                return None
        return buf
    
    # mrcnn works
    ROOT_DIR = os.path.abspath("./")
    from mrcnn import utils
    import mrcnn.model as modellib
    from mrcnn import visualize
    sys.path.append(os.path.join(ROOT_DIR, "mrcnn/milkvsvida"))
    import milkvsvida

    # Directory to save logs and trained model
    MODEL_DIR = os.path.join(ROOT_DIR, "milkvsvida_logs")
    
    # Local path to trained weights file
    MODEL_PATH = "/home/wang/VidaVSMilk-using-Mask-RCNN-master/mrcnn/mask_rcnn_milkvsvida_0030.h5"
    # Download trained weights from Releases if needed
    if not os.path.exists(MODEL_PATH):
        print("pre trained model not exists! ")
        exit()

    class InferenceConfig(milkvsvida.MilkVSVidaConfig):
        # Set batch size to 1 since we'll be running inference on
        # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
        GPU_COUNT = 1
        IMAGES_PER_GPU = 1

    config = InferenceConfig()
    config.display()

    print("model configuring... ")
    # Create model object in inference mode.
    model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)

    print("pre trained model loading... ")
    # Load weights trained on MS-COCO
    model.load_weights(MODEL_PATH, by_name=True)

    class_names = ['BG', 'milk', 'vida', 'not_defined']
    images=[]


    print('start binding and listening...')
    #接受TCP连接并返回（conn,address）,其中conn是新的套接字对象，可以用来接收和发送数据。addr是连接客户端的地址。
    #没有连接则等待有连接
    conn, addr = s.accept()
    print('connect from:'+str(addr))
    length=640*480*3

    while 1:
        #start = time.time()#用于计算帧率信息
        #length = recvall(conn,16)#获得图片文件的长度,16代表获取长度
        print('waiting image...')
        stringData = recvall(conn, int(length))#根据获得的文件长度，获取图片文件
        if stringData != None:
            data = np.frombuffer(stringData, np.uint8)#将获取到的字符流数据转换成1维数组
        else:
            print('socket disconnect !!')
            conn, addr = s.accept()
            print('connect from:'+str(addr))
            continue
        
        images=[]
        #decimg=cv2.imdecode(data,cv2.IMREAD_COLOR)#将数组解码成图像
        data=data.reshape((480,640,-1))
        #print(data)
        #skimage.io.imshow(data)
        images.append(data)
        #cv2.imwrite("./test.jpg",decimg)
        #print(decimg)
        #cv2.imshow('SERVER',decimg)#显示图像

        #end = time.time()
        #seconds = end - start
        #fps  = 1/seconds;
        #conn.send(bytes(str(int(fps)),encoding='utf-8'))

        # image_path="./0281.jpg"
        # imagencode=skimage.io.imread(image_path)
        # images=[imagencode]
# start mrcnn dete...
        results=[]
        print("####### dete image start...")
        # Run detection
        results.append(model.detect(images, verbose=1)[0])
        print("##### dete finished.")

        # Visualize results
        r = results[0]
        print('size1:',len(r['masks']),'size2:',len(r['masks'][0]),'size3:',len(r['masks'][0][0]))# this is (480.640.object_num)
        print( r['class_ids'])
        #masked = visualize.display_instances(images[0], r['rois'], r['masks'], r['class_ids'], class_names, r['scores'])
        
        img_panel = np.zeros((480,640,3),np.int8)
        if(len(r['class_ids'])>0):
            mask_bit = visualize.apply_mask(img_panel, r['masks'][:, :, 0], color=(1.0,1.0,1.0), alpha=1.0)
        else:
            mask_bit = visualize.apply_mask(img_panel,np.ones((480,640), np.int8) , color=(1.0,1.0,1.0), alpha=1.0)
# done dete works


#re send data

        data_resend = np.array(mask_bit)
        #将numpy矩阵转换成字符形式，以便在网络中传输
        data_resend = data_resend.tostring()
        
        #先发送要发送的数据的长度
        #ljust() 方法返回一个原字符串左对齐,并使用空格填充至指定长度的新字符串
        print(str('resending data:'), len(data_resend))
        #sock.send(str.encode(str(len(stringData)).ljust(16)));
        #发送数据
        conn.send(data_resend)
    s.close()




     
if __name__ == '__main__':
    socket_process()

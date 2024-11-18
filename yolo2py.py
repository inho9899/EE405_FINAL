from yolov5.detect import run as detection
import torch

## python3 detect.py --weights runs/train/exp/weights/best.pt --img 640 --conf 0.7 --source ../data/test/test2.png

## class Data : x, y, class

class Data :
    def __init__(self, x, y, class_name) :
        self.x = x
        self.y = y
        self.class_name = int(class_name)

[pred] = detection(imgsz = (640, 640), weights = 'yolov5/runs/train/exp/weights/best.pt', source = 'data/test/test1.png')
pred = pred.cuda().to('cpu').numpy()

d = []

for i, k in enumerate(pred) :
    d.append(Data(k[0], k[1], k[5]))

d = sorted(d, key = lambda x : x.y)



res = []

label = ['red', 'blue', 'none']

for i in range(0, len(d), 3) :
    pr = [d[i], d[i+1], d[i+2]]
    
    ## sort by x
    pr = sorted(pr, key = lambda x : x.x)
    
    res.append(pr)

final_pred = []

    
for i in res :
    final_pred.append([label[i[0].class_name], label[i[1].class_name], label[i[2].class_name]])
    
for row in final_pred:
    print(" ".join(map(str, row)))  # 값들을 공백으로 구분
        
    
    
    
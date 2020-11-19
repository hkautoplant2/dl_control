#!/bin/sh
# This is a comment!
echo Start communicating to HOST	

#scp foo.txt autoplant2@192.168.1.102:~/


#ssh root@540776e7638c 'tlt-infer faster_rcnn -e /workspace/examples/faster_rcnn/specs/fastrcnn_retrain_pruned12.txt'

scp /home/jetson/Downloads/test_image2.png autoplant2@192.168.1.102:~/

ssh autoplant2@192.168.1.102 'docker cp test_image2.png pedantic_chatelet:/workspace/tlt-experiments/data/Test/images'

ssh autoplant2@192.168.1.102 'docker exec -i e3a86180333e tlt-infer faster_rcnn -e /workspace/examples/faster_rcnn/specs/fastrcnn_retrain_pruned12.txt'

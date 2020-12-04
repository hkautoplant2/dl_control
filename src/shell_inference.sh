#!/bin/sh
# This is a comment!
echo Start communicating to HOST	


#Copy zed image into host locally
scp -r ~/run_inf_jet/* autoplant2@192.168.1.102:~/inference/run_inf_host

#Copy zed image from host into container
ssh autoplant2@192.168.1.102 'docker cp inference/run_inf_host/. e3a046e3d419:/workspace/run_inf_cont'

#Execute inference on the image
ssh autoplant2@192.168.1.102 'docker exec -i e3a046e3d419 tlt-infer faster_rcnn -e /workspace/examples/faster_rcnn/specs/fastrcnn_retrain.txt'

#Copy resulting labeled image and label file to local host map 
ssh autoplant2@192.168.1.102 'docker cp e3a046e3d419:/workspace/res_inf_cont/. ~/inference/res_inf_host'

#Copy resulting labeled image and label file into inference log map
ssh autoplant2@192.168.1.102 'cp -r ~/inference/res_inf_host/. ~/inference/log_inf_host'

#Copy results from host to jetson map
scp -r autoplant2@192.168.1.102:~/inference/res_inf_host/* ~/res_inf_jet

echo Inference has been run on network



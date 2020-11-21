#!/bin/sh
# This is a comment!
echo Start communicating to HOST	

#Clean up inference result map on jetson
rm ~/res_inf_jet/*

#Copy zed image into host locally
scp -r ~/run_inf_jet/* autoplant2@192.168.1.102:~/inference/run_inf_host

#Copy zed image from host into container
ssh autoplant2@192.168.1.102 'docker cp inference/run_inf_host/. e3a86180333e:/workspace/run_inf_cont'

#Execute inference on the image
ssh autoplant2@192.168.1.102 'docker exec -i e3a86180333e tlt-infer faster_rcnn -e /workspace/examples/faster_rcnn/specs/fastrcnn_retrain_pruned12.txt'

#Copy resulting labeled image and label file to local host map 
ssh autoplant2@192.168.1.102 'docker cp e3a86180333e:/workspace/res_inf_cont/. ~/inference/res_inf_host'

#Copy resulting labeled image and label file into inference log map
ssh autoplant2@192.168.1.102 'cp -r ~/inference/res_inf_host/. ~/inference/log_inf_host'

#Copy results from host to jetson map
scp -r autoplant2@192.168.1.102:~/inference/res_inf_host/* ~/res_inf_jet

#Clean run inference folder for network
ssh autoplant2@192.168.1.102 'docker exec -i e3a86180333e rm -rf run_inf_cont'

#Clean inference result map for network
ssh autoplant2@192.168.1.102 'docker exec -i e3a86180333e rm -rf res_inf_cont'

#Create the removed directories again (ugly solution)
ssh autoplant2@192.168.1.102 'docker exec -i e3a86180333e mkdir run_inf_cont'
ssh autoplant2@192.168.1.102 'docker exec -i e3a86180333e mkdir res_inf_cont'

#Clean up host run inference map
ssh autoplant2@192.168.1.102 'rm inference/run_inf_host/*'

#Clean up host resulting inference map
ssh autoplant2@192.168.1.102 'rm inference/res_inf_host/*'

#Clean up jetsons run inference maps
rm ~/run_inf_jet/*




#!/bin/sh
# This is a comment!
echo Start cleaning up directories	

#Clean up inference result map on jetson
rm ~/res_inf_jet/*

#Clean run inference folder for network
ssh autoplant2@192.168.1.102 'docker exec -i e3a046e3d419 rm -rf run_inf_cont'

#Clean inference result map for network
ssh autoplant2@192.168.1.102 'docker exec -i e3a046e3d419 rm -rf res_inf_cont'

#Create the removed directories again (ugly solution)
ssh autoplant2@192.168.1.102 'docker exec -i e3a046e3d419 mkdir run_inf_cont'
ssh autoplant2@192.168.1.102 'docker exec -i e3a046e3d419 mkdir res_inf_cont'

#Clean up host run inference map
ssh autoplant2@192.168.1.102 'rm inference/run_inf_host/*'

#Clean up host resulting inference map
ssh autoplant2@192.168.1.102 'rm inference/res_inf_host/*'

#Clean up jetsons run inference maps
rm ~/run_inf_jet/*

echo Directories are cleaned up




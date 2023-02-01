./build/SlamTester \
-algorithm=dso \
-camConfig=/Users/zhongzhaoqun/Downloads/Euroc/camera_euroc.txt \
-dataBag=/Users/zhongzhaoqun/Downloads/MH-01-easy/MH_01_easy.bag \
-dataset=euroc \
-groundTruth=/Users/zhongzhaoqun/Downloads/MH-01-easy/state_groundtruth_estimate0/data.csv \
-ciExtrinsic=/Users/zhongzhaoqun/Downloads/MH-01-easy/cam0/sensor.yaml \
-resizeAndUndistort=true \
-algoConfig=/Users/zhongzhaoqun/Downloads/Euroc/euroc_config.yaml \
-skipCamMsgIfLag=false \
-showOrigCamStream=true \
-viewer=socket \
-socketConfig=/Users/zhongzhaoqun/CLionProjects/SlamTester/socket_publisher/config.yaml 
clc
clear
close all

%%
rs2 = Capture.RealSense();

rs2 = rs2.startStreaming();
for i = 1:200
    img = rs2.getRGBFrame();
    imshow(img);
end

rs2.stopStreaming();

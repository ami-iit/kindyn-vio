function publishImage(publisher, img, img_encoding)
if isempty(img)
    return
end
img_msg = rosmessage('sensor_msgs/Image');
img_msg.Encoding = img_encoding;
writeImage(img_msg,img);
send(publisher, img_msg);
end


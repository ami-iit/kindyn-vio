classdef RealSense
    properties
        pipe realsense.pipeline;  % pipeline to manage streaming
        profile realsense.pipeline_profile; % camera settings
        config realsense.config;
        device realsense.device; % device wrapper
        frameset realsense.frameset; % frameset
    end
    
    methods
        function obj = RealSense()
            obj.pipe = realsense.pipeline(); 
            obj.config = realsense.config();
            obj.config.enable_stream(realsense.stream.color, 640, 480);            
        end
        
        function obj = startStreaming(obj)
            % start streaming on arbitrary camera with default settings
            obj.profile = obj.pipe.start(obj.config);
            % get device name
            obj.device = obj.profile.get_device();
            name = obj.device.get_info(realsense.camera_info.name);
            disp(['Starting stream from ', name]);
            
            % skip first 100 frames for a color correction
            % and for the camera to settle
            disp(['Waiting for the camera to settle...']);
            for i = 1:75      
                    obj.frameset = obj.pipe.wait_for_frames();                                
            end 
            disp(['Camera ready to stream.']);
        end
        
        function stopStreaming(obj)            
            obj.pipe.stop();
            obj.pipe.delete();
            disp(['Camera stopped stream.']);
        end
        
        function img = cvtRealSenseFrame(obj, img, encoding)
            width = img.get_width();
            height = img.get_height();
            data = img.get_data();
            switch encoding
                case {'COLOR2RGB', 'COLORIZEDDEPTH2RGB'}
                    img = permute(reshape(data',[3, width, height]), [3 2 1]);                     
                otherwise
                    disp(['Cannot convert to such an encoding. ', ...
                        'Available encodings: ' ... 
                        'COLOR2RGB, COLORIZEDDEPTH2RGB']);                    
            end
        end
        
        function img = getRGBFrame(obj)    
            obj.frameset = obj.pipe.wait_for_frames();
            img = obj.frameset.get_color_frame();
            img = obj.cvtRealSenseFrame(img, 'COLOR2RGB');
        end
        
        
    end
end


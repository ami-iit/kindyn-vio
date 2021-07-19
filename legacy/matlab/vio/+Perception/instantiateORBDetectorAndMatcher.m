function [orb_tracker, desc_matcher] = instantiateORBDetectorAndMatcher()
orb_tracker = cv.ORB();
orb_tracker.MaxFeatures = 1000;   

opts = {'LSH', 'TableNumber',6, 'KeySize',12, 'MultiProbeLevel',1};
desc_matcher = cv.DescriptorMatcher('FlannBasedMatcher', 'Index', opts);
end


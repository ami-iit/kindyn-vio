function trans = rpypos2trans(rpy, pos)
R = Core.rpy2rot(rpy(1), rpy(2), rpy(3));
trans = [R pos; zeros(1, 3) 1];
end


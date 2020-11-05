function p_global = getBaseToCam(cam, cameraParams,I)
    CB_xyz = [263.8 101.6 -46.0761];
%     a = sqrt(CB_xyz(1)^2+CB_xyz(2)^2);
    T_baseToCB = fwkintrans(CB_xyz(3), 0, CB_xyz(1), 0)*...
        fwkintrans(0, 3.1415/2,CB_xyz(2), 0)*...
        fwkintrans(0, 3.1415/2, 0, 3.1415);
    
    %T_camToCB = getCamToCheckerboard(cam,cameraParams);
%     T_camToCB = [0.0097    -0.8379    0.5458  119.9130;
%                  0.9995     0.0251    0.0209  111.4456;
%                  -0.0312    0.5453    0.8377  278.7317;
%                  0          0         0       1.0000];
             
    T_camToCB = [
        0.0214   -0.8417    0.5395  118.9411
        0.9998    0.0206   -0.0074  101.2393
       -0.0049    0.5395    0.8420  271.8500
             0         0         0    1.0000];

    
    p_CB = pointsToWorld(cameraParams, T_camToCB(1:3,1:3),T_camToCB(1:3,4),I);
    %T_CBToCam = pinv(T_camToCB);
    
%     T_baseToCam = T_camToCB/T_baseToCB;
%     T_baseToCam = T_baseToCB * T_CBToCam;

    p_global = T_baseToCB * [transpose(p_CB); CB_xyz(3);1];
    
    

end
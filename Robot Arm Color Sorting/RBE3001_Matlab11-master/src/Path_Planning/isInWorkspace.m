function ptInWs = isInWorkspace(x,y,z)
    ptInWs = 1;
    
    if((x < 66) || (x > 350))
        ptInWs = 0;
    elseif ((y < -300) || (y > 300))
        ptInWs = 0;
    elseif ((z < -45) || (z > 300))
        ptInWs = 0;   
    end
end
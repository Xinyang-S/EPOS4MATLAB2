function [mV] = encTomV(startPos,currentPos)
%transform encoder readings to mV
    %to convert from mV to degrees: (mV/20)-100
    %device: cpt = 6400, range of -100 to 100 degrees, range from 0 to 4000 mV
    positionNow = (startPos-currentPos)*90/6400 + 100; %degrees, neutral = ~180
    if (positionNow > 399)
        positionNow = 399; %max based on analog pin
    elseif (positionNow < 0)
        positionNow = 0; %min based on analog pin
    end
    mV = positionNow*20; % degrees*10, analog output pin, analog mV value, max 4000
end
    


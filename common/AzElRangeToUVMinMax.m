function [u,v] = AzElRangeToUVMinMax(az,el,range,minEl,maxEl,minAz,maxAz,rows,cols)
%Calculate slope
mel = double((rows)/(abs(double(maxEl-minEl))));
maz = double((cols)/(abs(double(maxAz-minAz))));

%Calculate the (u,v) coordinate
u = floor(double(cols - (maz)*((az-minAz))));
v = floor(double(rows - (mel)*(el-minEl)));
end
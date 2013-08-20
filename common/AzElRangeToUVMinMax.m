function [u,v] = AzElRangeToUVMinMax(az,el,range,minEl,maxEl,minAz,maxAz,rows,cols)
%Calculate slope
mel = double((rows-1)/(abs(double(maxEl-minEl))));
maz = double((cols-1)/(abs(double(maxAz-minAz))));

%Calculate the (u,v) coordinate
u = round(double(cols - (maz)*((az-minAz))));
v = round(double(rows - (mel)*(el-minEl)));
end